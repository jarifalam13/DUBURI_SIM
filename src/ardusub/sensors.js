// Sensor and state-estimate emulation feeding the firmware port:
//  - IMU: gyro/accel with noise, 2-pole low-pass (INS_GYRO_FILTER / INS_ACCEL_FILTER)
//  - AHRS attitude: truth + small slowly varying error, board trim (AHRS_TRIM_X/Y)
//  - Depth: external pressure sensor (Bar02 / Bar30 / custom) reading the
//    absolute water pressure at its mounting point (compressible-water
//    hydrostatics + wave pressure), with range limit, resolution, noise, rate
//    and delay; converted by AP_Baro's water formula
//    (ground_pressure - p) / 9800 / BARO_SPEC_GRAV and fused with the
//    accelerometer by a 3rd-order complementary filter (classic inertial nav)
//  - Internal (autopilot) barometer: enclosure air pressure, for FS_PRESS
//  - Leak detector: wet as soon as water has entered the housing (FS_LEAK)
//  - Optional horizontal positioning (DVL/USBL-like) for position hold

import { qFromEuler, qToEuler, qmul, qFromAxisAngleVec, qToMat, m3mulv, wrap360cd } from '../core/math.js';
import { LowPassFilter2p } from './control.js';
import { GaussMarkov } from '../core/rng.js';
import { depthSensorSpec } from '../vehicle/config.js';

export class SensorSuite {
  constructor(rng) {
    this.rng = rng;
    this.attErr = [new GaussMarkov(rng), new GaussMarkov(rng), new GaussMarkov(rng)];
    this.cfg = {};
    this.reset();
  }
  configure(cfg, params, loopHz, env) {
    this.cfg = cfg;
    this.env = env;
    this.loopHz = loopHz;
    this.gyroF = [0, 1, 2].map(() => new LowPassFilter2p(loopHz, params.INS_GYRO_FILTER));
    this.accF = [0, 1, 2].map(() => new LowPassFilter2p(loopHz, params.INS_ACCEL_FILTER));
    this.attErr.forEach((g) => { g.sigma = ((cfg.attitudeNoise ?? 0) * Math.PI) / 180; g.tau = 5; });
    this.trim = [params.AHRS_TRIM_X ?? 0, params.AHRS_TRIM_Y ?? 0];
    this.specGrav = params.BARO_SPEC_GRAV ?? 1;
    this.depthDelay = Math.max(0, (params.EK3_HGT_DELAY ?? 60) / 1000);
    this.spec = depthSensorSpec(cfg);
    // AP_Baro calibrates ground pressure at boot: the vehicle is powered on at the surface
    this.groundPressure = this.quantize(env ? env.pAtm : 101325);
  }
  quantize(p) {
    const s = this.spec;
    if (!s) return p;
    let v = Math.min(s.maxPa, Math.max(0, p));
    if (s.resPa > 0) v = Math.round(v / s.resPa) * s.resPa;
    return v;
  }
  reset() {
    this.t = 0;
    this.gyro = [0, 0, 0];
    this.gyroLatest = [0, 0, 0];
    this.accelBody = [0, 0, -9.80665];
    this.quat = [1, 0, 0, 0];
    this.euler = [0, 0, 0];
    this.baroQueue = [];
    this.nextBaro = 0;
    this.baroAlt = null; // m, up positive (negative underwater)
    this.inav = null;
    this.posHist = [];
    this.nextPos = 0;
    this.posMeas = null;
    this.depthHealthy = true;
    this.pressureMeas = NaN; // Pa absolute, external sensor
    this.pressureTrue = NaN;
    this.depthOverRange = false;
    this.internalPressure = 101325;
    this.leak = false;
  }

  // plant: VehiclePlant; dt: firmware loop period
  update(plant, dt) {
    const c = this.cfg, rng = this.rng;
    this.t += dt;
    // --- gyro ---
    const w = plant.rates;
    const gn = c.gyroNoise ?? 0;
    for (let i = 0; i < 3; i++) this.gyroLatest[i] = this.gyroF[i].apply(w[i] + gn * rng.gauss());
    this.gyro = this.gyroLatest.slice();
    // --- attitude estimate (truth + slowly varying error), board trim ---
    let q = plant.quaternion;
    if ((c.attitudeNoise ?? 0) > 0) q = qmul(q, qFromAxisAngleVec(this.attErr.map((g) => g.step(dt))));
    const e = qToEuler(q);
    e[0] -= this.trim[0];
    e[1] -= this.trim[1];
    this.euler = e;
    this.quat = qFromEuler(e[0], e[1], e[2]);
    // --- accelerometer ---
    const f = plant.specificForceBody();
    const an = c.accelNoise ?? 0;
    for (let i = 0; i < 3; i++) this.accelBody[i] = this.accF[i].apply(f[i] + an * rng.gauss());
    const R = qToMat(this.quat);
    this.accelEf = m3mulv(R, this.accelBody); // specific force in NED
    // --- pressure / depth ---
    const env = this.env;
    const pos = plant.position;
    const depthTrue = pos[2] + env.surfaceElevation(pos[0], pos[1]);
    if (this.t >= this.nextBaro) {
      this.nextBaro = this.t + 1 / Math.max(1, c.depthRate ?? 50);
      // absolute pressure at the sensor's mounting point
      const sp = c.depthSensorPos || [0, 0, 0];
      const Rq = qToMat(plant.quaternion);
      const sx = pos[0] + Rq[0] * sp[0] + Rq[1] * sp[1] + Rq[2] * sp[2];
      const sy = pos[1] + Rq[3] * sp[0] + Rq[4] * sp[1] + Rq[5] * sp[2];
      const sz = pos[2] + Rq[6] * sp[0] + Rq[7] * sp[1] + Rq[8] * sp[2];
      const pTrue = env.absolutePressureAt(sx, sy, sz);
      const pm = this.quantize(pTrue + (c.depthNoise ?? 0) * env.rho * env.g * rng.gauss());
      this.pressureTrue = pTrue;
      this.depthOverRange = pTrue > this.spec.maxPa;
      // AP_Baro (BARO_TYPE_WATER): altitude = (ground_pressure - pressure) / 9800 / specific gravity
      const alt = (this.groundPressure - pm) / 9800 / this.specGrav;
      this.baroQueue.push({ t: this.t + this.depthDelay, alt, p: pm });
    }
    while (this.baroQueue.length && this.baroQueue[0].t <= this.t) { const b = this.baroQueue.shift(); this.baroAlt = b.alt; this.pressureMeas = b.p; }
    // --- enclosure: autopilot barometer and leak detector ---
    const fl = plant.flood;
    this.internalPressure = fl ? fl.pIn : env.pAtm;
    this.leak = !!(fl && plant.m.hull?.leakSensor && fl.volume > 1e-6);
    // --- inertial navigation (vertical, NEU cm) ---
    const azUp = -(this.accelEf[2] + 9.80665); // m/s^2 up
    if (!this.inav) {
      const z0 = this.baroAlt ?? -depthTrue;
      const v = plant.velocityBody;
      const Rt = qToMat(plant.quaternion);
      const vned = m3mulv(Rt, v);
      this.inav = { z: z0, vz: -vned[2], bias: 0, x: plant.position[0], y: plant.position[1], vx: vned[0], vy: vned[1] };
      this.zHist = [];
    }
    const nav = this.inav;
    const tau = 0.6;
    const k1 = 3 / tau, k2 = 3 / (tau * tau), k3 = 1 / (tau * tau * tau);
    // compare with the position from depthDelay ago
    this.zHist.push({ t: this.t, z: nav.z });
    while (this.zHist.length > 2 && this.zHist[0].t < this.t - this.depthDelay) this.zHist.shift();
    const zPast = this.zHist[0].z;
    const err = this.baroAlt !== null ? this.baroAlt - zPast : 0;
    nav.bias += err * k3 * dt;
    nav.vz += (azUp + nav.bias) * dt + err * k2 * dt;
    nav.z += nav.vz * dt + err * k1 * dt;
    // --- horizontal positioning (optional) ---
    const Rt = qToMat(plant.quaternion);
    const vned = m3mulv(Rt, plant.velocityBody);
    if (c.positioning) {
      if (this.t >= this.nextPos) {
        this.nextPos = this.t + 1 / Math.max(1, c.positionRate ?? 10);
        const pn = c.positionNoise ?? 0;
        this.posMeas = [plant.position[0] + pn * rng.gauss(), plant.position[1] + pn * rng.gauss()];
      }
      const a = 1 - Math.exp(-dt / 0.3);
      nav.vx += (vned[0] - nav.vx) * a;
      nav.vy += (vned[1] - nav.vy) * a;
      nav.x += nav.vx * dt; nav.y += nav.vy * dt;
      if (this.posMeas) { nav.x += (this.posMeas[0] - nav.x) * 2 * dt; nav.y += (this.posMeas[1] - nav.y) * 2 * dt; }
    }
    this.velNed = vned;
  }

  // ---- AHRS-style accessors ----
  get roll() { return this.euler[0]; }
  get pitch() { return this.euler[1]; }
  get yaw() { return this.euler[2]; }
  get yawSensor() { return wrap360cd(Math.trunc((this.euler[2] * 18000) / Math.PI)); }
  get rollSensor() { return Math.trunc((this.euler[0] * 18000) / Math.PI); }
  get pitchSensor() { return Math.trunc((this.euler[1] * 18000) / Math.PI); }
  get cosRoll() { return Math.cos(this.euler[0]); }
  get cosPitch() { return Math.cos(this.euler[1]); }
  get cosYaw() { return Math.cos(this.euler[2]); }
  get sinYaw() { return Math.sin(this.euler[2]); }
  // inertial nav (cm, NEU)
  get positionZUpCm() { return this.inav ? this.inav.z * 100 : 0; }
  get velocityZUpCms() { return this.inav ? this.inav.vz * 100 : 0; }
  get zAccelCmss() { return -(this.accelEf[2] + 9.80665) * 100; }
  get positionXYCm() { return this.inav ? [this.inav.x * 100, this.inav.y * 100] : [0, 0]; }
  get velocityXYCms() { return this.inav ? [this.inav.vx * 100, this.inav.vy * 100] : [0, 0]; }
  get depthMeasured() { return this.baroAlt === null ? 0 : -this.baroAlt; }
  positionOk() { return !!this.cfg.positioning; }
}
