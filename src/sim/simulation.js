// Simulation orchestrator: environment + vehicle plant + sensors + ArduSub
// firmware, stepped at the firmware loop rate with physics sub-steps.

import { RNG } from '../core/rng.js';
import { qFromEuler, qToEuler, qToMat, m3mulv, RAD, DEG } from '../core/math.js';
import { Environment } from '../physics/environment.js';
import { VehiclePlant } from '../physics/body.js';
import { buildPlantModel, cgOf } from '../vehicle/model.js';
import { thrusterLimits } from '../vehicle/config.js';
import { analyzeLayout } from '../alloc/allocation.js';
import { FRAMES, frameTable } from '../alloc/frames.js';
import { SensorSuite } from '../ardusub/sensors.js';
import { ArduSubSim, MODE } from '../ardusub/sub.js';
import { Telemetry } from './telemetry.js';

export function buildMixer(cfg, geo) {
  const warnings = [];
  const cg = cgOf(cfg, geo);
  const th = cfg.thrusters.map((t) => ({ ...t, ...thrusterLimits(t) }));
  const analysis = analyzeLayout(th, cg, { threshold: cfg.mixer.threshold ?? 0.02, enabledAxes: cfg.mixer.axisEnable });
  let factors, variant;
  const src = cfg.mixer.source;
  if (src === 'frame') {
    const fc = cfg.params.FRAME_CONFIG;
    factors = frameTable(fc);
    variant = FRAMES[fc]?.mixer ?? 'generic';
    if (FRAMES[fc]?.warn) warnings.push(FRAMES[fc].warn);
    if (factors.length !== cfg.thrusters.length) warnings.push(`FRAME_CONFIG ${fc} drives ${factors.length} motors but the vehicle has ${cfg.thrusters.length} thrusters.`);
  } else if (src === 'custom' && Array.isArray(cfg.mixer.custom) && cfg.mixer.custom.length) {
    factors = cfg.mixer.custom.map((r) => r.slice(0, 6).map(Number));
    variant = cfg.mixer.saturation === 'clamp' ? 'generic' : cfg.mixer.saturation;
  } else {
    factors = analysis.factors.map((r) => r.slice());
    variant = cfg.mixer.saturation === 'clamp' ? 'generic' : cfg.mixer.saturation || 'priority';
  }
  return { factors, variant, analysis, source: src, warnings, cg };
}

export class Simulation {
  constructor(cfg, geo, opts = {}) {
    this.opts = opts;
    this.telemetry = new Telemetry(opts.telemetryCapacity ?? 36000);
    this.timeScale = 1;
    this.paused = false;
    this.acc = 0;
    this.pilot = { x: 0, y: 0, z: 500, r: 0, s: 0, t: 0, buttons: 0 };
    this.buttonLatch = 0;
    this.nextSend = 0;
    this.listeners = new Set();
    this.rebuild(cfg, geo, { keepState: false });
  }

  rebuild(cfg = this.cfg, geo = this.geo, { keepState = true } = {}) {
    const prev = keepState && this.plant ? { y: Float64Array.from(this.plant.y), armed: this.fw?.armed, mode: this.fw?.mode, time: this.time, thrusters: this.plant.thrusters.map((t) => t.T), flood: { ...this.plant.flood } } : null;
    this.cfg = cfg;
    this.geo = geo;
    this.rng = new RNG(cfg.sim?.seed ?? 1);
    this.env = new Environment(cfg.environment, this.rng);
    const { plant, info } = buildPlantModel(cfg, geo, this.env);
    this.model = plant;
    this.info = info;
    this.mixer = buildMixer(cfg, geo);
    this.plant = new VehiclePlant(plant, this.env);
    this.sensors = new SensorSuite(this.rng);
    const loopHz = Math.min(1000, Math.max(50, cfg.params.SCHED_LOOP_RATE || 400));
    this.loopHz = loopHz;
    this.sensors.configure(cfg.sensors, cfg.params, loopHz, this.env);
    this.fw = new ArduSubSim({ params: cfg.params, mixer: this.mixer, sensors: this.sensors });
    this.substeps = Math.max(1, Math.round((cfg.sim?.physicsRate ?? 800) / loopHz));
    this.dtFw = 1 / loopHz;
    this.dtPhys = this.dtFw / this.substeps;
    this.recordEvery = Math.max(1, Math.round(loopHz / 50));
    this.loopCount = 0;
    if (prev) {
      this.plant.y.set(prev.y);
      this.plant.thrusters.forEach((t, i) => (t.T = prev.thrusters[i] ?? 0));
      if (prev.flood) this.plant.flood = { ...prev.flood };
      // the pool may have shrunk around the vehicle
      const inside = this.keepInPool(this.plant.position);
      if (inside.moved) { this.plant.y[0] = inside.p[0]; this.plant.y[1] = inside.p[1]; this.plant.y[2] = inside.p[2]; for (let i = 7; i < 13; i++) this.plant.y[i] = 0; }
      this.time = prev.time;
      this.fw.time = prev.time;
      this.sensors.update(this.plant, this.dtFw);
      if (prev.mode !== undefined) this.fw.setMode(prev.mode, 'restore');
      if (prev.armed) this.fw.arm('restore');
    } else {
      this.reset();
    }
    this.emit('rebuild');
  }

  // Clamp a CG position so the whole vehicle fits inside the pool (walls and floor).
  keepInPool(pos) {
    const fl = this.env.floor, r = Math.max(0.05, this.plant?.boundR || 0.3);
    const hx = Math.max(0, fl.L / 2 - r), hy = Math.max(0, fl.W / 2 - r);
    const p = [Math.min(hx, Math.max(-hx, pos[0])), Math.min(hy, Math.max(-hy, pos[1])), pos[2]];
    p[2] = Math.min(p[2], Math.max(0, fl.depthAt(p[0]) - r));
    const moved = Math.abs(p[0] - pos[0]) + Math.abs(p[1] - pos[1]) + Math.abs(p[2] - pos[2]) > 1e-9;
    return { p, moved };
  }

  reset(pose) {
    const s = this.cfg.start || {};
    const p = this.keepInPool(pose?.position ?? s.position ?? [0, 0, 1]).p;
    const yaw = (pose?.yaw ?? s.yaw ?? 0) * DEG;
    this.plant.reset({ position: p.slice(), quaternion: qFromEuler(0, 0, yaw) });
    this.sensors.reset();
    this.sensors.update(this.plant, this.dtFw);
    this.fw = new ArduSubSim({ params: this.cfg.params, mixer: this.mixer, sensors: this.sensors });
    this.time = 0;
    this.acc = 0;
    this.nextSend = 0;
    this.loopCount = 0;
    this.telemetry.clear();
    this.emit('reset');
  }

  on(fn) { this.listeners.add(fn); return () => this.listeners.delete(fn); }
  emit(ev, data) { for (const fn of this.listeners) fn(ev, data); }

  setPilotInput(msg) {
    Object.assign(this.pilot, msg);
    this.buttonLatch |= msg.buttons >>> 0;
  }

  // one firmware loop + physics sub-steps
  loop() {
    const fw = this.fw;
    if (this.time >= this.nextSend) {
      const sendRate = this.cfg.joystick?.sendRate ?? 50;
      this.nextSend = this.time + 1 / sendRate;
      if (!this.opts.noPilot) fw.handleManualControl({ ...this.pilot, buttons: this.pilot.buttons | this.buttonLatch });
      this.buttonLatch = 0;
    }
    this.sensors.update(this.plant, this.dtFw);
    const pwm = fw.fastLoop(this.dtFw);
    for (let k = 0; k < this.substeps; k++) {
      this.env.step(this.dtPhys);
      this.plant.step(this.dtPhys, pwm);
    }
    this.time += this.dtFw;
    this.loopCount++;
    if (this.loopCount % this.recordEvery === 0) this.record();
  }

  // advance by wall-clock seconds; returns number of firmware loops executed
  advance(realDt, maxLoops = 400) {
    if (this.paused) return 0;
    this.acc += Math.min(realDt, 0.25) * this.timeScale;
    let n = 0;
    while (this.acc >= this.dtFw && n < maxLoops) { this.loop(); this.acc -= this.dtFw; n++; }
    if (n >= maxLoops) this.acc = 0; // can't keep up: drop time rather than spiral
    return n;
  }

  runFor(seconds) {
    const n = Math.round(seconds / this.dtFw);
    for (let i = 0; i < n; i++) this.loop();
  }

  record() {
    const p = this.plant, fw = this.fw, s = this.sensors;
    const e = qToEuler(p.quaternion);
    const R = qToMat(p.quaternion);
    const st = fw.status();
    // targets only exist while a controller is active (NaN breaks the graph line)
    const attOn = fw.armed && fw.mode !== MODE.MANUAL;
    const depthOn = fw.armed && (fw.mode === MODE.ALT_HOLD || fw.mode === MODE.POSHOLD || fw.mode === MODE.SURFACE);
    const rec = {
      t: this.time, armed: fw.armed ? 1 : 0, mode: fw.mode,
      roll: e[0] * RAD, pitch: e[1] * RAD, yaw: e[2] * RAD,
      roll_t: attOn ? st.targetEuler[0] * RAD : NaN, pitch_t: attOn ? st.targetEuler[1] * RAD : NaN, yaw_t: attOn && fw.mode !== MODE.ACRO ? st.targetEuler[2] * RAD : NaN,
      ahrs_roll: s.roll * RAD, ahrs_pitch: s.pitch * RAD, ahrs_yaw: s.yaw * RAD,
      p: p.rates[0] * RAD, q: p.rates[1] * RAD, r: p.rates[2] * RAD,
      p_t: attOn ? st.rateTarget[0] * RAD : NaN, q_t: attOn ? st.rateTarget[1] * RAD : NaN, r_t: attOn ? st.rateTarget[2] * RAD : NaN,
      depth: p.position[2], depth_meas: s.depthMeasured, depth_t: depthOn ? st.depthTarget : NaN, climb: -m3mulv(R, p.velocityBody)[2],
      x: p.position[0], y: p.position[1], z: p.position[2],
      u: p.velocityBody[0], v: p.velocityBody[1], w: p.velocityBody[2],
      in_roll: st.inputs[0], in_pitch: st.inputs[1], in_yaw: st.inputs[2], in_thr: st.inputs[3], in_fwd: st.inputs[4], in_lat: st.inputs[5],
      rll_P: fw.att.pidRoll.info.P, rll_I: fw.att.pidRoll.info.I, rll_D: fw.att.pidRoll.info.D,
      pit_P: fw.att.pidPitch.info.P, pit_I: fw.att.pidPitch.info.I, pit_D: fw.att.pidPitch.info.D,
      yaw_P: fw.att.pidYaw.info.P, yaw_I: fw.att.pidYaw.info.I, yaw_D: fw.att.pidYaw.info.D,
      saturated: st.saturated ? 1 : 0, submerged: p.diag.submerged ?? 1, gain: fw.gain,
      press_abs: s.pressureMeas / 100, press_int: s.internalPressure / 100, flood: p.flood.volume * 1e6,
      floor_dist: this.env.floorDepth(p.position[0]) - p.position[2],
      thrust_total: p.thrusters.reduce((a, t) => a + Math.abs(t.Teff), 0),
    };
    for (let i = 0; i < p.thrusters.length && i < 12; i++) {
      rec[`m${i + 1}_out`] = st.motorsOut[i] ?? 0;
      rec[`m${i + 1}_pwm`] = st.pwm[i] ?? 1500;
      rec[`m${i + 1}_N`] = p.thrusters[i].Teff;
    }
    this.telemetry.push(rec);
  }

  // convenience for UI/tests
  state() {
    const p = this.plant;
    return { time: this.time, position: p.position, quaternion: p.quaternion, euler: qToEuler(p.quaternion), velocity: p.velocityBody, rates: p.rates, diag: p.diag, thrusters: p.thrusters.map((t) => ({ pwm: t.pwm, T: t.Teff })), fw: this.fw.status() };
  }
}

export { MODE };
