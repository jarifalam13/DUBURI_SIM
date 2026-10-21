// Port of the parts of AC_PosControl (ArduSub 4.5.7) used by ArduSub's depth
// hold, surface and position hold modes. Units follow ArduPilot: cm, cm/s, cm/s^2, NEU.

import { AC_P_1D, AC_P_2D, AC_PID_Basic, AC_PID, AC_PID_2D } from './pid.js';
import { shapeVelAccel, shapePosVelAccel, updatePosVelAccel, updateVelAccel, stoppingDistance, angleToAccel, accelToAngle } from './control.js';
import { clamp, isPositive, isZero } from '../core/math.js';

const OVERSPEED_GAIN_Z = 2.0;
const RELAX_TC = 0.16;
const THROTTLE_CUTOFF_HZ = 2.0;
const STOP_UP_MAX = 300, STOP_DOWN_MAX = 200;

export class PosControl {
  constructor(ahrs, motors, att) {
    this.ahrs = ahrs; this.motors = motors; this.att = att;
    this.dt = 0.0025;
    this.pPosZ = new AC_P_1D(3);
    this.pidVelZ = new AC_PID_Basic();
    this.pidAccelZ = new AC_PID();
    this.pPosXY = new AC_P_2D(1);
    this.pidVelXY = new AC_PID_2D();
    this.velMaxDown = -150; this.velMaxUp = 250; this.accelMaxZ = 250; this.jerkMaxZ = 500; this.shapingJerkZ = 5;
    this.velMaxXY = 500; this.accelMaxXY = 100; this.shapingJerkXY = 5;
    this.posTarget = [0, 0, 0]; this.velDesired = [0, 0, 0]; this.velTarget = [0, 0, 0]; this.accelDesired = [0, 0, 0]; this.accelTarget = [0, 0, 0];
    this.posOffsetZ = 0; this.velOffsetZ = 0; this.accelOffsetZ = 0; this.posOffsetTargetZ = 0;
    this.limitZ = 0; this.limitXY = [0, 0];
    this.lastUpdateZ = -1e9; this.lastUpdateXY = -1e9; this.tick = 0;
    this.rollTarget = 0; this.pitchTarget = 0; this.leanAngleMaxCd = 0;
    this.angleMaxCd = 4500;
  }
  setParams(P) {
    this.pPosZ.kp = P.PSC_POSZ_P;
    this.pidVelZ.setGains({ p: P.PSC_VELZ_P, i: P.PSC_VELZ_I, d: P.PSC_VELZ_D, imax: P.PSC_VELZ_IMAX, ff: P.PSC_VELZ_FF, flte: P.PSC_VELZ_FLTE, fltd: P.PSC_VELZ_FLTD });
    this.pidAccelZ.setGains({ p: P.PSC_ACCZ_P, i: P.PSC_ACCZ_I, d: P.PSC_ACCZ_D, imax: P.PSC_ACCZ_IMAX, ff: P.PSC_ACCZ_FF, fltt: P.PSC_ACCZ_FLTT, flte: P.PSC_ACCZ_FLTE, fltd: P.PSC_ACCZ_FLTD, pdmx: P.PSC_ACCZ_PDMX });
    this.shapingJerkZ = P.PSC_JERK_Z;
    this.pPosXY.kp = P.PSC_POSXY_P;
    this.pidVelXY.setGains({ p: P.PSC_VELXY_P, i: P.PSC_VELXY_I, d: P.PSC_VELXY_D, imax: P.PSC_VELXY_IMAX, flte: P.PSC_VELXY_FLTE, fltd: P.PSC_VELXY_FLTD });
    this.shapingJerkXY = P.PSC_JERK_XY;
    this.angleMaxCd = P.ANGLE_MAX;
  }
  // called once per main loop by the firmware so is_active checks work
  advanceTick() { this.tick++; }

  // ---------------- vertical ----------------
  setMaxSpeedAccelZ(speedDown, speedUp, accel) {
    speedDown = -Math.abs(speedDown);
    if (speedDown < 0) this.velMaxDown = speedDown;
    if (speedUp > 0) this.velMaxUp = speedUp;
    if (accel > 0) this.accelMaxZ = accel;
    this.jerkMaxZ = this.shapingJerkZ * 100;
    if (isPositive(this.pidAccelZ.filtT)) this.jerkMaxZ = Math.min(this.jerkMaxZ, (Math.min(980.665, this.accelMaxZ) * (2 * Math.PI * this.pidAccelZ.filtT)) / 5);
    if (isPositive(this.pidAccelZ.filtE)) this.jerkMaxZ = Math.min(this.jerkMaxZ, (Math.min(980.665, this.accelMaxZ) * (2 * Math.PI * this.pidAccelZ.filtE)) / 5);
  }
  setCorrectionSpeedAccelZ(speedDown, speedUp, accel) {
    this.pPosZ.setLimits(-Math.abs(speedDown), speedUp, accel, 0);
  }
  initZController() {
    const a = this.ahrs;
    this.posTarget[2] = a.positionZUpCm;
    const v = a.velocityZUpCms;
    this.velDesired[2] = v; this.velTarget[2] = v;
    this.pidVelZ.resetFilter(); this.pidVelZ.setIntegrator(0);
    this.accelDesired[2] = clamp(a.zAccelCmss, -this.accelMaxZ, this.accelMaxZ);
    this.accelTarget[2] = this.accelDesired[2];
    this.pidAccelZ.resetFilter();
    this.posOffsetTargetZ = 0; this.posOffsetZ = 0; this.velOffsetZ = 0; this.accelOffsetZ = 0;
    this.pidAccelZ.setIntegrator((this.att.throttleIn - this.motors.throttleHover) * 1000 - this.pidAccelZ.kp * (this.accelTarget[2] - a.zAccelCmss) - this.pidAccelZ.kff * this.accelTarget[2]);
    this.lastUpdateZ = this.tick;
  }
  relaxZController(throttle) {
    this.initZController();
    this.pidAccelZ.relaxIntegrator((throttle - this.motors.throttleHover) * 1000, this.dt, RELAX_TC);
  }
  overspeedGain() {
    if (this.velDesired[2] < this.velMaxDown && !isZero(this.velMaxDown)) return (OVERSPEED_GAIN_Z * this.velDesired[2]) / this.velMaxDown;
    if (this.velDesired[2] > this.velMaxUp && !isZero(this.velMaxUp)) return (OVERSPEED_GAIN_Z * this.velDesired[2]) / this.velMaxUp;
    return 1;
  }
  inputVelAccelZ(vel, accel, limitOutput = true) {
    const g = this.overspeedGain();
    const aMax = this.accelMaxZ * g, jMax = this.jerkMaxZ * g;
    const st = { pos: this.posTarget[2], vel: this.velDesired[2] };
    updatePosVelAccel(st, this.accelDesired[2], this.dt, this.limitZ, this.pPosZ.error, this.pidVelZ.error);
    this.posTarget[2] = st.pos; this.velDesired[2] = st.vel;
    const s = { accel: this.accelDesired[2] };
    shapeVelAccel(vel, accel, this.velDesired[2], s, -clamp(aMax, 0, 750), aMax, jMax, this.dt, limitOutput);
    this.accelDesired[2] = s.accel;
    return updateVelAccel(vel, accel, this.dt, 0, 0);
  }
  updatePosOffsetZ(target) {
    const st = { pos: this.posOffsetZ, vel: this.velOffsetZ };
    updatePosVelAccel(st, this.accelOffsetZ, this.dt, Math.min(this.limitZ, 0), this.pPosZ.error, this.pidVelZ.error);
    this.posOffsetZ = st.pos; this.velOffsetZ = st.vel;
    const s = { accel: this.accelOffsetZ };
    shapePosVelAccel(target, 0, 0, this.posOffsetZ, this.velOffsetZ, s, this.velMaxDown, this.velMaxUp, -this.accelMaxZ, this.accelMaxZ, this.jerkMaxZ, this.dt, false);
    this.accelOffsetZ = s.accel;
  }
  setPosTargetZFromClimbRateCm(vel) {
    this.posTarget[2] -= this.posOffsetZ;
    this.velDesired[2] -= this.velOffsetZ;
    this.accelDesired[2] -= this.accelOffsetZ;
    this.inputVelAccelZ(vel, 0);
    this.updatePosOffsetZ(this.posOffsetTargetZ);
    this.posTarget[2] += this.posOffsetZ;
    this.velDesired[2] += this.velOffsetZ;
    this.accelDesired[2] += this.accelOffsetZ;
  }
  getPosTargetZCm() { return this.posTarget[2]; }
  setPosTargetZCm(z) { this.posTarget[2] = z; }
  getMaxSpeedUpCms() { return this.velMaxUp; }

  updateZController() {
    if (this.tick - this.lastUpdateZ > 1) this.initZController();
    this.lastUpdateZ = this.tick;
    const a = this.ahrs, m = this.motors;
    const r = this.pPosZ.updateAll(this.posTarget[2], a.positionZUpCm);
    this.posTarget[2] = r.target;
    this.velTarget[2] = r.out + this.velDesired[2];
    const vz = a.velocityZUpCms;
    this.accelTarget[2] = this.pidVelZ.updateAll(this.velTarget[2], vz, this.dt, m.limit.throttle_lower, m.limit.throttle_upper) + this.accelDesired[2];
    if (m.throttleHover * 1000 > this.pidAccelZ.kimax) this.pidAccelZ.kimax = m.throttleHover * 1000;
    let thr = this.pidAccelZ.updateAll(this.accelTarget[2], a.zAccelCmss, this.dt, m.limit.throttle_lower || m.limit.throttle_upper) * 0.001;
    thr += this.pidAccelZ.getFF() * 0.001;
    thr += m.throttleHover;
    this.att.setThrottleOut(thr, true, THROTTLE_CUTOFF_HZ);
    this.limitZ = m.limit.throttle_upper ? 1 : m.limit.throttle_lower ? -1 : 0;
    this.lastThrottle = thr;
  }

  // ---------------- horizontal (position hold) ----------------
  setMaxSpeedAccelXY(speed, accel) {
    this.velMaxXY = speed; this.accelMaxXY = accel;
    this.jerkMaxXY = this.shapingJerkXY * 100;
  }
  setCorrectionSpeedAccelXY(speed, accel) { this.pPosXY.setLimits(speed, accel, 0); }
  initXYController() {
    const a = this.ahrs;
    const p = a.positionXYCm, v = a.velocityXYCms;
    this.posTarget[0] = p[0]; this.posTarget[1] = p[1];
    this.velDesired[0] = v[0]; this.velDesired[1] = v[1];
    this.velTarget[0] = v[0]; this.velTarget[1] = v[1];
    this.accelDesired[0] = 0; this.accelDesired[1] = 0;
    this.pidVelXY.resetFilter();
    this.pidVelXY.setIntegrator([this.accelTarget[0], this.accelTarget[1]]);
    this.lastUpdateXY = this.tick;
  }
  initXYControllerStoppingPoint() {
    this.initXYController();
    const a = this.ahrs;
    const p = a.positionXYCm, v = a.velocityXYCms;
    const vt = Math.hypot(v[0], v[1]);
    this.posTarget[0] = p[0]; this.posTarget[1] = p[1];
    if (vt > 0) {
      const d = stoppingDistance(clamp(vt, 0, this.velMaxXY), this.pPosXY.kp, this.accelMaxXY);
      if (d > 0) { this.posTarget[0] += (v[0] * d) / vt; this.posTarget[1] += (v[1] * d) / vt; }
    }
    this.velDesired[0] = this.velDesired[1] = 0;
    this.accelDesired[0] = this.accelDesired[1] = 0;
  }
  updateXYController() {
    if (this.tick - this.lastUpdateXY > 1) this.initXYController();
    this.lastUpdateXY = this.tick;
    const a = this.ahrs;
    const r = this.pPosXY.updateAll([this.posTarget[0], this.posTarget[1]], a.positionXYCm);
    this.posTarget[0] = r.target[0]; this.posTarget[1] = r.target[1];
    this.velTarget[0] = r.out[0] + this.velDesired[0];
    this.velTarget[1] = r.out[1] + this.velDesired[1];
    const acc = this.pidVelXY.updateAll([this.velTarget[0], this.velTarget[1]], a.velocityXYCms, this.dt, this.limitXY);
    this.accelTarget[0] = acc[0] + this.accelDesired[0];
    this.accelTarget[1] = acc[1] + this.accelDesired[1];
    const angleMax = Math.min(this.att.getAltholdLeanAngleMaxCd(), this.angleMaxCd);
    const accelMax = angleToAccel(angleMax * 0.01) * 100;
    const l = Math.hypot(this.accelTarget[0], this.accelTarget[1]);
    this.limitXY = [0, 0];
    if (l > accelMax && l > 0) {
      this.limitXY = [this.accelTarget[0], this.accelTarget[1]];
      this.accelTarget[0] *= accelMax / l; this.accelTarget[1] *= accelMax / l;
    }
    // accel_to_lean_angles
    const fwd = this.accelTarget[0] * a.cosYaw + this.accelTarget[1] * a.sinYaw;
    const right = -this.accelTarget[0] * a.sinYaw + this.accelTarget[1] * a.cosYaw;
    this.pitchTarget = accelToAngle(-fwd * 0.01) * 100;
    const cp = Math.cos((this.pitchTarget * Math.PI) / 18000);
    this.rollTarget = accelToAngle(right * cp * 0.01) * 100;
  }
}
