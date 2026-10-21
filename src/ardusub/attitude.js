// Port of AC_AttitudeControl + AC_AttitudeControl_Sub (ArduSub 4.5.7).
// Quaternion attitude controller with input shaping, angle P (sqrt controller),
// thrust/heading error split and body-rate PIDs driving the motor mixer.

import { AC_PID } from './pid.js';
import { sqrtController, invSqrtController } from './control.js';
import {
  qmul, qconj, qnormalize, qrotate, qFromEuler, qToEuler, qFromAxisAngleVec, qFromAxisAngle, qToAxisAngle,
  wrapPI, clamp, isZero, isPositive, isNegative, vcross, vdot, vlen, DEG, RAD,
} from '../core/math.js';

const THRUST_ERROR_ANGLE = 30 * DEG;
const YAW_MAX_ERROR_ANGLE = 45 * DEG;
const ACCEL_RP_MIN = 40 * DEG, ACCEL_RP_MAX = 720 * DEG;
const ACCEL_Y_MIN = 10 * DEG, ACCEL_Y_MAX = 120 * DEG;
const ANGLE_LIMIT_THROTTLE_MAX = 0.8;
const ANGLE_LIMIT_MIN_DEG = 10;
const RATE_RELAX_TC = 0.16;

export class AttitudeControlSub {
  constructor(ahrs, motors) {
    this.ahrs = ahrs;
    this.motors = motors;
    this.dt = 0.0025;
    this.pidRoll = new AC_PID();
    this.pidPitch = new AC_PID();
    this.pidYaw = new AC_PID();
    this.angP = [6, 6, 6];
    this.useSqrt = true;
    this.rateRpTc = 0;
    this.rateYTc = 0;
    this.attitudeTarget = [1, 0, 0, 0];
    this.eulerAngleTarget = [0, 0, 0];
    this.eulerRateTarget = [0, 0, 0];
    this.angVelTarget = [0, 0, 0];
    this.angVelBody = [0, 0, 0];
    this.attitudeAngError = [1, 0, 0, 0];
    this.thrustAngle = 0;
    this.thrustErrorAngle = 0;
    this.feedforwardScalar = 1;
    this.althold_lean_angle_max = 0;
    this.throttleIn = 0;
    this.p = {};
  }

  setParams(P) {
    const rp = (ax) => ({ p: P[`ATC_RAT_${ax}_P`], i: P[`ATC_RAT_${ax}_I`], d: P[`ATC_RAT_${ax}_D`], imax: P[`ATC_RAT_${ax}_IMAX`], fltt: P[`ATC_RAT_${ax}_FLTT`], flte: P[`ATC_RAT_${ax}_FLTE`], fltd: P[`ATC_RAT_${ax}_FLTD`], pdmx: P[`ATC_RAT_${ax}_PDMX`] });
    this.pidRoll.setGains(rp('RLL'));
    this.pidPitch.setGains(rp('PIT'));
    this.pidYaw.setGains(rp('YAW'));
    this.angP = [P.ATC_ANG_RLL_P, P.ATC_ANG_PIT_P, P.ATC_ANG_YAW_P];
    this.p = {
      accelRollMax: P.ATC_ACCEL_R_MAX, accelPitchMax: P.ATC_ACCEL_P_MAX, accelYawMax: P.ATC_ACCEL_Y_MAX,
      rateFF: !!P.ATC_RATE_FF_ENAB, inputTc: P.ATC_INPUT_TC, slewYaw: P.ATC_SLEW_YAW,
      angVelRollMax: P.ATC_RATE_R_MAX, angVelPitchMax: P.ATC_RATE_P_MAX, angVelYawMax: P.ATC_RATE_Y_MAX,
      angleLimitTc: P.ATC_ANG_LIM_TC,
    };
  }

  // ---- accessors (ArduPilot units) ----
  accelRollMaxRadss() { return (this.p.accelRollMax * 0.01) * DEG; }
  accelPitchMaxRadss() { return (this.p.accelPitchMax * 0.01) * DEG; }
  accelYawMaxRadss() { return (this.p.accelYawMax * 0.01) * DEG; }
  slewYawMaxRads() {
    const degs = !isPositive(this.p.angVelYawMax) ? this.p.slewYaw * 0.01 : Math.min(this.p.angVelYawMax, this.p.slewYaw * 0.01);
    return degs * DEG;
  }
  getAttTargetEulerCd() { return this.eulerAngleTarget.map((v) => v * RAD * 100); }
  getAltholdLeanAngleMaxCd() { return Math.max(this.althold_lean_angle_max * RAD, ANGLE_LIMIT_MIN_DEG) * 100; }

  // ---- resets ----
  relaxAttitudeControllers() {
    this.attitudeTarget = this.ahrs.quat.slice();
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    this.attitudeAngError = [1, 0, 0, 0];
    this.angVelTarget = this.ahrs.gyro.slice();
    this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || [0, 0, 0];
    this.angVelBody = this.ahrs.gyro.slice();
    this.thrustErrorAngle = 0;
    for (const pid of [this.pidRoll, this.pidPitch, this.pidYaw]) { pid.resetFilter(); pid.resetI(); }
  }
  resetRateControllerITerms() { for (const pid of [this.pidRoll, this.pidPitch, this.pidYaw]) pid.resetI(); }
  resetRateControllerITermsSmoothly() { for (const pid of [this.pidRoll, this.pidPitch, this.pidYaw]) pid.relaxIntegrator(0, this.dt, RATE_RELAX_TC); }

  // ---- input functions ----
  inputEulerAngleRollPitchEulerRateYaw(rollCd, pitchCd, yawRateCds) {
    const roll = rollCd * 0.01 * DEG, pitch = pitchCd * 0.01 * DEG, yawRate = yawRateCds * 0.01 * DEG;
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    if (this.p.rateFF) {
      const ea = eulerAccelLimit(this.eulerAngleTarget, [this.accelRollMaxRadss(), this.accelPitchMaxRadss(), this.accelYawMaxRadss()]);
      this.eulerRateTarget[0] = inputShapingAngle(wrapPI(roll - this.eulerAngleTarget[0]), this.p.inputTc, ea[0], this.eulerRateTarget[0], 0, 0, this.dt);
      this.eulerRateTarget[1] = inputShapingAngle(wrapPI(pitch - this.eulerAngleTarget[1]), this.p.inputTc, ea[1], this.eulerRateTarget[1], 0, 0, this.dt);
      this.eulerRateTarget[2] = inputShapingAngVel(this.eulerRateTarget[2], yawRate, ea[2], this.dt, this.rateYTc);
      this.angVelTarget = eulerRateToAngVel(this.eulerAngleTarget, this.eulerRateTarget);
      angVelLimit(this.angVelTarget, this.p.angVelRollMax * DEG, this.p.angVelPitchMax * DEG, this.p.angVelYawMax * DEG);
      this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    } else {
      this.eulerAngleTarget[0] = roll;
      this.eulerAngleTarget[1] = pitch;
      this.eulerAngleTarget[2] += yawRate * this.dt;
      this.attitudeTarget = qFromEuler(...this.eulerAngleTarget);
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
    }
    this.attitudeControllerRunQuat();
  }

  inputEulerAngleRollPitchYaw(rollCd, pitchCd, yawCd, slewYaw) {
    const roll = rollCd * 0.01 * DEG, pitch = pitchCd * 0.01 * DEG, yaw = yawCd * 0.01 * DEG;
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    const slewMax = this.slewYawMaxRads();
    if (this.p.rateFF) {
      const ea = eulerAccelLimit(this.eulerAngleTarget, [this.accelRollMaxRadss(), this.accelPitchMaxRadss(), this.accelYawMaxRadss()]);
      this.eulerRateTarget[0] = inputShapingAngle(wrapPI(roll - this.eulerAngleTarget[0]), this.p.inputTc, ea[0], this.eulerRateTarget[0], 0, 0, this.dt);
      this.eulerRateTarget[1] = inputShapingAngle(wrapPI(pitch - this.eulerAngleTarget[1]), this.p.inputTc, ea[1], this.eulerRateTarget[1], 0, 0, this.dt);
      this.eulerRateTarget[2] = inputShapingAngle(wrapPI(yaw - this.eulerAngleTarget[2]), this.p.inputTc, ea[2], this.eulerRateTarget[2], 0, 0, this.dt);
      if (slewYaw) this.eulerRateTarget[2] = clamp(this.eulerRateTarget[2], -slewMax, slewMax);
      this.angVelTarget = eulerRateToAngVel(this.eulerAngleTarget, this.eulerRateTarget);
      angVelLimit(this.angVelTarget, this.p.angVelRollMax * DEG, this.p.angVelPitchMax * DEG, this.p.angVelYawMax * DEG);
      this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    } else {
      this.eulerAngleTarget[0] = roll;
      this.eulerAngleTarget[1] = pitch;
      if (slewYaw) {
        const err = clamp(wrapPI(yaw - this.eulerAngleTarget[2]), -slewMax * this.dt, slewMax * this.dt);
        this.eulerAngleTarget[2] = wrapPI(err + this.eulerAngleTarget[2]);
      } else this.eulerAngleTarget[2] = yaw;
      this.attitudeTarget = qFromEuler(...this.eulerAngleTarget);
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
    }
    this.attitudeControllerRunQuat();
  }

  inputRateBfRollPitchYaw(rollCds, pitchCds, yawCds) {
    const r = rollCds * 0.01 * DEG, p = pitchCds * 0.01 * DEG, y = yawCds * 0.01 * DEG;
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    if (this.p.rateFF) {
      this.angVelTarget[0] = inputShapingAngVel(this.angVelTarget[0], r, this.accelRollMaxRadss(), this.dt, this.rateRpTc);
      this.angVelTarget[1] = inputShapingAngVel(this.angVelTarget[1], p, this.accelPitchMaxRadss(), this.dt, this.rateRpTc);
      this.angVelTarget[2] = inputShapingAngVel(this.angVelTarget[2], y, this.accelYawMaxRadss(), this.dt, this.rateYTc);
      this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    } else {
      const upd = qFromAxisAngleVec([r * this.dt, p * this.dt, y * this.dt]);
      this.attitudeTarget = qnormalize(qmul(this.attitudeTarget, upd));
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
    }
    this.attitudeControllerRunQuat();
  }

  // ---- core ----
  attitudeControllerRunQuat() {
    const body = this.ahrs.quat;
    const { attitudeError, thrustAngle, thrustErrorAngle, target } = this.thrustHeadingRotationAngles(this.attitudeTarget, body);
    this.attitudeTarget = target;
    this.thrustAngle = thrustAngle;
    this.thrustErrorAngle = thrustErrorAngle;
    let avb = this.updateAngVelTargetFromAttError(attitudeError);
    angVelLimit(avb, this.p.angVelRollMax * DEG, this.p.angVelPitchMax * DEG, this.p.angVelYawMax * DEG);
    const rotTargetToBody = qmul(qconj(body), this.attitudeTarget);
    const ff = qrotate(rotTargetToBody, this.angVelTarget);
    this.feedforwardScalar = 1;
    const gyroZ = this.ahrs.gyro[2];
    if (thrustErrorAngle > THRUST_ERROR_ANGLE * 2) {
      avb[2] = gyroZ;
    } else if (thrustErrorAngle > THRUST_ERROR_ANGLE) {
      this.feedforwardScalar = 1 - (thrustErrorAngle - THRUST_ERROR_ANGLE) / THRUST_ERROR_ANGLE;
      avb[0] += ff[0] * this.feedforwardScalar;
      avb[1] += ff[1] * this.feedforwardScalar;
      avb[2] += ff[2];
      avb[2] = gyroZ * (1 - this.feedforwardScalar) + avb[2] * this.feedforwardScalar;
    } else {
      avb = [avb[0] + ff[0], avb[1] + ff[1], avb[2] + ff[2]];
    }
    this.angVelBody = avb;
    if (this.p.rateFF) {
      const upd = qFromAxisAngleVec([this.angVelTarget[0] * this.dt, this.angVelTarget[1] * this.dt, this.angVelTarget[2] * this.dt]);
      this.attitudeTarget = qmul(this.attitudeTarget, upd);
    }
    this.attitudeTarget = qnormalize(this.attitudeTarget);
    this.attitudeAngError = qmul(qconj(body), this.attitudeTarget);
  }

  thrustVectorRotationAngles(target, body) {
    const up = [0, 0, -1];
    const tv = qrotate(target, up), bv = qrotate(body, up);
    const thrustAngle = Math.acos(clamp(vdot(up, bv), -1, 1));
    let cross = vcross(bv, tv);
    const thrustErrorAngle = Math.acos(clamp(vdot(bv, tv), -1, 1));
    const l = vlen(cross);
    if (isZero(l) || isZero(thrustErrorAngle)) cross = up;
    else cross = cross.map((v) => v / l);
    cross = qrotate(qconj(body), cross);
    const correction = qFromAxisAngle(cross, thrustErrorAngle);
    const rot = qToAxisAngle(correction);
    const headingQ = qmul(qmul(qconj(correction), qconj(body)), target);
    const rotH = qToAxisAngle(headingQ);
    return { correction, attitudeError: [rot[0], rot[1], rotH[2]], thrustAngle, thrustErrorAngle };
  }

  thrustHeadingRotationAngles(target, body) {
    const r = this.thrustVectorRotationAngles(target, body);
    const err = r.attitudeError;
    let newTarget = target;
    const headingAccelMax = clamp(this.accelYawMaxRadss() / 2, ACCEL_Y_MIN, ACCEL_Y_MAX);
    if (!isZero(this.pidYaw.kp)) {
      const headingErrorMax = Math.min(invSqrtController(1 / this.pidYaw.kp, this.angP[2], headingAccelMax), YAW_MAX_ERROR_ANGLE);
      if (!isZero(this.angP[2]) && Math.abs(err[2]) > headingErrorMax) {
        err[2] = clamp(wrapPI(err[2]), -headingErrorMax, headingErrorMax);
        const hq = qFromAxisAngleVec([0, 0, err[2]]);
        newTarget = qmul(qmul(body, r.correction), hq);
      }
    }
    return { attitudeError: err, thrustAngle: r.thrustAngle, thrustErrorAngle: r.thrustErrorAngle, target: newTarget };
  }

  updateAngVelTargetFromAttError(e) {
    const out = [0, 0, 0];
    const rp = (k, accel) => (this.useSqrt && !isZero(accel) ? sqrtController(e[k], this.angP[k], clamp(accel / 2, ACCEL_RP_MIN, ACCEL_RP_MAX), this.dt) : this.angP[k] * e[k]);
    out[0] = rp(0, this.accelRollMaxRadss());
    out[1] = rp(1, this.accelPitchMaxRadss());
    const ay = this.accelYawMaxRadss();
    out[2] = this.useSqrt && !isZero(ay) ? sqrtController(e[2], this.angP[2], clamp(ay / 2, ACCEL_Y_MIN, ACCEL_Y_MAX), this.dt) : this.angP[2] * e[2];
    return out;
  }

  rateControllerRun() {
    const g = this.ahrs.gyroLatest;
    const lim = this.motors.limit;
    this.motors.setRoll(this.pidRoll.updateAll(this.angVelBody[0], g[0], this.dt, lim.roll));
    this.motors.setPitch(this.pidPitch.updateAll(this.angVelBody[1], g[1], this.dt, lim.pitch));
    this.motors.setYaw(this.pidYaw.updateAll(this.angVelBody[2], g[2], this.dt, lim.yaw));
  }

  setThrottleOut(throttle, applyAngleBoost, filtCutoff) {
    this.throttleIn = throttle;
    this.updateAltholdLeanAngleMax(throttle);
    this.motors.setThrottleFilterCutoff(filtCutoff);
    this.motors.setThrottle(throttle);
  }
  updateAltholdLeanAngleMax(throttle) {
    const thrMax = this.motors.throttleThrustMax;
    if (isZero(thrMax)) { this.althold_lean_angle_max = 0; return; }
    const target = Math.acos(clamp(throttle / (ANGLE_LIMIT_THROTTLE_MAX * thrMax), 0, 1));
    this.althold_lean_angle_max += (this.dt / (this.dt + this.p.angleLimitTc)) * (target - this.althold_lean_angle_max);
  }

  preArmChecks() {
    for (const [n, v] of [['ANG_PIT', this.angP[1]], ['ANG_RLL', this.angP[0]], ['ANG_YAW', this.angP[2]]]) if (!isPositive(v)) return `ATC_${n}_P must be > 0`;
    for (const [n, pid] of [['RAT_RLL', this.pidRoll], ['RAT_PIT', this.pidPitch], ['RAT_YAW', this.pidYaw]]) {
      if (!isPositive(pid.kp)) return `ATC_${n}_P must be > 0`;
      if (!isPositive(pid.ki)) return `ATC_${n}_I must be > 0`;
      if (isNegative(pid.kd)) return `ATC_${n}_D must be >= 0`;
    }
    return null;
  }
}

// ---- free functions (static in C++) ----
export function inputShapingAngle(errorAngle, inputTc, accelMax, targetAngVel, desiredAngVel, maxAngVel, dt) {
  desiredAngVel += sqrtController(errorAngle, 1 / Math.max(inputTc, 0.01), accelMax, dt);
  if (isPositive(maxAngVel)) desiredAngVel = clamp(desiredAngVel, -maxAngVel, maxAngVel);
  return inputShapingAngVel(targetAngVel, desiredAngVel, accelMax, dt, 0);
}
export function inputShapingAngVel(targetAngVel, desiredAngVel, accelMax, dt, inputTc) {
  if (isPositive(inputTc)) {
    const err = desiredAngVel - targetAngVel;
    const acc = sqrtController(err, 1 / Math.max(inputTc, 0.01), 0, dt);
    desiredAngVel = targetAngVel + acc * dt;
  }
  if (isPositive(accelMax)) {
    const d = accelMax * dt;
    return clamp(desiredAngVel, targetAngVel - d, targetAngVel + d);
  }
  return desiredAngVel;
}
export function angVelLimit(v, rollMax, pitchMax, yawMax) {
  if (isZero(rollMax) || isZero(pitchMax)) {
    if (!isZero(rollMax)) v[0] = clamp(v[0], -rollMax, rollMax);
    if (!isZero(pitchMax)) v[1] = clamp(v[1], -pitchMax, pitchMax);
  } else {
    const tx = v[0] / rollMax, ty = v[1] / pitchMax;
    const l = Math.hypot(tx, ty);
    if (l > 1) { v[0] = (tx * rollMax) / l; v[1] = (ty * pitchMax) / l; }
  }
  if (!isZero(yawMax)) v[2] = clamp(v[2], -yawMax, yawMax);
  return v;
}
export function eulerAccelLimit(e, a) {
  const sphi = clamp(Math.abs(Math.sin(e[0])), 0.1, 1), cphi = clamp(Math.abs(Math.cos(e[0])), 0.1, 1);
  const sth = clamp(Math.abs(Math.sin(e[1])), 0.1, 1), cth = clamp(Math.abs(Math.cos(e[1])), 0.1, 1);
  if (isZero(a[0]) || isZero(a[1]) || isZero(a[2]) || isNegative(a[0]) || isNegative(a[1]) || isNegative(a[2])) return a.slice();
  return [a[0], Math.min(a[1] / cphi, a[2] / sphi), Math.min(Math.min(a[0] / sth, a[1] / (sphi * cth)), a[2] / (cphi * cth))];
}
export function eulerRateToAngVel(e, r) {
  const st = Math.sin(e[1]), ct = Math.cos(e[1]), sp = Math.sin(e[0]), cp = Math.cos(e[0]);
  return [r[0] - st * r[2], cp * r[1] + sp * ct * r[2], -sp * r[1] + ct * cp * r[2]];
}
export function angVelToEulerRate(e, w) {
  const st = Math.sin(e[1]), ct = Math.cos(e[1]), sp = Math.sin(e[0]), cp = Math.cos(e[0]);
  if (isZero(ct)) return null;
  return [w[0] + sp * (st / ct) * w[1] + cp * (st / ct) * w[2], cp * w[1] - sp * w[2], (sp / ct) * w[1] + (cp / ct) * w[2]];
}
