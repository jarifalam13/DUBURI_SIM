// Port of AP_Motors6DOF (ArduSub 4.5.7) + the AP_MotorsMulticopter pieces it
// relies on (throttle filter, spool logic). Extra mixer variants are offered
// for automatically generated frames (not in the firmware): 'scale' preserves
// the commanded wrench direction under saturation, 'priority' keeps attitude
// authority first.

import { LowPassFilter } from './control.js';
import { clamp } from '../core/math.js';
import { frameTable, FRAMES } from '../alloc/frames.js';

export const SPOOL = { SHUT_DOWN: 0, GROUND_IDLE: 1, SPOOLING_UP: 2, THROTTLE_UNLIMITED: 3, SPOOLING_DOWN: 4 };

export class Motors6DOF {
  constructor() {
    this.factors = []; // [roll, pitch, yaw, throttle, forward, lateral] per motor
    this.variant = 'generic';
    this.reverse = new Array(12).fill(1);
    this.pwmMin = 1100; this.pwmMax = 1900;
    this.fvCoupling = 1;
    this.spoolTime = 0.5;
    this.throttleHover = 0.5;
    this.armed = false;
    this.spool = SPOOL.SHUT_DOWN;
    this.desiredSpool = SPOOL.SHUT_DOWN;
    this.throttleThrustMax = 0;
    this.maxThrottle = 1;
    this.rollIn = 0; this.pitchIn = 0; this.yawIn = 0; this.throttleIn = 0; this.forwardIn = 0; this.lateralIn = 0;
    this.throttleFilter = new LowPassFilter(0);
    this.limit = { roll: false, pitch: false, yaw: false, throttle_lower: false, throttle_upper: false };
    this.thrust = [];
    this.pwm = [];
    this.test = null; // {motor, pwm} during motor test
    this.saturated = false;
  }

  configure({ factors, variant, params }) {
    this.factors = factors.map((r) => r.slice());
    this.variant = variant;
    const P = params;
    for (let i = 0; i < 12; i++) this.reverse[i] = (P[`MOT_${i + 1}_DIRECTION`] ?? 1) < 0 ? -1 : 1;
    this.pwmMin = P.MOT_PWM_MIN ?? 1100;
    this.pwmMax = P.MOT_PWM_MAX ?? 1900;
    this.fvCoupling = P.MOT_FV_CPLNG_K ?? 1;
    this.spoolTime = P.MOT_SPOOL_TIME ?? 0.5;
    this.throttleHover = P.MOT_THST_HOVER ?? 0.5;
    this.thrust = new Array(this.factors.length).fill(0);
    this.pwm = new Array(this.factors.length).fill(1500);
  }

  static fromFrameConfig(frameConfig) {
    const f = FRAMES[frameConfig] ?? FRAMES[5];
    return { factors: frameTable(frameConfig), variant: f.mixer };
  }

  setRoll(v) { this.rollIn = v; }
  setPitch(v) { this.pitchIn = v; }
  setYaw(v) { this.yawIn = v; }
  setThrottle(v) { this.throttleIn = v; }
  setForward(v) { this.forwardIn = v; }
  setLateral(v) { this.lateralIn = v; }
  setThrottleFilterCutoff(hz) { this.throttleFilter.setCutoff(hz); }
  setDesiredSpoolState(s) { this.desiredSpool = s; }
  getThrottleBidirectional() { return clamp(2 * (this.throttleFilter.get() - 0.5), -1, 1); }
  getThrottle() { return clamp(this.throttleFilter.get(), 0, 1); }

  arm() { this.armed = true; }
  disarm() { this.armed = false; this.desiredSpool = SPOOL.SHUT_DOWN; }

  // AP_MotorsMulticopter::output_logic (spool state machine)
  outputLogic(dt) {
    if (!this.armed) { this.desiredSpool = SPOOL.SHUT_DOWN; this.spool = SPOOL.SHUT_DOWN; }
    const spoolUp = Math.max(this.spoolTime, 0.05);
    const step = dt / spoolUp;
    switch (this.spool) {
      case SPOOL.SHUT_DOWN:
        if (this.desiredSpool !== SPOOL.SHUT_DOWN) { this.spool = SPOOL.GROUND_IDLE; break; }
        this.spinUpRatio = 0;
        this.throttleThrustMax = 0;
        break;
      case SPOOL.GROUND_IDLE:
        if (this.desiredSpool === SPOOL.SHUT_DOWN) {
          this.spinUpRatio = (this.spinUpRatio ?? 0) - step;
          if (this.spinUpRatio <= 0) { this.spinUpRatio = 0; this.spool = SPOOL.SHUT_DOWN; }
        } else if (this.desiredSpool === SPOOL.THROTTLE_UNLIMITED) {
          this.spinUpRatio = (this.spinUpRatio ?? 0) + step;
          if (this.spinUpRatio >= 1) { this.spinUpRatio = 1; this.spool = SPOOL.SPOOLING_UP; }
        }
        this.throttleThrustMax = 0;
        break;
      case SPOOL.SPOOLING_UP:
        if (this.desiredSpool !== SPOOL.THROTTLE_UNLIMITED) { this.spool = SPOOL.SPOOLING_DOWN; break; }
        this.spinUpRatio = 1;
        this.throttleThrustMax += step;
        if (this.throttleThrustMax >= Math.min(this.getThrottle(), 1)) { this.throttleThrustMax = 1; this.spool = SPOOL.THROTTLE_UNLIMITED; }
        else if (this.throttleThrustMax < 0) this.throttleThrustMax = 0;
        break;
      case SPOOL.THROTTLE_UNLIMITED:
        if (this.desiredSpool !== SPOOL.THROTTLE_UNLIMITED) { this.spool = SPOOL.SPOOLING_DOWN; break; }
        this.spinUpRatio = 1;
        this.throttleThrustMax = 1;
        break;
      case SPOOL.SPOOLING_DOWN:
        if (this.desiredSpool === SPOOL.THROTTLE_UNLIMITED) { this.spool = SPOOL.SPOOLING_UP; break; }
        this.throttleThrustMax -= step;
        if (this.throttleThrustMax <= 0) this.throttleThrustMax = 0;
        if (this.throttleThrustMax >= 1) this.throttleThrustMax = 1;
        else if (this.throttleThrustMax === 0) this.spool = SPOOL.GROUND_IDLE;
        break;
    }
  }

  // AP_MotorsMulticopter::output() order
  output(dt) {
    if (this.armed) {
      this.throttleFilter.apply(this.throttleIn, dt);
      if (this.throttleFilter.get() < 0) this.throttleFilter.reset(0);
      if (this.throttleFilter.get() > 1) this.throttleFilter.reset(1);
    } else this.throttleFilter.reset(0);
    this.outputLogic(dt);
    this.outputArmedStabilizing();
    // output_to_motors
    const spinning = this.spool === SPOOL.SPOOLING_UP || this.spool === SPOOL.THROTTLE_UNLIMITED || this.spool === SPOOL.SPOOLING_DOWN;
    for (let i = 0; i < this.factors.length; i++) {
      if (this.test && this.armed) this.pwm[i] = this.test.motor === i ? clamp(this.test.pwm, this.pwmMin, this.pwmMax) : 1500;
      else this.pwm[i] = spinning ? this.thrustToPwm(this.thrust[i]) : 1500;
    }
    return this.pwm;
  }

  thrustToPwm(t) {
    const up = this.pwmMax - 1500, down = 1500 - this.pwmMin;
    return Math.trunc(1500 + t * (t > 0 ? up : down));
  }

  outputArmedStabilizing() {
    const n = this.factors.length;
    const roll = this.rollIn, pitch = this.pitchIn, yaw = this.yawIn;
    let thr = this.getThrottleBidirectional();
    const fwd = this.forwardIn, lat = this.lateralIn;
    const L = this.limit;
    L.roll = L.pitch = L.yaw = L.throttle_lower = L.throttle_upper = false;
    const tmax = this.throttleThrustMax;
    if (thr <= -tmax) { thr = -tmax; L.throttle_lower = true; }
    if (thr >= tmax) { thr = tmax; L.throttle_upper = true; }
    const F = this.factors, rev = this.reverse;
    const out = this.thrust;
    this.saturated = false;
    if (this.variant === 'vectored6dof') {
      thr = clamp(thr, -1, this.maxThrottle);
      let rptMax = 1, yflMax = 1;
      const rpt = new Array(n), yfl = new Array(n);
      for (let i = 0; i < n; i++) {
        rpt[i] = roll * F[i][0] + pitch * F[i][1] + thr * F[i][3];
        yfl[i] = yaw * F[i][2] + fwd * F[i][4] + lat * F[i][5];
        rptMax = Math.max(rptMax, Math.abs(rpt[i]));
        yflMax = Math.max(yflMax, Math.abs(yfl[i]));
      }
      this.saturated = rptMax > 1 || yflMax > 1;
      for (let i = 0; i < n; i++) out[i] = clamp(rev[i] * (rpt[i] / rptMax + yfl[i] / yflMax), -1, 1);
    } else if (this.variant === 'vectored') {
      const fcLimit = Math.max(0, 1 - this.fvCoupling * Math.abs(thr));
      const dir = [-1, -1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0];
      for (let i = 0; i < n; i++) {
        const rpy = roll * F[i][0] + pitch * F[i][1] + yaw * F[i][2];
        let f = fwd;
        if (f !== 0 && dir[i] !== 0 && (fwd < 0) === (dir[i] < 0)) f = clamp(fwd, -fcLimit, fcLimit);
        const lin = thr * F[i][3] + f * F[i][4] + lat * F[i][5];
        const raw = rev[i] * (rpy + lin);
        if (Math.abs(raw) > 1) this.saturated = true;
        out[i] = clamp(raw, -1, 1);
      }
    } else if (this.variant === 'scale' || this.variant === 'priority') {
      const att = new Array(n), lin = new Array(n);
      for (let i = 0; i < n; i++) {
        att[i] = roll * F[i][0] + pitch * F[i][1] + yaw * F[i][2];
        lin[i] = thr * F[i][3] + fwd * F[i][4] + lat * F[i][5];
      }
      if (this.variant === 'scale') {
        let m = 1;
        for (let i = 0; i < n; i++) m = Math.max(m, Math.abs(att[i] + lin[i]));
        this.saturated = m > 1;
        for (let i = 0; i < n; i++) out[i] = rev[i] * ((att[i] + lin[i]) / m);
      } else {
        let ma = 1;
        for (let i = 0; i < n; i++) ma = Math.max(ma, Math.abs(att[i]));
        for (let i = 0; i < n; i++) att[i] /= ma;
        // largest s in [0,1] so that |att + s*lin| <= 1 for every motor
        let s = 1;
        for (let i = 0; i < n; i++) {
          const a = att[i], l = lin[i];
          if (Math.abs(l) < 1e-12) continue;
          const lim = l > 0 ? (1 - a) / l : (-1 - a) / l;
          s = Math.min(s, Math.max(0, lim));
        }
        this.saturated = ma > 1 || s < 1;
        for (let i = 0; i < n; i++) out[i] = clamp(rev[i] * (att[i] + s * lin[i]), -1, 1);
      }
    } else {
      for (let i = 0; i < n; i++) {
        const raw = rev[i] * (roll * F[i][0] + pitch * F[i][1] + yaw * F[i][2] + thr * F[i][3] + fwd * F[i][4] + lat * F[i][5]);
        if (Math.abs(raw) > 1) this.saturated = true;
        out[i] = clamp(raw, -1, 1);
      }
    }
  }
}
