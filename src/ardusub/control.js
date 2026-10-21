// Port of ArduPilot libraries/AP_Math/control.cpp (ArduSub 4.5.7) and the
// low-pass filter helpers. Function names follow the C++ originals.

import { clamp, isPositive, isNegative, isZero, sq } from '../core/math.js';

export function calcLowpassAlpha(dt, cutoffHz) {
  if (!(cutoffHz > 0) || !(dt > 0)) return 1;
  const rc = 1 / (2 * Math.PI * cutoffHz);
  return dt / (dt + rc);
}

export function sqrtController(error, p, secondOrdLim, dt) {
  let rate;
  if (isNegative(secondOrdLim) || isZero(secondOrdLim)) rate = error * p;
  else if (isZero(p)) {
    if (isPositive(error)) rate = Math.sqrt(2 * secondOrdLim * error);
    else if (isNegative(error)) rate = -Math.sqrt(2 * secondOrdLim * -error);
    else rate = 0;
  } else {
    const linearDist = secondOrdLim / sq(p);
    if (error > linearDist) rate = Math.sqrt(Math.max(0, 2 * secondOrdLim * (error - linearDist / 2)));
    else if (error < -linearDist) rate = -Math.sqrt(Math.max(0, 2 * secondOrdLim * (-error - linearDist / 2)));
    else rate = error * p;
  }
  if (isPositive(dt)) return clamp(rate, -Math.abs(error) / dt, Math.abs(error) / dt);
  return rate;
}

export function sqrtController2(err, p, lim, dt) {
  const l = Math.hypot(err[0], err[1]);
  if (!isPositive(l)) return [0, 0];
  const c = sqrtController(l, p, lim, dt);
  return [err[0] * (c / l), err[1] * (c / l)];
}

export function invSqrtController(output, p, Dmax) {
  if (isPositive(Dmax) && isZero(p)) return (output * output) / (2 * Dmax);
  if ((isNegative(Dmax) || isZero(Dmax)) && !isZero(p)) return output / p;
  if ((isNegative(Dmax) || isZero(Dmax)) && isZero(p)) return 0;
  const linearVelocity = Dmax / p;
  if (Math.abs(output) < linearVelocity) return output / p;
  const linearDist = Dmax / sq(p);
  const stopping = linearDist * 0.5 + sq(output) / (2 * Dmax);
  return output > 0 ? stopping : -stopping;
}
export const stoppingDistance = (velocity, p, accelMax) => invSqrtController(velocity, p, accelMax);

export function shapeAccel(accelInput, s, jerkMax, dt) {
  // s: {accel}
  if (!isPositive(jerkMax) || !isPositive(dt)) return;
  s.accel += clamp(accelInput - s.accel, -jerkMax * dt, jerkMax * dt);
}

export function shapeVelAccel(velInput, accelInput, vel, s, accelMin, accelMax, jerkMax, dt, limitTotal) {
  if (!isNegative(accelMin) || !isPositive(accelMax) || !isPositive(jerkMax)) return;
  const velError = velInput - vel;
  const KPa = isPositive(velError) ? jerkMax / accelMax : jerkMax / -accelMin;
  let accelTarget = sqrtController(velError, KPa, jerkMax, dt);
  accelTarget = clamp(accelTarget, accelMin, accelMax);
  accelTarget += accelInput;
  if (limitTotal) accelTarget = clamp(accelTarget, accelMin, accelMax);
  shapeAccel(accelTarget, s, jerkMax, dt);
}

export function shapePosVelAccel(posInput, velInput, accelInput, pos, vel, s, velMin, velMax, accelMin, accelMax, jerkMax, dt, limitTotal) {
  if (isPositive(velMin) || isNegative(velMax) || !isNegative(accelMin) || !isPositive(accelMax) || !isPositive(jerkMax)) return;
  const posError = posInput - pos;
  let accelTcMax, KPv;
  if (isPositive(posError)) { accelTcMax = -0.5 * accelMin; KPv = (0.5 * jerkMax) / -accelMin; }
  else { accelTcMax = 0.5 * accelMax; KPv = (0.5 * jerkMax) / accelMax; }
  let velTarget = sqrtController(posError, KPv, accelTcMax, dt);
  if (isNegative(velMin) || isPositive(velMax)) velTarget = clamp(velTarget, velMin, velMax);
  velTarget += velInput;
  if (limitTotal) velTarget = clamp(velTarget, velMin, velMax);
  shapeVelAccel(velTarget, accelInput, vel, s, accelMin, accelMax, jerkMax, dt, limitTotal);
}

// update_vel_accel: returns new velocity
export function updateVelAccel(vel, accel, dt, limit, velError) {
  let dv = accel * dt;
  if (isPositive(dv * limit) && isPositive(velError * limit)) {
    if (isNegative(vel * limit)) dv = clamp(dv, -Math.abs(vel), Math.abs(vel));
    else dv = 0;
  }
  return vel + dv;
}
// update_pos_vel_accel on an object {pos, vel}
export function updatePosVelAccel(st, accel, dt, limit, posError, velError) {
  let dp = st.vel * dt + accel * 0.5 * sq(dt);
  if (isPositive(dp * limit) && isPositive(posError * limit)) dp = 0;
  st.pos += dp;
  st.vel = updateVelAccel(st.vel, accel, dt, limit, velError);
}

export function inputExpo(input, expo) {
  input = clamp(input, -1, 1);
  if (expo < 0.95) return ((1 - expo) * input) / (1 - expo * Math.abs(input));
  return input;
}
export const angleToAccel = (deg) => 9.80665 * Math.tan((deg * Math.PI) / 180);
export const accelToAngle = (accel) => (Math.atan(accel / 9.80665) * 180) / Math.PI;

// --- filters ---
export class LowPassFilter {
  constructor(cutoff = 0) { this.cutoff = cutoff; this.out = 0; this.init = false; }
  setCutoff(c) { this.cutoff = c; }
  apply(x, dt) {
    if (!this.init) { this.out = x; this.init = true; return x; }
    this.out += (x - this.out) * calcLowpassAlpha(dt, this.cutoff);
    return this.out;
  }
  reset(v = 0) { this.out = v; this.init = true; }
  get() { return this.out; }
}

// 2-pole Butterworth (ArduPilot DigitalBiquadFilter / LowPassFilter2p)
export class LowPassFilter2p {
  constructor(sampleHz, cutoffHz) { this.configure(sampleHz, cutoffHz); }
  configure(sampleHz, cutoffHz) {
    this.sampleHz = sampleHz; this.cutoffHz = cutoffHz;
    this.d1 = null; this.d2 = null;
    if (!(cutoffHz > 0) || cutoffHz >= sampleHz / 2) { this.pass = true; return; }
    this.pass = false;
    const fr = sampleHz / cutoffHz;
    const ohm = Math.tan(Math.PI / fr);
    const c = 1 + 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm;
    this.b0 = (ohm * ohm) / c; this.b1 = 2 * this.b0; this.b2 = this.b0;
    this.a1 = (2 * (ohm * ohm - 1)) / c;
    this.a2 = (1 - 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm) / c;
  }
  apply(x) {
    if (this.pass) return x;
    if (this.d1 === null) { const v = x / (this.b0 + this.b1 + this.b2); this.d1 = v; this.d2 = v; }
    const d0 = x - this.d1 * this.a1 - this.d2 * this.a2;
    const out = d0 * this.b0 + this.d1 * this.b1 + this.d2 * this.b2;
    this.d2 = this.d1; this.d1 = d0;
    return out;
  }
  reset() { this.d1 = null; this.d2 = null; }
}
