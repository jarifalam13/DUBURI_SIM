// Ports of ArduPilot AC_PID, AC_PID_Basic, AC_P_1D, AC_P_2D, AC_PID_2D (4.5.7).

import { calcLowpassAlpha, sqrtController, invSqrtController } from './control.js';
import { clamp, isPositive, isNegative, isZero } from '../core/math.js';

export class AC_PID {
  constructor(g = {}) {
    this.kp = g.p ?? 0; this.ki = g.i ?? 0; this.kd = g.d ?? 0; this.kff = g.ff ?? 0;
    this.kimax = g.imax ?? 0; this.filtT = g.fltt ?? 0; this.filtE = g.flte ?? 0; this.filtD = g.fltd ?? 0;
    this.kpdmax = g.pdmx ?? 0;
    this.target = 0; this.error = 0; this.derivative = 0; this.targetDerivative = 0; this.integrator = 0;
    this.resetFilterFlag = true;
    this.info = { target: 0, actual: 0, error: 0, P: 0, I: 0, D: 0, FF: 0, limit: false };
  }
  setGains(g) {
    if (g.p !== undefined) this.kp = g.p;
    if (g.i !== undefined) this.ki = g.i;
    if (g.d !== undefined) this.kd = g.d;
    if (g.ff !== undefined) this.kff = g.ff;
    if (g.imax !== undefined) this.kimax = Math.abs(g.imax);
    if (g.fltt !== undefined) this.filtT = g.fltt;
    if (g.flte !== undefined) this.filtE = g.flte;
    if (g.fltd !== undefined) this.filtD = g.fltd;
    if (g.pdmx !== undefined) this.kpdmax = g.pdmx;
  }
  updateAll(target, measurement, dt, limit = false, boost = 1) {
    if (!Number.isFinite(target) || !Number.isFinite(measurement)) return 0;
    if (this.resetFilterFlag) {
      this.resetFilterFlag = false;
      this.target = target;
      this.error = this.target - measurement;
      this.derivative = 0;
      this.targetDerivative = 0;
    } else {
      const errorLast = this.error, targetLast = this.target;
      const error = this.target - measurement;
      this.target += calcLowpassAlpha(dt, this.filtT) * (target - this.target);
      this.error += calcLowpassAlpha(dt, this.filtE) * (error - this.error);
      if (dt > 0) {
        const derivative = (this.error - errorLast) / dt;
        this.derivative += calcLowpassAlpha(dt, this.filtD) * (derivative - this.derivative);
        this.targetDerivative = (this.target - targetLast) / dt;
      }
    }
    this.updateI(dt, limit);
    let P = this.error * this.kp, D = this.derivative * this.kd;
    P *= boost; D *= boost;
    if (isPositive(this.kpdmax)) {
      const s = Math.abs(P + D);
      if (s > this.kpdmax) { const k = this.kpdmax / s; P *= k; D *= k; }
    }
    const i = this.info;
    i.target = this.target; i.actual = measurement; i.error = this.error; i.P = P; i.D = D; i.I = this.integrator; i.FF = this.target * this.kff;
    return P + D + this.integrator;
  }
  updateI(dt, limit) {
    if (!isZero(this.ki) && isPositive(dt)) {
      if (!limit || (isPositive(this.integrator) && isNegative(this.error)) || (isNegative(this.integrator) && isPositive(this.error))) {
        this.integrator += this.error * this.ki * dt;
        this.integrator = clamp(this.integrator, -this.kimax, this.kimax);
      }
    } else this.integrator = 0;
    this.info.limit = limit;
  }
  resetI() { this.integrator = 0; this.info.I = 0; }
  resetFilter() { this.resetFilterFlag = true; }
  setIntegrator(v) { this.integrator = clamp(v, -this.kimax, this.kimax); }
  relaxIntegrator(v, dt, tc) {
    v = clamp(v, -this.kimax, this.kimax);
    if (isPositive(dt)) this.integrator += (v - this.integrator) * (dt / (dt + tc));
  }
  getFF() { return this.info.FF; }
}

export class AC_PID_Basic {
  constructor(g = {}) {
    this.kp = g.p ?? 0; this.ki = g.i ?? 0; this.kd = g.d ?? 0; this.kff = g.ff ?? 0;
    this.kimax = g.imax ?? 0; this.filtE = g.flte ?? 0; this.filtD = g.fltd ?? 0;
    this.target = 0; this.error = 0; this.derivative = 0; this.integrator = 0; this.resetFilterFlag = true;
  }
  setGains(g) {
    for (const [k, f] of [['p', 'kp'], ['i', 'ki'], ['d', 'kd'], ['ff', 'kff'], ['flte', 'filtE'], ['fltd', 'filtD']]) if (g[k] !== undefined) this[f] = g[k];
    if (g.imax !== undefined) this.kimax = Math.abs(g.imax);
  }
  updateAll(target, measurement, dt, limitNeg = false, limitPos = false) {
    if (!Number.isFinite(target) || !Number.isFinite(measurement)) return 0;
    this.target = target;
    if (this.resetFilterFlag) {
      this.resetFilterFlag = false;
      this.error = this.target - measurement;
      this.derivative = 0;
    } else {
      const last = this.error;
      this.error += calcLowpassAlpha(dt, this.filtE) * (this.target - measurement - this.error);
      if (dt > 0) this.derivative += calcLowpassAlpha(dt, this.filtD) * ((this.error - last) / dt - this.derivative);
    }
    if (!isZero(this.ki)) {
      if (!((limitNeg && isNegative(this.error)) || (limitPos && isPositive(this.error)))) {
        this.integrator += this.error * this.ki * dt;
        this.integrator = clamp(this.integrator, -this.kimax, this.kimax);
      }
    } else this.integrator = 0;
    return this.error * this.kp + this.integrator + this.derivative * this.kd + this.target * this.kff;
  }
  resetFilter() { this.resetFilterFlag = true; }
  resetI() { this.integrator = 0; }
  setIntegrator(v) { this.integrator = clamp(v, -this.kimax, this.kimax); }
}

export class AC_P_1D {
  constructor(kp = 1) { this.kp = kp; this.errorMin = 0; this.errorMax = 0; this.D1max = 0; this.error = 0; }
  setLimits(outMin, outMax, DOutMax = 0, D2OutMax = 0) {
    this.D1max = 0; this.errorMin = 0; this.errorMax = 0;
    if (isPositive(DOutMax)) this.D1max = DOutMax;
    if (isPositive(D2OutMax) && isPositive(this.kp)) this.D1max = Math.min(this.D1max, D2OutMax / this.kp);
    if (isNegative(outMin) && isPositive(this.kp)) this.errorMin = invSqrtController(outMin, this.kp, this.D1max);
    if (isPositive(outMax) && isPositive(this.kp)) this.errorMax = invSqrtController(outMax, this.kp, this.D1max);
  }
  // returns {out, target} (target may be pulled toward the measurement)
  updateAll(target, measurement) {
    this.error = target - measurement;
    if (isNegative(this.errorMin) && this.error < this.errorMin) { this.error = this.errorMin; target = measurement + this.error; }
    else if (isPositive(this.errorMax) && this.error > this.errorMax) { this.error = this.errorMax; target = measurement + this.error; }
    return { out: sqrtController(this.error, this.kp, this.D1max, 0), target };
  }
}

export class AC_P_2D {
  constructor(kp = 1) { this.kp = kp; this.errorMax = 0; this.D1max = 0; this.error = [0, 0]; }
  setLimits(outMax, DOutMax = 0, D2OutMax = 0) {
    this.D1max = 0; this.errorMax = 0;
    if (isPositive(DOutMax)) this.D1max = DOutMax;
    if (isPositive(D2OutMax) && isPositive(this.kp)) this.D1max = Math.min(this.D1max, D2OutMax / this.kp);
    if (isPositive(outMax) && isPositive(this.kp)) this.errorMax = invSqrtController(outMax, this.kp, this.D1max);
  }
  updateAll(target, meas) {
    let ex = target[0] - meas[0], ey = target[1] - meas[1];
    const l = Math.hypot(ex, ey);
    if (isPositive(this.errorMax) && l > this.errorMax) {
      const k = this.errorMax / l; ex *= k; ey *= k;
      target = [meas[0] + ex, meas[1] + ey];
    }
    this.error = [ex, ey];
    const L = Math.hypot(ex, ey);
    if (!isPositive(L)) return { out: [0, 0], target };
    const c = sqrtController(L, this.kp, this.D1max, 0);
    return { out: [(ex * c) / L, (ey * c) / L], target };
  }
}

export class AC_PID_2D {
  constructor(g = {}) {
    this.kp = g.p ?? 0; this.ki = g.i ?? 0; this.kd = g.d ?? 0; this.kff = g.ff ?? 0; this.kimax = g.imax ?? 0;
    this.filtE = g.flte ?? 0; this.filtD = g.fltd ?? 0;
    this.error = [0, 0]; this.derivative = [0, 0]; this.integrator = [0, 0]; this.target = [0, 0]; this.resetFilterFlag = true;
  }
  setGains(g) {
    for (const [k, f] of [['p', 'kp'], ['i', 'ki'], ['d', 'kd'], ['ff', 'kff'], ['flte', 'filtE'], ['fltd', 'filtD']]) if (g[k] !== undefined) this[f] = g[k];
    if (g.imax !== undefined) this.kimax = Math.abs(g.imax);
  }
  updateAll(target, meas, dt, limit = [0, 0]) {
    this.target = target.slice();
    const e = [target[0] - meas[0], target[1] - meas[1]];
    if (this.resetFilterFlag) {
      this.resetFilterFlag = false;
      this.error = e; this.derivative = [0, 0];
    } else {
      const last = this.error.slice();
      const a = calcLowpassAlpha(dt, this.filtE);
      this.error = [this.error[0] + a * (e[0] - this.error[0]), this.error[1] + a * (e[1] - this.error[1])];
      if (dt > 0) {
        const ad = calcLowpassAlpha(dt, this.filtD);
        this.derivative = [0, 1].map((k) => this.derivative[k] + ad * ((this.error[k] - last[k]) / dt - this.derivative[k]));
      }
    }
    let dI = [this.error[0] * this.ki * dt, this.error[1] * this.ki * dt];
    if (limit[0] || limit[1]) if (dI[0] * limit[0] + dI[1] * limit[1] > 0) dI = [0, 0];
    this.integrator = [this.integrator[0] + dI[0], this.integrator[1] + dI[1]];
    const il = Math.hypot(...this.integrator);
    if (il > this.kimax && il > 0) this.integrator = this.integrator.map((v) => (v * this.kimax) / il);
    return [0, 1].map((k) => this.error[k] * this.kp + this.integrator[k] + this.derivative[k] * this.kd + this.target[k] * this.kff);
  }
  resetFilter() { this.resetFilterFlag = true; }
  resetI() { this.integrator = [0, 0]; }
  setIntegrator(v) { this.integrator = v.slice(); const l = Math.hypot(...v); if (l > this.kimax && l > 0) this.integrator = v.map((x) => (x * this.kimax) / l); }
}
