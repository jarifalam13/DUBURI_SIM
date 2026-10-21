// Thruster model: ESC PWM -> static thrust curve -> rotor lag -> in-flow,
// ventilation and hull-interaction corrections.

import { clamp } from '../core/math.js';

// Build a PWM->thrust table from a few shape parameters (used for presets and
// for thrusters without measured data).
export function parametricCurve({ fwdMax = 51.5, revMax = 40.2, deadband = 25, expo = 1.25, pwmMin = 1100, pwmMax = 1900 } = {}) {
  const pts = [];
  for (let pwm = pwmMin; pwm <= pwmMax; pwm += 10) {
    let T = 0;
    if (pwm > 1500 + deadband) T = fwdMax * Math.pow((pwm - 1500 - deadband) / (pwmMax - 1500 - deadband), expo);
    else if (pwm < 1500 - deadband) T = -revMax * Math.pow((1500 - deadband - pwm) / (1500 - deadband - pwmMin), expo);
    pts.push([pwm, +T.toFixed(4)]);
  }
  return pts;
}

export const THRUSTER_PRESETS = {
  't200-16v': {
    label: 'Blue Robotics T200 @ 16 V (approx.)',
    curve: { fwdMax: 51.5, revMax: 40.2, deadband: 25, expo: 1.25 },
    propDiameter: 0.076, maxRpm: 3500, J0: 0.9, tauUp: 0.08, tauDown: 0.12, mass: 0.344,
  },
  't200-12v': {
    label: 'Blue Robotics T200 @ 12 V (approx.)',
    curve: { fwdMax: 36.3, revMax: 28.4, deadband: 25, expo: 1.25 },
    propDiameter: 0.076, maxRpm: 3000, J0: 0.9, tauUp: 0.09, tauDown: 0.13, mass: 0.344,
  },
  't500-20v': {
    label: 'Blue Robotics T500 @ 20 V (approx.)',
    curve: { fwdMax: 157, revMax: 117, deadband: 25, expo: 1.3 },
    propDiameter: 0.104, maxRpm: 3200, J0: 0.9, tauUp: 0.12, tauDown: 0.16, mass: 0.9,
  },
  'generic-small': {
    label: 'Generic small thruster (≈2 kgf)',
    curve: { fwdMax: 20, revMax: 15, deadband: 30, expo: 1.3 },
    propDiameter: 0.06, maxRpm: 4000, J0: 0.85, tauUp: 0.07, tauDown: 0.1, mass: 0.2,
  },
};

export class ThrustCurve {
  constructor(table) {
    this.set(table);
  }
  set(table) {
    const t = (table && table.length >= 2 ? table : parametricCurve()).map(([p, T]) => [+p, +T]).filter(([p, T]) => Number.isFinite(p) && Number.isFinite(T));
    t.sort((a, b) => a[0] - b[0]);
    this.pwm = Float64Array.from(t.map((r) => r[0]));
    this.T = Float64Array.from(t.map((r) => r[1]));
    this.fwdMax = Math.max(0, ...this.T);
    this.revMax = Math.max(0, ...this.T.map((v) => -v));
  }
  thrust(pwm) {
    const P = this.pwm, T = this.T, n = P.length;
    if (pwm <= P[0]) return T[0];
    if (pwm >= P[n - 1]) return T[n - 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (P[mid] <= pwm) lo = mid; else hi = mid;
    }
    const f = (pwm - P[lo]) / (P[hi] - P[lo]);
    return T[lo] + f * (T[hi] - T[lo]);
  }
}

// Parse "pwm, thrust" text (CSV / TSV / spaces), thrust in N or kgf.
export function parseCurveText(text, unit = 'N') {
  const k = unit === 'kgf' ? 9.80665 : unit === 'lbf' ? 4.448222 : 1;
  const rows = [];
  for (const line of text.split(/\r?\n/)) {
    const nums = line.split(/[\s,;\t]+/).map(Number).filter((v) => Number.isFinite(v));
    if (nums.length >= 2 && nums[0] > 500 && nums[0] < 2500) rows.push([nums[0], nums[1] * k]);
  }
  if (rows.length < 3) throw new Error('Need at least 3 rows of "PWM, thrust"');
  return rows;
}

// Runtime state of one thruster.
export class ThrusterState {
  constructor(def) {
    this.def = def;
    this.curve = new ThrustCurve(def.table);
    this.T = 0; // static-equivalent thrust (N), rotor lag applied
    this.Tcmd = 0;
    this.Teff = 0; // force actually applied last step (N)
    this.pwm = 1500;
  }
  // advance rotor lag over dt for a given ESC PWM
  update(pwm, dt) {
    this.pwm = pwm;
    // prop polarity flips the physical thrust direction for a given PWM
    this.Tcmd = this.curve.thrust(pwm) * (this.def.polarity ?? 1) * (this.def.enabled === false ? 0 : 1) * (this.def.health ?? 1);
    const tau = Math.abs(this.Tcmd) > Math.abs(this.T) ? (this.def.tauUp ?? 0.08) : (this.def.tauDown ?? 0.12);
    const a = tau > 1e-6 ? 1 - Math.exp(-dt / tau) : 1;
    this.T += (this.Tcmd - this.T) * a;
  }
  // effective thrust given inflow velocity along the push direction and depth
  effective(inflowAlongPush, depthBelowSurface) {
    const d = this.def;
    const T = this.T;
    if (T === 0) return 0;
    const Tmax = T > 0 ? this.curve.fwdMax : this.curve.revMax;
    let k = 1;
    if (d.inflowModel !== false && Tmax > 0) {
      const nRatio = Math.sqrt(Math.min(1, Math.abs(T) / Tmax));
      const V0 = (d.J0 ?? 0.9) * (d.propDiameter ?? 0.076) * ((d.maxRpm ?? 3500) / 60) * Math.max(nRatio, 0.05);
      k = clamp(1 - inflowAlongPush / V0, 0, 1.25);
    }
    // ventilation / emergence near the free surface
    const D = d.propDiameter ?? 0.076;
    let vent = 1;
    if (depthBelowSurface < 1.5 * D) vent = depthBelowSurface <= 0 ? 0 : Math.sqrt(clamp(depthBelowSurface / (1.5 * D), 0, 1));
    const ded = 1 - clamp(d.deduction ?? 0, 0, 0.9);
    return T * k * vent * ded;
  }
}
