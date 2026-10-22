// Thruster model: ESC PWM -> static thrust curve (mirrored about 1500 us for a
// reversed unit) -> bus-voltage scaling -> rotor-speed dynamics (separate
// acceleration / braking time constants, reversal dwell) -> in-flow,
// ventilation and hull-interaction corrections; electrical current draw
// I(T, V) and propeller reaction torque (CW/CCW handedness).

import { clamp } from '../core/math.js';

// Build a PWM->thrust table from a few shape parameters (used for presets and
// for thrusters without measured data). The ESC deadband edges are explicit
// breakpoints so thrust is exactly zero inside 1500 +/- deadband.
export function parametricCurve({ fwdMax = 51.5, revMax = 40.2, deadband = 25, expo = 1.25, pwmMin = 1100, pwmMax = 1900 } = {}) {
  const db = clamp(+deadband || 0, 0, 0.9 * Math.min(1500 - pwmMin, pwmMax - 1500));
  const lo = 1500 - db, hi = 1500 + db;
  const pts = [[lo, 0], [1500, 0], [hi, 0]];
  for (let pwm = hi + 10; pwm < pwmMax; pwm += 10) pts.push([pwm, +(fwdMax * Math.pow((pwm - hi) / (pwmMax - hi), expo)).toFixed(4)]);
  for (let pwm = lo - 10; pwm > pwmMin; pwm -= 10) pts.push([pwm, +(-revMax * Math.pow((lo - pwm) / (lo - pwmMin), expo)).toFixed(4)]);
  pts.push([pwmMax, fwdMax], [pwmMin, -revMax]);
  const seen = new Set();
  return pts.sort((a, b) => a[0] - b[0]).filter(([p]) => (seen.has(p) ? false : (seen.add(p), true)));
}

const KGF = 9.80665;
// Full-throttle (1100 / 1900 us) thrust and current vs supply voltage.
// T200: Blue Robotics published performance data (12 / 16 / 20 V points:
// 3.71 / 5.25 / 6.70 kgf forward, 2.92 / 4.10 / 5.05 kgf reverse, about
// 17 / 24 / 32 A), intermediate voltages on the fitted ~V^1.2 law.
// T500: approximate, scaled from the 20 V point with the same law.
export const ELECTRICAL = {
  t200: { V: [10, 12, 14, 16, 18, 20], fwd: [2.98, 3.71, 4.46, 5.25, 6.0, 6.7].map((k) => k * KGF), rev: [2.35, 2.92, 3.51, 4.1, 4.6, 5.05].map((k) => k * KGF), I: [13.6, 17.0, 20.5, 24.0, 28.0, 32.0] },
  t500: { V: [10, 12, 14, 16, 18, 20], fwd: [68.3, 85.1, 102.4, 120.1, 138.3, 157], rev: [50.9, 63.4, 76.3, 89.5, 103.1, 117], I: [16.5, 20.6, 24.8, 29.1, 33.5, 38.0] },
};

export const THRUSTER_PRESETS = {
  't200-16v': {
    label: 'Blue Robotics T200 @ 16 V (approx.)',
    curve: { fwdMax: 51.5, revMax: 40.2, deadband: 25, expo: 1.25 },
    // tauUp / tauDown are rotor-speed time constants (thrust ~ n|n|)
    propDiameter: 0.076, maxRpm: 3500, J0: 0.9, J0Rev: 0.75, tauUp: 0.05, tauDown: 0.075, mass: 0.344,
    voltage: 16, elec: 't200', torqueRatio: 0.12, reverseDwell: 0.03,
  },
  't200-12v': {
    label: 'Blue Robotics T200 @ 12 V (approx.)',
    curve: { fwdMax: 36.3, revMax: 28.4, deadband: 25, expo: 1.25 },
    propDiameter: 0.076, maxRpm: 3000, J0: 0.9, J0Rev: 0.75, tauUp: 0.055, tauDown: 0.08, mass: 0.344,
    voltage: 12, elec: 't200', torqueRatio: 0.12, reverseDwell: 0.03,
  },
  't500-20v': {
    label: 'Blue Robotics T500 @ 20 V (approx.)',
    curve: { fwdMax: 157, revMax: 117, deadband: 25, expo: 1.3 },
    propDiameter: 0.104, maxRpm: 3200, J0: 0.9, J0Rev: 0.75, tauUp: 0.075, tauDown: 0.1, mass: 0.9,
    voltage: 20, elec: 't500', torqueRatio: 0.12, reverseDwell: 0.04,
  },
  'generic-small': {
    label: 'Generic small thruster (≈2 kgf)',
    curve: { fwdMax: 20, revMax: 15, deadband: 30, expo: 1.3 },
    propDiameter: 0.06, maxRpm: 4000, J0: 0.85, J0Rev: 0.7, tauUp: 0.045, tauDown: 0.065, mass: 0.2,
    voltage: 16, elec: null, torqueRatio: 0.12, reverseDwell: 0.03,
  },
};

// piecewise-linear in range, ~V^1.2 power law outside
function interpV(Vs, Ys, V) {
  const n = Vs.length;
  if (V <= Vs[0]) return Ys[0] * Math.pow(Math.max(V, 0) / Vs[0], 1.2);
  if (V >= Vs[n - 1]) return Ys[n - 1] * Math.pow(V / Vs[n - 1], 1.2);
  let i = 0;
  while (V > Vs[i + 1]) i++;
  const f = (V - Vs[i]) / (Vs[i + 1] - Vs[i]);
  return Ys[i] + f * (Ys[i + 1] - Ys[i]);
}

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
//   n: signed normalised rotor speed (+1 = full forward-thrust speed at the
//      curve's voltage; sign = direction of thrust along the axis).
//   T: static (bollard) thrust along the axis, T = Tside * n|n|.
export class ThrusterState {
  constructor(def) {
    this.def = def;
    this.curve = new ThrustCurve(def.table);
    this.preset = THRUSTER_PRESETS[def.preset] ?? null;
    const pr = this.preset ?? {};
    const pick = (k, dflt) => def[k] ?? pr[k] ?? dflt;
    this.Vnom = pick('tableVoltage', null) ?? pick('voltage', 16);
    const ek = def.elec !== undefined ? def.elec : pr.elec;
    this.elec = typeof ek === 'object' && ek ? ek : ELECTRICAL[ek] ?? null;
    this.torqueRatio = pick('torqueRatio', 0.12);
    this.reverseDwell = Math.max(0, pick('reverseDwell', 0.03));
    this.J0Rev = pick('J0Rev', (def.J0 ?? 0.9) * 0.83);
    this.escMinV = pick('escMinV', 6.5);
    this.idleA = pick('idleCurrent', 0.05);
    this.n = 0;
    this.dwell = 0;
    this.Tcmd = 0;
    this.Teff = 0; // force actually applied last step (N)
    this.Q = 0; // reaction torque about the axis applied last step (N m)
    this.vent = 1;
    this.pwm = 1500;
    this.vbus = this.Vnom;
    this.current = 0; // A drawn from the bus
  }
  side(sign) { return sign >= 0 ? this.curve.fwdMax : this.curve.revMax; }
  get T() { const n = this.n; return n === 0 ? 0 : this.side(n) * n * Math.abs(n); }
  set T(v) { const s = this.side(v); this.n = s > 0 && v ? Math.sign(v) * Math.sqrt(Math.abs(v) / s) : 0; }
  // full-throttle thrust on one side at bus voltage V relative to the curve's voltage
  voltageScale(V, sign) {
    if (!(V > this.escMinV)) return 0;
    const e = this.elec;
    if (!e) return Math.pow(V / this.Vnom, 1.2);
    const Y = sign >= 0 ? e.fwd : e.rev;
    return interpV(e.V, Y, V) / interpV(e.V, Y, this.Vnom);
  }
  // full-throttle current at bus voltage V (A)
  maxCurrent(V) {
    if (this.elec) return interpV(this.elec.V, this.elec.I, V);
    // momentum theory: P = T^1.5 / sqrt(2 rho A) / eta, eta ~ 0.32 (fits the T200)
    const D = this.def.propDiameter ?? 0.076, A = (Math.PI * D * D) / 4;
    const Tm = Math.max(this.curve.fwdMax, this.curve.revMax) * Math.pow(V / this.Vnom, 1.2);
    return Math.pow(Tm, 1.5) / Math.sqrt(2 * 1000 * A) / 0.32 / Math.max(V, 1);
  }
  // advance the rotor over dt for a given ESC PWM and bus voltage
  update(pwm, dt, vbus = this.Vnom) {
    this.pwm = pwm;
    this.vbus = vbus;
    const d = this.def;
    // A reversed unit (CCW prop or swapped phases, MOT_n_DIRECTION = -1 on a
    // correctly built vehicle) reaches its efficient side with reverse rotation:
    // T_rev(pwm) = T(3000 - pwm), mirrored about 1500 us, not -T(pwm).
    const pol = d.polarity ?? 1;
    let base;
    if (pol < 0) base = d.tableOnReversedUnit ? -this.curve.thrust(pwm) : this.curve.thrust(3000 - pwm);
    else base = this.curve.thrust(pwm);
    const on = d.enabled === false ? 0 : 1;
    this.Tcmd = base * on * (d.health ?? 1) * this.voltageScale(vbus, base);
    const sc = this.side(this.Tcmd);
    const nCmd = sc > 0 && this.Tcmd ? Math.sign(this.Tcmd) * Math.sqrt(Math.abs(this.Tcmd) / sc) : 0;
    const tauUp = d.tauUp ?? 0.05, tauDown = d.tauDown ?? 0.075;
    if (this.dwell > 0) {
      // ESC re-commutating after a reversal: rotor held at rest
      this.dwell -= dt;
      if (nCmd === 0) this.dwell = 0;
    } else {
      const n = this.n;
      // braking (toward smaller |n|, including the first half of a reversal) uses tauDown
      const decel = (n !== 0 && Math.sign(nCmd) !== Math.sign(n)) || Math.abs(nCmd) < Math.abs(n);
      const tau = decel ? tauDown : tauUp;
      const a = tau > 1e-6 ? 1 - Math.exp(-dt / tau) : 1;
      let nn = n + (nCmd - n) * a;
      if (n !== 0 && nCmd !== 0 && Math.sign(nn) !== Math.sign(n)) { nn = 0; this.dwell = this.reverseDwell; }
      if (Math.abs(nn) < 1e-6) nn = 0;
      this.n = nn;
    }
    // bus current: I0 + Imax(V) (|T| / Tmax(V))^1.5 (shaft power ~ T^1.5)
    const T = this.T;
    if (!on || T === 0) this.current = on ? this.idleA : 0;
    else {
      const Tm = this.side(T) * Math.max(this.voltageScale(vbus, T), 1e-6);
      this.current = this.idleA + this.maxCurrent(vbus) * Math.pow(Math.min(1.5, Math.abs(T) / Tm), 1.5);
    }
  }
  // effective thrust given inflow velocity along the push direction and depth
  effective(inflowAlongPush, depthBelowSurface) {
    const d = this.def;
    const T = this.T;
    if (T === 0) { this.vent = 1; return 0; }
    let k = 1;
    if (d.inflowModel !== false) {
      // zero-thrust advance speed: J0 n D (reverse props are less efficient: J0Rev)
      const J0 = T > 0 ? (d.J0 ?? 0.9) : this.J0Rev;
      const V0 = J0 * (d.propDiameter ?? 0.076) * ((d.maxRpm ?? 3500) / 60) * Math.max(Math.abs(this.n), 0.05);
      k = clamp(1 - inflowAlongPush / V0, 0, 1.25);
    }
    // ventilation / emergence near the free surface
    const D = d.propDiameter ?? 0.076;
    let vent = 1;
    if (depthBelowSurface < 1.5 * D) vent = depthBelowSurface <= 0 ? 0 : Math.sqrt(clamp(depthBelowSurface / (1.5 * D), 0, 1));
    this.vent = vent;
    // forward thrust: jet leaves along -axis (deduction); reverse: along +axis (deductionRev)
    const dd = T >= 0 ? (d.deduction ?? 0) : (d.deductionRev ?? d.deduction ?? 0);
    const ded = 1 - clamp(dd, 0, 0.9);
    return T * k * vent * ded;
  }
  // Propeller reaction torque about +axis on the vehicle: Q = (KQ/KT) D T,
  // opposite to the prop's spin. hand = +1: the prop spins positively about
  // +axis when pushing along +axis (right-handed / CW seen from behind).
  reactionTorque() {
    const T = this.T;
    if (T === 0) return 0;
    const hand = this.def.hand ?? 1;
    return -hand * this.torqueRatio * (this.def.propDiameter ?? 0.076) * T * this.vent;
  }
}
