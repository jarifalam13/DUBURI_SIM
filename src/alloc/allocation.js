// Thruster allocation analysis and automatic "maximum-DOF" mixer generation.
//
// Wrench order used here: [Fx, Fy, Fz, Mx, My, Mz] in the body FRD frame about
// the CG. ArduSub's six pilot/controller inputs map onto unit wrenches:
//   roll -> +Mx, pitch -> +My, yaw -> +Mz, throttle -> -Fz (up),
//   forward -> +Fx, lateral -> +Fy.

import { vcross, vsub, vnorm, svd, pinv, zeros } from '../core/math.js';
import { FACTOR_AXES } from './frames.js';

export const AXES = FACTOR_AXES; // roll, pitch, yaw, throttle, forward, lateral
export const AXIS_LABEL = { roll: 'Roll', pitch: 'Pitch', yaw: 'Yaw', throttle: 'Heave', forward: 'Surge', lateral: 'Sway' };
export const AXIS_ROW = { forward: 0, lateral: 1, throttle: 2, roll: 3, pitch: 4, yaw: 5 };
export const AXIS_SIGN = { forward: 1, lateral: 1, throttle: -1, roll: 1, pitch: 1, yaw: 1 };
export const AXIS_UNIT = { forward: 'N', lateral: 'N', throttle: 'N', roll: 'N·m', pitch: 'N·m', yaw: 'N·m' };

export function thrusterNominal(t) {
  const fwd = Math.max(1e-6, t.fwdMax ?? 50), rev = Math.max(1e-6, t.revMax ?? 40);
  return { fwd, rev, nom: 0.5 * (fwd + rev) };
}

// 6 x N allocation matrix (columns scaled by nominal thrust) about point `cg`.
export function buildB(thrusters, cg = [0, 0, 0]) {
  const N = thrusters.length;
  const B = zeros(6, N);
  const nom = [];
  thrusters.forEach((t, i) => {
    const a = vnorm(t.axis);
    const r = vsub(t.pos, cg);
    const m = vcross(r, a);
    const F = thrusterNominal(t).nom * (t.enabled === false ? 0 : 1);
    nom.push(F);
    B[0][i] = a[0] * F; B[1][i] = a[1] * F; B[2][i] = a[2] * F;
    B[3][i] = m[0] * F; B[4][i] = m[1] * F; B[5][i] = m[2] * F;
  });
  return { B, nom };
}

function rowsOf(B, axes) {
  return axes.map((ax) => B[AXIS_ROW[ax]].map((v) => v * AXIS_SIGN[ax]));
}

function subsets(list) {
  const out = [];
  const n = list.length;
  for (let mask = 1; mask < 1 << n; mask++) {
    const s = [];
    for (let i = 0; i < n; i++) if (mask & (1 << i)) s.push(list[i]);
    out.push(s);
  }
  return out.sort((a, b) => b.length - a.length);
}

/**
 * Analyse a thruster layout.
 * @param thrusters [{pos, axis, fwdMax, revMax, enabled}]
 * @param cg        centre of gravity in the same frame
 * @param opts      { threshold (relative sigma), enabledAxes: {roll:true,...} }
 */
export function analyzeLayout(thrusters, cg = [0, 0, 0], opts = {}) {
  const threshold = opts.threshold ?? 0.02;
  const enabledAxes = AXES.filter((a) => (opts.enabledAxes ? opts.enabledAxes[a] !== false : true));
  const N = thrusters.length;
  const result = { N, controllable: [], status: {}, factors: zeros(N, 6), authority: {}, coupling: {}, singular: [], rank: 0, warnings: [] };
  if (N === 0) {
    AXES.forEach((a) => (result.status[a] = 'none'));
    result.warnings.push('No thrusters defined.');
    return result;
  }
  const { B, nom } = buildB(thrusters, cg);
  const lever = Math.sqrt(thrusters.reduce((s, t) => s + vsub(t.pos, cg).reduce((q, v) => q + v * v, 0), 0) / N) || 0.1;
  const Fref = nom.reduce((s, v) => s + v, 0) / N || 1;
  const scale = AXES.map((a) => (['roll', 'pitch', 'yaw'].includes(a) ? 1 / (Fref * lever) : 1 / Fref));
  const scaled = (axes) => rowsOf(B, axes).map((row, i) => row.map((v) => v * scale[AXES.indexOf(axes[i])]));
  const full = svd(scaled(AXES));
  result.singular = full.S;
  const smax = full.S[0] || 1;
  result.rank = full.S.filter((s) => s > smax * 1e-9).length;

  // Pick the largest axis subset that is independently controllable.
  let best = null;
  const candidates = [];
  for (const S of subsets(enabledAxes)) {
    if (S.length > N) continue;
    if (best && S.length < best.axes.length) break;
    const sv = svd(scaled(S)).S;
    const smin = sv[Math.min(S.length, N) - 1] ?? 0;
    const rel = smin / smax;
    candidates.push({ axes: S, rel });
    if (rel >= threshold && (!best || rel > best.rel)) best = { axes: S, rel };
  }
  const chosen = best ? best.axes : [];
  result.controllable = AXES.filter((a) => chosen.includes(a));
  result.conditioning = best ? best.rel : 0;

  // Per-axis status: controllable / weak (achievable only coupled) / none
  const P = pinv(scaled(AXES));
  for (const a of AXES) {
    if (chosen.includes(a)) { result.status[a] = 'full'; continue; }
    if (!enabledAxes.includes(a)) { result.status[a] = 'disabled'; continue; }
    // how much of a pure command on this axis lies in the reachable space
    const e = AXES.map((x) => (x === a ? 1 : 0));
    const u = P.map((row) => row.reduce((s, v, j) => s + v * e[j], 0));
    const w = scaled(AXES).map((row) => row.reduce((s, v, j) => s + v * u[j], 0));
    const along = w[AXES.indexOf(a)];
    result.status[a] = along > 0.15 ? 'weak' : 'none';
  }

  if (chosen.length) {
    const BS = rowsOf(B, chosen);
    const Pinv = pinv(BS);
    chosen.forEach((ax, ci) => {
      const target = chosen.map((_, j) => (j === ci ? 1 : 0));
      const u = Pinv.map((row) => row.reduce((s, v, j) => s + v * target[j], 0)); // normalised thrust units
      const umax = Math.max(...u.map(Math.abs)) || 1;
      const k = AXES.indexOf(ax);
      u.forEach((v, i) => (result.factors[i][k] = clean(v / umax)));
      // authority (pure axis, respecting asymmetric thrust)
      const lim = (sgn) => {
        let s = Infinity;
        u.forEach((v, i) => {
          if (Math.abs(v) < 1e-12) return;
          const { fwd, rev, nom: Fn } = thrusterNominal(thrusters[i]);
          const cap = (v * sgn > 0 ? fwd : rev) / Fn;
          s = Math.min(s, cap / Math.abs(v));
        });
        return Number.isFinite(s) ? s : 0;
      };
      result.authority[ax] = { pos: lim(1), neg: lim(-1), unit: AXIS_UNIT[ax] };
      // coupling into the other axes (per unit of this axis)
      const w = AXES.map((other) => rowsOf(B, [other])[0].reduce((s, v, j) => s + v * u[j], 0));
      result.coupling[ax] = {};
      AXES.forEach((other, j) => {
        if (other !== ax && Math.abs(w[j]) > 1e-9) result.coupling[ax][other] = w[j];
      });
    });
  }
  AXES.forEach((a) => { if (!result.authority[a]) result.authority[a] = { pos: 0, neg: 0, unit: AXIS_UNIT[a] }; });
  result.candidates = candidates.slice(0, 12);
  result.lever = lever;
  result.Fref = Fref;
  if (result.controllable.length === 6) result.summary = 'Full 6-DOF control';
  else result.summary = `${result.controllable.length}-DOF control (${result.controllable.map((a) => AXIS_LABEL[a]).join(', ') || 'none'})`;
  return result;
}

const clean = (v) => (Math.abs(v) < 1e-9 ? 0 : Math.round(v * 1e6) / 1e6);

/**
 * Wrench produced per unit of each ArduSub input for a given factor table,
 * including motor direction / prop polarity signs.
 * Returns 6x6 matrix W[input][wrenchRow] in N / N·m.
 */
export function effectiveWrench(thrusters, factors, cg = [0, 0, 0], netSigns = null) {
  const { B } = buildB(thrusters, cg);
  const W = zeros(6, 6);
  for (let k = 0; k < 6; k++) {
    for (let i = 0; i < thrusters.length; i++) {
      const f = (factors[i] ? factors[i][k] : 0) * (netSigns ? netSigns[i] : 1);
      if (!f) continue;
      for (let r = 0; r < 6; r++) W[k][r] += B[r][i] * f;
    }
  }
  return W;
}

// Compare what a factor table actually does against the ideal per-axis command.
export function tableQuality(thrusters, factors, cg, netSigns) {
  const W = effectiveWrench(thrusters, factors, cg, netSigns);
  const rows = AXES.map((ax, k) => {
    const w = W[k];
    const main = w[AXIS_ROW[ax]] * AXIS_SIGN[ax];
    const off = AXES.filter((o) => o !== ax).map((o) => ({ axis: o, value: w[AXIS_ROW[o]] * AXIS_SIGN[o] })).filter((x) => Math.abs(x.value) > 1e-6);
    return { axis: ax, main, off, reversed: main < -1e-9 };
  });
  return { W, rows };
}
