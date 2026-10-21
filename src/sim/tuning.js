// Step-response tests and simulation-based auto-tuning of ArduSub gains.
// Everything runs on fresh headless Simulation instances, so results are
// reproducible and do not disturb the live session.

import { Simulation, MODE } from './simulation.js';
import { qToEuler, RAD, DEG, wrapPI } from '../core/math.js';
import { effectiveWrench } from '../alloc/allocation.js';
import { thrusterLimits } from '../vehicle/config.js';

export const STEP_AXES = {
  roll: { label: 'Roll', unit: '°', size: 15, band: 0.75 },
  pitch: { label: 'Pitch', unit: '°', size: 15, band: 0.75 },
  yaw: { label: 'Yaw', unit: '°', size: 45, band: 1.0 },
  depth: { label: 'Depth', unit: 'm', size: 0.5, band: 0.02 },
};

const yieldToUI = () => new Promise((r) => setTimeout(r, 0));

function withParams(cfg, params) {
  return { ...cfg, params: { ...cfg.params, ...params }, sim: { ...cfg.sim } };
}

function measure(sim, axis) {
  const e = qToEuler(sim.plant.quaternion);
  if (axis === 'roll') return e[0] * RAD;
  if (axis === 'pitch') return e[1] * RAD;
  if (axis === 'yaw') return e[2] * RAD;
  return sim.plant.position[2];
}

export function stepMetrics(t, y, y0, yf, band) {
  const step = yf - y0;
  const sgn = Math.sign(step) || 1;
  const n = t.length;
  let t10 = NaN, t90 = NaN, peak = -Infinity, settle = 0;
  for (let i = 0; i < n; i++) {
    const prog = (y[i] - y0) / step;
    if (Number.isNaN(t10) && prog >= 0.1) t10 = t[i];
    if (Number.isNaN(t90) && prog >= 0.9) t90 = t[i];
    peak = Math.max(peak, (y[i] - yf) * sgn);
  }
  const tol = Math.max(band, 0.02 * Math.abs(step));
  for (let i = n - 1; i >= 0; i--) if (Math.abs(y[i] - yf) > tol) { settle = t[i]; break; }
  let crossings = 0, prev = 0, reached = false;
  for (let i = 0; i < n; i++) {
    const s = Math.sign(y[i] - yf);
    if (!reached && (y[i] - yf) * sgn >= 0) reached = true;
    if (reached && s !== 0 && prev !== 0 && s !== prev && Math.abs(y[i] - yf) > tol * 0.25) crossings++;
    if (s !== 0) prev = s;
  }
  const tail = y.filter((_, i) => t[i] > t[n - 1] - 1);
  const ss = tail.reduce((a, v) => a + v, 0) / Math.max(1, tail.length) - yf;
  const tail2 = y.filter((_, i) => t[i] > t[n - 1] - 2);
  const rms = Math.sqrt(tail2.reduce((a, v) => a + (v - yf) * (v - yf), 0) / Math.max(1, tail2.length));
  return {
    tailRms: rms,
    riseTime: Number.isNaN(t10) || Number.isNaN(t90) ? NaN : t90 - t10,
    overshoot: Math.max(0, (peak / Math.abs(step)) * 100),
    settlingTime: settle,
    steadyError: ss,
    oscillations: crossings,
    reached: !Number.isNaN(t90),
  };
}

/**
 * Run one step test on a fresh simulation.
 * opts: { axis, size, settle, duration, params, noise }
 */
export function runStepTest(cfg, geo, opts = {}) {
  const axis = opts.axis ?? 'yaw';
  const meta = STEP_AXES[axis];
  const size = opts.size ?? meta.size;
  const c = withParams(cfg, opts.params || {});
  // Step tests run in open water (a deep virtual tank, 3 m down) so the
  // result describes the vehicle, not the walls or floor of the test pool.
  c.start = { position: [0, 0, opts.startDepth ?? 3], yaw: 0 };
  if (opts.noise === false) c.sensors = { ...c.sensors, gyroNoise: 0, accelNoise: 0, attitudeNoise: 0, depthNoise: 0 };
  c.environment = { ...c.environment, pool: { length: 60, width: 60, depth: 12, floor: 'flat', freeboard: 0, style: 'tiles' }, current: [0, 0, 0], turbulence: { sigma: 0, tau: 2 }, waves: { amplitude: 0, period: 2, direction: 0 } };
  c.hull = { ...(c.hull || {}), floodOverRating: false };
  const sim = new Simulation(c, geo, { telemetryCapacity: 16 });
  const fw = sim.fw;
  fw.setMode(MODE.ALT_HOLD);
  sim.runFor(0.4);
  fw.arm();
  const settle = opts.settle ?? 3;
  const yaw0 = qToEuler(sim.plant.quaternion)[2] * RAD;
  // hold level attitude through the GCS attitude-target path while settling
  const hold = (r, p, y) => fw.setAttitudeTarget(r, p, y);
  hold(0, 0, yaw0);
  sim.runFor(settle);
  const y0 = measure(sim, axis);
  let yf;
  if (axis === 'depth') {
    yf = y0 + size;
    fw.depthTargetOverride = -yf * 100;
  } else {
    yf = axis === 'yaw' ? y0 + size : size;
    hold(axis === 'roll' ? size : 0, axis === 'pitch' ? size : 0, axis === 'yaw' ? wrapPI((y0 + size) * DEG) * RAD : yaw0);
  }
  const dur = opts.duration ?? (axis === 'yaw' ? 7 : 6);
  const n = Math.round(dur / sim.dtFw);
  const t = new Float32Array(Math.ceil(n / 2)), y = new Float32Array(t.length), sat = new Uint8Array(t.length), tgt = new Float32Array(t.length);
  let k = 0;
  let unwrap = 0, last = y0;
  for (let i = 0; i < n; i++) {
    if (i % 400 === 0 && axis !== 'depth') hold(axis === 'roll' ? size : 0, axis === 'pitch' ? size : 0, axis === 'yaw' ? wrapPI((y0 + size) * DEG) * RAD : yaw0);
    sim.loop();
    if (i % 2 === 0 && k < t.length) {
      let v = measure(sim, axis);
      if (axis === 'yaw') { // unwrap across +-180
        let d = v + unwrap - last;
        if (d > 180) unwrap -= 360; else if (d < -180) unwrap += 360;
        v += unwrap; last = v;
      }
      t[k] = i * sim.dtFw; y[k] = v; sat[k] = fw.motors.saturated ? 1 : 0;
      tgt[k] = axis === 'depth' ? -fw.pos.posTarget[2] / 100 : axis === 'yaw' ? yf : yf;
      k++;
    }
  }
  const metrics = stepMetrics(t, y, y0, yf, meta.band);
  metrics.saturation = sat.reduce((a, v) => a + v, 0) / sat.length;
  metrics.diverged = !y.every(Number.isFinite) || Math.max(...Array.from(y).map((v) => Math.abs(v - yf))) > Math.abs(size) * 6 + (axis === 'depth' ? 2 : 90);
  return { axis, size, y0, yf, t, y, target: tgt, metrics, unit: meta.unit };
}

export function stepCost(m, axis) {
  if (m.diverged || !m.reached) return 1e3 + (m.diverged ? 1e3 : 0);
  const band = STEP_AXES[axis].band;
  return 0.5 * m.settlingTime + 0.05 * m.overshoot + 0.4 * m.oscillations + 3 * m.saturation + 0.6 * (Number.isFinite(m.riseTime) ? m.riseTime : 3) + 2 * (m.tailRms / band);
}

// ---------------------------------------------------------------------------
// Auto-tune
// ---------------------------------------------------------------------------
const AX = { roll: 'RLL', pitch: 'PIT', yaw: 'YAW' };

export function plantAxisEstimates(sim) {
  const m = sim.model;
  const th = sim.cfg.thrusters.map((t) => ({ ...t, ...thrusterLimits(t) }));
  const signs = sim.cfg.thrusters.map((t, i) => (t.polarity ?? 1) * ((sim.cfg.params[`MOT_${i + 1}_DIRECTION`] ?? 1) < 0 ? -1 : 1));
  const W = effectiveWrench(th, sim.mixer.factors, sim.mixer.cg, signs);
  const tau = Math.max(...sim.cfg.thrusters.map((t) => t.tauUp ?? 0.08));
  return {
    roll: { J: m.Ig[0] + m.MA[3][3], K: Math.abs(W[0][3]) },
    pitch: { J: m.Ig[4] + m.MA[4][4], K: Math.abs(W[1][4]) },
    yaw: { J: m.Ig[8] + m.MA[5][5], K: Math.abs(W[2][5]) },
    heave: { M: m.mass + m.MA[2][2], K: Math.abs(W[3][2]) },
    motorTau: tau,
  };
}

function attitudeGains(est, axis, wc) {
  const { J, K } = est[axis];
  if (!(K > 1e-6)) return null;
  const P = (wc * J) / K;
  const isYaw = axis === 'yaw';
  return {
    [`ATC_RAT_${AX[axis]}_P`]: round(P),
    [`ATC_RAT_${AX[axis]}_I`]: round(P * (isYaw ? 0.25 : 0.6) * Math.min(1, wc / 6)),
    [`ATC_RAT_${AX[axis]}_D`]: round(isYaw ? 0 : P * 0.025),
    [`ATC_ANG_${AX[axis]}_P`]: round(Math.min(12, Math.max(2, wc * (isYaw ? 0.6 : 0.75)))),
  };
}
function depthGains(est, s) {
  const { M, K } = est.heave;
  if (!(K > 1e-6)) return null;
  // throttle (0..1) -> bidirectional x2 -> force K; accel loop gain ~ unity at s=1
  const accP = (5 * M) / K;
  return {
    PSC_ACCZ_P: round(accP * s),
    PSC_ACCZ_I: round(accP * s * 0.5),
    PSC_VELZ_P: round(Math.min(20, 3 * s)),
    PSC_POSZ_P: round(Math.min(6, 1.5 * s)),
  };
}
const round = (v) => +(+v).toPrecision(3);

/**
 * Auto-tune all controllable axes. Returns { params, report }.
 * Every candidate is scored on a full step and a half step the other way, the
 * current gains compete as a candidate, and an axis keeps its current gains
 * unless the tuned ones are clearly better.
 * onProgress(fraction, text)
 */
export async function autoTune(cfg, geo, { axes = ['roll', 'pitch', 'yaw', 'depth'], onProgress = () => {}, margin = 0.85, quick = false } = {}) {
  const base = new Simulation(cfg, geo, { telemetryCapacity: 16 });
  const est = plantAxisEstimates(base);
  const controllable = base.mixer.analysis.controllable;
  const report = {};
  const out = {};
  const total = axes.length;
  let done = 0;
  for (const axis of axes) {
    const prog = (f, txt) => onProgress((done + f) / total, txt);
    if (axis !== 'depth' && !controllable.includes(axis) && cfg.mixer.source === 'auto') { report[axis] = { skipped: 'axis not controllable with this thruster layout' }; done++; continue; }
    const make = axis === 'depth' ? (s) => depthGains(est, s) : (s) => attitudeGains(est, axis, s);
    if (!make(1)) { report[axis] = { skipped: 'no control authority on this axis' }; done++; continue; }
    const size = STEP_AXES[axis].size;
    const evaluate = (params) => {
      const a = runStepTest(cfg, geo, { axis, params, size });
      const b = runStepTest(cfg, geo, { axis, params, size: -size / 2 });
      return { r: a, cost: 0.6 * stepCost(a.metrics, axis) + 0.4 * stepCost(b.metrics, axis) };
    };
    const grid = axis === 'depth' ? (quick ? [0.6, 1, 1.6] : [0.5, 0.8, 1.2, 1.8, 2.6]) : quick ? [3, 6, 10] : [2, 3.5, 5, 7, 10, 14, 20];
    const steps = grid.length + (quick ? 2 : 6) + 2;
    let k = 0;
    const tick = async (txt) => { prog(k++ / steps, `${STEP_AXES[axis].label}: ${txt}`); await yieldToUI(); };
    await tick('current gains');
    const before = evaluate({});
    let best = null;
    for (const s of grid) {
      await tick(`trial ${grid.indexOf(s) + 1}`);
      const e = evaluate(make(s));
      if (!best || e.cost < best.cost) best = { s, ...e };
    }
    // golden-section refinement around the best grid point
    const idx = grid.indexOf(best.s);
    let lo = grid[Math.max(0, idx - 1)], hi = grid[Math.min(grid.length - 1, idx + 1)];
    for (let it = 0; it < (quick ? 1 : 3); it++) {
      await tick('refining');
      const a = lo + 0.382 * (hi - lo), b = lo + 0.618 * (hi - lo);
      const ea = evaluate(make(a)), eb = evaluate(make(b));
      if (ea.cost < best.cost) best = { s: a, ...ea };
      if (eb.cost < best.cost) best = { s: b, ...eb };
      if (ea.cost < eb.cost) hi = b; else lo = a;
    }
    // back off for robustness unless that costs too much performance
    await tick('robustness margin');
    const backed = evaluate(make(best.s * margin));
    const chosen = backed.cost <= best.cost * 1.1 ? { s: best.s * margin, ...backed } : best;
    const keep = before.cost <= chosen.cost * 1.03;
    const params = keep ? {} : make(chosen.s);
    Object.assign(out, params);
    const after = keep ? before : chosen;
    report[axis] = {
      keep, before: before.r.metrics, after: after.r.metrics, beforeCost: before.cost, afterCost: after.cost,
      bandwidth: chosen.s, params, suggested: make(chosen.s),
      curves: { before: { t: before.r.t, y: before.r.y }, after: { t: after.r.t, y: after.r.y }, target: after.r.yf, y0: after.r.y0 },
    };
    done++;
  }
  onProgress(1, 'Done');
  return { params: out, report, estimates: est };
}
