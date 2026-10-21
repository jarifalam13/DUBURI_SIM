// Turn a vehicle config + geometry analysis into the numeric plant model used
// by VehiclePlant, plus human-readable derived quantities for the UI.

import { vsub, vnorm, vadd } from '../core/math.js';
import { VoxelGrid, rayHitDistance } from '../geometry/mesh.js';
import { ellipsoidAddedMass, dragFromProjections } from '../hydro/water.js';
import { shiftMassMatrix } from '../hydro/bem.js';
import { parametricCurve } from '../physics/thruster.js';
import { DEFAULT_HULL } from './config.js';

const zeros6 = () => Array.from({ length: 6 }, () => new Array(6).fill(0));
const diag6 = (d) => { const M = zeros6(); for (let i = 0; i < 6; i++) M[i][i] = d[i]; return M; };
export const isMat6 = (M) => Array.isArray(M) && M.length === 6 && M.every((r) => Array.isArray(r) && r.length === 6 && r.every(Number.isFinite));

// Parse a pasted 6 x 6 matrix (rows on lines, any of comma / tab / space / ;)
export function parseMatrix6(text) {
  const nums = String(text).replace(/[[\]]/g, ' ').split(/[\s,;]+/).filter(Boolean).map(Number);
  if (nums.length !== 36 || !nums.every(Number.isFinite)) throw new Error(`Expected 36 numbers, found ${nums.filter(Number.isFinite).length}`);
  return Array.from({ length: 6 }, (_, i) => nums.slice(6 * i, 6 * i + 6));
}

export function cgOf(cfg, geo) {
  if (cfg.mass.cgMode === 'manual') return cfg.mass.cg.slice();
  return vadd(geo.centroid, cfg.mass.cgOffset || [0, 0, 0]);
}

export function inertiaTensor(cfg, geo, cg) {
  const I = cfg.mass.inertia;
  if (I.mode === 'manual') {
    // CAD tools report products of inertia as P = integral(x y dm); tensor uses -P.
    return [I.Ixx, -I.Pxy, -I.Pxz, -I.Pxy, I.Iyy, -I.Pyz, -I.Pxz, -I.Pyz, I.Izz];
  }
  const rho = cfg.mass.mass / Math.max(geo.volume, 1e-9);
  const Ic = geo.inertiaUnit.map((v) => v * rho);
  const d = vsub(geo.solidCentroid || geo.centroid, cg);
  const m = cfg.mass.mass, dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
  const out = Ic.slice();
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) out[i * 3 + j] += m * ((i === j ? dd : 0) - d[i] * d[j]);
  return out;
}

// Jet/hull interaction: fraction of the thruster jet that hits the vehicle
// within a few diameters, mapped to a thrust-deduction factor.
export function jetBlockage(wet, pos, jetDir, D) {
  if (!wet) return 0;
  const grid = wet instanceof VoxelGrid ? wet : Object.assign(new VoxelGrid(wet.nx, wet.ny, wet.nz, wet.cell, wet.origin), { data: wet.data });
  const a = vnorm(jetDir);
  const t0 = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const u = vnorm([a[1] * t0[2] - a[2] * t0[1], a[2] * t0[0] - a[0] * t0[2], a[0] * t0[1] - a[1] * t0[0]]);
  const v = [a[1] * u[2] - a[2] * u[1], a[2] * u[0] - a[0] * u[2], a[0] * u[1] - a[1] * u[0]];
  const L = 4 * D;
  const start = 0.8 * D + grid.cell;
  let acc = 0, n = 0;
  const offsets = [[0, 0]];
  for (let k = 0; k < 6; k++) offsets.push([0.7 * Math.cos((k * Math.PI) / 3), 0.7 * Math.sin((k * Math.PI) / 3)]);
  for (const [ou, ov] of offsets) {
    const r = D / 2;
    const o = [0, 1, 2].map((q) => pos[q] + a[q] * start + (u[q] * ou + v[q] * ov) * r);
    const d = rayHitDistance(grid, o, a, L);
    n++;
    if (Number.isFinite(d)) acc += 1 - d / L;
  }
  return Math.min(0.5, 0.5 * (acc / n));
}

export function buildPlantModel(cfg, geo, env) {
  const warnings = [];
  const cg = cgOf(cfg, geo);
  const mass = cfg.mass.mass;
  if (!(mass > 0)) throw new Error('Mass must be positive');
  const rho = env.rho;

  // buoyancy
  let volume, cb;
  if (cfg.buoyancy.mode === 'manual') { volume = cfg.buoyancy.volume; cb = cfg.buoyancy.cb.slice(); }
  else { volume = geo.volume; cb = geo.centroid.slice(); }
  const scale = geo.volume > 0 ? volume / geo.volume : 1;
  const shift = vsub(cb, geo.centroid);
  const buoyEls = geo.buoyEls.map((e) => ({ p: vsub(vadd(e.p, shift), cg), v: e.v * scale, h: e.h }));

  // inertia
  const Ig = inertiaTensor(cfg, geo, cg);
  if (Ig[0] <= 0 || Ig[4] <= 0 || Ig[8] <= 0) throw new Error('Inertia must be positive');

  // added mass
  let MA, maSource;
  const am = cfg.hydro.addedMass;
  if (am.mode === 'bem' && am.matrix) { MA = shiftMassMatrix(am.matrix, am.ref || [0, 0, 0], cg); maSource = 'BEM'; }
  else if (am.mode === 'matrix' && isMat6(am.userMatrix)) { MA = am.userRef === 'origin' ? shiftMassMatrix(am.userMatrix, [0, 0, 0], cg) : am.userMatrix.map((r) => r.slice()); maSource = 'your 6×6 matrix'; }
  else if (am.mode === 'manual') { MA = diag6(am.diag); maSource = 'manual'; }
  else {
    const s = geo.bbox.size;
    const e = ellipsoidAddedMass(s[0] / 2, s[1] / 2, s[2] / 2, rho).values;
    MA = diag6(e);
    maSource = 'ellipsoid estimate';
  }
  // keep the mass matrix symmetric positive definite-ish
  for (let i = 0; i < 6; i++) if (MA[i][i] < 0) { warnings.push('Negative added-mass diagonal clamped to 0'); MA[i][i] = 0; }

  // damping
  let DL, DQ, dSource;
  const dm = cfg.hydro.damping;
  if (dm.mode === 'cfd' && dm.DL && dm.DQ) {
    DL = dm.DL; DQ = dm.DQ; dSource = 'CFD';
    const off = Math.hypot(...vsub(dm.ref || cg, cg));
    if (off > 0.01) warnings.push(`CFD damping was computed about a point ${(off * 100).toFixed(1)} cm from the current CG; re-run the CFD lab for best accuracy.`);
  } else if (dm.mode === 'matrix' && isMat6(dm.userDL) && isMat6(dm.userDQ)) { DL = dm.userDL.map((r) => r.slice()); DQ = dm.userDQ.map((r) => r.slice()); dSource = 'your 6×6 matrices'; }
  else if (dm.mode === 'manual') { DL = diag6(dm.linear); DQ = diag6(dm.quadratic); dSource = 'manual'; }
  else {
    const d = dragFromProjections(geo.projections, rho, dm.Cd ?? 1.0);
    DL = diag6(d.linear); DQ = diag6(d.quadratic); dSource = `projected-area estimate (Cd ${dm.Cd ?? 1})`;
  }

  // thrusters
  const thrusters = cfg.thrusters.map((t, i) => {
    const axis = vnorm(t.axis);
    if (Math.hypot(...t.axis) < 1e-9) warnings.push(`${t.name || 'Thruster ' + (i + 1)} has a zero direction vector`);
    const table = t.table && t.table.length >= 2 ? t.table : parametricCurve(t.curve);
    let deduction = t.deduction ?? 0, deductionRev = t.deductionRev ?? 0;
    if (t.deductionMode === 'auto') {
      // forward thrust pushes along +axis, so the jet leaves along -axis
      deduction = jetBlockage(geo.wet, t.pos, axis.map((v) => -v), t.propDiameter ?? 0.076);
      deductionRev = jetBlockage(geo.wet, t.pos, axis, t.propDiameter ?? 0.076);
    }
    return { ...t, axis, pos: vsub(t.pos, cg), table, deduction, deductionRev, _index: i };
  });

  const contacts = geo.contacts.map((c) => vsub(c, cg));
  // pressure housing: depth rating, compressibility, floodable air volume
  const H = { ...DEFAULT_HULL, ...(cfg.hull || {}) };
  const hull = {
    rating: Math.max(0, +H.depthRating || 0),
    beta: Math.max(0, +H.compressibility || 0) / 100 / 1e6, // % per 10 bar -> 1/Pa
    floodOverRating: H.floodOverRating !== false,
    leakArea: Math.max(0, +H.leakArea || 0) * 1e-6, // mm^2 -> m^2
    airVolume: (+H.airVolume > 0 ? +H.airVolume / 1000 : 0.2 * volume), // L -> m^3 (auto: 20% of the displaced volume)
    leakSensor: H.leakSensor !== false,
  };
  const plant = { mass, Ig, MA, DL, DQ, volume, cb: vsub(cb, cg), buoyEls, contacts, thrusters, hullHeight: geo.bbox.size[2], hull };

  const g = env.g;
  const W = mass * g, B = rho * g * volume;
  const cbRel = plant.cb;
  const GM = -cbRel[2]; // CB above CG (z down) gives positive restoring arm
  const info = {
    cg, cb, mass, volume, rho,
    weightN: W, buoyancyN: B, netBuoyancyN: B - W, netBuoyancyKg: (B - W) / g,
    GM,
    staticRollDeg: GM > 1e-6 ? (Math.atan2(-cbRel[1], GM) * 180) / Math.PI : NaN,
    staticPitchDeg: GM > 1e-6 ? (Math.atan2(cbRel[0], GM) * 180) / Math.PI : NaN,
    rollPeriod: GM > 1e-6 ? 2 * Math.PI * Math.sqrt((Ig[0] + MA[3][3]) / (B * GM)) : NaN,
    pitchPeriod: GM > 1e-6 ? 2 * Math.PI * Math.sqrt((Ig[4] + MA[4][4]) / (B * GM)) : NaN,
    Ig, MA, DL, DQ, maSource, dSource,
    deductions: thrusters.map((t) => [t.deduction, t.deductionRev]),
    hull,
    warnings,
  };
  if (GM <= 0) warnings.push('Centre of buoyancy is not above the centre of gravity: the vehicle has no passive roll/pitch stability.');
  if (Math.abs(info.netBuoyancyKg) > 0.05 * mass) warnings.push(`Vehicle is ${info.netBuoyancyKg > 0 ? 'positively' : 'negatively'} buoyant by ${Math.abs(info.netBuoyancyKg).toFixed(2)} kg (more than 5% of mass).`);
  return { plant, info };
}
