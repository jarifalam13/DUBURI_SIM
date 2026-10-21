// Water body: pool/tank geometry (any size, flat or sloped floor), water
// properties, hydrostatic pressure, current + turbulence, surface waves.
// World frame is NED with the origin at the still-water surface, pool centre.

import { GaussMarkov } from '../core/rng.js';
import { waterDensity, kinematicViscosity, eos80SecantBulkModulus, toT68 } from '../hydro/water.js';
import { GRAVITY_MSS } from '../core/math.js';

export const DEFAULT_POOL = {
  length: 25, // m, along north (x)
  width: 12, // m, along east (y)
  depth: 4, // m, deepest point (the whole floor when flat)
  floor: 'flat', // 'flat' | 'sloped'
  shallowDepth: 1.2, // m, shallow-end depth (sloped floor)
  shallowEnd: 'south', // 'south' | 'north'
  slopeStart: 8, // m from the shallow-end wall where the floor starts to drop
  slopeEnd: 14, // m from the shallow-end wall where it reaches full depth
  freeboard: 0.3, // m of wall above the water (visual)
  style: 'tiles', // 'tiles' (pool) | 'natural' (lake / sea bottom, no visible walls)
};

export const DEFAULT_ENVIRONMENT = {
  temperature: 26, // degC (indoor pool)
  salinity: 0, // g/kg (fresh)
  densityOverride: 0, // kg/m^3, 0 = compute from temperature & salinity
  airPressure: 1013.25, // hPa at the water surface
  pool: { ...DEFAULT_POOL },
  current: [0, 0, 0], // m/s NED
  turbulence: { sigma: 0, tau: 2.0 }, // m/s, s
  waves: { amplitude: 0, period: 2.0, direction: 0 }, // m, s, deg (from north, heading of propagation)
  visibility: 30, // m, water clarity (visual only)
  surfaceDrag: 0.6, // extra translational drag factor when breaking the surface
  contact: { stiffness: 30000, damping: 900, friction: 0.5 },
  gravity: GRAVITY_MSS,
};

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

// Floor depth along the pool. A sloped floor is flat at the shallow end, drops
// linearly between slopeStart and slopeEnd (metres from the shallow-end wall)
// and is flat again at the deep end. x is the north coordinate.
export function floorProfile(pool = DEFAULT_POOL) {
  const L = Math.max(0.1, +pool.length || DEFAULT_POOL.length);
  const W = Math.max(0.1, +pool.width || DEFAULT_POOL.width);
  const Dd = Math.max(0.05, +pool.depth || DEFAULT_POOL.depth);
  const flat = { L, W, sloped: false, minDepth: Dd, maxDepth: Dd, shallowDepth: Dd, deepDepth: Dd, breaks: [], depthAt: () => Dd, slopeAt: () => 0, a: 0, b: L, north: false };
  if (pool.floor !== 'sloped') return flat;
  const Ds = clamp(+pool.shallowDepth || Dd, 0.05, Dd);
  if (Dd - Ds < 1e-6) return flat;
  const north = pool.shallowEnd === 'north';
  let a = clamp(Number.isFinite(+pool.slopeStart) ? +pool.slopeStart : 0, 0, L);
  let b = clamp(Number.isFinite(+pool.slopeEnd) ? +pool.slopeEnd : L, 0, L);
  if (b < a) [a, b] = [b, a];
  // steeper than 60° is a wall, not a slope: widen it around its middle
  const minRun = Math.min(L, (Dd - Ds) * Math.tan(Math.PI / 6));
  if (b - a < minRun) {
    const mid = clamp((a + b) / 2, minRun / 2, L - minRun / 2);
    a = mid - minRun / 2; b = mid + minRun / 2;
  }
  const k = (Dd - Ds) / (b - a);
  const s = north ? (x) => L / 2 - x : (x) => x + L / 2; // distance from the shallow-end wall
  const xOf = north ? (ss) => L / 2 - ss : (ss) => ss - L / 2;
  return {
    L, W, sloped: true, minDepth: Ds, maxDepth: Dd, shallowDepth: Ds, deepDepth: Dd, a, b, north,
    breaks: [xOf(a), xOf(b)].sort((p, q) => p - q),
    depthAt: (x) => { const ss = s(x); return ss <= a ? Ds : ss >= b ? Dd : Ds + k * (ss - a); },
    // d(depth)/d(north)
    slopeAt: (x) => { const ss = s(x); return ss > a && ss < b ? (north ? -k : k) : 0; },
  };
}

export class Environment {
  constructor(cfg = DEFAULT_ENVIRONMENT, rng) {
    this.rng = rng;
    this.gm = [new GaussMarkov(rng), new GaussMarkov(rng), new GaussMarkov(rng)];
    this.configure(cfg);
    this.time = 0;
    this.gust = [0, 0, 0];
  }
  configure(cfg) {
    this.cfg = { ...DEFAULT_ENVIRONMENT, ...cfg, pool: { ...DEFAULT_POOL, ...(cfg.pool || {}) }, turbulence: { ...DEFAULT_ENVIRONMENT.turbulence, ...(cfg.turbulence || {}) }, waves: { ...DEFAULT_ENVIRONMENT.waves, ...(cfg.waves || {}) }, contact: { ...DEFAULT_ENVIRONMENT.contact, ...(cfg.contact || {}) } };
    this.rho = this.cfg.densityOverride > 0 ? this.cfg.densityOverride : waterDensity(this.cfg.temperature, this.cfg.salinity);
    this.nu = kinematicViscosity(this.cfg.temperature, this.cfg.salinity);
    this.g = this.cfg.gravity;
    this.pAtm = (Number.isFinite(+this.cfg.airPressure) && this.cfg.airPressure > 0 ? +this.cfg.airPressure : 1013.25) * 100; // Pa
    this.floor = floorProfile(this.cfg.pool);
    const t = this.cfg.turbulence;
    this.gm.forEach((g, i) => { g.sigma = t.sigma * (i === 2 ? 0.5 : 1); g.tau = t.tau; });
    const w = this.cfg.waves;
    this.waveOmega = w.period > 0 ? (2 * Math.PI) / w.period : 0;
    this.waveK = (this.waveOmega * this.waveOmega) / this.g; // deep-water dispersion
    const b = (w.direction * Math.PI) / 180;
    this.waveDir = [Math.cos(b), Math.sin(b)];
    this.buildPressureTable();
  }

  // ---- geometry ----
  floorDepth(x) { return this.floor.depthAt(x); }
  floorSlope(x) { return this.floor.slopeAt(x); }

  // ---- hydrostatics ----
  // Water is compressible: density follows the EOS-80 secant bulk modulus,
  //   rho(p) = rho_surface / (1 - p / K(S, T, p)),
  // and the gauge pressure integrates dp/dz = rho(p) g (RK4, tabulated down to
  // just below the deepest floor, extrapolated beyond).
  densityAtGauge(pPa) {
    const pb = Math.max(0, pPa) / 1e5;
    return this.rho / (1 - pb / eos80SecantBulkModulus(this.salinityPSU, this.T68, pb));
  }
  buildPressureTable() {
    this.salinityPSU = Math.max(0, +this.cfg.salinity || 0);
    this.T68 = toT68(+this.cfg.temperature || 20);
    const zmax = Math.max(1, this.floor.maxDepth) * 1.02 + 2;
    const N = 512, dz = zmax / N;
    const p = new Float64Array(N + 1);
    const f = (pp) => this.g * this.densityAtGauge(pp);
    for (let i = 0; i < N; i++) {
      const k1 = f(p[i]), k2 = f(p[i] + 0.5 * dz * k1), k3 = f(p[i] + 0.5 * dz * k2), k4 = f(p[i] + dz * k3);
      p[i + 1] = p[i] + (dz / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
    }
    this.pTable = p; this.pDz = dz; this.pN = N;
  }
  // gauge pressure [Pa] at a depth below the still-water surface
  pressureGauge(depth) {
    if (!(depth > 0)) return 0;
    const u = depth / this.pDz;
    if (u >= this.pN) { const pe = this.pTable[this.pN]; return pe + (depth - this.pN * this.pDz) * this.g * this.densityAtGauge(pe); }
    const i = Math.floor(u), fr = u - i;
    return this.pTable[i] + (this.pTable[i + 1] - this.pTable[i]) * fr;
  }
  densityAtDepth(depth) { return this.densityAtGauge(this.pressureGauge(depth)); }
  // gauge pressure at a world point (NED): hydrostatic + linear-wave dynamic part
  pressureAt(x, y, z) {
    const eta = this.surfaceElevation(x, y);
    if (z <= -eta) return 0; // above the local free surface
    const zEff = eta ? z + eta * Math.exp(-this.waveK * Math.max(0, z)) : z;
    return this.pressureGauge(zEff);
  }
  absolutePressureAt(x, y, z) { return this.pAtm + this.pressureAt(x, y, z); }

  step(dt) {
    this.time += dt;
    for (let i = 0; i < 3; i++) this.gust[i] = this.gm[i].step(dt);
  }
  // free-surface elevation (positive up) at horizontal position
  surfaceElevation(x, y, t = this.time) {
    const w = this.cfg.waves;
    if (!(w.amplitude > 0) || !this.waveOmega) return 0;
    const ph = this.waveK * (x * this.waveDir[0] + y * this.waveDir[1]) - this.waveOmega * t;
    return w.amplitude * Math.cos(ph);
  }
  // fluid velocity (NED) at a world point; z positive down
  fluidVelocity(x, y, z, t = this.time) {
    const c = this.cfg.current;
    let vx = c[0] + this.gust[0], vy = c[1] + this.gust[1], vz = c[2] + this.gust[2];
    const w = this.cfg.waves;
    if (w.amplitude > 0 && this.waveOmega) {
      const ph = this.waveK * (x * this.waveDir[0] + y * this.waveDir[1]) - this.waveOmega * t;
      const decay = Math.exp(-this.waveK * Math.max(0, z));
      const uh = w.amplitude * this.waveOmega * decay * Math.cos(ph);
      vx += uh * this.waveDir[0];
      vy += uh * this.waveDir[1];
      vz += -w.amplitude * this.waveOmega * decay * Math.sin(ph); // NED (down positive)
    }
    return [vx, vy, vz];
  }
}
