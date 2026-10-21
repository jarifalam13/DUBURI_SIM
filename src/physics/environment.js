// Water body: pool geometry, water properties, current + turbulence, surface waves.
// World frame is NED with the origin at the still-water surface, pool centre.

import { GaussMarkov } from '../core/rng.js';
import { waterDensity, kinematicViscosity } from '../hydro/water.js';
import { GRAVITY_MSS } from '../core/math.js';

export const DEFAULT_ENVIRONMENT = {
  temperature: 26, // degC (indoor pool)
  salinity: 0, // g/kg (fresh)
  densityOverride: 0, // kg/m^3, 0 = compute from temperature & salinity
  pool: { length: 25, width: 12, depth: 4 },
  current: [0, 0, 0], // m/s NED
  turbulence: { sigma: 0, tau: 2.0 }, // m/s, s
  waves: { amplitude: 0, period: 2.0, direction: 0 }, // m, s, deg (from north, heading of propagation)
  surfaceDrag: 0.6, // extra translational drag factor when breaking the surface
  contact: { stiffness: 30000, damping: 900, friction: 0.5 },
  gravity: GRAVITY_MSS,
};

export class Environment {
  constructor(cfg = DEFAULT_ENVIRONMENT, rng) {
    this.rng = rng;
    this.gm = [new GaussMarkov(rng), new GaussMarkov(rng), new GaussMarkov(rng)];
    this.configure(cfg);
    this.time = 0;
    this.gust = [0, 0, 0];
  }
  configure(cfg) {
    this.cfg = { ...DEFAULT_ENVIRONMENT, ...cfg, pool: { ...DEFAULT_ENVIRONMENT.pool, ...(cfg.pool || {}) }, turbulence: { ...DEFAULT_ENVIRONMENT.turbulence, ...(cfg.turbulence || {}) }, waves: { ...DEFAULT_ENVIRONMENT.waves, ...(cfg.waves || {}) }, contact: { ...DEFAULT_ENVIRONMENT.contact, ...(cfg.contact || {}) } };
    this.rho = this.cfg.densityOverride > 0 ? this.cfg.densityOverride : waterDensity(this.cfg.temperature, this.cfg.salinity);
    this.nu = kinematicViscosity(this.cfg.temperature, this.cfg.salinity);
    this.g = this.cfg.gravity;
    const t = this.cfg.turbulence;
    this.gm.forEach((g, i) => { g.sigma = t.sigma * (i === 2 ? 0.5 : 1); g.tau = t.tau; });
    const w = this.cfg.waves;
    this.waveOmega = w.period > 0 ? (2 * Math.PI) / w.period : 0;
    this.waveK = (this.waveOmega * this.waveOmega) / this.g; // deep-water dispersion
    const b = (w.direction * Math.PI) / 180;
    this.waveDir = [Math.cos(b), Math.sin(b)];
  }
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
