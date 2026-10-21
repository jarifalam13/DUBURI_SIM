// Deterministic pseudo-random numbers so every run (and every auto-tune trial)
// is reproducible from a seed.

export class RNG {
  constructor(seed = 1) {
    this.seed(seed);
  }
  seed(s) {
    this.state = (s >>> 0) || 0x9e3779b9;
    this._spare = null;
  }
  // mulberry32
  next() {
    let t = (this.state = (this.state + 0x6d2b79f5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  uniform(a = 0, b = 1) {
    return a + (b - a) * this.next();
  }
  // standard normal via Box-Muller (cached spare)
  gauss() {
    if (this._spare !== null) {
      const s = this._spare;
      this._spare = null;
      return s;
    }
    let u = 0, v = 0;
    while (u <= 1e-12) u = this.next();
    v = this.next();
    const mag = Math.sqrt(-2 * Math.log(u));
    this._spare = mag * Math.sin(2 * Math.PI * v);
    return mag * Math.cos(2 * Math.PI * v);
  }
}

// First-order Gauss-Markov process (coloured noise), used for turbulence and
// sensor bias drift. sigma = stationary std-dev, tau = correlation time [s].
export class GaussMarkov {
  constructor(rng, sigma = 0, tau = 1) {
    this.rng = rng;
    this.sigma = sigma;
    this.tau = tau;
    this.x = 0;
  }
  step(dt) {
    if (this.sigma <= 0) {
      this.x = 0;
      return 0;
    }
    const a = Math.exp(-dt / Math.max(this.tau, 1e-6));
    this.x = a * this.x + this.sigma * Math.sqrt(1 - a * a) * this.rng.gauss();
    return this.x;
  }
}
