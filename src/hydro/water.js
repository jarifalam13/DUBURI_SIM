// Water properties and closed-form hydrodynamic estimates used as defaults
// before (or instead of) the CFD/BEM analysis.

// Fresh-water density (Tanaka et al. 2001 / Thiesen form), plus a linear
// salinity correction (~0.78 kg/m^3 per g/kg, good to ~1 kg/m^3 for 0-40 g/kg).
export function waterDensity(tempC = 20, salinity = 0) {
  const T = tempC;
  const rhoFresh = 999.974950 * (1 - ((T - 3.983035) ** 2 * (T + 301.797)) / (522528.9 * (T + 69.34881)));
  return rhoFresh + 0.78 * salinity;
}

// Dynamic viscosity of water [Pa s] (Vogel equation) with a mild salinity factor.
export function waterViscosity(tempC = 20, salinity = 0) {
  const mu = 2.414e-5 * 10 ** (247.8 / (tempC + 273.15 - 140));
  return mu * (1 + 0.0023 * salinity);
}

export function kinematicViscosity(tempC = 20, salinity = 0) {
  return waterViscosity(tempC, salinity) / waterDensity(tempC, salinity);
}

// Lamb's added-mass coefficients for an ellipsoid with semi-axes a, b, c
// (a along x, b along y, c along z), deeply submerged in unbounded fluid.
export function ellipsoidAddedMass(a, b, c, rho = 1000) {
  const integral = (s1) => {
    // integral_0^inf dl / ((s1+l) * sqrt((a2+l)(b2+l)(c2+l))), with l = t/(1-t)
    const a2 = a * a, b2 = b * b, c2 = c * c;
    const n = 4000;
    let sum = 0;
    for (let i = 0; i < n; i++) {
      const t = (i + 0.5) / n;
      const l = t / (1 - t);
      const dl = 1 / ((1 - t) * (1 - t));
      sum += dl / ((s1 + l) * Math.sqrt((a2 + l) * (b2 + l) * (c2 + l)));
    }
    return (sum / n) * a * b * c;
  };
  const al = integral(a * a), be = integral(b * b), ga = integral(c * c);
  const V = (4 / 3) * Math.PI * a * b * c;
  const m = rho * V;
  const rot = (p2, q2, ip, iq) => {
    const num = (p2 - q2) ** 2 * (iq - ip);
    const den = 2 * (p2 - q2) + (p2 + q2) * (ip - iq);
    if (Math.abs(p2 - q2) < 1e-12 * (p2 + q2) || Math.abs(den) < 1e-18) return 0;
    return (m / 5) * (num / den);
  };
  return {
    coeff: [al, be, ga],
    values: [
      (al / (2 - al)) * m,
      (be / (2 - be)) * m,
      (ga / (2 - ga)) * m,
      rot(b * b, c * c, be, ga),
      rot(c * c, a * a, ga, al),
      rot(a * a, b * b, al, be),
    ],
  };
}

// Default drag model from projected areas (bluff body, Cd ~ 1) with a small
// linear term so the model behaves sensibly at very low speed.
export function dragFromProjections(proj, rho = 1000, Cd = 1.0, linearSpeed = 0.1, linearRate = 0.3) {
  const q = (A) => 0.5 * rho * Cd * A;
  const quad = [q(proj.area[0]), q(proj.area[1]), q(proj.area[2]), q(proj.k3.roll), q(proj.k3.pitch), q(proj.k3.yaw)];
  const lin = [quad[0] * linearSpeed, quad[1] * linearSpeed, quad[2] * linearSpeed, quad[3] * linearRate, quad[4] * linearRate, quad[5] * linearRate];
  return { linear: lin, quadratic: quad };
}
