// Water properties and closed-form hydrodynamic estimates used as defaults
// before (or instead of) the CFD/BEM analysis.

// ---- UNESCO EOS-80 equation of state of seawater -------------------------
// Fofonoff & Millard (1983), UNESCO Technical Papers in Marine Science 44.
// S practical salinity (≈ g/kg), T in °C on the IPTS-68 scale, p sea pressure
// (absolute minus one atmosphere) in bar.
export const toT68 = (t90) => t90 * 1.00024;

// one-atmosphere density [kg/m^3]
export function eos80Density0(S, T) {
  const rw = 999.842594 + T * (6.793952e-2 + T * (-9.095290e-3 + T * (1.001685e-4 + T * (-1.120083e-6 + T * 6.536332e-9))));
  const A = 8.24493e-1 + T * (-4.0899e-3 + T * (7.6438e-5 + T * (-8.2467e-7 + T * 5.3875e-9)));
  const B = -5.72466e-3 + T * (1.0227e-4 - T * 1.6546e-6);
  return rw + A * S + B * S * Math.sqrt(S) + 4.8314e-4 * S * S;
}

// secant bulk modulus K(S,T,p) [bar]
export function eos80SecantBulkModulus(S, T, p) {
  const Kw = 19652.21 + T * (148.4206 + T * (-2.327105 + T * (1.360477e-2 - T * 5.155288e-5)));
  const Aw = 3.239908 + T * (1.43713e-3 + T * (1.16092e-4 - T * 5.77905e-7));
  const Bw = 8.50935e-5 + T * (-6.12293e-6 + T * 5.2787e-8);
  const sr = Math.sqrt(S);
  const K0 = Kw + S * (54.6746 + T * (-0.603459 + T * (1.09987e-2 - T * 6.1670e-5))) + S * sr * (7.944e-2 + T * (1.6483e-2 - T * 5.3009e-4));
  const A = Aw + S * (2.2838e-3 + T * (-1.0981e-5 - T * 1.6078e-6)) + 1.91075e-4 * S * sr;
  const B = Bw + S * (-9.9348e-7 + T * (2.0816e-8 + T * 9.1697e-10));
  return K0 + p * (A + B * p);
}

// in-situ density [kg/m^3]
export function eos80Density(S, T, p) {
  return eos80Density0(S, T) / (1 - p / eos80SecantBulkModulus(S, T, p));
}

// Water density at the surface. Fresh water from Tanaka et al. (2001), the
// most accurate pure-water formula; the salt contribution from EOS-80.
export function waterDensity(tempC = 20, salinity = 0) {
  const T = tempC;
  const rhoFresh = 999.974950 * (1 - ((T - 3.983035) ** 2 * (T + 301.797)) / (522528.9 * (T + 69.34881)));
  if (!(salinity > 0)) return rhoFresh;
  const T68 = toT68(T);
  return rhoFresh + eos80Density0(salinity, T68) - eos80Density0(0, T68);
}

// Standard atmosphere: air pressure [hPa] at a site elevation [m above sea level].
export const airPressureAtAltitude = (h) => 1013.25 * Math.pow(1 - 2.25577e-5 * h, 5.25588);

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
