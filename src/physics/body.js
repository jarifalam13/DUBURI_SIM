// 6-DOF underwater vehicle plant (Fossen 2011 formulation, body FRD frame,
// origin at the CG):
//
//   (M_RB + M_A) nu_dot + C_RB(nu) nu + C_A(nu_r) nu_r + D(nu_r) nu_r + g(eta) = tau
//
// with full 6x6 added mass (from BEM or estimates), coupled linear + quadratic
// damping (from CFD or estimates), surface-aware buoyancy from volume
// elements, thruster forces with in-flow/ventilation effects, current,
// turbulence and waves, and penalty contacts with the pool walls and floor.

import { qToMat, qnormalize, clamp } from '../core/math.js';
import { ThrusterState } from './thruster.js';

function inv6(M) {
  const n = 6;
  const A = M.map((r, i) => [...r, ...Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    if (Math.abs(A[p][c]) < 1e-12) throw new Error('Mass matrix is singular');
    [A[c], A[p]] = [A[p], A[c]];
    const d = A[c][c];
    for (let k = 0; k < 2 * n; k++) A[c][k] /= d;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = A[r][c];
      if (f === 0) continue;
      for (let k = 0; k < 2 * n; k++) A[r][k] -= f * A[c][k];
    }
  }
  return A.map((r) => r.slice(n));
}

export class VehiclePlant {
  constructor(model, env) {
    this.env = env;
    this.t = 0;
    this.y = new Float64Array(13);
    this.k = [new Float64Array(13), new Float64Array(13), new Float64Array(13), new Float64Array(13)];
    this.tmp = new Float64Array(13);
    this.diag = {};
    this.setModel(model);
    this.reset();
  }

  setModel(m) {
    this.m = m;
    const A = m.MA;
    const M = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (_, j) => A[i][j]));
    for (let i = 0; i < 3; i++) M[i][i] += m.mass;
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) M[3 + i][3 + j] += m.Ig[i * 3 + j];
    this.M = M;
    this.Minv = inv6(M);
    this.thrusters = m.thrusters.map((d, i) => {
      const prev = this.thrusters && this.thrusters[i];
      const t = new ThrusterState(d);
      if (prev) t.T = prev.T;
      return t;
    });
    // bounding radius for the fully-submerged shortcut
    let R = 0;
    for (const e of m.buoyEls) R = Math.max(R, Math.hypot(e.p[0], e.p[1], e.p[2]) + Math.hypot(e.h[0], e.h[1], e.h[2]));
    this.boundR = R;
    this.extraForce = [0, 0, 0, 0, 0, 0]; // body-frame disturbance hook (tether, payload...)
  }

  reset(pose = {}) {
    const y = this.y;
    y.fill(0);
    const p = pose.position ?? [0, 0, 1.0];
    y[0] = p[0]; y[1] = p[1]; y[2] = p[2];
    const q = pose.quaternion ?? [1, 0, 0, 0];
    y[3] = q[0]; y[4] = q[1]; y[5] = q[2]; y[6] = q[3];
    if (pose.velocity) { y[7] = pose.velocity[0]; y[8] = pose.velocity[1]; y[9] = pose.velocity[2]; }
    if (pose.rates) { y[10] = pose.rates[0]; y[11] = pose.rates[1]; y[12] = pose.rates[2]; }
    for (const t of this.thrusters) { t.T = 0; t.Tcmd = 0; t.Teff = 0; t.pwm = 1500; }
    this.accelNed = [0, 0, 0];
    this.nuDot = [0, 0, 0, 0, 0, 0];
    this.t = 0;
  }

  get position() { return [this.y[0], this.y[1], this.y[2]]; }
  get quaternion() { return [this.y[3], this.y[4], this.y[5], this.y[6]]; }
  get velocityBody() { return [this.y[7], this.y[8], this.y[9]]; }
  get rates() { return [this.y[10], this.y[11], this.y[12]]; }

  // Generalised forces and state derivative.
  deriv(y, out, record = false) {
    const m = this.m, env = this.env;
    const q = [y[3], y[4], y[5], y[6]];
    const R = qToMat(q);
    const u = y[7], v = y[8], w = y[9], p = y[10], qq = y[11], r = y[12];
    const pz = y[2];
    const eta = env.surfaceElevation(y[0], y[1]);

    // fluid velocity in body frame
    const fl = env.fluidVelocity(y[0], y[1], pz);
    const ub = R[0] * fl[0] + R[3] * fl[1] + R[6] * fl[2];
    const vb = R[1] * fl[0] + R[4] * fl[1] + R[7] * fl[2];
    const wb = R[2] * fl[0] + R[5] * fl[1] + R[8] * fl[2];
    const ur = u - ub, vr = v - vb, wr = w - wb;

    const tau = [0, 0, 0, 0, 0, 0];
    const rho = env.rho, g = env.g;

    // --- gravity at CG (NED down) ---
    const W = m.mass * g;
    tau[0] += R[6] * W; tau[1] += R[7] * W; tau[2] += R[8] * W;

    // --- buoyancy ---
    let subFrac = 1;
    const fb = [0, 0, 0], mb = [0, 0, 0];
    const depthCG = pz + eta; // depth below the local free surface (eta positive up)
    if (depthCG > this.boundR + 1e-3) {
      const B = rho * g * m.volume;
      const fx = -R[6] * B, fy = -R[7] * B, fz = -R[8] * B;
      fb[0] = fx; fb[1] = fy; fb[2] = fz;
      const c = m.cb;
      mb[0] = c[1] * fz - c[2] * fy; mb[1] = c[2] * fx - c[0] * fz; mb[2] = c[0] * fy - c[1] * fx;
    } else {
      let vsub = 0;
      for (const e of m.buoyEls) {
        const ez = pz + R[6] * e.p[0] + R[7] * e.p[1] + R[8] * e.p[2];
        const ex = y[0] + R[0] * e.p[0] + R[1] * e.p[1] + R[2] * e.p[2];
        const ey = y[1] + R[3] * e.p[0] + R[4] * e.p[1] + R[5] * e.p[2];
        const d = ez + env.surfaceElevation(ex, ey);
        const hz = Math.abs(R[6]) * e.h[0] + Math.abs(R[7]) * e.h[1] + Math.abs(R[8]) * e.h[2];
        const frac = clamp((d + hz) / (2 * hz + 1e-9), 0, 1);
        if (frac <= 0) continue;
        const dv = e.v * frac;
        vsub += dv;
        const B = rho * g * dv;
        const fx = -R[6] * B, fy = -R[7] * B, fz = -R[8] * B;
        fb[0] += fx; fb[1] += fy; fb[2] += fz;
        mb[0] += e.p[1] * fz - e.p[2] * fy; mb[1] += e.p[2] * fx - e.p[0] * fz; mb[2] += e.p[0] * fy - e.p[1] * fx;
      }
      subFrac = m.volume > 0 ? vsub / m.volume : 0;
    }
    for (let i = 0; i < 3; i++) { tau[i] += fb[i]; tau[3 + i] += mb[i]; }

    // --- damping (scaled by submerged fraction, + surface wave-making drag) ---
    const nur = [ur, vr, wr, p, qq, r];
    const sd = Math.max(subFrac, 0.02);
    const surfBoost = 1 + (env.cfg.surfaceDrag ?? 0) * 4 * subFrac * (1 - subFrac);
    const DL = m.DL, DQ = m.DQ;
    const absnu = nur.map((x) => Math.abs(x) * x);
    const fd = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < 6; i++) {
      let s = 0;
      const dl = DL[i], dq = DQ[i];
      for (let j = 0; j < 6; j++) s += dl[j] * nur[j] + dq[j] * absnu[j];
      fd[i] = -s * sd * (i < 3 ? surfBoost : 1);
      tau[i] += fd[i];
    }

    // --- Coriolis: rigid body (origin at CG) ---
    const Ig = m.Ig;
    const Iw = [Ig[0] * p + Ig[1] * qq + Ig[2] * r, Ig[3] * p + Ig[4] * qq + Ig[5] * r, Ig[6] * p + Ig[7] * qq + Ig[8] * r];
    const ms = m.mass;
    // m * (w x v)
    tau[0] -= ms * (qq * w - r * v);
    tau[1] -= ms * (r * u - p * w);
    tau[2] -= ms * (p * v - qq * u);
    // w x (I w)
    tau[3] -= qq * Iw[2] - r * Iw[1];
    tau[4] -= r * Iw[0] - p * Iw[2];
    tau[5] -= p * Iw[1] - qq * Iw[0];

    // --- Coriolis: added mass (relative velocity), full 6x6 ---
    const A = m.MA;
    const pt = [0, 0, 0], pr = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      pt[i] = A[i][0] * ur + A[i][1] * vr + A[i][2] * wr + A[i][3] * p + A[i][4] * qq + A[i][5] * r;
      pr[i] = A[3 + i][0] * ur + A[3 + i][1] * vr + A[3 + i][2] * wr + A[3 + i][3] * p + A[3 + i][4] * qq + A[3 + i][5] * r;
    }
    // [w x pt ; vr x pt + w x pr]
    tau[0] -= qq * pt[2] - r * pt[1];
    tau[1] -= r * pt[0] - p * pt[2];
    tau[2] -= p * pt[1] - qq * pt[0];
    tau[3] -= (vr * pt[2] - wr * pt[1]) + (qq * pr[2] - r * pr[1]);
    tau[4] -= (wr * pt[0] - ur * pt[2]) + (r * pr[0] - p * pr[2]);
    tau[5] -= (ur * pt[1] - vr * pt[0]) + (p * pr[1] - qq * pr[0]);

    // --- thrusters ---
    const ft = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < this.thrusters.length; i++) {
      const th = this.thrusters[i];
      const d = th.def;
      if (th.T === 0) { if (record) th.Teff = 0; continue; }
      const a = d.axis, rp = d.pos;
      // local relative water velocity at the thruster
      const lx = ur + qq * rp[2] - r * rp[1];
      const ly = vr + r * rp[0] - p * rp[2];
      const lz = wr + p * rp[1] - qq * rp[0];
      const sgn = th.T > 0 ? 1 : -1;
      const inflow = sgn * (lx * a[0] + ly * a[1] + lz * a[2]);
      const tzw = pz + R[6] * rp[0] + R[7] * rp[1] + R[8] * rp[2];
      const txw = y[0] + R[0] * rp[0] + R[1] * rp[1] + R[2] * rp[2];
      const tyw = y[1] + R[3] * rp[0] + R[4] * rp[1] + R[5] * rp[2];
      const Te = th.effective(inflow, tzw + env.surfaceElevation(txw, tyw));
      if (record) th.Teff = Te;
      const fx = Te * a[0], fy = Te * a[1], fz = Te * a[2];
      ft[0] += fx; ft[1] += fy; ft[2] += fz;
      ft[3] += rp[1] * fz - rp[2] * fy; ft[4] += rp[2] * fx - rp[0] * fz; ft[5] += rp[0] * fy - rp[1] * fx;
    }
    for (let i = 0; i < 6; i++) tau[i] += ft[i];

    // --- contacts with pool floor and walls ---
    const fc = [0, 0, 0, 0, 0, 0];
    const pool = env.cfg.pool, ct = env.cfg.contact;
    const hx = pool.length / 2, hy = pool.width / 2, D = pool.depth;
    for (const c of m.contacts) {
      const wx = y[0] + R[0] * c[0] + R[1] * c[1] + R[2] * c[2];
      const wy = y[1] + R[3] * c[0] + R[4] * c[1] + R[5] * c[2];
      const wz = pz + R[6] * c[0] + R[7] * c[1] + R[8] * c[2];
      let nX = 0, nY = 0, nZ = 0, pen = 0;
      if (wz > D) { pen = wz - D; nZ = -1; }
      else if (wx > hx) { pen = wx - hx; nX = -1; }
      else if (wx < -hx) { pen = -hx - wx; nX = 1; }
      else if (wy > hy) { pen = wy - hy; nY = -1; }
      else if (wy < -hy) { pen = -hy - wy; nY = 1; }
      if (pen <= 0) continue;
      // contact point velocity (NED)
      const bvx = u + qq * c[2] - r * c[1], bvy = v + r * c[0] - p * c[2], bvz = w + p * c[1] - qq * c[0];
      const Vx = R[0] * bvx + R[1] * bvy + R[2] * bvz, Vy = R[3] * bvx + R[4] * bvy + R[5] * bvz, Vz = R[6] * bvx + R[7] * bvy + R[8] * bvz;
      const vn = Vx * nX + Vy * nY + Vz * nZ;
      let Fn = ct.stiffness * pen - ct.damping * vn;
      if (Fn <= 0) continue;
      let Fx = Fn * nX, Fy = Fn * nY, Fz = Fn * nZ;
      const tx = Vx - vn * nX, ty = Vy - vn * nY, tz = Vz - vn * nZ;
      const tl = Math.hypot(tx, ty, tz);
      const mu = ct.friction * Fn * (tl / Math.max(tl, 0.02));
      if (tl > 1e-9) { Fx -= (mu * tx) / tl; Fy -= (mu * ty) / tl; Fz -= (mu * tz) / tl; }
      // to body
      const bx = R[0] * Fx + R[3] * Fy + R[6] * Fz, by = R[1] * Fx + R[4] * Fy + R[7] * Fz, bz = R[2] * Fx + R[5] * Fy + R[8] * Fz;
      fc[0] += bx; fc[1] += by; fc[2] += bz;
      fc[3] += c[1] * bz - c[2] * by; fc[4] += c[2] * bx - c[0] * bz; fc[5] += c[0] * by - c[1] * bx;
    }
    for (let i = 0; i < 6; i++) tau[i] += fc[i] + this.extraForce[i];

    // --- solve M nu_dot = tau ---
    const Mi = this.Minv;
    for (let i = 0; i < 6; i++) {
      let s = 0;
      for (let j = 0; j < 6; j++) s += Mi[i][j] * tau[j];
      out[7 + i] = s;
    }
    // kinematics
    out[0] = R[0] * u + R[1] * v + R[2] * w;
    out[1] = R[3] * u + R[4] * v + R[5] * w;
    out[2] = R[6] * u + R[7] * v + R[8] * w;
    out[3] = 0.5 * (-q[1] * p - q[2] * qq - q[3] * r);
    out[4] = 0.5 * (q[0] * p + q[2] * r - q[3] * qq);
    out[5] = 0.5 * (q[0] * qq + q[3] * p - q[1] * r);
    out[6] = 0.5 * (q[0] * r + q[1] * qq - q[2] * p);

    if (record) {
      this.diag = { thrust: ft, buoyancy: [...fb, ...mb], damping: fd, contact: fc, submerged: subFrac, fluidBody: [ub, vb, wb], weight: W, buoyancyMag: Math.hypot(fb[0], fb[1], fb[2]) };
      // inertial acceleration of the CG (NED) and specific force (body)
      const vd = [out[7] + qq * w - r * v, out[8] + r * u - p * w, out[9] + p * v - qq * u];
      this.accelNed = [R[0] * vd[0] + R[1] * vd[1] + R[2] * vd[2], R[3] * vd[0] + R[4] * vd[1] + R[5] * vd[2], R[6] * vd[0] + R[7] * vd[1] + R[8] * vd[2]];
      this.nuDot = [out[7], out[8], out[9], out[10], out[11], out[12]];
    }
  }

  // Advance by dt with ESC PWM per thruster (held constant over the step).
  step(dt, pwm) {
    for (let i = 0; i < this.thrusters.length; i++) this.thrusters[i].update(pwm ? pwm[i] ?? 1500 : 1500, dt);
    const y = this.y, [k1, k2, k3, k4] = this.k, t = this.tmp;
    this.deriv(y, k1, true);
    for (let i = 0; i < 13; i++) t[i] = y[i] + 0.5 * dt * k1[i];
    this.deriv(t, k2);
    for (let i = 0; i < 13; i++) t[i] = y[i] + 0.5 * dt * k2[i];
    this.deriv(t, k3);
    for (let i = 0; i < 13; i++) t[i] = y[i] + dt * k3[i];
    this.deriv(t, k4);
    for (let i = 0; i < 13; i++) y[i] += (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    const qn = qnormalize([y[3], y[4], y[5], y[6]]);
    y[3] = qn[0]; y[4] = qn[1]; y[5] = qn[2]; y[6] = qn[3];
    for (let i = 0; i < 13; i++) if (!Number.isFinite(y[i])) throw new Error('Physics diverged (non-finite state). Check mass/inertia/damping values.');
    this.t += dt;
  }

  specificForceBody() {
    // accelerometer: R^T (a - g)
    const R = qToMat(this.quaternion);
    const g = this.env.g;
    const a = [this.accelNed[0], this.accelNed[1], this.accelNed[2] - g];
    return [R[0] * a[0] + R[3] * a[1] + R[6] * a[2], R[1] * a[0] + R[4] * a[1] + R[7] * a[2], R[2] * a[0] + R[5] * a[1] + R[8] * a[2]];
  }
}
