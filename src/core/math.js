// Core math utilities shared by physics, allocation and the ArduSub port.
// Conventions:
//   - Vectors are plain arrays [x, y, z].
//   - Quaternions are [w, x, y, z] (Hamilton), representing body -> NED rotation
//     (same meaning as ArduPilot's get_quat_body_to_ned()).
//   - Euler angles follow ArduPilot's 3-2-1 convention: roll (x), pitch (y), yaw (z).

export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;
export const GRAVITY_MSS = 9.80665; // ArduPilot GRAVITY_MSS

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const isZero = (v) => Math.abs(v) < 1.1920929e-7; // FLT_EPSILON like ArduPilot's is_zero
export const isPositive = (v) => v >= 1.1920929e-7;
export const isNegative = (v) => v <= -1.1920929e-7;
export const sq = (v) => v * v;
export const sign = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);

export function wrapPI(a) {
  // wrap to (-pi, pi]
  let r = a % (2 * Math.PI);
  if (r > Math.PI) r -= 2 * Math.PI;
  else if (r <= -Math.PI) r += 2 * Math.PI;
  return r;
}
export function wrap2PI(a) {
  let r = a % (2 * Math.PI);
  if (r < 0) r += 2 * Math.PI;
  return r;
}
export function wrap180cd(cd) {
  let r = cd % 36000;
  if (r > 18000) r -= 36000;
  if (r < -18000) r += 36000;
  return r;
}
export function wrap360cd(cd) {
  let r = cd % 36000;
  if (r < 0) r += 36000;
  return r;
}

// ---------------- vec3 ----------------
export const v3 = (x = 0, y = 0, z = 0) => [x, y, z];
export const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const vscale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const vcross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const vlen = (a) => Math.hypot(a[0], a[1], a[2]);
export function vnorm(a) {
  const l = vlen(a);
  return l > 1e-12 ? [a[0] / l, a[1] / l, a[2] / l] : [0, 0, 0];
}
export const vcopy = (a) => [a[0], a[1], a[2]];
export const vneg = (a) => [-a[0], -a[1], -a[2]];

// ---------------- mat3 (row-major arrays of 9) ----------------
export function m3mulv(m, v) {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
}
export function m3tmulv(m, v) {
  // transpose(m) * v
  return [
    m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
    m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
    m[2] * v[0] + m[5] * v[1] + m[8] * v[2],
  ];
}
export function m3mul(a, b) {
  const r = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  return r;
}
export function m3transpose(m) {
  return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
}
export function m3inv(m) {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-18) throw new Error('singular 3x3 matrix');
  const inv = 1 / det;
  return [
    A * inv, -(b * i - c * h) * inv, (b * f - c * e) * inv,
    B * inv, (a * i - c * g) * inv, -(a * f - c * d) * inv,
    C * inv, -(a * h - b * g) * inv, (a * e - b * d) * inv,
  ];
}
export const m3identity = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];

// ---------------- quaternion ----------------
export const qidentity = () => [1, 0, 0, 0];
export function qmul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}
export const qconj = (q) => [q[0], -q[1], -q[2], -q[3]];
export const qinverse = qconj; // unit quaternions only
export function qnormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-12) return [1, 0, 0, 0];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
export function qrotate(q, v) {
  // rotate vector v by quaternion q: q * v * q^-1
  const [w, x, y, z] = q;
  const ix = w * v[0] + y * v[2] - z * v[1];
  const iy = w * v[1] + z * v[0] - x * v[2];
  const iz = w * v[2] + x * v[1] - y * v[0];
  const iw = -x * v[0] - y * v[1] - z * v[2];
  return [
    ix * w + iw * -x + iy * -z - iz * -y,
    iy * w + iw * -y + iz * -x - ix * -z,
    iz * w + iw * -z + ix * -y - iy * -x,
  ];
}
export function qrotateInv(q, v) {
  return qrotate(qconj(q), v);
}
export function qToMat(q) {
  // rotation matrix (row-major) equal to rotating by q (body -> ned for attitude)
  const [w, x, y, z] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
  ];
}
export function qFromEuler(roll, pitch, yaw) {
  // ArduPilot Quaternion::from_euler (3-2-1)
  const cr2 = Math.cos(roll * 0.5), cp2 = Math.cos(pitch * 0.5), cy2 = Math.cos(yaw * 0.5);
  const sr2 = Math.sin(roll * 0.5), sp2 = Math.sin(pitch * 0.5), sy2 = Math.sin(yaw * 0.5);
  return [
    cr2 * cp2 * cy2 + sr2 * sp2 * sy2,
    sr2 * cp2 * cy2 - cr2 * sp2 * sy2,
    cr2 * sp2 * cy2 + sr2 * cp2 * sy2,
    cr2 * cp2 * sy2 - sr2 * sp2 * cy2,
  ];
}
export function qToEuler(q) {
  // ArduPilot Quaternion::to_euler -> [roll, pitch, yaw]
  const [q1, q2, q3, q4] = q;
  const roll = Math.atan2(2 * (q1 * q2 + q3 * q4), 1 - 2 * (q2 * q2 + q3 * q3));
  const pitch = Math.asin(clamp(2 * (q1 * q3 - q4 * q2), -1, 1));
  const yaw = Math.atan2(2 * (q1 * q4 + q2 * q3), 1 - 2 * (q3 * q3 + q4 * q4));
  return [roll, pitch, yaw];
}
export function qFromAxisAngleVec(v) {
  // ArduPilot Quaternion::from_axis_angle(Vector3f v) where |v| = angle
  const theta = vlen(v);
  if (theta < 1e-12) return [1, 0, 0, 0];
  return qFromAxisAngle(vscale(v, 1 / theta), theta);
}
export function qFromAxisAngle(axis, theta) {
  if (Math.abs(theta) < 1e-12) return [1, 0, 0, 0];
  const st2 = Math.sin(0.5 * theta);
  return [Math.cos(0.5 * theta), axis[0] * st2, axis[1] * st2, axis[2] * st2];
}
export function qToAxisAngle(q) {
  // ArduPilot Quaternion::to_axis_angle -> rotation vector
  let l = Math.hypot(q[1], q[2], q[3]);
  let v = [q[1], q[2], q[3]];
  if (l > 1e-12) {
    v = vscale(v, 1 / l);
    return vscale(v, wrapPI(2 * Math.atan2(l, q[0])));
  }
  return [0, 0, 0];
}

// ---------------- dense matrices (row-major, array of rows) ----------------
export function zeros(r, c) {
  return Array.from({ length: r }, () => new Array(c).fill(0));
}
export function matT(A) {
  const r = A.length, c = A[0].length;
  const T = zeros(c, r);
  for (let i = 0; i < r; i++) for (let j = 0; j < c; j++) T[j][i] = A[i][j];
  return T;
}
export function matMul(A, B) {
  const r = A.length, n = B.length, c = B[0].length;
  const C = zeros(r, c);
  for (let i = 0; i < r; i++)
    for (let k = 0; k < n; k++) {
      const a = A[i][k];
      if (a === 0) continue;
      const Bk = B[k];
      const Ci = C[i];
      for (let j = 0; j < c; j++) Ci[j] += a * Bk[j];
    }
  return C;
}
export function matVec(A, x) {
  return A.map((row) => row.reduce((s, a, j) => s + a * x[j], 0));
}

// One-sided Jacobi SVD. Returns {U (m x k), S (k), V (n x k)} with k = min(m, n),
// singular values sorted descending. Robust for the small matrices used here.
export function svd(A) {
  const m = A.length, n = A[0].length;
  const transpose = m < n;
  const M = transpose ? matT(A) : A.map((r) => r.slice());
  const rows = M.length, cols = M[0].length; // rows >= cols
  const V = zeros(cols, cols);
  for (let i = 0; i < cols; i++) V[i][i] = 1;
  for (let sweep = 0; sweep < 80; sweep++) {
    let off = 0;
    for (let p = 0; p < cols - 1; p++) {
      for (let q = p + 1; q < cols; q++) {
        let alpha = 0, beta = 0, gamma = 0;
        for (let i = 0; i < rows; i++) {
          alpha += M[i][p] * M[i][p];
          beta += M[i][q] * M[i][q];
          gamma += M[i][p] * M[i][q];
        }
        if (Math.abs(gamma) <= 1e-15 * Math.sqrt(alpha * beta) || gamma === 0) continue;
        off = Math.max(off, Math.abs(gamma) / Math.sqrt(alpha * beta));
        const zeta = (beta - alpha) / (2 * gamma);
        const t = Math.sign(zeta || 1) / (Math.abs(zeta) + Math.sqrt(1 + zeta * zeta));
        const c = 1 / Math.sqrt(1 + t * t), s = c * t;
        for (let i = 0; i < rows; i++) {
          const mp = M[i][p], mq = M[i][q];
          M[i][p] = c * mp - s * mq;
          M[i][q] = s * mp + c * mq;
        }
        for (let i = 0; i < cols; i++) {
          const vp = V[i][p], vq = V[i][q];
          V[i][p] = c * vp - s * vq;
          V[i][q] = s * vp + c * vq;
        }
      }
    }
    if (off < 1e-14) break;
  }
  const S = new Array(cols).fill(0);
  const U = zeros(rows, cols);
  for (let j = 0; j < cols; j++) {
    let nrm = 0;
    for (let i = 0; i < rows; i++) nrm += M[i][j] * M[i][j];
    nrm = Math.sqrt(nrm);
    S[j] = nrm;
    for (let i = 0; i < rows; i++) U[i][j] = nrm > 1e-300 ? M[i][j] / nrm : 0;
  }
  // sort descending
  const idx = S.map((s, i) => i).sort((a, b) => S[b] - S[a]);
  const Ss = idx.map((i) => S[i]);
  const Us = U.map((row) => idx.map((i) => row[i]));
  const Vs = V.map((row) => idx.map((i) => row[i]));
  return transpose ? { U: Vs, S: Ss, V: Us } : { U: Us, S: Ss, V: Vs };
}

// Moore-Penrose pseudo-inverse with relative tolerance.
export function pinv(A, rtol = 1e-10) {
  const { U, S, V } = svd(A);
  const tol = (S[0] || 0) * rtol;
  const m = A.length, n = A[0].length, k = S.length;
  const P = zeros(n, m);
  for (let t = 0; t < k; t++) {
    if (S[t] <= tol || S[t] === 0) continue;
    const inv = 1 / S[t];
    for (let i = 0; i < n; i++) {
      const vi = V[i][t] * inv;
      if (vi === 0) continue;
      for (let j = 0; j < m; j++) P[i][j] += vi * U[j][t];
    }
  }
  return P;
}

export function rankOf(A, rtol = 1e-9) {
  const { S } = svd(A);
  const tol = (S[0] || 0) * rtol;
  return S.filter((s) => s > tol).length;
}

// Solve small dense linear system Ax=b with partial pivoting (A square).
export function solve(A, b) {
  const n = A.length;
  const M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-14) throw new Error('singular system');
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

export function fmt(v, d = 2) {
  if (!Number.isFinite(v)) return '—';
  return v.toFixed(d);
}
