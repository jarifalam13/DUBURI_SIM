// ArduSub built-in frame motor-factor tables, copied from
// libraries/AP_Motors/AP_Motors6DOF.cpp (ArduSub 4.5.7), plus representative
// thruster geometries consistent with those tables (for presets).
//
// Factor order (ArduSub add_motor_raw_6dof): roll, pitch, yaw, throttle, forward, lateral.

const S = Math.SQRT1_2;

export const FACTOR_AXES = ['roll', 'pitch', 'yaw', 'throttle', 'forward', 'lateral'];

const T_BLUEROV1 = [
  [0, 0, -1.0, 0, 1.0, 0],
  [0, 0, 1.0, 0, 1.0, 0],
  [-0.5, 0.5, 0, 0.45, 0, 0],
  [0.5, 0.5, 0, 0.45, 0, 0],
  [0, -1.0, 0, 1.0, 0, 0],
  [-0.25, 0, 0, 0, 0, 1.0],
];
const T_V6_90 = [
  [1.0, 1.0, 0, 1.0, 0, 0],
  [0, 0, 1.0, 0, 1.0, 0],
  [1.0, -1.0, 0, 1.0, 0, 0],
  [0, 0, 0, 0, 0, 1.0],
  [0, 0, 0, 0, 0, 1.0],
  [-1.0, 1.0, 0, 1.0, 0, 0],
  [0, 0, -1.0, 0, 1.0, 0],
  [-1.0, -1.0, 0, 1.0, 0, 0],
];
const T_V6 = [
  [0, 0, 1.0, 0, -1.0, 1.0],
  [0, 0, -1.0, 0, -1.0, -1.0],
  [0, 0, -1.0, 0, 1.0, 1.0],
  [0, 0, 1.0, 0, 1.0, -1.0],
  [1.0, -1.0, 0, -1.0, 0, 0],
  [-1.0, -1.0, 0, -1.0, 0, 0],
  [1.0, 1.0, 0, -1.0, 0, 0],
  [-1.0, 1.0, 0, -1.0, 0, 0],
];
const T_V = [
  [0, 0, 1.0, 0, -1.0, 1.0],
  [0, 0, -1.0, 0, -1.0, -1.0],
  [0, 0, -1.0, 0, 1.0, 1.0],
  [0, 0, 1.0, 0, 1.0, -1.0],
  [1.0, 0, 0, -1.0, 0, 0],
  [-1.0, 0, 0, -1.0, 0, 0],
];
const T_SIMPLE3 = [
  [0, 0, -1.0, 0, 1.0, 0],
  [0, 0, 1.0, 0, 1.0, 0],
  [0, 0, 0, -1.0, 0, 0],
];
const T_DEFAULT = [
  [0, 0, -1.0, 0, 1.0, 0],
  [0, 0, 1.0, 0, 1.0, 0],
  [1.0, 0, 0, -1.0, 0, 0],
  [-1.0, 0, 0, -1.0, 0, 0],
  [0, 0, 0, 0, 0, 1.0],
];

// FRAME_CONFIG values (AP_Motors6DOF::sub_frame_t)
export const FRAMES = {
  0: { key: 'BLUEROV1', name: 'BlueROV1', table: T_BLUEROV1, mixer: 'generic' },
  1: { key: 'VECTORED', name: 'Vectored (BlueROV2, 6 thrusters)', table: T_V, mixer: 'vectored' },
  2: { key: 'VECTORED_6DOF', name: 'Vectored 6DOF (BlueROV2 Heavy, 8 thrusters)', table: T_V6, mixer: 'vectored6dof' },
  3: { key: 'VECTORED_6DOF_90DEG', name: 'Vectored 6DOF 90°', table: T_V6_90, mixer: 'generic' },
  4: { key: 'SIMPLEROV_3', name: 'SimpleROV-3', table: T_SIMPLE3, mixer: 'generic' },
  5: { key: 'SIMPLEROV_4', name: 'SimpleROV-4 (uses default 5-motor table)', table: T_DEFAULT, mixer: 'generic' },
  6: { key: 'SIMPLEROV_5', name: 'SimpleROV-5 (uses default 5-motor table)', table: T_DEFAULT, mixer: 'generic' },
  // In ArduSub 4.5.7 the CUSTOM case has its `break` commented out and falls
  // through to SIMPLEROV_3. We reproduce that faithfully and warn in the UI.
  7: { key: 'CUSTOM', name: 'Custom (falls through to SimpleROV-3 in firmware)', table: T_SIMPLE3, mixer: 'generic', warn: 'ArduSub 4.5 has no runtime custom frame: FRAME_CONFIG=7 behaves like SimpleROV-3.' },
};

export function frameTable(frameConfig) {
  const f = FRAMES[frameConfig] ?? FRAMES[5];
  return f.table.map((r) => r.slice());
}
// add_motor_raw_6dof testing_order: every built-in Sub frame tests motors in board order 1..N
export function frameTestOrder(frameConfig) {
  const f = FRAMES[frameConfig] ?? FRAMES[5];
  return f.table.map((_, i) => i + 1);
}

// --- Representative geometries -------------------------------------------
// Positions in the vehicle frame (FRD, metres, relative to the CG).
// `axis` is the direction the thruster pushes the vehicle for a positive mixer
// output (the frame convention). Values for the BlueROV2 family are close to
// published dimensions; others are derived from the factor tables.

export function heavyLayout() {
  const hx = 0.156, hy = 0.111, hz = 0.085; // horizontal thrusters
  const vx = 0.12, vy = 0.218, vz = -0.07; // vertical thrusters (outboard, upper)
  return [
    { name: 'T1 front-right', pos: [hx, hy, hz], axis: [-S, S, 0] },
    { name: 'T2 front-left', pos: [hx, -hy, hz], axis: [-S, -S, 0] },
    { name: 'T3 rear-right', pos: [-hx, hy, hz], axis: [S, S, 0] },
    { name: 'T4 rear-left', pos: [-hx, -hy, hz], axis: [S, -S, 0] },
    { name: 'T5 vert front-right', pos: [vx, vy, vz], axis: [0, 0, 1] },
    { name: 'T6 vert front-left', pos: [vx, -vy, vz], axis: [0, 0, 1] },
    { name: 'T7 vert rear-right', pos: [-vx, vy, vz], axis: [0, 0, 1] },
    { name: 'T8 vert rear-left', pos: [-vx, -vy, vz], axis: [0, 0, 1] },
  ];
}

export function vectoredLayout() {
  const L = heavyLayout().slice(0, 4);
  return [
    ...L,
    { name: 'T5 vert right', pos: [0, 0.11, -0.06], axis: [0, 0, 1] },
    { name: 'T6 vert left', pos: [0, -0.11, -0.06], axis: [0, 0, 1] },
  ];
}

// Heuristic geometry from a factor table (used for frames without a published layout).
export function layoutFromTable(table, X = 0.18, Y = 0.15, Z = 0.06) {
  return table.map((f, i) => {
    const [r, p, y, t, fw, lat] = f;
    let axis, pos;
    if (fw || lat) {
      const n = Math.hypot(fw, lat);
      axis = [fw / n, lat / n, 0];
      if (fw && lat) {
        // choose the corner that yields the right yaw sign
        let best = null;
        for (const sx of [1, -1]) for (const sy of [1, -1]) {
          const N = sx * X * axis[1] - sy * Y * axis[0];
          if (!best || N * Math.sign(y || 1) > best.v) best = { v: N * Math.sign(y || 1), pos: [sx * X, sy * Y, 0] };
        }
        pos = best.pos;
      } else if (fw) {
        pos = [0, y ? (-Math.sign(y) * Math.sign(fw)) * Y : 0, 0];
      } else {
        pos = [y ? Math.sign(y) * Math.sign(lat) * X : 0, 0, 0];
      }
      if (r) pos[2] = -Math.sign(r) * Math.sign(lat || 1) * Z;
    } else {
      const dz = t ? -Math.sign(t) : 1;
      axis = [0, 0, dz];
      pos = [p ? -Math.sign(p) * dz * X * Math.min(1, Math.abs(p) * 2) : 0, r ? Math.sign(r) * dz * Y * Math.min(1, Math.abs(r) * 2) : 0, -Z];
    }
    return { name: `T${i + 1}`, pos, axis };
  });
}

export function frameLayout(frameConfig) {
  if (frameConfig === 2) return heavyLayout();
  if (frameConfig === 1) return vectoredLayout();
  return layoutFromTable(frameTable(frameConfig));
}
