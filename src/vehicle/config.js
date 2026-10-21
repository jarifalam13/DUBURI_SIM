// Vehicle configuration: one JSON document that fully describes a vehicle
// (geometry source, mass properties, hydrodynamics, thrusters, mixer, sensors,
// ArduSub parameters, joystick). Versioned so saved files stay loadable.

import { heavyLayout } from '../alloc/frames.js';
import { defaultParams } from '../ardusub/params.js';
import { THRUSTER_PRESETS } from '../physics/thruster.js';
import { DEFAULT_ENVIRONMENT } from '../physics/environment.js';
import { DEFAULT_SHAPE } from './shapes.js';

export const CONFIG_SCHEMA = 'duburi.vehicle';

// Pressure housing (electronics / battery enclosures) of the vehicle.
export const DEFAULT_HULL = {
  depthRating: 100, // m: rated depth of the pressure housings
  compressibility: 0, // % of the displaced volume lost per 10 bar (about 100 m); 0 = rigid
  airVolume: 0, // L of air inside the housings (0 = auto: 20% of the displaced volume)
  leakArea: 2, // mm²: effective leak hole once a housing fails
  floodOverRating: true, // housings start to leak below the rated depth
  leakSensor: true, // a leak detector is fitted (ArduSub LEAK1_PIN set)
};

// External depth/pressure sensors (absolute pressure range, resolution).
export const DEPTH_SENSORS = {
  bar02: { label: 'Bar02 (2 bar · 10 m)', maxBar: 2, resMbar: 0.016 },
  bar30: { label: 'Bar30 (30 bar · 300 m)', maxBar: 30, resMbar: 0.2 },
  custom: { label: 'Custom' },
};
export function depthSensorSpec(s = {}) {
  const p = DEPTH_SENSORS[s.depthSensor] ?? DEPTH_SENSORS.bar30;
  const custom = s.depthSensor === 'custom';
  const maxBar = custom ? Math.max(0.1, +s.depthSensorMax || 30) : p.maxBar;
  const resMbar = custom ? Math.max(0, +s.depthSensorRes || 0) : p.resMbar;
  return { maxPa: maxBar * 1e5, resPa: resMbar * 100, maxBar, resMbar, label: custom ? `Custom (${maxBar} bar)` : p.label };
}
export const CONFIG_VERSION = 1;

export function thrusterFromPreset(base, presetKey = 't200-16v') {
  const p = THRUSTER_PRESETS[presetKey] ?? THRUSTER_PRESETS['t200-16v'];
  return {
    name: base.name ?? 'Thruster',
    pos: base.pos ?? [0, 0, 0],
    axis: base.axis ?? [1, 0, 0],
    polarity: base.polarity ?? 1,
    preset: presetKey,
    curve: { ...p.curve },
    table: null,
    propDiameter: p.propDiameter,
    maxRpm: p.maxRpm,
    J0: p.J0,
    tauUp: p.tauUp,
    tauDown: p.tauDown,
    deductionMode: 'auto',
    deduction: 0,
    deductionRev: 0,
    inflowModel: true,
    enabled: true,
  };
}

// Thrust limits of a thruster definition (measured table wins over the parametric curve).
export function thrusterLimits(t) {
  if (Array.isArray(t.table) && t.table.length >= 2) {
    const T = t.table.map((r) => +r[1]);
    return { fwdMax: Math.max(1e-6, ...T), revMax: Math.max(1e-6, ...T.map((v) => -v)) };
  }
  return { fwdMax: t.curve?.fwdMax ?? 50, revMax: t.curve?.revMax ?? 40 };
}

export function defaultConfig() {
  return {
    schema: CONFIG_SCHEMA,
    version: CONFIG_VERSION,
    name: 'DUBURI sample ROV',
    model: { source: 'sample', units: 'mm', forward: '+X', up: '+Z', origin: 'bbox', offset: [0, 0, 0], color: '#dfe6ea', sealedHull: false, fileName: null, shape: { ...DEFAULT_SHAPE } },
    mass: { mass: 13.5, source: 'estimate', cgMode: 'cb-offset', cgOffset: [0, 0, 0.025], cg: [0, 0, 0], inertia: { mode: 'auto', Ixx: 0.3, Iyy: 0.4, Izz: 0.45, Pxy: 0, Pxz: 0, Pyz: 0 } },
    buoyancy: { mode: 'auto', volume: 0.0135, cb: [0, 0, 0] },
    hydro: {
      addedMass: { mode: 'estimate', diag: [6, 8, 15, 0.15, 0.15, 0.15], matrix: null, ref: [0, 0, 0], meta: null },
      damping: { mode: 'estimate', Cd: 1.0, linear: [4, 6, 6, 0.1, 0.1, 0.1], quadratic: [20, 30, 40, 1.5, 1.5, 1.5], DL: null, DQ: null, ref: [0, 0, 0], meta: null },
    },
    thrusters: heavyLayout().map((t) => thrusterFromPreset(t, 't200-16v')),
    mixer: { source: 'auto', saturation: 'priority', threshold: 0.02, axisEnable: { roll: true, pitch: true, yaw: true, throttle: true, forward: true, lateral: true }, custom: null },
    sensors: { gyroNoise: 0.0015, accelNoise: 0.04, attitudeNoise: 0.02, depthNoise: 0.002, depthRate: 50, depthSensor: 'bar30', depthSensorMax: 30, depthSensorRes: 0.2, depthSensorPos: [0, 0, 0], positioning: true, positionNoise: 0.01, positionRate: 10 },
    hull: { ...DEFAULT_HULL },
    environment: JSON.parse(JSON.stringify(DEFAULT_ENVIRONMENT)),
    start: { position: [0, 0, 1.0], yaw: 0 },
    sim: { physicsRate: 800, seed: 1 },
    params: defaultParams(),
    extraParams: {},
    joystick: {
      deadzone: 0.06,
      expo: 0,
      sendRate: 50,
      axes: { x: { axis: 3, invert: true }, y: { axis: 2, invert: false }, z: { axis: 1, invert: true }, r: { axis: 0, invert: false } },
      triggers: { left: 'none', right: 'none' },
      // browser "standard gamepad" index -> MAVLink/QGC (SDL) button number
      buttonMap: { 0: 0, 1: 1, 2: 2, 3: 3, 8: 4, 16: 5, 9: 6, 10: 7, 11: 8, 4: 9, 5: 10, 12: 11, 13: 12, 14: 13, 15: 14 },
    },
  };
}

// Merge a (possibly older / partial) config onto defaults.
export function normalizeConfig(cfg) {
  const d = defaultConfig();
  if (!cfg || typeof cfg !== 'object') return d;
  if (cfg.schema && cfg.schema !== CONFIG_SCHEMA) throw new Error('Not a DUBURI vehicle file');
  const merge = (a, b) => {
    if (Array.isArray(a) || Array.isArray(b)) return b ?? a;
    if (a && typeof a === 'object' && b && typeof b === 'object') {
      const o = { ...a };
      for (const k of Object.keys(b)) o[k] = k in a ? merge(a[k], b[k]) : b[k];
      return o;
    }
    return b === undefined ? a : b;
  };
  const out = merge(d, cfg);
  const known = defaultParams();
  const params = { ...known, ...(cfg.params || {}) };
  const extra = { ...(cfg.extraParams || {}) };
  // Parameters the simulator started to simulate after this vehicle was saved:
  // the value from the user's file (kept in extraParams) wins over the new default.
  for (const k of Object.keys(extra)) {
    if (!(k in known)) continue;
    const saved = !!cfg.params && k in cfg.params;
    if (!saved || (params[k] === known[k] && extra[k] !== known[k])) params[k] = extra[k];
    delete extra[k];
  }
  out.params = params;
  out.extraParams = extra;
  out.hull = { ...DEFAULT_HULL, ...(out.hull || {}) };
  if (!out.hull.leakSensorSource) {
    const det = leakDetectorFromParams({ ...extra, ...params });
    if (det !== null) { out.hull.leakSensor = det; out.hull.leakSensorSource = 'params'; }
  }
  out.thrusters = (cfg.thrusters || d.thrusters).map((t) => ({ ...thrusterFromPreset(t, t.preset || 't200-16v'), ...t }));
  out.version = CONFIG_VERSION;
  out.schema = CONFIG_SCHEMA;
  return out;
}

// Apply an ArduSub .param map to a config. Known parameters are simulated;
// unknown ones are retained for export.
export function applyParamFile(cfg, parsed, { syncPolarity = true } = {}) {
  const known = defaultParams();
  const report = { applied: 0, kept: 0, polarityChanged: [] };
  cfg.extraParams = cfg.extraParams || {};
  for (const [k, v] of Object.entries(parsed)) {
    if (k in known) { cfg.params[k] = v; delete cfg.extraParams[k]; report.applied++; }
    else { cfg.extraParams[k] = v; report.kept++; }
  }
  // the leak detector the file configures (LEAKn_PIN / LEAKn_TYPE)
  const det = leakDetectorFromParams(parsed);
  if (det !== null) { cfg.hull = { ...DEFAULT_HULL, ...(cfg.hull || {}), leakSensor: det, leakSensorSource: 'params' }; report.leakDetector = det; }
  if (syncPolarity) {
    // Assume the real vehicle flies correctly: a reversed MOT_n_DIRECTION means
    // that thruster's prop/ESC pushes the other way on the hardware.
    cfg.thrusters.forEach((t, i) => {
      const dir = cfg.params[`MOT_${i + 1}_DIRECTION`] ?? 1;
      const pol = dir < 0 ? -1 : 1;
      if (t.polarity !== pol) { t.polarity = pol; report.polarityChanged.push(i + 1); }
    });
  }
  return report;
}

// ArduSub 4.5 leak detectors: LEAKn_PIN >= 0 with LEAKn_TYPE 0 (digital) or 1 (analog).
// Returns true / false when the parameters say, null when they don't mention it.
export function leakDetectorFromParams(all) {
  let seen = false;
  for (let i = 1; i <= 3; i++) {
    const pin = all[`LEAK${i}_PIN`];
    if (pin === undefined) continue;
    seen = true;
    const type = all[`LEAK${i}_TYPE`];
    if (pin >= 0 && (type === undefined || type >= 0)) return true;
  }
  return seen ? false : null;
}

export function allParamsForExport(cfg) {
  return { ...cfg.extraParams, ...cfg.params };
}
