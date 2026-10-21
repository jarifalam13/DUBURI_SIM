// Vehicle configuration: one JSON document that fully describes a vehicle
// (geometry source, mass properties, hydrodynamics, thrusters, mixer, sensors,
// ArduSub parameters, joystick). Versioned so saved files stay loadable.

import { heavyLayout } from '../alloc/frames.js';
import { defaultParams } from '../ardusub/params.js';
import { THRUSTER_PRESETS } from '../physics/thruster.js';
import { DEFAULT_ENVIRONMENT } from '../physics/environment.js';

export const CONFIG_SCHEMA = 'duburi.vehicle';
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

export function defaultConfig() {
  return {
    schema: CONFIG_SCHEMA,
    version: CONFIG_VERSION,
    name: 'DUBURI sample ROV',
    model: { source: 'sample', units: 'mm', forward: '+X', up: '+Z', origin: 'bbox', offset: [0, 0, 0], color: '#dfe6ea', sealedHull: false, fileName: null },
    mass: { mass: 13.5, cgMode: 'cb-offset', cgOffset: [0, 0, 0.025], cg: [0, 0, 0], inertia: { mode: 'auto', Ixx: 0.3, Iyy: 0.4, Izz: 0.45, Pxy: 0, Pxz: 0, Pyz: 0 } },
    buoyancy: { mode: 'auto', volume: 0.0135, cb: [0, 0, 0] },
    hydro: {
      addedMass: { mode: 'estimate', diag: [6, 8, 15, 0.15, 0.15, 0.15], matrix: null, ref: [0, 0, 0], meta: null },
      damping: { mode: 'estimate', Cd: 1.0, linear: [4, 6, 6, 0.1, 0.1, 0.1], quadratic: [20, 30, 40, 1.5, 1.5, 1.5], DL: null, DQ: null, ref: [0, 0, 0], meta: null },
    },
    thrusters: heavyLayout().map((t) => thrusterFromPreset(t, 't200-16v')),
    mixer: { source: 'auto', saturation: 'priority', threshold: 0.02, axisEnable: { roll: true, pitch: true, yaw: true, throttle: true, forward: true, lateral: true }, custom: null },
    sensors: { gyroNoise: 0.0015, accelNoise: 0.04, attitudeNoise: 0.02, depthNoise: 0.002, depthRate: 50, positioning: true, positionNoise: 0.01, positionRate: 10 },
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
  out.params = { ...defaultParams(), ...(cfg.params || {}) };
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
  for (const [k, v] of Object.entries(parsed)) {
    if (k in known) { cfg.params[k] = v; report.applied++; }
    else { cfg.extraParams[k] = v; report.kept++; }
  }
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

export function allParamsForExport(cfg) {
  return { ...cfg.extraParams, ...cfg.params };
}
