// ArduSub 4.5.7 parameters that the simulator emulates, with defaults taken
// from the firmware source (library defaults + ArduSub's defaults_table).
// Parameters present in an imported file but not listed here are kept
// verbatim so a .param file round-trips without loss.

const P = [];
const def = (name, value, o = {}) => P.push({ name, default: value, ...o });

// ---- Frame / motors ----
def('FRAME_CONFIG', 1, { group: 'Frame', desc: 'Frame configuration (motor layout and mixing table)', values: { 0: 'BlueROV1', 1: 'Vectored', 2: 'Vectored 6DOF', 3: 'Vectored 6DOF 90°', 4: 'SimpleROV-3', 5: 'SimpleROV-4', 6: 'SimpleROV-5', 7: 'Custom' }, reboot: true });
for (let i = 1; i <= 12; i++) def(`MOT_${i}_DIRECTION`, 1, { group: 'Motors', desc: `Motor ${i} normal (1) or reverse (-1)`, values: { 1: 'normal', '-1': 'reverse' } });
def('MOT_PWM_MIN', 1100, { group: 'Motors', unit: 'µs', min: 800, max: 1500, desc: 'PWM output minimum' });
def('MOT_PWM_MAX', 1900, { group: 'Motors', unit: 'µs', min: 1500, max: 2200, desc: 'PWM output maximum' });
def('MOT_FV_CPLNG_K', 1.0, { group: 'Motors', min: 0, max: 1.5, desc: 'Vectored frame only: forward/vertical to pitch decoupling factor' });
def('MOT_SPOOL_TIME', 0.5, { group: 'Motors', unit: 's', min: 0, max: 2, desc: 'Time to spool up after arming' });
def('MOT_THST_HOVER', 0.5, { group: 'Motors', min: 0.2, max: 0.8, desc: 'Hover throttle (ROVs are neutral: 0.5)' });
def('SCHED_LOOP_RATE', 400, { group: 'Motors', unit: 'Hz', min: 50, max: 800, desc: 'Main loop rate (attitude controller rate)', reboot: true });

// ---- Attitude controller ----
for (const ax of ['RLL', 'PIT', 'YAW']) def(`ATC_ANG_${ax}_P`, 6.0, { group: 'Attitude', min: 0, max: 12, desc: `${ax} angle controller P gain (rate per angle error)` });
const rateDefaults = {
  RLL: { P: 0.135, I: 0.09, D: 0.0036, IMAX: 0.444, FLT: 30 },
  PIT: { P: 0.135, I: 0.09, D: 0.0036, IMAX: 0.444, FLT: 30 },
  YAW: { P: 0.18, I: 0.018, D: 0, IMAX: 0.222, FLT: 5 },
};
for (const [ax, d] of Object.entries(rateDefaults)) {
  const g = 'Attitude rate ' + ax;
  def(`ATC_RAT_${ax}_P`, d.P, { group: g, min: 0, max: 2, desc: `${ax} rate controller P gain (motor output per rad/s error)` });
  def(`ATC_RAT_${ax}_I`, d.I, { group: g, min: 0, max: 2, desc: `${ax} rate controller I gain` });
  def(`ATC_RAT_${ax}_D`, d.D, { group: g, min: 0, max: 0.1, desc: `${ax} rate controller D gain` });
  def(`ATC_RAT_${ax}_IMAX`, d.IMAX, { group: g, min: 0, max: 1, desc: `${ax} rate controller I term maximum` });
  def(`ATC_RAT_${ax}_FLTT`, d.FLT, { group: g, unit: 'Hz', min: 0, max: 100, desc: `${ax} target filter frequency` });
  def(`ATC_RAT_${ax}_FLTE`, 0, { group: g, unit: 'Hz', min: 0, max: 100, desc: `${ax} error filter frequency (0 = off)` });
  def(`ATC_RAT_${ax}_FLTD`, d.FLT, { group: g, unit: 'Hz', min: 0, max: 100, desc: `${ax} derivative filter frequency` });
  def(`ATC_RAT_${ax}_PDMX`, 0, { group: g, min: 0, max: 1, desc: `${ax} P+D output limit (0 = off)` });
  def(`ATC_RAT_${ax}_FF`, 0, { group: g, desc: 'Feed-forward (not applied by the ArduSub 4.5 rate loop)', inert: true });
  def(`ATC_RAT_${ax}_SMAX`, 0, { group: g, desc: 'Slew-rate limiter (not simulated)', inert: true });
  def(`ATC_RAT_${ax}_D_FF`, 0, { group: g, desc: 'Derivative feed-forward (not applied by ArduSub 4.5)', inert: true });
}
def('ATC_ACCEL_R_MAX', 110000, { group: 'Attitude', unit: 'cdeg/s²', min: 0, max: 180000, desc: 'Roll acceleration limit for input shaping (0 disables)' });
def('ATC_ACCEL_P_MAX', 110000, { group: 'Attitude', unit: 'cdeg/s²', min: 0, max: 180000, desc: 'Pitch acceleration limit for input shaping (0 disables)' });
def('ATC_ACCEL_Y_MAX', 110000, { group: 'Attitude', unit: 'cdeg/s²', min: 0, max: 180000, desc: 'Yaw acceleration limit for input shaping (0 disables)' });
def('ATC_RATE_FF_ENAB', 1, { group: 'Attitude', values: { 0: 'Disabled', 1: 'Enabled' }, desc: 'Rate feed-forward / input shaping enable' });
def('ATC_INPUT_TC', 0.15, { group: 'Attitude', unit: 's', min: 0, max: 1, desc: 'Attitude control input time constant (smaller = crisper)' });
def('ATC_SLEW_YAW', 6000, { group: 'Attitude', unit: 'cdeg/s', min: 500, max: 18000, desc: 'Yaw target slew rate when holding heading' });
def('ATC_RATE_R_MAX', 0, { group: 'Attitude', unit: 'deg/s', min: 0, max: 1080, desc: 'Roll rate limit (0 = off)' });
def('ATC_RATE_P_MAX', 0, { group: 'Attitude', unit: 'deg/s', min: 0, max: 1080, desc: 'Pitch rate limit (0 = off)' });
def('ATC_RATE_Y_MAX', 180, { group: 'Attitude', unit: 'deg/s', min: 0, max: 1080, desc: 'Yaw rate limit (0 = off)' });
def('ATC_ANG_LIM_TC', 1.0, { group: 'Attitude', unit: 's', desc: 'Depth-hold lean-angle limit time constant' });
def('ATC_ANGLE_BOOST', 1, { group: 'Attitude', desc: 'Angle boost (no effect in ArduSub 6DOF mixing)', inert: true });
def('ATC_THR_MIX_MIN', 0.1, { group: 'Attitude', desc: 'Throttle mix (not used by ArduSub 6DOF mixing)', inert: true });
def('ATC_THR_MIX_MAX', 0.5, { group: 'Attitude', desc: 'Throttle mix (not used by ArduSub 6DOF mixing)', inert: true });
def('ATC_THR_MIX_MAN', 0.1, { group: 'Attitude', desc: 'Throttle mix (not used by ArduSub 6DOF mixing)', inert: true });
def('ANGLE_MAX', 4500, { group: 'Pilot', unit: 'cdeg', min: 1000, max: 8000, desc: 'Maximum lean angle commanded by the pilot' });

// ---- Position controller ----
def('PSC_POSZ_P', 3.0, { group: 'Depth control', min: 0, max: 10, desc: 'Depth position P (cm/s per cm)' });
def('PSC_VELZ_P', 8.0, { group: 'Depth control', min: 0, max: 30, desc: 'Vertical velocity P (cm/s² per cm/s)' });
def('PSC_VELZ_I', 0, { group: 'Depth control', min: 0, max: 10, desc: 'Vertical velocity I' });
def('PSC_VELZ_D', 0, { group: 'Depth control', min: 0, max: 5, desc: 'Vertical velocity D' });
def('PSC_VELZ_IMAX', 1000, { group: 'Depth control', unit: 'cm/s²', desc: 'Vertical velocity I max' });
def('PSC_VELZ_FF', 0, { group: 'Depth control', desc: 'Vertical velocity feed-forward' });
def('PSC_VELZ_FLTE', 5, { group: 'Depth control', unit: 'Hz', desc: 'Vertical velocity error filter' });
def('PSC_VELZ_FLTD', 5, { group: 'Depth control', unit: 'Hz', desc: 'Vertical velocity derivative filter' });
def('PSC_ACCZ_P', 0.5, { group: 'Depth control', min: 0, max: 3, desc: 'Vertical acceleration P (throttle per cm/s², x0.001)' });
def('PSC_ACCZ_I', 0.1, { group: 'Depth control', min: 0, max: 3, desc: 'Vertical acceleration I' });
def('PSC_ACCZ_D', 0, { group: 'Depth control', min: 0, max: 1, desc: 'Vertical acceleration D' });
def('PSC_ACCZ_IMAX', 100, { group: 'Depth control', desc: 'Vertical acceleration I max (raised to hover*1000 at run time)' });
def('PSC_ACCZ_FF', 0, { group: 'Depth control', desc: 'Vertical acceleration feed-forward' });
def('PSC_ACCZ_FLTT', 0, { group: 'Depth control', unit: 'Hz', desc: 'Vertical accel target filter' });
def('PSC_ACCZ_FLTE', 20, { group: 'Depth control', unit: 'Hz', desc: 'Vertical accel error filter' });
def('PSC_ACCZ_FLTD', 0, { group: 'Depth control', unit: 'Hz', desc: 'Vertical accel derivative filter' });
def('PSC_ACCZ_PDMX', 0, { group: 'Depth control', desc: 'Vertical accel P+D limit (0 = off)' });
def('PSC_JERK_Z', 50, { group: 'Depth control', unit: 'm/s³', desc: 'Vertical jerk limit for shaping' });
def('PSC_POSXY_P', 1.0, { group: 'Position control', desc: 'Horizontal position P' });
def('PSC_VELXY_P', 6.0, { group: 'Position control', desc: 'Horizontal velocity P' });
def('PSC_VELXY_I', 0.5, { group: 'Position control', desc: 'Horizontal velocity I' });
def('PSC_VELXY_D', 0, { group: 'Position control', desc: 'Horizontal velocity D' });
def('PSC_VELXY_IMAX', 1000, { group: 'Position control', desc: 'Horizontal velocity I max' });
def('PSC_VELXY_FLTE', 5, { group: 'Position control', unit: 'Hz', desc: 'Horizontal velocity error filter' });
def('PSC_VELXY_FLTD', 5, { group: 'Position control', unit: 'Hz', desc: 'Horizontal velocity derivative filter' });
def('PSC_JERK_XY', 5, { group: 'Position control', unit: 'm/s³', desc: 'Horizontal jerk limit' });
def('WPNAV_SPEED', 100, { group: 'Position control', unit: 'cm/s', desc: 'Default horizontal speed (position hold)' });
def('WPNAV_SPEED_UP', 250, { group: 'Position control', unit: 'cm/s', desc: 'Climb speed used by Surface mode (limited by PILOT_SPEED_UP)' });
def('WPNAV_ACCEL', 250, { group: 'Position control', unit: 'cm/s²', desc: 'Horizontal acceleration (position hold)' });

// ---- Pilot ----
def('PILOT_SPEED_UP', 100, { group: 'Pilot', unit: 'cm/s', min: 20, max: 500, desc: 'Max climb rate in depth hold' });
def('PILOT_SPEED_DN', 0, { group: 'Pilot', unit: 'cm/s', min: 0, max: 500, desc: 'Max descent rate (0 = use PILOT_SPEED_UP)' });
def('PILOT_ACCEL_Z', 100, { group: 'Pilot', unit: 'cm/s²', min: 50, max: 500, desc: 'Vertical acceleration in depth hold' });
def('PILOT_THR_FILT', 0, { group: 'Pilot', unit: 'Hz', desc: 'Pilot throttle filter (0 = off)' });
def('THR_DZ', 100, { group: 'Pilot', min: 0, max: 400, desc: 'Throttle deadzone around mid-stick in depth hold' });
def('ACRO_RP_P', 4.5, { group: 'Pilot', desc: 'Acro roll/pitch rate gain' });
def('ACRO_YAW_P', 3.375, { group: 'Pilot', desc: 'Pilot yaw rate gain (deg/s per input unit /100)' });
def('ACRO_EXPO', 0.3, { group: 'Pilot', min: 0, max: 1, desc: 'Acro roll/pitch expo' });
def('ACRO_TRAINER', 2, { group: 'Pilot', values: { 0: 'Disabled', 1: 'Leveling', 2: 'Leveling and limited' }, desc: 'Acro trainer' });
def('ACRO_BAL_ROLL', 1, { group: 'Pilot', desc: 'Acro roll balance' });
def('ACRO_BAL_PITCH', 1, { group: 'Pilot', desc: 'Acro pitch balance' });

// ---- Joystick ----
def('JS_GAIN_DEFAULT', 0.5, { group: 'Joystick', min: 0.1, max: 1, desc: 'Default pilot gain' });
def('JS_GAIN_MAX', 1.0, { group: 'Joystick', min: 0.2, max: 1, desc: 'Maximum pilot gain' });
def('JS_GAIN_MIN', 0.25, { group: 'Joystick', min: 0.1, max: 0.8, desc: 'Minimum pilot gain' });
def('JS_GAIN_STEPS', 4, { group: 'Joystick', min: 1, max: 10, desc: 'Number of gain steps' });
def('JS_THR_GAIN', 1.0, { group: 'Joystick', min: 0.5, max: 4, desc: 'Throttle gain scalar' });
def('JS_LIGHTS_STEPS', 8, { group: 'Joystick', min: 1, max: 10, desc: 'Light brightness steps' });
export const DEFAULT_BUTTONS = [
  [0, 0], [5, 0], [7, 0], [6, 0], [4, 0], [1, 0], [3, 0], [21, 0],
  [48, 0], [23, 27], [22, 26], [42, 47], [43, 46], [33, 45], [32, 44], [0, 0],
];
for (let i = 0; i < 32; i++) {
  const d = DEFAULT_BUTTONS[i] || [0, 0];
  def(`BTN${i}_FUNCTION`, d[0], { group: 'Buttons', desc: `Button ${i} function`, enum: 'button' });
  def(`BTN${i}_SFUNCTION`, d[1], { group: 'Buttons', desc: `Button ${i} function with shift`, enum: 'button' });
}

// ---- RC input ----
const dz = [30, 30, 30, 40, 30, 30];
for (let i = 1; i <= 8; i++) {
  const g = 'RC input';
  def(`RC${i}_MIN`, 1100, { group: g, unit: 'µs' });
  def(`RC${i}_MAX`, 1900, { group: g, unit: 'µs' });
  def(`RC${i}_TRIM`, i === 3 ? 1100 : 1500, { group: g, unit: 'µs' });
  def(`RC${i}_DZ`, i <= 6 ? dz[i - 1] : 0, { group: g, unit: 'µs' });
  def(`RC${i}_REVERSED`, 0, { group: g, values: { 0: 'Normal', 1: 'Reversed' } });
}

// ---- Failsafes / misc ----
def('FS_PILOT_INPUT', 2, { group: 'Failsafe', values: { 0: 'Disabled', 1: 'Warn only', 2: 'Disarm' }, desc: 'Action when pilot input is lost' });
def('FS_PILOT_TIMEOUT', 3.0, { group: 'Failsafe', unit: 's', desc: 'Pilot input timeout' });
def('FS_LEAK_ENABLE', 1, { group: 'Failsafe', values: { 0: 'Disabled', 1: 'Warn only', 2: 'Enter surface mode' }, desc: 'Action when the leak detector senses water' });
def('FS_PRESS_ENABLE', 0, { group: 'Failsafe', values: { 0: 'Disabled', 1: 'Warn only' }, desc: 'Action when the internal (enclosure) pressure exceeds FS_PRESS_MAX' });
def('FS_PRESS_MAX', 105000, { group: 'Failsafe', unit: 'Pa', min: 0, max: 200000, desc: 'Internal pressure failsafe threshold' });
def('SURFACE_DEPTH', -10, { group: 'Depth', unit: 'cm', desc: 'Depth reading at which the vehicle counts as surfaced' });
def('BARO_SPEC_GRAV', 1.0, { group: 'Depth', min: 0.98, max: 1.05, desc: 'Specific gravity used to convert pressure to depth (1.0 fresh, 1.024 sea)' });
def('INS_GYRO_FILTER', 20, { group: 'Sensors', unit: 'Hz', desc: 'Gyro low-pass filter' });
def('INS_ACCEL_FILTER', 20, { group: 'Sensors', unit: 'Hz', desc: 'Accelerometer low-pass filter' });
def('AHRS_TRIM_X', 0, { group: 'Sensors', unit: 'rad', desc: 'Roll trim of the autopilot board' });
def('AHRS_TRIM_Y', 0, { group: 'Sensors', unit: 'rad', desc: 'Pitch trim of the autopilot board' });
def('EK3_HGT_DELAY', 60, { group: 'Sensors', unit: 'ms', desc: 'Depth sensor delay relative to IMU' });

export const PARAM_DEFS = P;
export const PARAM_INDEX = Object.fromEntries(P.map((p) => [p.name, p]));

export function defaultParams() {
  return Object.fromEntries(P.map((p) => [p.name, p.default]));
}

// Parse Mission Planner (NAME,VALUE) or QGroundControl (tab separated with ids) files.
export function parseParamFile(text) {
  const out = {};
  const errors = [];
  text.split(/\r?\n/).forEach((line, n) => {
    const s = line.trim();
    if (!s || s.startsWith('#')) return;
    let name, value;
    const qgc = s.split('\t');
    if (qgc.length >= 4 && /^\d+$/.test(qgc[0]) && /^\d+$/.test(qgc[1])) { name = qgc[2]; value = qgc[3]; }
    else {
      const m = /^([A-Za-z0-9_]+)[\s,]+([-+0-9.eE]+)/.exec(s);
      if (!m) { errors.push(`line ${n + 1}: ${s.slice(0, 40)}`); return; }
      name = m[1]; value = m[2];
    }
    const v = Number(value);
    if (!Number.isFinite(v)) { errors.push(`line ${n + 1}: bad value for ${name}`); return; }
    out[name.toUpperCase()] = v;
  });
  return { params: out, errors };
}

export function formatParamFile(params, { style = 'mp', header = true } = {}) {
  const names = Object.keys(params).sort();
  const lines = [];
  if (header) lines.push(`# DUBURI simulator parameter export ${new Date().toISOString()}`);
  for (const n of names) {
    const v = params[n];
    const s = Number.isInteger(v) ? String(v) : String(+v.toPrecision(9));
    lines.push(style === 'qgc' ? `1\t1\t${n}\t${s}\t9` : `${n},${s}`);
  }
  return lines.join('\n') + '\n';
}

export function validateParam(name, value) {
  const d = PARAM_INDEX[name];
  if (!Number.isFinite(value)) return 'Not a number';
  if (!d) return null;
  if (d.min !== undefined && value < d.min) return `Below minimum ${d.min}`;
  if (d.max !== undefined && value > d.max) return `Above maximum ${d.max}`;
  return null;
}
