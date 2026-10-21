// Port of ArduSub joystick.cpp: MANUAL_CONTROL -> RC overrides, pilot gain,
// trims, input hold, roll/pitch toggle and button functions (AP_JSButton).

export const BUTTON_FUNCTIONS = {
  0: 'none', 1: 'shift', 2: 'arm_toggle', 3: 'arm', 4: 'disarm',
  5: 'mode_manual', 6: 'mode_stabilize', 7: 'mode_depth_hold', 8: 'mode_poshold', 9: 'mode_auto', 10: 'mode_circle', 11: 'mode_guided', 12: 'mode_acro', 13: 'mode_surftrak',
  21: 'mount_center', 22: 'mount_tilt_up', 23: 'mount_tilt_down', 24: 'camera_trigger', 25: 'camera_source_toggle', 26: 'mount_pan_right', 27: 'mount_pan_left',
  31: 'lights1_cycle', 32: 'lights1_brighter', 33: 'lights1_dimmer', 34: 'lights2_cycle', 35: 'lights2_brighter', 36: 'lights2_dimmer',
  41: 'gain_toggle', 42: 'gain_inc', 43: 'gain_dec', 44: 'trim_roll_inc', 45: 'trim_roll_dec', 46: 'trim_pitch_inc', 47: 'trim_pitch_dec', 48: 'input_hold_set', 49: 'roll_pitch_toggle',
  51: 'relay_1_on', 52: 'relay_1_off', 53: 'relay_1_toggle', 54: 'relay_2_on', 55: 'relay_2_off', 56: 'relay_2_toggle', 57: 'relay_3_on', 58: 'relay_3_off', 59: 'relay_3_toggle',
  61: 'servo_1_inc', 62: 'servo_1_dec', 63: 'servo_1_min', 64: 'servo_1_max', 65: 'servo_1_center',
  66: 'servo_2_inc', 67: 'servo_2_dec', 68: 'servo_2_min', 69: 'servo_2_max', 70: 'servo_2_center',
  71: 'servo_3_inc', 72: 'servo_3_dec', 73: 'servo_3_min', 74: 'servo_3_max', 75: 'servo_3_center',
  91: 'custom_1', 92: 'custom_2', 93: 'custom_3', 94: 'custom_4', 95: 'custom_5', 96: 'custom_6',
  101: 'relay_4_on', 102: 'relay_4_off', 103: 'relay_4_toggle', 104: 'relay_1_momentary', 105: 'relay_2_momentary', 106: 'relay_3_momentary', 107: 'relay_4_momentary',
  108: 'script_1', 109: 'script_2', 110: 'script_3', 111: 'script_4',
};
export const buttonFunctionLabel = (n) => (BUTTON_FUNCTIONS[n] ?? `function ${n}`).replace(/_/g, ' ');

// QGC / SDL GameController button numbering used by ArduSub's BTNn params
export const SDL_BUTTON_NAMES = ['A', 'B', 'X', 'Y', 'Back', 'Guide', 'Start', 'L-stick', 'R-stick', 'LB', 'RB', 'D-pad up', 'D-pad down', 'D-pad left', 'D-pad right'];

export class JoystickHandler {
  constructor(sub) {
    this.sub = sub;
    this.camTilt = 1500; this.camPan = 1500;
    this.lights1 = 0; this.lights2 = 0;
    this.rollTrim = 0; this.pitchTrim = 0; this.zTrim = 0; this.xTrim = 0; this.yTrim = 0;
    this.xLast = 0; this.yLast = 0; this.zLast = 500;
    this.buttonsPrev = 0;
    this.controlsResetSinceInputHold = true;
    this.inputHoldEngaged = false;
    this.rollPitchFlag = 0;
    this.lightsIncreasing = [true, true];
    this.gainToggleLow = false;
    this.relays = [false, false, false, false];
    this.servos = [1500, 1500, 1500];
    this.cameraSource = 1;
  }

  init(P) {
    const steps = Math.max(1, P.JS_GAIN_STEPS);
    const s = this.sub;
    if (steps === 1 || (P.JS_GAIN_DEFAULT < P.JS_GAIN_MAX + 0.01 && P.JS_GAIN_DEFAULT > P.JS_GAIN_MIN - 0.01)) s.gain = Math.min(Math.max(P.JS_GAIN_DEFAULT, P.JS_GAIN_MIN), P.JS_GAIN_MAX);
    else s.gain = P.JS_GAIN_MIN + (Math.trunc(steps / 2) - 1) * ((P.JS_GAIN_MAX - P.JS_GAIN_MIN) / (steps - 1));
    s.gain = Math.min(Math.max(s.gain, 0.1), 1);
  }

  buttonFunction(i, shift) {
    const P = this.sub.params;
    return shift ? P[`BTN${i}_SFUNCTION`] ?? 0 : P[`BTN${i}_FUNCTION`] ?? 0;
  }

  // MANUAL_CONTROL message: x,y,r in [-1000,1000], z in [0,1000], buttons bitmask
  transform(x, y, z, r, buttons, s = 0, t = 0) {
    const sub = this.sub, P = sub.params;
    const rpyScale = 0.4 * sub.gain;
    const throttleScale = 0.8 * sub.gain * P.JS_THR_GAIN;
    const rpyCenter = 1500;
    const throttleBase = 1500 - 500 * throttleScale;
    this.camTilt = 1500; this.camPan = 1500;
    let shift = false;
    for (let i = 0; i < 32; i++) if (buttons & (1 << i) && this.buttonFunction(i, false) === 1) shift = true;
    for (let i = 0; i < 32; i++) {
      const bit = 1 << i;
      if (buttons & bit) this.handlePress(i, shift, !!(this.buttonsPrev & bit));
      else if (this.buttonsPrev & bit) this.handleRelease(i, shift);
    }
    this.buttonsPrev = buttons >>> 0;
    if (this.rollPitchFlag === 1) {
      this.pitchTrim = Math.trunc(-x * rpyScale);
      this.rollTrim = Math.trunc(y * rpyScale);
    }
    let zTot, yTot, xTot;
    if (!this.controlsResetSinceInputHold) {
      zTot = this.zTrim + 500; yTot = this.yTrim; xTot = this.xTrim;
      this.controlsResetSinceInputHold = Math.abs(z - 500) < 50 && Math.abs(y) < 50 && Math.abs(x) < 50;
    } else {
      zTot = z + this.zTrim; yTot = y + this.yTrim; xTot = x + this.xTrim;
    }
    const c = (v) => Math.trunc(Math.min(1900, Math.max(1100, v)));
    const rc = sub.rc;
    rc.pitch.setOverride(c(s + this.pitchTrim + rpyCenter));
    rc.roll.setOverride(c(t + this.rollTrim + rpyCenter));
    rc.throttle.setOverride(c(zTot * throttleScale + throttleBase));
    rc.yaw.setOverride(c(r * rpyScale + rpyCenter));
    if (this.rollPitchFlag === 0) {
      rc.forward.setOverride(c(xTot * rpyScale + rpyCenter));
      rc.lateral.setOverride(c(yTot * rpyScale + rpyCenter));
    } else {
      rc.forward.setOverride(c(this.xTrim * rpyScale + rpyCenter));
      rc.lateral.setOverride(c(this.yTrim * rpyScale + rpyCenter));
    }
    rc.all[6].setOverride(this.camPan);
    rc.all[7].setOverride(this.camTilt);
    this.xLast = x; this.yLast = y; this.zLast = z;
  }

  handlePress(i, shift, held) {
    const sub = this.sub, P = sub.params;
    const fn = this.buttonFunction(i, shift);
    const gainSteps = () => {
      const minG = Math.min(Math.max(P.JS_GAIN_MIN, 0.1), 0.8);
      const maxG = Math.min(Math.max(P.JS_GAIN_MAX, minG), 1);
      const n = Math.min(Math.max(Math.trunc(P.JS_GAIN_STEPS), 1), 10);
      return { minG, maxG, n };
    };
    const lightStep = 1000 / Math.max(1, P.JS_LIGHTS_STEPS);
    switch (fn) {
      case 2: sub.armed ? sub.disarm('joystick') : sub.arm('joystick'); break;
      case 3: sub.arm('joystick'); break;
      case 4: sub.disarm('joystick'); break;
      case 5: sub.setMode('MANUAL', 'button'); break;
      case 6: sub.setMode('STABILIZE', 'button'); break;
      case 7: sub.setMode('ALT_HOLD', 'button'); break;
      case 8: sub.setMode('POSHOLD', 'button'); break;
      case 9: sub.setMode('AUTO', 'button'); break;
      case 10: sub.setMode('CIRCLE', 'button'); break;
      case 11: sub.setMode('GUIDED', 'button'); break;
      case 12: sub.setMode('ACRO', 'button'); break;
      case 13: sub.setMode('SURFTRAK', 'button'); break;
      case 21: sub.cameraTilt = 0; sub.cameraPan = 0; break;
      case 22: this.camTilt = 1900; break;
      case 23: this.camTilt = 1100; break;
      case 25: if (!held) { this.cameraSource = this.cameraSource === 1 ? 2 : 1; sub.text(6, `Video Toggle: Source ${this.cameraSource}`); } break;
      case 26: this.camPan = 1900; break;
      case 27: this.camPan = 1100; break;
      case 31: case 34: if (!held) {
        const k = fn === 31 ? 0 : 1;
        const key = k === 0 ? 'lights1' : 'lights2';
        this[key] = Math.min(1000, Math.max(0, this[key] + (this.lightsIncreasing[k] ? lightStep : -lightStep)));
        if (this[key] >= 1000 || this[key] <= 0) this.lightsIncreasing[k] = !this.lightsIncreasing[k];
      } break;
      case 32: if (!held) this.lights1 = Math.min(1000, this.lights1 + lightStep); break;
      case 33: if (!held) this.lights1 = Math.max(0, this.lights1 - lightStep); break;
      case 35: if (!held) this.lights2 = Math.min(1000, this.lights2 + lightStep); break;
      case 36: if (!held) this.lights2 = Math.max(0, this.lights2 - lightStep); break;
      case 41: if (!held) { this.gainToggleLow = !this.gainToggleLow; sub.gain = this.gainToggleLow ? 0.5 : 1; sub.text(6, `#Gain: ${Math.round(sub.gain * 100)}%`); } break;
      case 42: case 43: if (!held) {
        const { minG, maxG, n } = gainSteps();
        if (n === 1) sub.gain = Math.min(Math.max(P.JS_GAIN_DEFAULT, minG), maxG);
        else sub.gain = Math.min(Math.max(sub.gain + (fn === 42 ? 1 : -1) * ((maxG - minG) / (n - 1)), minG), maxG);
        sub.text(6, `#Gain is ${Math.round(sub.gain * 100)}%`);
      } break;
      case 44: this.rollTrim = Math.min(200, Math.max(-200, this.rollTrim + 10)); break;
      case 45: this.rollTrim = Math.min(200, Math.max(-200, this.rollTrim - 10)); break;
      case 46: this.pitchTrim = Math.min(200, Math.max(-200, this.pitchTrim + 10)); break;
      case 47: this.pitchTrim = Math.min(200, Math.max(-200, this.pitchTrim - 10)); break;
      case 48: if (sub.armed && !held) {
        this.zTrim = Math.abs(this.zLast - 500) > 50 ? this.zLast - 500 : 0;
        this.xTrim = Math.abs(this.xLast) > 50 ? this.xLast : 0;
        this.yTrim = Math.abs(this.yLast) > 50 ? this.yLast : 0;
        const last = this.inputHoldEngaged;
        this.inputHoldEngaged = !!(this.zTrim || this.xTrim || this.yTrim);
        if (this.inputHoldEngaged) sub.text(6, '#Input Hold Set');
        else if (last) sub.text(6, '#Input Hold Disabled');
        this.controlsResetSinceInputHold = !this.inputHoldEngaged;
      } break;
      case 49: if (!held) { this.rollPitchFlag = this.rollPitchFlag ? 0 : 1; sub.text(6, this.rollPitchFlag ? '#Attitude Control' : '#Movement Control'); } break;
      case 51: this.relays[0] = true; break;
      case 52: this.relays[0] = false; break;
      case 53: if (!held) this.relays[0] = !this.relays[0]; break;
      case 54: this.relays[1] = true; break;
      case 55: this.relays[1] = false; break;
      case 56: if (!held) this.relays[1] = !this.relays[1]; break;
      case 63: this.servos[0] = 1100; break;
      case 64: this.servos[0] = 1900; break;
      case 65: this.servos[0] = 1500; break;
      default: break;
    }
  }
  handleRelease(i, shift) {
    const fn = this.buttonFunction(i, shift);
    if (fn >= 104 && fn <= 107) this.relays[fn - 104] = false;
  }
  setNeutralControls() {
    const rc = this.sub.rc;
    for (let i = 0; i < 6; i++) rc.all[i].setOverride(1500);
    this.pitchTrim = 0; this.rollTrim = 0;
  }
  clearInputHold() { this.xTrim = 0; this.yTrim = 0; this.zTrim = 0; this.inputHoldEngaged = false; }
}
