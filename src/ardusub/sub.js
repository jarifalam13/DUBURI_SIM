// ArduSub 4.5.7 "firmware" emulation: fast loop ordering, arming, modes,
// pilot input handling, surface detection and failsafes, wired to the ported
// attitude controller, position controller and 6-DOF motor mixer.

import { AttitudeControlSub, eulerRateToAngVel } from './attitude.js';
import { PosControl } from './poscontrol.js';
import { Motors6DOF, SPOOL } from './motors.js';
import { makeSubChannels, configureSubChannels } from './rc.js';
import { JoystickHandler } from './joystick.js';
import { clamp, wrap180cd, qToEuler } from '../core/math.js';

export const MODE = { STABILIZE: 0, ACRO: 1, ALT_HOLD: 2, AUTO: 3, GUIDED: 4, CIRCLE: 7, SURFACE: 9, POSHOLD: 16, MANUAL: 19, MOTOR_DETECT: 20, SURFTRAK: 21 };
export const MODE_LABEL = { 0: 'Stabilize', 1: 'Acro', 2: 'Depth Hold', 3: 'Auto', 4: 'Guided', 7: 'Circle', 9: 'Surface', 16: 'Position Hold', 19: 'Manual', 20: 'Motor Detect', 21: 'SurfTrak' };
export const SIM_MODES = [MODE.MANUAL, MODE.STABILIZE, MODE.ACRO, MODE.ALT_HOLD, MODE.SURFACE, MODE.POSHOLD];
const MODE_INFO = {
  [MODE.MANUAL]: { manualThrottle: true, gps: false },
  [MODE.STABILIZE]: { manualThrottle: true, gps: false },
  [MODE.ACRO]: { manualThrottle: true, gps: false },
  [MODE.ALT_HOLD]: { manualThrottle: false, gps: false },
  [MODE.SURFACE]: { manualThrottle: false, gps: false },
  [MODE.POSHOLD]: { manualThrottle: false, gps: true },
};
const ROLL_PITCH_INPUT_MAX = 4500;
const ACRO_YAW_P_DEFAULT = 3.375;
const ACRO_LEVEL_MAX_ANGLE = 3000;

export class ArduSubSim {
  constructor({ params, mixer, sensors }) {
    this.params = { ...params };
    this.sensors = sensors;
    this.motors = new Motors6DOF();
    this.att = new AttitudeControlSub(sensors, this.motors);
    this.pos = new PosControl(sensors, this.motors, this.att);
    this.rc = makeSubChannels(this.params);
    this.js = new JoystickHandler(this);
    this.messages = [];
    this.time = 0;
    this.armed = false;
    this.gain = 0.5;
    this.mode = MODE.MANUAL;
    this.lastPilotHeading = 0;
    this.lastPilotYawInputMs = -1e9;
    this.desiredClimbRate = 0;
    this.atSurface = false; this.atBottom = false; this.bottomCount = 0;
    this.failsafe = { pilotInput: false, lastPilotInput: 0 };
    this.attitudeTargetMsg = null;
    this.depthTargetOverride = null;
    this.motorTest = null;
    this.mixer = mixer;
    this.applyParams(this.params, true);
    this.js.init(this.params);
  }

  text(severity, msg) {
    this.messages.push({ t: this.time, severity, text: msg });
    if (this.messages.length > 200) this.messages.shift();
  }

  get loopHz() { return clamp(this.params.SCHED_LOOP_RATE || 400, 50, 1000); }

  applyParams(P, first = false) {
    this.params = { ...this.params, ...P };
    const p = this.params;
    this.att.setParams(p);
    this.pos.setParams(p);
    this.motors.configure({ factors: this.mixer.factors, variant: this.mixer.variant, params: p });
    configureSubChannels(this.rc, p);
    const dt = 1 / this.loopHz;
    this.att.dt = dt; this.pos.dt = dt;
    if (!first && this.mode !== undefined) this.text(6, 'Parameters updated');
  }
  setMixer(mixer) {
    this.mixer = mixer;
    this.motors.configure({ factors: mixer.factors, variant: mixer.variant, params: this.params });
  }

  // ---------------- arming ----------------
  preArm() {
    const e = this.att.preArmChecks();
    if (e) return e;
    if (!this.mixer.factors.length) return 'No motors configured';
    return null;
  }
  arm(source = 'gcs') {
    if (this.armed) return true;
    const err = this.preArm();
    if (err) { this.text(3, `PreArm: ${err}`); return false; }
    this.armed = true;
    this.motors.arm();
    this.initialArmedBearing = this.sensors.yawSensor;
    this.text(6, 'Armed');
    // modes re-sync their targets on the first armed loop
    this.lastPilotHeading = this.sensors.yawSensor;
    return true;
  }
  disarm(source = 'gcs') {
    if (!this.armed) return false;
    this.armed = false;
    this.motors.disarm();
    this.js.clearInputHold();
    this.motorTest = null;
    this.motors.test = null;
    this.text(6, 'Disarmed');
    return true;
  }

  // ---------------- modes ----------------
  setMode(m, reason = 'gcs') {
    if (typeof m === 'string') m = MODE[m];
    if (m === this.mode) return true;
    const info = MODE_INFO[m];
    if (!info) { this.text(4, `Flight mode not supported in the simulator: ${MODE_LABEL[m] ?? m}`); return false; }
    if (info.gps && !this.sensors.positionOk()) { this.text(4, `Mode change failed: ${MODE_LABEL[m]} requires position`); return false; }
    if (!this.sensors.depthHealthy && MODE_INFO[this.mode]?.manualThrottle && !info.manualThrottle) { this.text(4, `Mode change failed: ${MODE_LABEL[m]} need alt estimate`); return false; }
    if (!this.modeInit(m)) { this.text(4, `Flight mode change failed ${MODE_LABEL[m]}`); return false; }
    this.mode = m;
    this.text(6, `Mode ${MODE_LABEL[m]}`);
    return true;
  }
  modeInit(m) {
    const P = this.params;
    switch (m) {
      case MODE.MANUAL: case MODE.ACRO:
        this.pos.setPosTargetZCm(0);
        this.js.setNeutralControls();
        return true;
      case MODE.STABILIZE:
        this.pos.setPosTargetZCm(0);
        this.lastPilotHeading = this.sensors.yawSensor;
        return true;
      case MODE.ALT_HOLD:
      case MODE.SURFACE:
        if (!this.sensors.depthHealthy) return false;
        this.pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP, P.PILOT_ACCEL_Z);
        this.pos.setCorrectionSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP, P.PILOT_ACCEL_Z);
        this.pos.initZController();
        this.lastPilotHeading = this.sensors.yawSensor;
        return true;
      case MODE.POSHOLD:
        if (!this.sensors.positionOk()) return false;
        this.pos.setMaxSpeedAccelXY(P.WPNAV_SPEED, P.WPNAV_ACCEL);
        this.pos.setCorrectionSpeedAccelXY(P.WPNAV_SPEED, P.WPNAV_ACCEL);
        this.pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP, P.PILOT_ACCEL_Z);
        this.pos.setCorrectionSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP, P.PILOT_ACCEL_Z);
        this.pos.initXYControllerStoppingPoint();
        this.pos.initZController();
        this.att.setThrottleOut(0.5, true, P.PILOT_THR_FILT);
        this.att.relaxAttitudeControllers();
        this.pos.relaxZController(0.5);
        this.lastPilotHeading = this.sensors.yawSensor;
        return true;
    }
    return false;
  }
  pilotSpeedDn() { return this.params.PILOT_SPEED_DN === 0 ? Math.abs(this.params.PILOT_SPEED_UP) : Math.abs(this.params.PILOT_SPEED_DN); }

  // ---------------- pilot input ----------------
  handleManualControl(msg) {
    this.failsafe.lastPilotInput = this.time;
    if (this.failsafe.pilotInput) { this.failsafe.pilotInput = false; this.text(6, 'Manual control regained'); }
    this.js.transform(msg.x | 0, msg.y | 0, msg.z | 0, msg.r | 0, msg.buttons >>> 0, msg.s | 0, msg.t | 0);
  }
  setAttitudeTarget(rollDeg, pitchDeg, yawDeg) {
    this.attitudeTargetMsg = { t: this.time, roll: rollDeg, pitch: pitchDeg, yaw: yawDeg };
  }
  clearAttitudeTarget() { this.attitudeTargetMsg = null; }

  getPilotDesiredLeanAngles(rollIn, pitchIn, angleMax) {
    const amax = clamp(this.params.ANGLE_MAX, 1000, 8000);
    angleMax = clamp(angleMax, 1000, amax);
    const scaler = amax / ROLL_PITCH_INPUT_MAX;
    rollIn *= scaler; pitchIn *= scaler;
    const total = Math.hypot(pitchIn, rollIn);
    if (total > angleMax) { const r = angleMax / total; rollIn *= r; pitchIn *= r; }
    rollIn = (18000 / Math.PI) * Math.atan(Math.cos((pitchIn * Math.PI) / 18000) * Math.tan((rollIn * Math.PI) / 18000));
    return [rollIn, pitchIn];
  }
  getPilotDesiredYawRate(stick) { return stick * this.params.ACRO_YAW_P; }
  getPilotDesiredClimbRate(throttle) {
    if (this.failsafe.pilotInput) return 0;
    const mid = this.rc.throttle.getControlMid();
    const dz = clamp(this.params.THR_DZ, 0, 400);
    const top = mid + dz * this.gain, bottom = mid - dz * this.gain;
    throttle = clamp(throttle, 0, 1000);
    let rate = 0;
    if (throttle < bottom) rate = (this.pilotSpeedDn() * (throttle - bottom)) / bottom;
    else if (throttle > top) rate = (this.params.PILOT_SPEED_UP * (throttle - top)) / (1000 - top);
    this.desiredClimbRate = rate;
    return rate;
  }
  getPilotDesiredAngleRates(rollIn, pitchIn, yawIn) {
    const P = this.params;
    const total = Math.hypot(pitchIn, rollIn);
    if (total > ROLL_PITCH_INPUT_MAX) { const r = ROLL_PITCH_INPUT_MAX / total; rollIn *= r; pitchIn *= r; }
    const req = [0, 0, 0];
    if (P.ACRO_EXPO <= 0) { req[0] = rollIn * P.ACRO_RP_P; req[1] = pitchIn * P.ACRO_RP_P; }
    else {
      const expo = Math.min(P.ACRO_EXPO, 1);
      const f = (v) => { const x = v / ROLL_PITCH_INPUT_MAX; return ROLL_PITCH_INPUT_MAX * (expo * x * x * x + (1 - expo) * x) * P.ACRO_RP_P; };
      req[0] = f(rollIn); req[1] = f(pitchIn);
    }
    req[2] = yawIn * P.ACRO_YAW_P;
    if (P.ACRO_TRAINER !== 0) {
      const s = this.sensors;
      const rollA = wrap180cd(s.rollSensor), pitchA = wrap180cd(s.pitchSensor);
      const lvl = [-clamp(rollA, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * P.ACRO_BAL_ROLL, -clamp(pitchA, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * P.ACRO_BAL_PITCH, 0];
      const amax = P.ANGLE_MAX;
      if (P.ACRO_TRAINER === 2) {
        if (rollA > amax) lvl[0] -= P.ACRO_BAL_ROLL * (rollA - amax); else if (rollA < -amax) lvl[0] -= P.ACRO_BAL_ROLL * (rollA + amax);
        if (pitchA > amax) lvl[1] -= P.ACRO_BAL_PITCH * (pitchA - amax); else if (pitchA < -amax) lvl[1] -= P.ACRO_BAL_PITCH * (pitchA + amax);
      }
      const e = this.att.getAttTargetEulerCd().map((v) => (v * Math.PI) / 18000);
      let bf = eulerRateToAngVel(e, lvl);
      if (P.ACRO_TRAINER === 2) { req[0] += bf[0]; req[1] += bf[1]; req[2] += bf[2]; }
      else {
        const mix = clamp(1 - Math.max(Math.abs(rollIn), Math.abs(pitchIn), Math.abs(yawIn)) / 4500, 0, 1) * s.cosPitch;
        bf = bf.map((v) => v * mix);
        for (let k = 0; k < 3; k++) { const lim = Math.abs(Math.abs(req[k]) - Math.abs(bf[k])); req[k] = clamp(req[k] + bf[k], -lim, lim); }
      }
    }
    return req;
  }

  // shared heading-hold logic used by stabilize / depth hold / position hold
  runAttitude(targetRoll, targetPitch, targetYawRate) {
    const nowMs = this.time * 1000;
    if (targetYawRate !== 0) {
      this.att.inputEulerAngleRollPitchEulerRateYaw(targetRoll, targetPitch, targetYawRate);
      this.lastPilotHeading = this.sensors.yawSensor;
      this.lastPilotYawInputMs = nowMs;
    } else if (nowMs < this.lastPilotYawInputMs + 250) {
      this.att.inputEulerAngleRollPitchEulerRateYaw(targetRoll, targetPitch, 0);
      this.lastPilotHeading = this.sensors.yawSensor;
    } else {
      this.att.inputEulerAngleRollPitchYaw(targetRoll, targetPitch, this.lastPilotHeading, true);
    }
  }

  runMode() {
    const P = this.params, m = this.motors, rc = this.rc, att = this.att, pos = this.pos;
    const disarmedCommon = (thr) => {
      m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
      att.setThrottleOut(thr, true, P.PILOT_THR_FILT);
      att.relaxAttitudeControllers();
    };
    const yawInputDz = () => rc.yaw.pwmToAngleDzTrim(Math.trunc(rc.yaw.deadZone * this.gain), rc.yaw.trim);
    switch (this.mode) {
      case MODE.MANUAL: {
        if (!this.armed) { disarmedCommon(0); return; }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        m.setRoll(rc.roll.normInput());
        m.setPitch(rc.pitch.normInput());
        m.setYaw((rc.yaw.normInput() * P.ACRO_YAW_P) / ACRO_YAW_P_DEFAULT);
        m.setThrottle(rc.throttle.normInput());
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.STABILIZE: {
        if (!this.armed) { disarmedCommon(0); this.lastPilotHeading = this.sensors.yawSensor; return; }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, P.ANGLE_MAX);
        this.runAttitude(r, p, this.getPilotDesiredYawRate(yawInputDz()));
        att.setThrottleOut(rc.throttle.normInput(), false, P.PILOT_THR_FILT);
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.ACRO: {
        if (!this.armed) { disarmedCommon(0); return; }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        const t = this.getPilotDesiredAngleRates(rc.roll.controlIn, rc.pitch.controlIn, rc.yaw.controlIn);
        att.inputRateBfRollPitchYaw(t[0], t[1], t[2]);
        att.setThrottleOut(rc.throttle.normInput(), false, P.PILOT_THR_FILT);
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.ALT_HOLD: {
        pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP, P.PILOT_ACCEL_Z);
        if (!this.armed) {
          disarmedCommon(0.5);
          pos.relaxZController(m.throttleHover);
          this.lastPilotHeading = this.sensors.yawSensor;
          return;
        }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        const tm = this.attitudeTargetMsg;
        if (tm && this.time - tm.t < 5) {
          att.inputEulerAngleRollPitchYaw(tm.roll * 100, tm.pitch * 100, tm.yaw * 100, true);
          // (ArduSub: run_pre() returns early, control_depth()/run_post() still run)
        } else {
          const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, att.getAltholdLeanAngleMaxCd());
          this.runAttitude(r, p, this.getPilotDesiredYawRate(yawInputDz()));
        }
        this.controlDepth();
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.SURFACE: {
        if (!this.armed) {
          disarmedCommon(0);
          pos.initZController();
          return;
        }
        if (this.atSurface) { this.setMode(MODE.ALT_HOLD, 'surface complete'); return; }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, P.ANGLE_MAX);
        att.inputEulerAngleRollPitchEulerRateYaw(r, p, this.getPilotDesiredYawRate(rc.yaw.controlIn));
        const cmb = clamp(Math.abs(this.params.WPNAV_SPEED_UP ?? 250), 1, pos.getMaxSpeedUpCms());
        this.desiredClimbRate = cmb;
        pos.setPosTargetZFromClimbRateCm(cmb);
        pos.updateZController();
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.POSHOLD: {
        if (!this.armed) {
          disarmedCommon(0.5);
          pos.initXYControllerStoppingPoint();
          pos.relaxZController(0.5);
          this.lastPilotHeading = this.sensors.yawSensor;
          return;
        }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        const pl = rc.lateral.normInput(), pf = rc.forward.normInput();
        let latOut = 0, fwdOut = 0;
        if (this.sensors.positionOk()) {
          if (Math.abs(pl) > 0.1 || Math.abs(pf) > 0.1) pos.initXYControllerStoppingPoint();
          const amax = P.ANGLE_MAX;
          const lat = clamp(Math.trunc(pos.rollTarget), -amax, amax), fwd = clamp(Math.trunc(-pos.pitchTarget), -amax, amax);
          latOut = lat / amax; fwdOut = fwd / amax;
          pos.updateXYController();
        } else pos.initXYControllerStoppingPoint();
        m.setForward(fwdOut + pf);
        m.setLateral(latOut + pl);
        const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, P.ANGLE_MAX);
        this.runAttitude(r, p, this.getPilotDesiredYawRate(yawInputDz()));
        this.controlDepth();
        return;
      }
    }
  }

  controlDepth() {
    const pos = this.pos;
    if (this.depthTargetOverride !== null) {
      // simulator test hook: drive the depth target directly (cm up)
      pos.setPosTargetZCm(this.depthTargetOverride);
      pos.velDesired[2] = 0; pos.accelDesired[2] = 0;
      pos.updateZController();
      return;
    }
    let rate = this.getPilotDesiredClimbRate(this.rc.throttle.controlIn);
    rate = clamp(rate, -this.pilotSpeedDn(), this.params.PILOT_SPEED_UP);
    if (Math.abs(rate) < 0.05) {
      if (this.atSurface) pos.setPosTargetZCm(Math.min(pos.getPosTargetZCm(), this.params.SURFACE_DEPTH - 5));
      else if (this.atBottom) pos.setPosTargetZCm(Math.max(this.sensors.positionZUpCm + 10, pos.getPosTargetZCm()));
    }
    pos.setPosTargetZFromClimbRateCm(rate);
    pos.updateZController();
  }

  updateSurfaceBottom() {
    if (!this.armed) { this.atSurface = false; this.atBottom = false; return; }
    const depthAlt = -this.sensors.depthMeasured; // m, up positive
    const sd = this.params.SURFACE_DEPTH / 100;
    this.atSurface = this.atSurface ? depthAlt > sd - 0.05 : depthAlt > sd;
    const vStationary = Math.abs(this.sensors.velocityZUpCms) < 5;
    if (this.motors.limit.throttle_lower && vStationary) {
      this.bottomCount++;
      if (this.bottomCount > 1.0 * this.loopHz) this.atBottom = true;
    } else { this.bottomCount = 0; this.atBottom = false; }
  }

  checkFailsafes() {
    const P = this.params;
    if (P.FS_PILOT_INPUT === 0) { this.failsafe.pilotInput = false; return; }
    if (this.time < this.failsafe.lastPilotInput + P.FS_PILOT_TIMEOUT) { this.failsafe.pilotInput = false; return; }
    if (this.failsafe.pilotInput) return;
    this.failsafe.pilotInput = true;
    this.text(2, 'Lost manual control');
    this.js.setNeutralControls();
    if (P.FS_PILOT_INPUT === 2) this.disarm('failsafe');
  }

  // One iteration of ArduSub's fast loop. Sensors must already reflect the plant.
  fastLoop(dt) {
    this.time += dt;
    this.pos.advanceTick();
    this.att.dt = dt; this.pos.dt = dt;
    // run_rate_controller
    if (this.mode !== MODE.MANUAL && this.mode !== MODE.MOTOR_DETECT) this.att.rateControllerRun();
    // motors_output
    if (this.motorTest && this.armed) this.motors.test = this.motorTest;
    else this.motors.test = null;
    const pwm = this.motors.output(dt).slice();
    // update_flight_mode
    this.runMode();
    this.updateSurfaceBottom();
    if (!this._fsTick || this.time >= this._fsTick) { this._fsTick = this.time + 0.1; this.checkFailsafes(); }
    return pwm;
  }

  // telemetry snapshot (cheap)
  status() {
    const s = this.sensors;
    return {
      armed: this.armed, mode: this.mode, modeLabel: MODE_LABEL[this.mode], gain: this.gain,
      roll: s.roll, pitch: s.pitch, yaw: s.yaw,
      depth: s.depthMeasured, climb: s.velocityZUpCms / 100,
      targetEuler: this.att.eulerAngleTarget.slice(),
      rateTarget: this.att.angVelBody.slice(), gyro: s.gyroLatest.slice(),
      depthTarget: -this.pos.posTarget[2] / 100,
      motorsOut: this.motors.thrust.slice(), pwm: this.motors.pwm.slice(),
      saturated: this.motors.saturated, spool: this.motors.spool,
      lights: [this.js.lights1 / 10, this.js.lights2 / 10], camTilt: this.js.camTilt,
      inputHold: this.js.inputHoldEngaged, rollPitchMode: this.js.rollPitchFlag,
      atSurface: this.atSurface, atBottom: this.atBottom, failsafe: this.failsafe.pilotInput,
      throttle: this.motors.getThrottleBidirectional(),
      inputs: [this.motors.rollIn, this.motors.pitchIn, this.motors.yawIn, this.motors.getThrottleBidirectional(), this.motors.forwardIn, this.motors.lateralIn],
    };
  }
}
