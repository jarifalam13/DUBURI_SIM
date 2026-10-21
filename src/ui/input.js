// Pilot input: gamepad (Gamepad API, standard mapping), keyboard and touch
// sticks, turned into MAVLink MANUAL_CONTROL fields the way QGroundControl
// sends them (x fwd, y lateral, z throttle 0..1000, r yaw, buttons in SDL order).

const FLIGHT_KEYS = {
  KeyW: ['x', 1], KeyS: ['x', -1], KeyA: ['r', -1], KeyD: ['r', 1], KeyQ: ['y', -1], KeyE: ['y', 1],
  KeyR: ['z', 1], KeyF: ['z', -1], Space: ['z', 1], KeyC: ['z', -1],
  ArrowUp: ['x', 1], ArrowDown: ['x', -1], ArrowLeft: ['y', -1], ArrowRight: ['y', 1],
  KeyI: ['s', 1], KeyK: ['s', -1], KeyJ: ['t', -1], KeyL: ['t', 1],
};

export class InputManager {
  constructor(app) {
    this.app = app;
    this.keys = new Set();
    this.axesKb = { x: 0, y: 0, z: 0, r: 0, s: 0, t: 0 };
    this.touch = { lx: 0, ly: 0, rx: 0, ry: 0, active: false };
    this.gamepadIndex = null;
    this.gamepadName = null;
    this.gamepadBlocked = false;
    this.lastPadButtons = [];
    this.source = 'keyboard';
    window.addEventListener('keydown', (e) => this.onKey(e, true));
    window.addEventListener('keyup', (e) => this.onKey(e, false));
    window.addEventListener('blur', () => this.keys.clear());
    try {
      window.addEventListener('gamepadconnected', (e) => { this.gamepadIndex = e.gamepad.index; this.gamepadName = e.gamepad.id; this.app.toast?.(`Controller connected: ${shortName(e.gamepad.id)}`, 'good'); });
      window.addEventListener('gamepaddisconnected', (e) => { if (this.gamepadIndex === e.gamepad.index) { this.gamepadIndex = null; this.gamepadName = null; this.app.toast?.('Controller disconnected', 'bad'); } });
    } catch { /* ignore */ }
  }

  typing() {
    const a = document.activeElement;
    return a && (a.tagName === 'INPUT' || a.tagName === 'SELECT' || a.tagName === 'TEXTAREA' || a.isContentEditable);
  }

  onKey(e, down) {
    if (this.typing()) return;
    const code = e.code;
    if (down && !e.repeat) {
      const handled = this.app.hotkey?.(code, e);
      if (handled) { e.preventDefault(); return; }
    }
    if (code in FLIGHT_KEYS || code === 'ShiftLeft' || code === 'ShiftRight') {
      if (down) this.keys.add(code); else this.keys.delete(code);
      if (code.startsWith('Arrow') || code === 'Space') e.preventDefault();
    }
  }

  bindStick(el, side) {
    const knob = el.querySelector('i');
    let id = null;
    const set = (e) => {
      const r = el.getBoundingClientRect();
      let x = ((e.clientX - r.left) / r.width) * 2 - 1, y = ((e.clientY - r.top) / r.height) * 2 - 1;
      const l = Math.hypot(x, y);
      if (l > 1) { x /= l; y /= l; }
      knob.style.transform = `translate(${x * 40}px, ${y * 40}px)`;
      if (side === 'left') { this.touch.lx = x; this.touch.ly = y; } else { this.touch.rx = x; this.touch.ry = y; }
      this.touch.active = true;
    };
    el.addEventListener('pointerdown', (e) => { id = e.pointerId; el.setPointerCapture(id); set(e); });
    el.addEventListener('pointermove', (e) => { if (e.pointerId === id) set(e); });
    const end = (e) => {
      if (e.pointerId !== id) return;
      id = null; knob.style.transform = '';
      if (side === 'left') { this.touch.lx = 0; this.touch.ly = 0; } else { this.touch.rx = 0; this.touch.ry = 0; }
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
  }

  readGamepad() {
    let pads;
    try { pads = navigator.getGamepads ? navigator.getGamepads() : []; }
    catch { this.gamepadBlocked = true; return null; }
    let pad = null;
    if (this.gamepadIndex !== null && pads[this.gamepadIndex]) pad = pads[this.gamepadIndex];
    else for (const p of pads || []) if (p && p.connected) { pad = p; this.gamepadIndex = p.index; this.gamepadName = p.id; break; }
    return pad;
  }

  // returns MANUAL_CONTROL fields; dt for keyboard ramps
  sample(dt, cfg) {
    const jc = cfg.joystick;
    const dz = jc.deadzone ?? 0.06, expo = jc.expo ?? 0;
    const shape = (v) => {
      const a = Math.abs(v);
      if (a < dz) return 0;
      let s = (a - dz) / (1 - dz);
      s = (1 - expo) * s + expo * s * s * s;
      return Math.sign(v) * Math.min(1, s);
    };
    // keyboard: ramp towards key targets
    const target = { x: 0, y: 0, z: 0, r: 0, s: 0, t: 0 };
    for (const k of this.keys) { const m = FLIGHT_KEYS[k]; if (m) target[m[0]] += m[1]; }
    const ramp = Math.min(1, dt * 5);
    for (const k of Object.keys(target)) this.axesKb[k] += (Math.max(-1, Math.min(1, target[k])) - this.axesKb[k]) * ramp;
    const kbActive = Object.values(this.axesKb).some((v) => Math.abs(v) > 0.01);
    let x = this.axesKb.x, y = this.axesKb.y, z = this.axesKb.z, r = this.axesKb.r, s = this.axesKb.s, t = this.axesKb.t;
    let buttons = 0;
    const pad = this.readGamepad();
    this.padState = null;
    if (pad) {
      const ax = (m) => (m && pad.axes[m.axis] !== undefined ? (m.invert ? -1 : 1) * pad.axes[m.axis] : 0);
      const gx = shape(ax(jc.axes.x)), gy = shape(ax(jc.axes.y)), gz = shape(ax(jc.axes.z)), gr = shape(ax(jc.axes.r));
      const padActive = Math.abs(gx) + Math.abs(gy) + Math.abs(gz) + Math.abs(gr) > 0;
      if (padActive || !kbActive) { x = gx; y = gy; z = gz; r = gr; }
      const trig = (b) => (pad.buttons[b] ? pad.buttons[b].value : 0);
      const lt = trig(6), rt = trig(7);
      const applyTrig = (fn, v) => { if (fn === 'pitch') s += v; else if (fn === 'roll') t += v; else if (fn === 'down') z -= v; else if (fn === 'up') z += v; };
      applyTrig(jc.triggers.left, -lt);
      applyTrig(jc.triggers.right, rt);
      const map = jc.buttonMap || {};
      pad.buttons.forEach((b, i) => { if (b.pressed && map[i] !== undefined) buttons |= 1 << map[i]; });
      this.padState = { axes: Array.from(pad.axes), buttons: pad.buttons.map((b) => b.value), id: pad.id };
      this.source = padActive ? 'gamepad' : kbActive ? 'keyboard' : this.gamepadName ? 'gamepad' : 'keyboard';
    } else this.source = this.touch.active ? 'touch' : 'keyboard';
    if (this.touch.active && (this.touch.lx || this.touch.ly || this.touch.rx || this.touch.ry)) {
      r = shape(this.touch.lx); z = shape(-this.touch.ly); y = shape(this.touch.rx); x = shape(-this.touch.ry);
    }
    if (this.keys.has('ShiftLeft') || this.keys.has('ShiftRight')) buttons |= 1 << 5; // keyboard shift = Guide (shift) button
    const cl = (v) => Math.max(-1, Math.min(1, v));
    return {
      x: Math.round(cl(x) * 1000), y: Math.round(cl(y) * 1000), z: Math.round(500 + cl(z) * 500), r: Math.round(cl(r) * 1000),
      s: Math.round(cl(s) * 400), t: Math.round(cl(t) * 400), buttons: buttons >>> 0,
    };
  }
}

export function shortName(id = '') {
  const m = /^(.*?)\s*\(/.exec(id);
  return (m ? m[1] : id).slice(0, 40) || 'Gamepad';
}
export const KEY_HELP = [
  ['W / S', 'Forward / back'], ['A / D', 'Yaw left / right'], ['Q / E', 'Strafe left / right'], ['R / F (Space / C)', 'Ascend / descend'],
  ['I / K', 'Pitch input (6-DOF)'], ['J / L', 'Roll input (6-DOF)'], ['Enter', 'Arm / disarm'], ['1-6', 'Manual, Stabilize, Depth hold, Acro, Position hold, Surface'],
  ['+ / −', 'Pilot gain'], ['V', 'Next camera'], ['G', 'Graphs'], ['P', 'Pause'], ['Backspace', 'Reset vehicle'], ['Shift', 'Joystick shift button'],
];
