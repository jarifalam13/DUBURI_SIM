// Controller panel: live gamepad state, axis mapping, button functions
// (BTNn_FUNCTION / BTNn_SFUNCTION exactly as QGroundControl edits them) and
// keyboard help.

import { h, numberField, selectField, checkbox, fmt, toast } from '../dom.js';
import { BUTTON_FUNCTIONS, buttonFunctionLabel, SDL_BUTTON_NAMES } from '../../ardusub/joystick.js';
import { KEY_HELP, shortName } from '../input.js';

// browser "standard gamepad" button indices
const STD_NAMES = ['A / Cross', 'B / Circle', 'X / Square', 'Y / Triangle', 'LB', 'RB', 'LT', 'RT', 'Back / Share', 'Start / Options', 'Left stick press', 'Right stick press', 'D-pad up', 'D-pad down', 'D-pad left', 'D-pad right', 'Guide / Home'];
const AXIS_NAMES = ['Left stick X', 'Left stick Y', 'Right stick X', 'Right stick Y'];
const CONTROLS = [['x', 'Forward / back'], ['y', 'Lateral (strafe)'], ['z', 'Up / down (throttle)'], ['r', 'Yaw']];
const TRIG = [['none', 'Not used'], ['pitch', 'Pitch input'], ['roll', 'Roll input'], ['up', 'Ascend'], ['down', 'Descend']];

export default {
  id: 'joystick', title: 'Controller', short: 'Joystick', icon: 'gamepad', sub: 'Gamepad mapping and button functions',
  create(app) {
    const cfg = app.cfg;
    const J = cfg.joystick;
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const save = () => app.scheduleSave();

    // ---- status + live ----
    const status = h('div', { class: 'card' });
    const axesBox = h('div', { class: 'bars' });
    const btnBox = h('div', { class: 'row', style: { gap: '4px' } });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Controller'), status, axesBox, btnBox));

    function drawStatus() {
      const inp = app.input;
      const pad = inp.padState;
      status.replaceChildren(
        pad ? h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', {}, shortName(pad.id)), h('span', { class: 'pill good' }, 'Connected'))
          : inp.gamepadBlocked ? h('div', {}, h('b', {}, 'Controllers are blocked in this view'), h('p', { class: 'note' }, 'Download the simulator (Logs & files → Save simulator page) and open the HTML file in Chrome or Edge. The keyboard works everywhere.'))
            : h('div', {}, h('b', {}, 'No controller detected'), h('p', { class: 'note' }, 'Plug in an Xbox, PlayStation or Logitech gamepad and press any button. Browsers only reveal a controller after a button press.')));
      if (!pad) { axesBox.replaceChildren(); btnBox.replaceChildren(); return; }
      axesBox.replaceChildren(...pad.axes.map((v, i) => h('div', { class: 'bar-row' }, h('span', { class: 'note' }, AXIS_NAMES[i] ?? `Axis ${i}`),
        h('div', { class: 'bar-track' }, h('div', { class: 'bar-fill', style: { left: v < 0 ? `${50 + v * 50}%` : '50%', width: `${Math.abs(v) * 50}%`, background: 'var(--accent)' } })),
        h('span', { class: 'mono note', style: { textAlign: 'right' } }, `${i}: ${fmt(v, 2)}`))));
      btnBox.replaceChildren(...pad.buttons.map((v, i) => h('span', { class: 'kbd', title: STD_NAMES[i] ?? `Button ${i}`, style: { background: v > 0.5 ? 'var(--accent)' : '', color: v > 0.5 ? 'var(--accent-ink)' : '' } }, i)));
    }
    drawStatus();

    // ---- axes ----
    const axisOpts = [[-1, 'None'], ...[0, 1, 2, 3, 4, 5, 6, 7].map((i) => [i, `${i} · ${AXIS_NAMES[i] ?? 'Axis ' + i}`])];
    const axSec = h('div', { class: 'section' }, h('h3', {}, 'Sticks'),
      h('p', { class: 'note' }, 'Default is the QGroundControl / BlueOS layout: left stick throttle and yaw, right stick forward and lateral. Press "Detect" and move one stick fully.'));
    for (const [k, label] of CONTROLS) {
      const m = J.axes[k] || (J.axes[k] = { axis: -1, invert: false });
      const sel = selectField({ label, options: axisOpts, get: () => m.axis, set: (v) => { m.axis = +v; save(); } });
      const det = h('button', { class: 'btn', onclick: () => detectAxis(app, (ax, sign) => { m.axis = ax; m.invert = sign < 0; sel.refresh(); inv.querySelector('input').checked = m.invert; save(); toast(`${label}: axis ${ax}${m.invert ? ' (inverted)' : ''}`, 'good'); }) }, 'Detect');
      const inv = checkbox('Invert', () => m.invert, (v) => { m.invert = v; save(); });
      axSec.append(h('div', { style: { display: 'grid', gridTemplateColumns: '1fr auto auto', gap: '8px', alignItems: 'end' } }, sel, det, inv));
    }
    axSec.append(h('div', { class: 'grid3' },
      numberField({ label: 'Dead zone', step: 0.01, min: 0, max: 0.4, get: () => J.deadzone, set: (v) => { J.deadzone = v; save(); } }),
      numberField({ label: 'Expo', step: 0.05, min: 0, max: 1, get: () => J.expo, set: (v) => { J.expo = v; save(); } }),
      numberField({ label: 'Send rate', unit: 'Hz', step: 1, min: 2, max: 100, get: () => J.sendRate, set: (v) => { J.sendRate = v; save(); }, hint: 'MANUAL_CONTROL messages per second (QGC sends 25-50)' })),
      h('div', { class: 'grid2' },
        selectField({ label: 'Left trigger (LT)', options: TRIG, get: () => J.triggers.left, set: (v) => { J.triggers.left = v; save(); } }),
        selectField({ label: 'Right trigger (RT)', options: TRIG, get: () => J.triggers.right, set: (v) => { J.triggers.right = v; save(); } })),
      h('p', { class: 'note' }, 'Roll and pitch inputs use the MANUAL_CONTROL s/t extension fields, as BlueOS Cockpit sends them for 6-DOF frames.'));
    el.append(axSec);

    // ---- gain ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Pilot gain'),
      h('div', { class: 'grid2' }, ...['JS_GAIN_DEFAULT', 'JS_GAIN_MIN', 'JS_GAIN_MAX', 'JS_GAIN_STEPS', 'JS_THR_GAIN'].map((n) => numberField({ label: n, step: n === 'JS_GAIN_STEPS' ? 1 : 0.05, get: () => cfg.params[n], set: (v) => app.setParam(n, v) })))));

    // ---- buttons ----
    const fnOpts = Object.keys(BUTTON_FUNCTIONS).map((k) => [k, buttonFunctionLabel(+k)]);
    const sdlOpts = [[-1, 'Not used'], ...Array.from({ length: 16 }, (_, i) => [i, `${i} · ${SDL_BUTTON_NAMES[i] ?? 'Button ' + i}`])];
    const rows = STD_NAMES.map((name, std) => {
      const sdl = J.buttonMap[std] ?? -1;
      const mapSel = h('select', { 'aria-label': `${name} sends button`, onchange: () => { const v = +mapSel.value; if (v < 0) delete J.buttonMap[std]; else J.buttonMap[std] = v; save(); app.refreshPanel(); } }, sdlOpts.map(([v, l]) => h('option', { value: String(v) }, l)));
      mapSel.value = String(sdl);
      const fnSel = (shift) => {
        if (sdl < 0) return h('span', { class: 'note' }, '—');
        const p = `BTN${sdl}_${shift ? 'SFUNCTION' : 'FUNCTION'}`;
        const s = h('select', { 'aria-label': p, onchange: () => app.setParam(p, +s.value) }, fnOpts.map(([v, l]) => h('option', { value: v }, l)));
        s.value = String(cfg.params[p] ?? 0);
        return s;
      };
      return h('tr', {}, h('td', {}, name), h('td', {}, mapSel), h('td', {}, fnSel(false)), h('td', {}, fnSel(true)));
    });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Buttons'),
      h('p', { class: 'note' }, 'Each physical button sends a QGroundControl button number; ArduSub decides what it does with BTNn_FUNCTION, or BTNn_SFUNCTION while the shift button is held. These are the same parameters your Pixhawk stores.'),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, 'Button'), h('th', {}, 'Sends'), h('th', {}, 'Function'), h('th', {}, 'With shift'))), h('tbody', {}, rows)))));

    // ---- keyboard ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Keyboard'),
      h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('tbody', {}, KEY_HELP.map(([k, d]) => h('tr', {}, h('td', {}, h('span', { class: 'kbd' }, k)), h('td', { style: { whiteSpace: 'normal' } }, d))))))));

    return { el, tick: drawStatus };
  },
};

// Wait for one stick axis to move past 0.6 and report it.
function detectAxis(app, done) {
  const start = app.input.padState?.axes.slice();
  if (!start) { toast('Connect a controller first', 'bad'); return; }
  toast('Move the stick fully in the positive direction (forward, right, up or yaw right)…', '', 4000);
  const t0 = performance.now();
  const poll = () => {
    const a = app.input.padState?.axes;
    if (a) for (let i = 0; i < a.length; i++) if (Math.abs(a[i] - (start[i] ?? 0)) > 0.6) { done(i, a[i] - (start[i] ?? 0)); return; }
    if (performance.now() - t0 < 6000) setTimeout(poll, 50); else toast('No stick movement detected', 'bad');
  };
  poll();
}
