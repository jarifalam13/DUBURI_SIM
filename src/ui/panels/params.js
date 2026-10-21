// Parameters panel: every ArduSub parameter the simulator emulates, editable
// live like QGroundControl's parameter editor, plus .param import/export.

import { h, fmt, toast, confirmDialog, pickFile, saveFile, checkbox } from '../dom.js';
import { PARAM_DEFS, PARAM_INDEX, defaultParams, parseParamFile, formatParamFile, validateParam } from '../../ardusub/params.js';
import { applyParamFile, allParamsForExport } from '../../vehicle/config.js';
import { buttonFunctionLabel, BUTTON_FUNCTIONS } from '../../ardusub/joystick.js';
import DUBURI_PARAMS from '../../../profiles/duburi_pixhawk_param.param';

const GROUPS = [...new Set(PARAM_DEFS.map((p) => p.group))];
const STRUCTURAL = /^(FRAME_CONFIG|SCHED_LOOP_RATE)$/;
const state = { q: '', group: '', modified: false };

export async function importParamFile(app, file) {
  const text = await file.text();
  const { params, errors } = parseParamFile(text);
  const n = Object.keys(params).length;
  if (!n) { toast(`No parameters found in ${file.name}`, 'bad'); return; }
  const dirs = Object.keys(params).filter((k) => /^MOT_\d+_DIRECTION$/.test(k)).length;
  let sync = false;
  if (dirs) {
    sync = await confirmDialog('Match thruster spin directions?',
      'This file sets MOT_n_DIRECTION. If your real vehicle already flies correctly with these values, the props of the reversed motors must push the other way on the hardware. Match the simulated thrusters to that (recommended), or keep the thruster directions you set in the simulator?',
      { ok: 'Match my vehicle', cancel: 'Keep simulator thrusters' });
  }
  const rep = applyParamFile(app.cfg, params, { syncPolarity: sync });
  app.changed({ immediate: true });
  app.refreshPanel();
  toast(`${file.name}: ${rep.applied} parameters simulated, ${rep.kept} kept for export${errors.length ? `, ${errors.length} lines skipped` : ''}${rep.polarityChanged.length ? `; thrusters ${rep.polarityChanged.join(', ')} reversed` : ''}`, 'good', 7000);
}

export default {
  id: 'params', title: 'Parameters', short: 'Params', icon: 'params', sub: 'ArduSub 4.5 parameters, live like QGroundControl',
  create(app) {
    const cfg = app.cfg;
    const defaults = defaultParams();
    const el = h('div', { class: 'section', style: { gap: '14px' } });
    const isMod = (name) => Math.abs((cfg.params[name] ?? defaults[name]) - defaults[name]) > 1e-9;
    const modCount = PARAM_DEFS.filter((p) => isMod(p.name)).length;

    // ---- toolbar ----
    const search = h('input', { type: 'text', placeholder: 'Search name or description', value: state.q, 'aria-label': 'Search parameters', oninput: () => { state.q = search.value; render(); } });
    const groupSel = h('select', { 'aria-label': 'Parameter group', onchange: () => { state.group = groupSel.value; render(); } },
      h('option', { value: '' }, 'All groups'), GROUPS.map((g) => h('option', { value: g }, g)));
    groupSel.value = state.group;
    el.append(h('div', { class: 'section' },
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: async () => { const f = await pickFile('.param,.parm,.params,.txt'); if (f) await importParamFile(app, f); } }, 'Load .param file'),
        h('button', { class: 'btn', onclick: () => saveFile(`${slug(cfg.name)}.param`, formatParamFile(allParamsForExport(cfg), { style: 'mp' }), 'text/plain') }, 'Save (Mission Planner)'),
        h('button', { class: 'btn', onclick: () => saveFile(`${slug(cfg.name)}.params`, formatParamFile(allParamsForExport(cfg), { style: 'qgc' }), 'text/plain') }, 'Save (QGC)')),
      h('div', { class: 'grid2' }, search, groupSel),
      h('div', { class: 'row', style: { justifyContent: 'space-between' } },
        checkbox(`Only changed from default (${modCount})`, () => state.modified, (v) => { state.modified = v; render(); }),
        h('span', { class: 'note' }, `${PARAM_DEFS.length} simulated · ${Object.keys(cfg.extraParams || {}).length} kept from file`))));

    const list = h('div', { class: 'table-wrap', style: { maxHeight: 'none' } });
    el.append(list);

    function editor(p) {
      const v = cfg.params[p.name] ?? p.default;
      if (p.values || p.enum === 'button') {
        const opts = p.enum === 'button' ? Object.keys(BUTTON_FUNCTIONS).map((k) => [k, `${k} · ${buttonFunctionLabel(+k)}`]) : Object.entries(p.values);
        const sel = h('select', { 'aria-label': p.name, onchange: () => commit(p, Number(sel.value), sel) }, opts.map(([k, l]) => h('option', { value: String(k) }, p.enum === 'button' ? l : `${k} · ${l}`)));
        if (!opts.some(([k]) => Number(k) === v)) sel.append(h('option', { value: String(v) }, `${v} · (other)`));
        sel.value = String(v);
        return sel;
      }
      const inp = h('input', { type: 'number', step: 'any', value: String(+(+v).toPrecision(7)), 'aria-label': p.name, style: { width: '11ch' } });
      inp.addEventListener('change', () => commit(p, parseFloat(inp.value), inp));
      return inp;
    }
    function commit(p, value, input) {
      const err = validateParam(p.name, value);
      input.classList.toggle('invalid', !!err);
      if (err) { input.title = err; toast(`${p.name}: ${err}`, 'bad'); return; }
      input.title = '';
      app.setParam(p.name, value);
      if (STRUCTURAL.test(p.name)) toast(`${p.name} changed: the vehicle model was rebuilt (a real autopilot needs a reboot).`);
      const row = input.closest('tr');
      if (row) row.classList.toggle('mod', isMod(p.name));
    }

    function render() {
      const q = state.q.trim().toLowerCase();
      const rows = PARAM_DEFS.filter((p) => (!state.group || p.group === state.group || q) && (!q || p.name.toLowerCase().includes(q) || (p.desc || '').toLowerCase().includes(q)) && (!state.modified || isMod(p.name)));
      const shown = rows.slice(0, 400);
      list.replaceChildren(h('table', { class: 'data' },
        h('thead', {}, h('tr', {}, h('th', {}, 'Name'), h('th', {}, 'Value'), h('th', { class: 'num' }, 'Default'), h('th', {}, ''))),
        h('tbody', {}, shown.map((p) => h('tr', { class: isMod(p.name) ? 'mod' : '', title: p.desc || '' },
          h('td', { class: 'mono', style: { whiteSpace: 'normal', minWidth: '15ch' } }, p.name, h('div', { class: 'note', style: { fontFamily: 'var(--font-ui)', fontSize: '0.7rem', lineHeight: 1.35 } }, p.desc || '')),
          h('td', {}, h('div', { class: 'row', style: { flexWrap: 'nowrap', gap: '4px' } }, editor(p), p.unit ? h('span', { class: 'note' }, p.unit) : null)),
          h('td', { class: 'num note' }, fmtVal(p.default)),
          h('td', {}, p.inert ? h('span', { class: 'pill muted', title: 'Accepted and exported, but this ArduSub version does not use it' }, 'not used') : p.reboot ? h('span', { class: 'pill warn', title: 'Needs a reboot on the real autopilot' }, 'reboot') : null))))));
      if (!rows.length) list.replaceChildren(h('p', { class: 'note', style: { padding: '12px' } }, 'No parameters match.'));
    }
    render();

    // ---- defaults / profile ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Reset'),
      h('div', { class: 'row' },
        h('button', { class: 'btn', onclick: async () => {
          if (!await confirmDialog('Load the DUBURI Pixhawk profile?', 'Replaces all simulated parameters with the values from duburi_pixhawk_param.param (ArduSub 4.5.7, FRAME_CONFIG 2) and matches thruster directions to its MOT_n_DIRECTION.', { ok: 'Load profile' })) return;
          cfg.params = defaultParams();
          cfg.extraParams = {};
          applyParamFile(cfg, parseParamFile(DUBURI_PARAMS).params, { syncPolarity: true });
          app.changed({ immediate: true }); app.refreshPanel(); toast('DUBURI profile loaded', 'good');
        } }, 'Load DUBURI profile'),
        h('button', { class: 'btn danger', onclick: async () => {
          if (!await confirmDialog('Reset every parameter to the ArduSub default?', 'Your changes and imported values will be lost (parameters not simulated are kept for export).', { ok: 'Reset all', danger: true })) return;
          cfg.params = defaultParams();
          app.changed({ immediate: true }); app.refreshPanel(); toast('Parameters reset to ArduSub defaults');
        } }, 'Reset to ArduSub defaults')),
      h('p', { class: 'note' }, 'Gains take effect immediately, exactly as when you write a parameter from QGroundControl while the vehicle is in the water. Rows in orange differ from the ArduSub default.')));
    return { el };
  },
};

function fmtVal(v) { return Number.isInteger(v) ? String(v) : String(+v.toPrecision(5)); }
export function slug(s) { return String(s || 'vehicle').toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'vehicle'; }
