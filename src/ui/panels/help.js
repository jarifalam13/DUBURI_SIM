// Help panel: quick start, controls, conventions, what is simulated and the
// honest limits of the simulation.

import { h } from '../dom.js';
import { KEY_HELP } from '../input.js';
import { buttonFunctionLabel, SDL_BUTTON_NAMES } from '../../ardusub/joystick.js';

const STEPS = [
  ['Upload your design', 'Click "Upload design" (or drop the file anywhere): STL, OBJ or PLY from your CAD tool. Pick its units and which axis points forward and up. The hull is checked and voxelised; volume, centre of buoyancy and inertia are computed.'],
  ['Enter what you know', 'Vehicle: the real mass and centre of gravity (CAD), inertia if you have it. Thrusters: place each thruster ("Place on model" and click the hull), its direction and prop polarity, and paste a measured PWM-to-thrust table if you have one.'],
  ['Set the water', 'Pool & water: your pool or tank (any length, width and depth, flat or with a shallow end and slope), fresh or sea water, and the depth rating of your housings. Pressure, buoyancy and the depth sensor reading all follow from it.'],
  ['See what the layout can do', 'Frame & mixer shows which of the six axes your thrusters can control and how strongly. The automatic mixer always gives the most degrees of freedom the layout allows; you can also fly the exact ArduSub FRAME_CONFIG table.'],
  ['AUTO-TUNE', 'One click: added mass from the panel method and drag/damping from the CFD lab, each on several resolutions with an uncertainty, then roll, pitch, yaw and depth-hold gains tuned on that model. Every value is shown, editable, and only applied when you say so.'],
  ['Fly', 'Press Enter to arm, pick a mode (keys 1 to 6) and fly with the keyboard or a gamepad. G shows target vs actual graphs; Tuning runs step tests.'],
  ['Take it to the vehicle', 'Save .param (Logs & files or Parameters) and load it on the Pixhawk with QGroundControl or Mission Planner. Save the vehicle file to keep each design and compare them.'],
];

const SIMULATED = [
  'ArduSub 4.5.7 control code paths re-implemented line by line in JavaScript: quaternion attitude controller with input shaping, rate PIDs, depth (z) position controller, position hold, pilot input mapping (joystick.cpp), button functions, all six thruster mixers and saturation handling, spool-up, arming checks, surface detection and the pilot-input failsafe.',
  'Rigid-body 6-DOF dynamics (Fossen form) with the full 6 × 6 added-mass matrix and its Coriolis/Munk moments, coupled linear + quadratic damping, buoyancy from your hull voxels including partial submersion at the surface, and contact with the pool floor and walls.',
  'Added mass: constant-source panel method (BEM) on a smooth hull mesh, three resolutions with Richardson extrapolation; checked against Capytaine on the same meshes (within about 1%) and the exact ellipsoid solution (within 2%).',
  'Drag and damping: lattice-Boltzmann CFD (D3Q19, regularised collision, Smagorinsky LES, Bouzidi interpolated walls, momentum exchange) on the GPU, grid convergence with GCI uncertainty and Reynolds extrapolation; the sphere drag curve is reproduced within a few percent.',
  'Heavy numerics: GPU shaders for the CFD, C kernels compiled to WebAssembly (SIMD) for the panel method and the CPU fallback of the CFD.',
  'Thrusters: PWM-to-thrust curves (Blue Robotics T200/T500 or your own measured table), deadband, asymmetric forward/reverse thrust, motor lag, thrust loss with in-flow speed, ventilation near the surface and thrust loss when the jet hits the hull.',
  'Water: any pool, tank, lake or sea size and depth with a flat or sloped floor; density and viscosity from temperature and salinity; current with turbulence; linear surface waves.',
  'Pressure: absolute pressure from air pressure plus the water column, with compressible water (UNESCO EOS-80, checked against its published check values); buoyancy changes with depth through water density and hull compression; pressure housings leak past their rated depth (orifice flow against the compressed air inside), which trips the leak detector and ArduSub\'s FS_LEAK / FS_PRESS failsafes.',
  'Sensors: gyro and accelerometer noise and filtering, attitude estimate error; Bar02 / Bar30 / custom depth sensor reading absolute pressure at its mounting point with range limit, resolution, noise, rate and delay, converted exactly as AP_Baro does (9800 Pa/m × BARO_SPEC_GRAV); optional DVL/USBL position.',
];

const LIMITS = [
  'This is not the ArduSub firmware binary (no SITL). The control logic is ported faithfully, but the EKF is replaced by a simple estimator with realistic noise, and there is no compass, GPS or MAVLink link.',
  'The tether is not modelled. A real tether adds drag and pulls the vehicle, especially in sway and at depth.',
  'The CFD lab resolves the flow at a few thousand Reynolds number and extrapolates to your vehicle; this is reliable for open frames and boxy hulls (separation at sharp edges), less so for smooth streamlined hulls. Each result carries its grid uncertainty. The panel method assumes ideal (potential) flow; parts thinner than a voxel are thickened slightly.',
  'Thruster wake interaction between thrusters is not modelled. Water temperature is the same at every depth, and thrusters and electronics do not change with pressure.',
  'Results are as good as the mass, centre of gravity and thruster data you enter. Measure them on the real vehicle where you can.',
];

export default {
  id: 'help', title: 'Help', short: 'Help', icon: 'help', sub: 'Quick start, controls and fidelity',
  create(app) {
    const P = app.cfg.params;
    const map = app.cfg.joystick.buttonMap || {};
    const std = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'LT', 'RT', 'Back', 'Start', 'L-stick', 'R-stick', 'D-pad up', 'D-pad down', 'D-pad left', 'D-pad right', 'Guide'];
    const padRows = Object.entries(map).map(([s, q]) => {
      const f = P[`BTN${q}_FUNCTION`] ?? 0, sf = P[`BTN${q}_SFUNCTION`] ?? 0;
      return h('tr', {}, h('td', {}, h('span', { class: 'kbd' }, std[+s] ?? `#${s}`)), h('td', {}, buttonFunctionLabel(f)), h('td', { class: 'note' }, sf ? buttonFunctionLabel(sf) : ''));
    });
    return {
      el: h('div', { class: 'section', style: { gap: '20px' } },
        h('div', { class: 'section' }, h('h3', {}, 'Quick start'),
          h('ol', { style: { margin: 0, paddingLeft: '20px', display: 'flex', flexDirection: 'column', gap: '8px' } },
            STEPS.map(([t, d]) => h('li', { class: 'note' }, h('b', { style: { color: 'var(--ink)' } }, `${t}. `), d)))),
        h('div', { class: 'section' }, h('h3', {}, 'Keyboard'),
          h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('tbody', {}, KEY_HELP.map(([k, d]) => h('tr', {}, h('td', {}, h('span', { class: 'kbd' }, k)), h('td', { style: { whiteSpace: 'normal' } }, d))))))),
        h('div', { class: 'section' }, h('h3', {}, 'Gamepad (your BTN parameters)'),
          h('p', { class: 'note' }, 'Left stick: up/down and yaw. Right stick: forward and lateral. Buttons follow BTNn_FUNCTION; hold the shift button for the second column.'),
          h('div', { class: 'table-wrap' }, h('table', { class: 'data' }, h('thead', {}, h('tr', {}, h('th', {}, 'Button'), h('th', {}, 'Function'), h('th', {}, 'With shift'))), h('tbody', {}, padRows))),
          h('p', { class: 'note' }, 'Browsers hide controllers inside some embedded views. If yours is not detected, save the simulator page (Logs & files) and open the file in Chrome or Edge.')),
        h('div', { class: 'section' }, h('h3', {}, 'Conventions'),
          h('p', { class: 'note' }, 'Body axes follow ArduPilot: X forward, Y right, Z down. Positive roll is right side down, positive pitch is nose up, positive yaw is nose right. Thruster positions are in metres from the model origin; the world frame is North-East-Down with the pool surface at depth 0.')),
        h('div', { class: 'section' }, h('h3', {}, 'What is simulated'), h('ul', { style: { margin: 0, paddingLeft: '20px', display: 'flex', flexDirection: 'column', gap: '6px' } }, SIMULATED.map((s) => h('li', { class: 'note' }, s)))),
        h('div', { class: 'section' }, h('h3', {}, 'Limits to keep in mind'), h('ul', { style: { margin: 0, paddingLeft: '20px', display: 'flex', flexDirection: 'column', gap: '6px' } }, LIMITS.map((s) => h('li', { class: 'note' }, s)))),
        h('div', { class: 'section' }, h('h3', {}, 'About'),
          h('p', { class: 'note' }, 'DUBURI Simulator. Control logic ported from ArduPilot ArduSub 4.5.7 (GPL-3.0); this simulator is therefore GPL-3.0 as well. 3D rendering by three.js (MIT). Source: github.com/jarifalamclaude-max/simulator'),
          h('p', { class: 'note' }, `QGroundControl button numbers: ${SDL_BUTTON_NAMES.map((n, i) => `${i} ${n}`).join(', ')}.`))),
    };
  },
};
