# DUBURI — ROV Simulator

A browser-based underwater vehicle simulator for testing ROV/AUV designs and
ArduSub tuning before the vehicle goes in the water. Load your hull (STL), place
the thrusters, and fly it in a virtual pool with an ArduSub-style interface and
a gamepad.

> **Status:** work in progress. The physics, allocation and hydrodynamics core
> is in place and unit-tested; the ArduSub control port, CFD lab and user
> interface are being built.

## What it does

- **ArduSub control stack, ported from ArduSub 4.5.7 source** — motor mixer
  (`AP_Motors6DOF`), attitude controller (`AC_AttitudeControl_Sub`), depth
  controller (`AC_PosControl`), flight modes, joystick/button handling, and the
  real parameter names (`ATC_*`, `PSC_*`, `MOT_*`, `JS_*`, `BTNn_*`).
  Import/export `.param` files.
- **Automatic thruster allocation** — from thruster positions and directions it
  finds the largest set of independently controllable axes (full 6-DOF when the
  layout allows it, otherwise the maximum possible), builds an ArduSub-style
  factor table, and reports authority and cross-coupling per axis.
- **Hydrodynamics**
  - 6-DOF Fossen model with full 6×6 added mass and coupled damping.
  - Added mass from a potential-flow **boundary element method** (panel method)
    on your hull; validated against analytic sphere/ellipsoid results.
  - Drag/damping from a GPU **Lattice-Boltzmann CFD** virtual tow tank
    (in progress), with projected-area estimates as defaults.
  - Surface-aware buoyancy, thruster in-flow loss, ventilation near the surface,
    thruster–hull interaction, current, turbulence and waves.
- **Team profiles** — DUBURI header with a team logo/name/colour so several
  teams can use the same simulator.

## Repository layout

```
src/
  core/       math (quaternions, SVD, pseudo-inverse), seeded RNG
  geometry/   STL I/O, voxelisation, mass properties, smooth hull extraction
  alloc/      ArduSub frame tables, thruster allocation & DOF analysis
  hydro/      water properties, BEM added-mass solver
  physics/    thruster model, environment, 6-DOF vehicle plant
tests/        node:test unit tests (run with `npm test`)
```

## Development

```bash
npm install
npm test          # unit tests
```

## Conventions

- World frame: NED, origin at the still-water surface in the pool centre.
- Body frame: FRD (x forward, y starboard, z down), origin at the CG.
- Euler angles follow ArduPilot's 3-2-1 convention.
