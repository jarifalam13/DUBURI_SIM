// One-shot geometry analysis of a hull (already in the vehicle frame, metres).
// Heavy for big STLs, so the browser runs it in a worker.

import { bbox, topology, signedVolume, makeGrid, voxelizeSolid, voxelizeSurface, massProps, projections, buoyancyElements, contactPoints, union, fillEnclosed } from './mesh.js';

export function chooseCell(bb, targetCells = 56, minCell = 0.003) {
  const L = Math.max(...bb.size);
  return Math.max(minCell, L / targetCells);
}

export function analyzeGeometry(positions, opts = {}) {
  const onProgress = opts.onProgress || (() => {});
  const bb = bbox(positions);
  if (!Number.isFinite(bb.size[0]) || Math.max(...bb.size) <= 0) throw new Error('Empty or invalid geometry');
  onProgress(0.05, 'Checking mesh topology');
  const topo = topology(positions);
  const sv = signedVolume(positions);
  const cell = opts.cell ?? chooseCell(bb, opts.targetCells ?? 56);
  onProgress(0.15, 'Voxelising hull volume');
  const solid = makeGrid(bb, cell, 2);
  const vs = voxelizeSolid(positions, solid);
  onProgress(0.5, 'Marking thin features');
  const surf = makeGrid(bb, cell, 2);
  voxelizeSurface(positions, surf);
  // "wet" solid used for CFD walls and jet blockage: solid + surface cells
  const wet = union(solid, surf);
  onProgress(0.65, 'Mass properties');
  let mp = massProps(solid);
  let volumeSource = 'voxel';
  if (mp.volume <= 0) { mp = massProps(wet); volumeSource = 'surface-voxel'; }
  // Sealed cavities (e.g. hollow shells exported without internal parts) count
  // as displaced volume only if the user says the hull is sealed.
  const sealed = opts.sealedHull ? massProps(fillEnclosed(wet)) : null;
  onProgress(0.75, 'Projected areas');
  const proj = projections(wet, mp.centroid);
  onProgress(0.85, 'Buoyancy elements');
  const els = buoyancyElements(volumeSource === 'voxel' ? solid : wet, opts.buoyancyResolution ?? 14);
  const contacts = contactPoints(wet);
  onProgress(1, 'Done');
  return {
    bbox: bb,
    topology: topo,
    meshVolume: sv.volume,
    meshCentroid: sv.centroid,
    cell,
    gridDims: [solid.nx, solid.ny, solid.nz],
    openRayFraction: vs.openRayFraction,
    volume: sealed ? sealed.volume : mp.volume,
    centroid: sealed ? sealed.centroid : mp.centroid,
    inertiaUnit: mp.inertiaUnit,
    solidCentroid: mp.centroid,
    volumeSource: sealed ? 'sealed-hull' : volumeSource,
    projections: proj,
    buoyEls: els,
    contacts,
    wet: { nx: wet.nx, ny: wet.ny, nz: wet.nz, cell: wet.cell, origin: wet.origin, data: wet.data },
  };
}
