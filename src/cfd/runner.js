// Drives one GPU lattice-Boltzmann run from the page's animation frames:
// a few solver steps per frame (adapted to keep the page responsive), forces
// read back asynchronously, convergence checked on the averaged forces.

import { caseSettings, applyRamp, convergence, windowMean, runCoefficients, primary, decimate, snapshotStride } from './study.js';
import { LBMGPU } from './gpu.js';

export function runGPU(canvas, cs, Re, { onProgress = () => {}, onFrame = () => {}, signal, snapshot = false } = {}) {
  const st = caseSettings(cs, Re);
  const solver = new LBMGPU(canvas, { nx: cs.nx, ny: cs.ny, nz: cs.nz, mask: cs.mask, q: cs.q }, st.params);
  const cells = cs.nx * cs.ny * cs.nz;
  return new Promise((resolve, reject) => {
    const samples = [];
    let conv = { done: false };
    let perFrame = Math.max(1, Math.min(40, Math.round(1.5e6 / cells)));
    let last = performance.now();
    const t0 = last;
    let lastReport = 0;
    const finish = (err) => {
      let field = null;
      if (!err && snapshot) { try { field = solver.snapshot(snapshotStride(cs)); } catch (e) { console.warn('flow snapshot failed', e); } }
      try { solver.dispose(); } catch { /* ignore */ }
      if (err) { reject(err); return; }
      const end = samples[samples.length - 1].step;
      const W = st.win * st.Tc;
      const mean = windowMean(samples, end - W, end);
      const sec = (performance.now() - t0) / 1000;
      resolve({ Re, mean, steps: solver.steps, converged: !!conv.converged, rel: conv.rel, C: runCoefficients(cs, st, mean), history: decimate(samples.map((s) => [s.step / st.Tc, primary(s, cs, st)]), 240), seconds: sec, mlups: (cells * solver.steps) / sec / 1e6, field });
    };
    const frame = () => {
      if (signal?.aborted) { finish(new Error('cancelled')); return; }
      try {
        const now = performance.now();
        const dt = now - last;
        last = now;
        // keep the page near 30+ fps while the GPU works
        if (dt > 45) perFrame = Math.max(1, Math.floor(perFrame * 0.75));
        else if (dt < 26) perFrame = Math.min(400, Math.ceil(perFrame * 1.15));
        const r = solver.pollForce();
        if (r) {
          samples.push({ step: r.step, F: r.F, M: r.M });
          conv = convergence(samples, r.step, cs, st);
          if (now - lastReport > 150 || conv.done) {
            lastReport = now;
            onProgress({ step: r.step, fraction: Math.min(1, r.step / (st.tMax * st.Tc)), rel: conv.rel, value: primary(r, cs, st), t: r.step / st.Tc, mlups: (cells * solver.steps) / Math.max(1e-3, (now - t0) / 1000) / 1e6, perFrame });
          }
          if (conv.done) { onFrame(solver); finish(); return; }
        }
        let n = perFrame;
        while (n > 0) {
          const toSample = st.sampleEvery - (solver.steps % st.sampleEvery);
          const k = Math.min(n, toSample);
          applyRamp(solver, solver.steps, cs, st);
          solver.step(k);
          n -= k;
          if (solver.steps % st.sampleEvery === 0) solver.requestForce();
        }
        onFrame(solver);
        requestAnimationFrame(frame);
      } catch (e) { finish(e); }
    };
    requestAnimationFrame(frame);
  });
}
