// In-browser recalibration (calibrate.py measure) in a Web Worker: its own MuJoCo instance and its own copy of the
// variant, so the live view keeps running. Message in: {base, settings: {squeeze: {pollen, nectar}, feeder_u, per, seed}}.
// Messages out: {progress: [i, n]}, {done: calibration}, {error}.
import { loadMujoco } from './cdn.js';
import { Sim } from './sim.js';
import { Shooter } from './shooter.js';
import { fetchVariant } from './assets.js';
import { measureGrid } from './calib.js';

let stop = false;
self.onmessage = async (ev) => {
  if (ev.data.stop) { stop = true; return; }
  try {
    const { base, settings = {} } = ev.data;
    const mj = await loadMujoco();
    const v = await fetchVariant(base);
    const sim = new Sim(mj).loadScene(v.xml, v.files);
    const B = new Shooter(sim, v.manifest, { jitter: true, seed: settings.seed ?? 100 });
    if (settings.feeder_u != null) B.feeder_u = settings.feeder_u;
    const sq = settings.squeeze || {};
    const setBall = B.setBall.bind(B);
    B.setBall = (bt) => setBall(bt, sq[bt]);                // the current gap settings, not the design's defaults
    const t0 = performance.now();
    const cal = await measureGrid(B, (i, n) => self.postMessage({ progress: [i, n, (performance.now() - t0) / 1000] }),
      settings.per ?? 2, undefined, () => stop);
    if (!cal) { self.postMessage({ stopped: true }); return; }
    cal.settings = settings; cal.wall_s = (performance.now() - t0) / 1000; cal.sim_s = sim.time;
    self.postMessage({ done: cal });
  } catch (e) {
    self.postMessage({ error: String(e && e.stack || e) });
  }
};
