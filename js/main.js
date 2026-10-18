// The testbench app: the shooter live (MuJoCo WASM), drawn with three.js, the panel of shooter/testbench.py.
// Everything is reachable from the console as window.app.

import { loadMujoco } from './cdn.js';
import { Sim } from './sim.js';
import { Shooter, BALL_SPREAD } from './shooter.js';
import { View } from './view.js';
import { fetchVariant } from './assets.js';
import { CalMap } from './calib.js';
import { Strip } from './charts.js';
import { markdown } from './markdown.js';

const $ = (id) => document.getElementById(id);
const fmt = (x, d) => (x == null || !isFinite(x) ? '-' : (+x).toFixed(d));
const sgn = (x, d) => (x == null || !isFinite(x) ? '-' : (x >= 0 ? '+' : '') + (+x).toFixed(d));

const VIEWS = [  // name, azimuth, elevation, distance (MuJoCo free camera, as testbench.py), look-at (null: the shooter)
  ['ISO', 135, -20, 0.95, null], ['SIDE', 180, -2, 0.9, null], ['FRONT', 60, -12, 1.0, null], ['TOP', 180, -88, 0.9, null],
  ['FLIGHT', 160, -8, 3.2, [0, 0.9, 1.0]]];

const app = window.app = {
  mj: null, sim: null, shooter: null, view: null, manifest: null, variant: null, variants: [], base: null,
  map: null, maps: {},               // the calibration in use, and browser recalibrations per variant id
  s: { ball: 'pollen', mode: 'target', speed: 6.0, elev: 72.5, rpm: 2600, arm: 0, sq_pollen: 2.4, sq_nectar: 1.0,
       feeder_u: 1.0, burst: 6, slow: 1, auto: false, paused: false, view: 0, jitter: true, vary: true, collision: false },
  queue: 0, queueActive: false, lastFeed: null, tracker: null, recov: null, shots: [], cmd: [0, 2600, true],
  achieved: 0, fps: 60,
  prof: { phys: 0, draw: 0, frames: 0, steps: 0 },   // ms per frame accumulators (app.prof; reset by hand)
};

// ------------------------------------------------------------------ the variants index
// assets/variants/index.json: a list of {id, label, group, folder} (folder relative to the index's folder); the older
// {default, variants: [{id, name, path}]} form (path relative to assets/) is read too.
const INDEX = 'assets/variants/index.json';
function readIndex(idx) {
  const base = new URL(INDEX, document.baseURI);
  const list = Array.isArray(idx) ? idx : idx.variants;
  return list.map((v) => ({ id: v.id, label: v.label || v.name || v.id, group: v.group || '', describe: v.describe || '',
    url: v.folder ? new URL(v.folder.replace(/\/?$/, '/'), base).href : new URL('assets/' + v.path, document.baseURI).href }));
}

// ------------------------------------------------------------------ boot
async function boot() {
  buildSliders();
  wireUi();
  app.view = new View($('view'));
  VIEWS.forEach(([nm], i) => { const b = document.createElement('button'); b.textContent = nm; b.title = nm === 'FLIGHT' ? 'Camera: zoomed out to follow the ball in flight (key V)' : `Camera: ${nm.toLowerCase()} view of the shooter (key V)`; b.onclick = () => setView(i); $('views').appendChild(b); });
  app.strip = new Strip($('charts'), [
    { label: 'wheel rpm (target grey)', unit: 'rpm', color: '#f48434', fmt: (v) => fmt(v, 0), ref: true, span: 40 },
    { label: 'motor current', unit: 'A', color: '#78beff', fmt: (v) => fmt(v, 2), span: 0.5 },
    { label: 'battery', unit: 'V', color: '#56d48a', fmt: (v) => fmt(v, 2), span: 0.05 }]);
  $('loadmsg').textContent = 'Loading MuJoCo (WebAssembly, ~10 MB)...';
  const [mj, idx] = await Promise.all([loadMujoco(), fetch(INDEX, { cache: 'no-cache' }).then((r) => r.json())]);
  app.mj = mj; app.sim = new Sim(mj); app.variants = readIndex(idx);
  const groups = [...new Set(app.variants.map((v) => v.group))];
  $('variant').innerHTML = groups.map((g) => {
    const opts = app.variants.filter((v) => v.group === g).map((v) => `<option value="${v.id}" title="${v.describe}">${g ? g + ': ' : ''}${v.label}</option>`).join('');
    return g ? `<optgroup label="${g}">${opts}</optgroup>` : opts;
  }).join('');
  const want = new URLSearchParams(location.search).get('variant') || app.variants[0].id;
  $('variant').value = want;
  await loadVariant(want);
  fetch('assets/about.md', { cache: 'no-cache' }).then((r) => r.ok ? r.text() : '').then((t) => { $('aboutBody').innerHTML = markdown(t); });
  requestAnimationFrame(frame);
}

/** Load (or switch to) a variant: a new model in the same Sim, new controllers, the drawing rebuilt; UI state kept. */
async function loadVariant(id) {
  const v = app.variants.find((x) => x.id === id) || app.variants[0];
  $('loading').classList.remove('hide');
  $('loadmsg').textContent = `Loading ${v.label}...`;
  const data = await fetchVariant(v.url, (i, n) => { $('loadmsg').textContent = `Loading ${v.label}: meshes ${i}/${n}`; });
  const t0 = performance.now();
  if (app.shooter) app.shooter.detach();
  if (app.fireHook) app.sim.removeHook(app.fireHook);
  if (app.sampleHook) app.sim.removeHook(app.sampleHook);
  app.sim.loadScene(data.xml, data.files);
  app.manifest = data.manifest; app.variant = v; app.base = data.base;
  const B = app.shooter = new Shooter(app.sim, data.manifest, { jitter: app.s.jitter, seed: (Math.random() * 2 ** 31) | 0,
    ballSpread: app.s.vary ? Object.keys(data.manifest.controller.ball_spread || BALL_SPREAD) : null });
  B.onShot = onShot;
  app.fireHook = app.sim.addHook({ preStep: fireStep }, true);
  app.sampleHook = app.sim.addHook({ postStep: sampleStep });
  app.view.build(app.sim, B.balls);
  app.view.setShowCollision(app.s.collision);
  app.calFile = data.calibration;
  app.map = app.maps[v.id] || (data.calibration ? new CalMap(data.calibration) : null);
  if (!app.map && app.s.mode === 'target') app.s.mode = 'direct';
  const D = data.manifest.design, lim = D.angle_travel_deg;
  const sa = document.querySelector('.slider[data-key="arm"] input'); sa.min = -lim; sa.max = lim;
  const fixedAngle = !(lim > 1e-9);                       // a fixed-angle design: speed is the only target (no hood actuator)
  for (const k of ['arm', 'elev']) document.querySelector(`.slider[data-key="${k}"]`).style.display = fixedAngle ? 'none' : '';
  if (fixedAngle) app.s.arm = 0;
  if (!app._sqInit) { app.s.sq_pollen = D.squeeze_pollen_mm; app.s.sq_nectar = D.squeeze_nectar_mm; app.s.feeder_u = data.manifest.controller.feeder?.u ?? 1; app._sqInit = true; }
  document.querySelector('.slider[data-key="feeder_u"]').style.display = data.manifest.controller.feeder ? '' : 'none';
  app.tracker = null; app.queue = 0; app.recov = null; app.strip.clear();
  app._cmdKey = null; apply();
  setView(app.s.view);
  $('vtitle').textContent = (v.group ? v.group + ': ' : '') + v.label;
  $('vdesc').textContent = v.describe || '';
  $('design').textContent = `${data.manifest.design_file}${Object.keys(data.manifest.overrides || {}).length ? ' + overrides' : ''}`;
  $('status').textContent = `${v.id}: compiled in ${(performance.now() - t0).toFixed(0)} ms, ${data.manifest.counts.ngeom} geoms, ` +
    `timestep ${data.manifest.timestep * 1e3} ms, MuJoCo ${data.manifest.mujoco_version}`;
  history.replaceState(null, '', `?variant=${encodeURIComponent(v.id)}`);
  calInfo(); syncUi(); renderShot(); renderLog(); renderStats();
  $('loading').classList.add('hide');
}

// ------------------------------------------------------------------ the command (testbench.py command/apply)
/** The map solves at the selected squeeze (calib.js CalMap levels); a squeeze slider outside the calibrated range is
 *  clamped to it, and both squeeze sliders are sized to it. */
function syncSqueeze() {
  const M = app.map, s = app.s;
  if (!M || !M.setSqueeze) return;
  for (const bt of ['pollen', 'nectar']) {
    const R = M.sqRange(bt), inp = document.querySelector(`.slider[data-key="sq_${bt}"] input`);
    if (R) {
      if (inp) { inp.min = R[0]; inp.max = R[1]; }
      const c = Math.min(R[1], Math.max(R[0], s[`sq_${bt}`]));
      if (c !== s[`sq_${bt}`]) s[`sq_${bt}`] = c;
    }
    M.setSqueeze(bt, R ? s[`sq_${bt}`] : null);
  }
}

function command() {
  const s = app.s;
  syncSqueeze();
  if (s.mode === 'target' && app.map) {
    const key = `${s.ball}|${s.speed}|${s.elev}|${s[`sq_${s.ball}`]}|${app.map === app._cmdMap}`;
    if (key !== app._cmdKey) {
      app._cmdKey = key; app._cmdMap = app.map;
      if (!(app.map.T > 1e-9)) {                          // fixed angle (calibrate.py Map.solve): the rpm giving the speed
        let best = [0, 1600, false], err = Infinity;
        for (let i = 0; i <= 400; i++) { const q = 1600 + 2800 * i / 400, v = app.map.predict(s.ball, 0, q)[0];
          if (Math.abs(v - s.speed) < err) { err = Math.abs(v - s.speed); best = [0, q, Math.abs(v / s.speed - 1) < 0.005]; } }
        app._cmd = best; s.elev = Math.round(app.map.predict(s.ball, 0, best[1])[1] * 10) / 10;
      } else app._cmd = app.map.solve(s.ball, s.speed, s.elev);
    }
    return app._cmd;
  }
  return [s.arm, s.rpm, true];
}

function predicted(bt, arm, rpm) {
  if (!app.map) return null;
  const [v, e] = app.map.predict(bt, arm, rpm);
  return { speed: v, elevation: e };
}

function apply() {
  const s = app.s, B = app.shooter;
  app.cmd = command();
  const [arm, rpm] = app.cmd;
  B.setBall(s.ball, s[`sq_${s.ball}`]);
  B.setAngle(arm); B.setRpm(rpm);
  B.feeder_u = s.feeder_u;
  B.jitter = s.jitter;
  B.setBallSpread(s.vary ? Object.keys(B.M.controller.ball_spread || BALL_SPREAD) : null);
}

// ------------------------------------------------------------------ per step: firing (testbench.py advance) and sampling
function fireStep(sim) {
  const B = app.shooter;
  if (app.recov && B.settled()) {                       // recovery: from the last shot's exit until ready again
    app.recov.shot.recovery_s = sim.time - app.recov.t; app.recov = null; app.logDirty = true;
  }
  if (app.tracker || app.s.paused || !(app.queue > 0 || app.s.auto) || !B.settled()) return;
  const s = app.s, bt = s.ball, [arm, rpm] = app.cmd;
  const tgt = s.mode === 'target' && app.map ? { speed: s.speed, elevation: s.elev } : predicted(bt, arm, rpm);
  const now = sim.time;
  const tr = app.tracker = B.shoot(bt, tgt || undefined);
  tr.meta.interval = app.lastFeed !== null && app.queueActive ? now - app.lastFeed : null;
  tr.meta.variant = app.variant.id; tr.meta.mode = s.mode;
  app.lastFeed = now; app.queueActive = true;
  if (app.queue > 0) app.queue--;
  app.view.startTrail(tr.ball.body, now);
}

let sampleN = 0;
function sampleStep(sim) {
  if (++sampleN % 25) return;                           // every 5 ms of simulated time
  const B = app.shooter;
  app.strip.push([B.wheelRpm(), B.current, B.v_batt], [B.targetRpm()]);
}

function hideHint() { const h = $('hint'); if (h && !h.classList.contains('gone')) { h.classList.add('gone'); try { localStorage.setItem('shooterHintSeen', '1'); } catch (e) { /* private mode */ } } }

function onShot(r) {
  hideHint();
  r.n = app.shots.length + 1;
  app.shots.push(r); app.tracker = null;
  if (!r.jam) app.recov = { shot: r, t: r.t_exit };
  if (app.queue === 0 && !app.s.auto) app.queueActive = false;
  app.shotDirty = true; app.logDirty = true;
}

// ------------------------------------------------------------------ the loop
let last = performance.now(), budget = 0, statT = last, statSim = 0, frames = 0, uiT = 0, chartT = 0;
const CHUNK = 25;              // steps between trail samples and parking checks (5 ms simulated)
const FRAME_MS = 14;           // physics budget per animation frame; beyond it the sim runs slower than asked

function frame(now) {
  const sim = app.sim, B = app.shooter, ts = sim.timestep;
  const dt = Math.min((now - last) / 1000, 0.05); last = now; frames++;
  apply();
  if (!app.s.paused) {
    B.poseMocaps();
    budget += dt / app.s.slow;
    const want = Math.floor(budget / ts);
    const t0 = performance.now(); let done = 0;
    // the clock is checked after EVERY step (MEASURED 2026-10-08: a ball pressed into many convex pieces at once makes
    // single steps 10-80x slower; checked only per 25-step chunk, one such chunk froze the frame): the sim runs slower
    // for that moment instead of stalling the picture; trails are still sampled every CHUNK steps
    while (done < want) {
      sim.step(1); done += 1;
      if (done % CHUNK === 0) app.view.sampleTrails(sim);
      if (performance.now() - t0 > FRAME_MS) break;
    }
    if (done % CHUNK) app.view.sampleTrails(sim);
    B.parkAllFlown();
    app.prof.phys += performance.now() - t0; app.prof.steps += done;
    budget = done < want ? 0 : budget - done * ts;
    statSim += done * ts;
  }
  if (now - statT > 500) {
    app.achieved = statSim / ((now - statT) / 1000); app.fps = frames / ((now - statT) / 1000);
    statT = now; statSim = 0; frames = 0;
  }
  const tr0 = performance.now();
  app.view.sync(sim);
  app.view.render();
  app.prof.draw += performance.now() - tr0; app.prof.frames++;
  if (now - uiT > 100) { uiT = now; liveUi(); }
  if (now - chartT > 66) { chartT = now; app.strip.draw(); }
  if (app.shotDirty) { app.shotDirty = false; renderShot(); renderStats(); }
  if (app.logDirty) { app.logDirty = false; renderLog(); }
  requestAnimationFrame(frame);
}

// ------------------------------------------------------------------ panel
function buildSliders() {
  for (const el of document.querySelectorAll('.slider')) {
    const label = el.textContent.trim();
    el.innerHTML = `<span>${label}</span><span class="val"></span><input type="range" min="${el.dataset.min}" max="${el.dataset.max}" step="${el.dataset.step}">`;
    const inp = el.querySelector('input'), key = el.dataset.key;
    inp.addEventListener('input', () => { app.s[key] = +inp.value; syncUi(); });
  }
}

/** What the calibrated shooter can actually do for the current ball (Ben 2026-10-06: no "OUT OF RANGE" - size the
 *  sliders). Speed and angle limit each other, so each slider's range is what the map reaches AT THE OTHER'S VALUE:
 *  for every hood angle the wheel speed that gives the wanted exit speed (bisection: speed rises with rpm) -> the exit
 *  angles reachable at that speed; and for every wheel speed the hood angle that gives the wanted exit angle -> the
 *  speeds reachable at that angle. A small margin keeps the ends solvable (solve's tolerance: 0.5 %, 0.25 deg). */
function targetLimits() {
  const s = app.s, M = app.map;
  if (!M) return null;
  syncSqueeze();
  const key = `${s.ball}|${s.speed}|${s.elev}|${s[`sq_${s.ball}`]}|${app.variant?.id}`;
  if (app._limKey === key && app._limMap === M) return app._lim;
  const T = M.T, R0 = 1600, R1 = 4400, bt = s.ball;
  if (!(T > 1e-9)) {                                       // fixed angle: speed is the only target
    let vlo = Infinity, vhi = -Infinity;
    for (let j = 0; j <= 80; j++) { const v = M.predict(bt, 0, R0 + (R1 - R0) * j / 80)[0]; vlo = Math.min(vlo, v); vhi = Math.max(vhi, v); }
    const lim = { all: { speed: [vlo, vhi], elev: [45, 90] }, speed: [Math.ceil(vlo * 1.003 * 20) / 20, Math.floor(vhi * 0.997 * 20) / 20] };
    app._limKey = key; app._limMap = M; app._lim = lim; return lim;
  }
  const bis = (f, lo, hi) => { let flo = f(lo), fhi = f(hi); if (flo * fhi > 0) return null;
    for (let k = 0; k < 40; k++) { const mid = 0.5 * (lo + hi), fm = f(mid); if (flo * fm <= 0) { hi = mid; fhi = fm; } else { lo = mid; flo = fm; } }
    return 0.5 * (lo + hi); };
  let elo = Infinity, ehi = -Infinity, vlo = Infinity, vhi = -Infinity;
  for (let i = 0; i <= 80; i++) {                         // exit angles reachable at the wanted speed
    const a = -T + 2 * T * i / 80, r = bis((q) => M.predict(bt, a, q)[0] - s.speed, R0, R1);
    if (r !== null) { const e = M.predict(bt, a, r)[1]; elo = Math.min(elo, e); ehi = Math.max(ehi, e); }
  }
  for (let j = 0; j <= 80; j++) {                         // exit speeds reachable at the wanted angle
    const q = R0 + (R1 - R0) * j / 80, a = bis((x) => M.predict(bt, x, q)[1] - s.elev, -T, T);
    if (a !== null) { const v = M.predict(bt, a, q)[0]; vlo = Math.min(vlo, v); vhi = Math.max(vhi, v); }
  }
  // the whole map's reach (a fallback when nothing is reachable at the other slider's value)
  let Vlo = Infinity, Vhi = -Infinity, Elo = Infinity, Ehi = -Infinity;
  for (let i = 0; i <= 30; i++) for (let j = 0; j <= 30; j++) {
    const [v, e] = M.predict(bt, -T + 2 * T * i / 30, R0 + (R1 - R0) * j / 30);
    Vlo = Math.min(Vlo, v); Vhi = Math.max(Vhi, v); Elo = Math.min(Elo, e); Ehi = Math.max(Ehi, e);
  }
  const lim = { all: { speed: [Vlo, Vhi], elev: [Math.max(45, Elo), Math.min(90, Ehi)] } };
  if (isFinite(elo)) lim.elev = [Math.ceil((Math.max(45, elo) + 0.15) * 2) / 2, Math.floor((Math.min(90, ehi) - 0.15) * 2) / 2];
  if (isFinite(vlo)) lim.speed = [Math.ceil((vlo * 1.003) * 20) / 20, Math.floor((vhi * 0.997) * 20) / 20];
  app._limKey = key; app._limMap = M; app._lim = lim;
  return lim;
}

function applyLimits() {
  const s = app.s;
  for (let pass = 0; pass < 40; pass++) {                  // clamping one can move the other's range: settle
    const L = targetLimits();
    if (!L) return;
    let moved = false;
    for (const k of ['speed', 'elev']) {
      if (!L[k] && k === 'elev' && !(app.map.T > 1e-9)) continue;     // fixed angle: no angle target
      if (!L[k] || L[k][0] > L[k][1]) {                   // nothing reachable at the other's value: walk toward the middle
        const mid = 0.5 * (L.all[k][0] + L.all[k][1]), step = k === 'speed' ? 0.25 : 2.0;
        const c = Math.abs(mid - s[k]) <= step ? mid : s[k] + Math.sign(mid - s[k]) * step;
        s[k] = Math.round(c * (k === 'speed' ? 20 : 2)) / (k === 'speed' ? 20 : 2); app._cmdKey = null; moved = true; continue;
      }
      const inp = document.querySelector(`.slider[data-key="${k}"] input`);
      if (document.activeElement !== inp || pass > 0) { inp.min = L[k][0]; inp.max = L[k][1]; }
      const c = Math.min(L[k][1], Math.max(L[k][0], s[k]));
      if (c !== s[k]) { s[k] = c; app._cmdKey = null; moved = true; }
    }
    if (!moved) break;
  }
  // the map's reach can have small holes (a bumpy fit): if this pair does not solve, step the angle (then the speed) to
  // the nearest pair that does, so the command never reads OUT OF RANGE
  const M = app.map; if (!M) return;
  if (M.solve(s.ball, s.speed, s.elev)[2]) return;
  for (let d = 0.5; d <= 12; d += 0.5) for (const sg of [1, -1]) {
    const e = s.elev + sg * d;
    if (M.solve(s.ball, s.speed, e)[2]) { s.elev = e; app._cmdKey = null; return; }
  }
  for (let d = 0.05; d <= 1.5; d += 0.05) for (const sg of [1, -1]) {
    const v = Math.round((s.speed + sg * d) * 20) / 20;
    if (M.solve(s.ball, v, s.elev)[2]) { s.speed = v; app._cmdKey = null; return; }
  }
}

function syncUi() {
  const s = app.s;
  applyLimits();
  for (const el of document.querySelectorAll('.slider')) {
    const key = el.dataset.key, inp = el.querySelector('input'), d = +el.dataset.fmt;
    if (document.activeElement !== inp) inp.value = s[key];
    el.querySelector('.val').textContent = `${el.dataset.sign ? sgn(s[key], d) : fmt(s[key], d)} ${el.dataset.unit}`;
  }
  document.querySelectorAll('[data-ball]').forEach((b) => b.classList.toggle('on', b.dataset.ball === s.ball));
  document.querySelectorAll('[data-mode]').forEach((b) => { b.classList.toggle('on', b.dataset.mode === s.mode); if (b.dataset.mode === 'target') b.disabled = !app.map; });
  $('sliders-target').hidden = s.mode !== 'target'; $('sliders-direct').hidden = s.mode !== 'direct';
  $('burstN').textContent = `x${s.burst}`;
  $('auto').classList.toggle('on', s.auto);
  $('pause').textContent = s.paused ? 'Run (P)' : 'Pause (P)';
  document.querySelectorAll('#views button').forEach((b, i) => b.classList.toggle('on', i === s.view));
}

function liveUi() {
  const B = app.shooter, s = app.s;
  if (!B) return;
  const [arm, rpm, ok] = app.cmd;
  const sol = $('solved');
  if (s.mode === 'target' && app.map) sol.textContent = `solved: wheel ${fmt(rpm, 0)} rpm, hood ${sgn(arm, 2)} deg${ok ? '' : '   OUT OF RANGE'}`;
  else { const p = predicted(s.ball, arm, rpm); sol.textContent = p ? `predicted: ${fmt(p.speed, 2)} m/s at ${fmt(p.elevation, 1)} deg` : 'no calibration: no prediction'; }
  sol.classList.toggle('bad', !ok);
  $('l-rpm').textContent = `${fmt(B.wheelRpm(), 0).padStart(5)} / ${fmt(B.targetRpm(), 0)} rpm`;
  $('l-hood').textContent = `${sgn(B.armDeg(), 2).padStart(6)} / ${sgn(B.armTargetDeg(), 2)} deg`;
  $('l-gap').textContent = `${sgn(B.gapMm(), 2).padStart(6)} / ${sgn(B.gapTargetMm(), 2)} mm`;
  $('l-motor').textContent = B.motor ? `${fmt(B.current, 2)} A, battery ${fmt(B.v_batt, 2)} V${B.M.controller.v10 ? `, lift ${fmt(B.lift.current, 2)} A` : ''}` : '-';
  const st = s.paused ? 'PAUSED' : app.tracker ? 'FIRING' : B.settled() ? 'READY' : 'SETTLING';
  const badge = $('l-state'); badge.textContent = st; badge.className = 'badge ' + st.toLowerCase();
  $('l-queue').textContent = `queue ${app.queue}${s.auto ? ' +auto' : ''}`;
  $('perf').textContent = `${fmt(app.fps, 0)} fps  ${fmt(app.achieved, 2)}x real time${s.slow > 1 ? ` (slow motion ${s.slow}x)` : ''}`;
  $('footer').textContent = `t ${fmt(app.sim.time, 2)} s   sim ${fmt(app.achieved, 2)}x real time (asked ${fmt(1 / s.slow, 2)}x)   ${app.shooter.balls.length} balls, ${app.manifest.counts.ngeom} geoms`;
}

const TIPS = {
  droop: 'How much the flywheel slowed during the shot', recovery: 'Time from the ball leaving until the shooter was ready again',
  'squeeze tread': 'How far the ball was pressed into the flywheel tread', 'squeeze hood': 'How far the ball was pressed into the hood',
  'force tread': 'Largest push between ball and flywheel', 'force hood': 'Largest push between ball and hood',
  contact: 'How long the ball touched the shooter', slip: 'How much slower the ball left than the tread surface (0 = no slip)',
  spin: 'Ball spin as it left (backspin positive)', energy: 'Kinetic energy of the ball as it left',
  azimuth: 'Sideways angle of the shot (0 = straight)', 'motor peak': 'Highest flywheel motor current during the shot',
  'battery min': 'Lowest battery voltage during the shot', 'feeder peak': 'Highest feeder motor current during the shot',
  'wheel / hood': 'Wheel rpm and hood angle the shot was fired with', 'peak force': 'Largest total contact force on the ball',
  ball: 'This ball\'s drawn deviation from nominal: diameter and mass (stiffness and bounce vary too)',
};
const errClass = (x, good, warn) => (x == null ? '' : Math.abs(x) <= good ? 'good' : Math.abs(x) <= warn ? 'warn' : 'bad');

function renderShot() {
  const r = app.shots[app.shots.length - 1];
  const g = $('s-grid');
  if (!r) { $('s-speed').textContent = '-'; $('s-elev').textContent = '-'; $('s-verr').textContent = 'no shots yet: press FIRE'; $('s-eerr').textContent = ''; g.innerHTML = ''; return; }
  if (r.jam) {
    $('s-speed').textContent = 'JAM'; $('s-elev').textContent = r.dud ? 'dud' : '-';
    $('s-verr').textContent = r.taken ? 'taken by the wheel, never left' : 'the flywheel never took it'; $('s-eerr').textContent = '';
    g.innerHTML = ''; return;
  }
  $('s-speed').textContent = fmt(r.speed, 2); $('s-elev').textContent = fmt(r.elevation, 1);
  const ve = r.speed_err != null ? 100 * r.speed_err : null, ee = r.elev_err;
  $('s-verr').innerHTML = r.target ? `err <span class="${errClass(ve, 1, 3)}">${sgn(ve, 2)} %</span> vs ${fmt(r.target.speed, 2)}` : '<span class="dim">no target</span>';
  $('s-eerr').innerHTML = r.target ? `err <span class="${errClass(ee, 0.5, 1.5)}">${sgn(ee, 2)} deg</span> vs ${fmt(r.target.elevation, 1)}` : '';
  const items = [
    ['droop', `${fmt(100 * r.droop, 1)} %`], ['recovery', r.recovery_s != null ? `${fmt(r.recovery_s, 2)} s` : '...'],
    ['squeeze tread', `${fmt(r.squeeze_tread_mm, 2)} mm`], ['squeeze hood', `${fmt(r.squeeze_hood_mm, 2)} mm`],
    ['force tread', `${fmt(r.force_tread_n, 0)} N`], ['force hood', `${fmt(r.force_hood_n, 0)} N`],
    ['contact', `${fmt(r.contact_ms, 1)} ms`], ['slip', `${fmt(100 * r.slip, 1)} %`],
    ['spin', `${sgn(r.spin, 0)} rad/s`], ['energy', `${fmt(r.energy_j, 3)} J`],
    ['azimuth', `${sgn(r.azimuth, 2)} deg`], ['motor peak', `${fmt(r.motor_current_peak_a, 2)} A`],
    ['battery min', `${fmt(r.battery_min_v, 2)} V`], ['feeder peak', `${fmt(r.feeder_current_peak_a, 2)} A`],
    ['wheel / hood', `${fmt(r.rpm_target, 0)} rpm ${sgn(r.arm_deg, 2)}`], ['peak force', `${fmt(r.peak_force_n, 0)} N`]];
  if (r.ball_draw) items.push(['ball', `d ${sgn(100 * r.ball_draw.diameter, 2)} % m ${sgn(100 * r.ball_draw.mass, 1)} %`]);
  g.innerHTML = items.map(([k, v]) => `<div title="${TIPS[k] || ''}"><span>${k}</span><b>${v}</b></div>`).join('');
}

function renderLog() {
  const rows = app.shots.slice(-60).reverse().map((r) => {
    const t = r.target || {};
    const ve = r.speed_err != null ? 100 * r.speed_err : null;
    return `<tr><td>${r.n}</td><td class="${r.type}">${r.type.slice(0, 3).toUpperCase()}</td><td>${fmt(t.speed, 2)}</td>` +
      `<td>${r.jam ? 'JAM' : fmt(r.speed, 2)}</td><td class="${errClass(ve, 1, 3)}">${r.jam || ve == null ? '' : sgn(ve, 2) + '%'}</td>` +
      `<td>${fmt(t.elevation, 1)}</td><td>${r.jam ? '' : fmt(r.elevation, 1)}</td><td class="${errClass(r.elev_err, 0.5, 1.5)}">${r.jam || r.elev_err == null ? '' : sgn(r.elev_err, 2)}</td>` +
      `<td>${r.interval ? fmt(r.interval, 2) : '-'}</td><td>${r.recovery_s != null ? fmt(r.recovery_s, 2) : '-'}</td></tr>`;
  });
  $('log').querySelector('tbody').innerHTML = rows.join('');
  if (app.shots.length) renderShot();
}

function renderStats() {
  const out = [];
  for (const bt of ['pollen', 'nectar']) {
    const S = app.shots.filter((r) => r.type === bt && r.variant === app.variant.id);
    if (!S.length) continue;
    const ok = S.filter((r) => !r.jam && r.speed_err != null);
    const ev = ok.map((r) => 100 * r.speed_err), ee = ok.map((r) => r.elev_err);
    const rms = (a) => (a.length ? Math.sqrt(a.reduce((x, y) => x + y * y, 0) / a.length) : NaN);
    const mx = (a) => (a.length ? Math.max(...a.map(Math.abs)) : NaN);
    const iv = S.filter((r) => r.interval).map((r) => r.interval);
    const rate = iv.length ? 1 / (iv.reduce((a, b) => a + b, 0) / iv.length) : null;
    out.push(`<div class="${bt}">${bt.toUpperCase().padEnd(6)} ${ok.length}/${S.length} launched  speed rms ${fmt(rms(ev), 2)}% max ${fmt(mx(ev), 2)}%  ` +
      `angle rms ${fmt(rms(ee), 2)} max ${fmt(mx(ee), 2)} deg${rate ? `  ${fmt(rate, 2)} shots/s` : ''}</div>`);
  }
  $('stats').innerHTML = out.join('') || '<span class="dim">no shots on this variant yet</span>';
}

function calInfo() {
  const c = app.map?.cal;
  const v = app.variant.id;
  if (!app.map) $('calinfo').textContent = 'No calibration for this variant: DIRECT mode only. Recalibrate to measure one here (about 1-3 min).';
  else if (app.maps[v]) $('calinfo').textContent = `Measured in this browser (${c.pollen.n + c.nectar.n} shots, ${fmt(c.wall_s, 0)} s): natural bicubic interpolation through the grid means. Squeezes ${fmt(c.settings?.squeeze?.pollen, 1)} / ${fmt(c.settings?.squeeze?.nectar, 1)} mm, feeder ${fmt(c.settings?.feeder_u, 2)}.`;
  else $('calinfo').textContent = `From the Python calibration (${c.source}): the same smoothing splines as shooter/calibrate.py, ${c.pollen.n ?? '?'} + ${c.nectar.n ?? '?'} shots. Valid for the design's squeeze settings.`;
  $('calDownload').hidden = !app.maps[v];
}

// ------------------------------------------------------------------ recalibration (Web Worker)
function recalibrate() {
  if (app.worker) return;
  const s = app.s;
  const w = app.worker = new Worker(new URL('./recal_worker.js', import.meta.url), { type: 'module' });
  $('calprog').hidden = false; $('recalStop').hidden = false; $('recal').disabled = true;
  const bar = $('calprog').firstElementChild; bar.style.width = '0%';
  $('calinfo').textContent = 'Starting a second MuJoCo for the measurement...';
  const vid = app.variant.id;
  w.onmessage = (ev) => {
    const d = ev.data;
    if (d.progress) {
      const [i, n, t] = d.progress; bar.style.width = `${(100 * i / n).toFixed(1)}%`;
      $('calinfo').textContent = `Measuring the grid: shot ${i} / ${n}, ${fmt(t, 0)} s, about ${fmt(t / i * (n - i), 0)} s left`;
    } else if (d.done) {
      app.maps[vid] = new CalMap(d.done, 'interp');
      if (app.variant.id === vid) { app.map = app.maps[vid]; app._cmdKey = null; }
      endRecal(); calInfo(); syncUi();
    } else if (d.error || d.stopped) {
      endRecal(); calInfo();
      if (d.error) { $('calinfo').textContent = 'Recalibration failed: ' + d.error.split('\n')[0]; console.error(d.error); }
    }
  };
  w.postMessage({ base: app.base, settings: { squeeze: { pollen: s.sq_pollen, nectar: s.sq_nectar }, feeder_u: s.feeder_u, per: 2, seed: 100 } });
}
function endRecal() {
  if (app.worker) app.worker.terminate();
  app.worker = null; $('calprog').hidden = true; $('recalStop').hidden = true; $('recal').disabled = false;
}

// ------------------------------------------------------------------ wiring
function setView(i) {
  app.s.view = ((i % VIEWS.length) + VIEWS.length) % VIEWS.length;
  const [, az, el, dist, look] = VIEWS[app.s.view];
  const h = app.manifest ? app.manifest.design.height_m : 0.3, V = app.manifest?.view;   // a variant may aim the presets (the v2 feed: between cell and wheel)
  app.view.setView({ az, el, dist: look ? dist : dist * (V?.dist_scale ?? 1), lookat: look || V?.lookat || [0, -0.03, h - 0.04] });
  syncUi();
}

function wireUi() {
  const s = app.s;
  try { if (localStorage.getItem('shooterHintSeen')) $('hint').classList.add('gone'); } catch (e) { /* private mode */ }
  window.addEventListener('pointerdown', hideHint, { once: true });
  document.querySelectorAll('[data-ball]').forEach((b) => { b.onclick = () => { s.ball = b.dataset.ball; app._cmdKey = null; syncUi(); }; });
  document.querySelectorAll('[data-mode]').forEach((b) => { b.onclick = () => { s.mode = b.dataset.mode; syncUi(); }; });
  $('fire').onclick = () => { app.queue++; };
  $('burst').onclick = () => { app.queue += s.burst; };
  $('auto').onclick = () => { s.auto = !s.auto; syncUi(); };
  $('stop').onclick = () => { app.queue = 0; s.auto = false; syncUi(); };
  $('pause').onclick = () => { s.paused = !s.paused; syncUi(); };
  $('resetStats').onclick = () => { app.shots = []; app.shotDirty = app.logDirty = true; };
  $('resetSim').onclick = () => { app.shooter.reset(); app.view.clearTrails(); app.tracker = null; app.queue = 0; app.recov = null; app.strip.clear(); app.lastFeed = null; };
  $('jitter').onchange = () => { s.jitter = $('jitter').checked; };
  $('vary').onchange = () => { s.vary = $('vary').checked; apply(); };
  $('collision').onchange = () => { s.collision = $('collision').checked; app.view.setShowCollision(s.collision); };
  $('variant').onchange = () => loadVariant($('variant').value).catch(fail);
  $('recal').onclick = recalibrate;
  $('recalStop').onclick = () => { endRecal(); calInfo(); };
  $('calDownload').onclick = () => {
    const c = app.maps[app.variant.id]?.cal; if (!c) return;
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([JSON.stringify(c)], { type: 'application/json' }));
    a.download = `calibration_${app.variant.id}_browser.json`; a.click();
  };
  $('aboutBtn').onclick = () => { $('about').hidden = false; };
  $('aboutClose').onclick = () => { $('about').hidden = true; };
  $('about').onclick = (e) => { if (e.target.id === 'about') $('about').hidden = true; };
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'SELECT' || (e.target.tagName === 'INPUT' && e.target.type !== 'range' && e.target.type !== 'checkbox')) return;
    const k = e.key.toLowerCase();
    if (k === ' ') { e.preventDefault(); app.queue++; }
    else if (k === 'b') app.queue += s.burst;
    else if (k === 'a') s.auto = !s.auto;
    else if (k === 'r') { app.shots = []; app.shotDirty = app.logDirty = true; }
    else if (k === '1') s.ball = 'pollen';
    else if (k === '2') s.ball = 'nectar';
    else if (k === 'm' && app.map) s.mode = s.mode === 'target' ? 'direct' : 'target';
    else if (k === 'v') setView(s.view + 1);
    else if (k === 'p') s.paused = !s.paused;
    else if (k === 'escape') $('about').hidden = true;
    else if (e.key.startsWith('Arrow')) {
      if (e.target.type === 'range') return;
      e.preventDefault();
      const up = e.key === 'ArrowUp' ? 1 : e.key === 'ArrowDown' ? -1 : 0, rt = e.key === 'ArrowRight' ? 1 : e.key === 'ArrowLeft' ? -1 : 0;
      const T = app.manifest.design.angle_travel_deg;
      if (s.mode === 'target') { s.speed = Math.round(Math.min(9.5, Math.max(3.5, s.speed + 0.05 * up)) * 100) / 100; s.elev = Math.min(88, Math.max(50, s.elev + 0.5 * rt)); applyLimits(); }
      else { s.rpm = Math.min(4400, Math.max(1600, s.rpm + 25 * up)); s.arm = Math.min(T, Math.max(-T, s.arm + 0.25 * rt)); }
    } else return;
    syncUi();
  });
}

function fail(e) { $('loadmsg').textContent = 'ERROR: ' + (e?.message || e); $('loading').classList.remove('hide'); console.error(e); }
boot().catch(fail);
