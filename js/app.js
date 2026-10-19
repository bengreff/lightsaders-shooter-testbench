// The turret contact bench page: loads the scene (assets/turret/, written by sim/mechanism/turret/bench.py), runs MuJoCo in
// real time with the Turret controller (turret.js), draws with three.js (view.js), and wires the panel.
import { loadMujoco } from './cdn.js';
import { Sim } from './sim.js';
import { Turret } from './turret.js';
import { View } from './view.js';
import { Strip } from './charts.js';
import { CalMap } from './calib.js';

const $ = (id) => document.getElementById(id);
const BASE = 'assets/turret/';
const app = {
  s: { ball: 'pollen', mode: 'direct', rpm: 3000, ext: 0, speed: 5.5, elev: 85 },
  shots: [], n: 0, paused: false,
};

const SLIDERS = {
  target: [['speed', 'Target exit speed', 3.5, 8.5, 0.05, 'm/s', 2], ['elev', 'Target exit angle', 60, 100, 0.5, 'deg', 1]],
  direct: [['rpm', 'Flywheel speed', 2000, 3500, 25, 'rpm', 0], ['ext', 'Extension angle (+ = flatter)', -13.6, 13.6, 0.2, 'deg', 1]],
};

function buildSliders() {
  for (const [grp, list] of Object.entries(SLIDERS)) {
    const box = $(`sliders-${grp}`); box.innerHTML = '';
    for (const [k, label, lo, hi, step, unit, fmt] of list) {
      const el = document.createElement('div'); el.className = 'slider'; el.dataset.key = k;
      el.innerHTML = `<span>${label}</span><span class="val"></span><input type="range" min="${lo}" max="${hi}" step="${step}">`;
      const inp = el.querySelector('input'); inp.value = app.s[k];
      const show = () => { el.querySelector('.val').textContent = `${(+inp.value).toFixed(fmt)} ${unit}`; };
      inp.addEventListener('input', () => { app.s[k] = +inp.value; show(); apply(); });
      show(); box.appendChild(el);
    }
  }
}

/** Push the panel's settings to the turret (TARGET: solve rpm + extension from the calibration). */
function apply() {
  const s = app.s, T = app.T;
  for (const b of document.querySelectorAll('[data-ball]')) b.classList.toggle('on', b.dataset.ball === s.ball);
  for (const b of document.querySelectorAll('[data-mode]')) b.classList.toggle('on', b.dataset.mode === s.mode);
  $('sliders-target').style.display = s.mode === 'target' ? '' : 'none';
  $('sliders-direct').style.display = s.mode === 'direct' ? '' : 'none';
  let rpm = s.rpm, ext = s.ext;
  const sol = $('solved');
  if (s.mode === 'target' && app.map) {
    const [a, r, ok] = app.map.solve(s.ball, s.speed, s.elev);
    ext = a; rpm = r;
    sol.textContent = ok ? `solved: ${rpm.toFixed(0)} rpm, extension ${ext.toFixed(1)} deg` : 'out of the calibrated range';
    sol.className = ok ? 'small mono' : 'small mono bad';
  } else if (app.map) {
    const [v, e] = app.map.predict(s.ball, ext, rpm);
    sol.textContent = `calibration predicts ${v.toFixed(2)} m/s at ${e.toFixed(1)} deg`; sol.className = 'small mono';
  } else sol.textContent = s.mode === 'target' ? 'no calibration yet: DIRECT only' : '';
  app.want = { rpm, ext };
  T.setBall(s.ball); T.setRpm(rpm); T.setAngle(ext);
}

function fire(burst) {
  const T = app.T, s = app.s;
  if (T.state !== 'idle') return;
  const types = burst ? (($('mixed').checked) ? ['pollen', 'nectar', 'pollen', 'nectar'] : Array(4).fill(s.ball)) : [s.ball];
  if (!burst) T.load(types); else T.load(types);
  app.view.clearTrails();
  for (const b of T.balls) app.view.startTrail(b.body, app.sim.time);
  app.burstTypes = types; app.burstK = 0;
  T.fire(burst);
}

function onShot(r) {
  app.n += 1; const T = app.T;
  r.n = app.n; r.ext = app.want.ext; r.rpm = app.want.rpm; app.shots.unshift(r);
  $('s-speed').textContent = r.jam ? 'JAM' : r.speed.toFixed(2);
  $('s-elev').textContent = r.jam ? '-' : r.elevation.toFixed(1);
  const s = app.s;
  if (!r.jam && s.mode === 'target') {
    const dv = 100 * (r.speed / s.speed - 1), de = r.elevation - s.elev;
    $('s-verr').textContent = `${dv >= 0 ? '+' : ''}${dv.toFixed(1)} %`; $('s-verr').className = 'err mono ' + (Math.abs(dv) < 2 ? 'good' : Math.abs(dv) < 5 ? 'warn' : 'bad');
    $('s-eerr').textContent = `${de >= 0 ? '+' : ''}${de.toFixed(1)} deg`; $('s-eerr').className = 'err mono ' + (Math.abs(de) < 1 ? 'good' : Math.abs(de) < 3 ? 'warn' : 'bad');
  } else { $('s-verr').textContent = ''; $('s-eerr').textContent = ''; }
  $('s-grid').innerHTML = r.jam ? `<div><span>ball</span>${r.type}</div><div><span>result</span>${r.taken ? 'taken, dud' : 'never reached the flywheel'}</div>` :
    [['sideways', `${r.azimuth.toFixed(1)} deg`], ['spin', `${(r.spin * 60 / 2 / Math.PI).toFixed(0)} rpm`], ['contact', `${r.contact_ms.toFixed(1)} ms`],
     ['feed to exit', `${(1000 * r.time_to_exit).toFixed(0)} ms`], ['wheel droop', `${(100 * r.droop).toFixed(1)} %`], ['peak force', `${r.peak_force_n.toFixed(0)} N`]]
      .map(([k, v]) => `<div><span>${k}</span>${v}</div>`).join('');
  const tb = $('shots').querySelector('tbody');
  tb.innerHTML = app.shots.slice(0, 40).map((x) => `<tr class="${x.type}"><td>${x.n}</td><td>${x.type}</td><td>${x.rpm.toFixed(0)}</td><td>${x.ext.toFixed(1)}</td>` +
    (x.jam ? '<td colspan="3" class="bad">jam</td>' : `<td>${x.speed.toFixed(2)}</td><td>${x.elevation.toFixed(1)}</td><td>${x.azimuth.toFixed(1)}</td>`) +
    `<td>${(1000 * x.t).toFixed(0) % 100000}</td></tr>`).join('');
}

function live() {
  const T = app.T, sim = app.sim;
  $('l-rpm').textContent = `${T.wheelRpm().toFixed(0)} / ${T.targetRpm().toFixed(0)} rpm`;
  $('l-gap').textContent = `${T.gapMm().toFixed(1)} mm (${T.gapMm() > 12.7 ? 'NECTAR' : 'POLLEN'})`;
  $('l-ext').textContent = `${T.armDeg().toFixed(1)} deg`;
  $('l-lift').textContent = `${T.liftArmDeg().toFixed(1)} deg (${T.placer.holding ? 'held' : 'released'})`;
  $('l-motor').textContent = `${T.current.toFixed(1)} A, battery ${T.v_batt.toFixed(2)} V`;
  const st = T.state === 'idle' ? (T.settled() ? 'READY' : 'SETTLING') : T.state.toUpperCase().replace('_', ' ');
  const b = $('l-state'); b.textContent = st; b.className = 'badge ' + (st === 'READY' ? 'ready' : T.state === 'idle' ? '' : 'firing');
}

async function main() {
  $('loadmsg').textContent = 'Loading MuJoCo...';
  const mj = await loadMujoco();
  $('loadmsg').textContent = 'Loading the turret...';
  const manifest = await (await fetch(BASE + 'manifest.json')).json();
  const xml = await (await fetch(BASE + 'scene.xml')).text();
  const files = {};
  await Promise.all(manifest.files.map(async (f) => { files[f] = new Uint8Array(await (await fetch(BASE + f)).arrayBuffer()); }));
  let cal = null;
  try { const r = await fetch(BASE + 'calibration.json'); if (r.ok) cal = await r.json(); } catch (e) { /* none yet */ }
  const sim = app.sim = new Sim(mj).loadScene(xml, files);
  const T = app.T = new Turret(sim, manifest);
  T.onShot = onShot;
  app.map = cal ? new CalMap(cal) : null;
  if (app.map) app.s.mode = 'target';
  const view = app.view = new View($('view'));
  view.build(sim, T.balls);
  const VIEWS = { turret: { az: -130, el: -24, dist: 0.85, lookat: [0, 0, 0.21] }, side: { az: 180, el: -4, dist: 0.7, lookat: [0, 0.02, 0.2] },
    cell: { az: -60, el: -12, dist: 0.42, lookat: [0, 0.03, 0.15] }, comb: { az: 160, el: -20, dist: 0.32, lookat: [0, -0.03, 0.21] },
    top: { az: -90, el: -85, dist: 0.8, lookat: [0, 0, 0.2] } };
  for (const [k, v] of Object.entries(VIEWS)) {
    const bt = document.createElement('button'); bt.textContent = k; bt.onclick = () => view.setView(v); $('views').appendChild(bt);
  }
  view.setView(VIEWS.turret);
  const strip = new Strip($('charts'), [{ label: 'flywheel', unit: 'rpm', color: '#e8a63c', fmt: (v) => v.toFixed(0), ref: true }, { label: 'motor', unit: 'A', color: '#78beff', fmt: (v) => v.toFixed(1) }]);
  buildSliders();
  for (const b of document.querySelectorAll('[data-ball]')) b.onclick = () => { app.s.ball = b.dataset.ball; apply(); };
  for (const b of document.querySelectorAll('[data-mode]')) b.onclick = () => { if (b.dataset.mode === 'target' && !app.map) return; app.s.mode = b.dataset.mode; apply(); };
  $('fire').onclick = () => fire(false); $('burst').onclick = () => fire(true);
  $('reset').onclick = () => { T.reset(); view.clearTrails(); apply(); };
  $('collision').onchange = (e) => view.setShowCollision(e.target.checked);
  addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' && e.target.type !== 'checkbox') return;
    if (e.code === 'Space') { e.preventDefault(); fire(false); } else if (e.key === 'b') fire(true); else if (e.key === 'r') $('reset').click();
    else if (e.key === '1') { app.s.ball = 'pollen'; apply(); } else if (e.key === '2') { app.s.ball = 'nectar'; apply(); }
    else if (e.key === 'm' && app.map) { app.s.mode = app.s.mode === 'target' ? 'direct' : 'target'; apply(); }
  });
  $('notes').innerHTML = `CONTACT BENCH: only the surfaces that touch the ball, each floating in place. The hood and extension are placed (never pushed by the ball); the lift arm is an ideal hold + the spring law; the turret is held at 0. Physics: MuJoCo ${manifest.mujoco || ''} with the shooter rig's contact laws (tread law on the flywheel and lift wheel, guide law on every printed ball surface, the arena's ball law, air drag + Magnus) and the vendor motor models. ` +
    `Balls are placed straight into the carousel's pockets (the intake is not part of this bench). ${cal ? '' : 'No calibration yet: DIRECT mode only.'}`;
  apply();
  $('loading').classList.add('hide');
  // ------------------------------------------------------------------ real-time loop
  let last = performance.now(), simAcc = 0, frames = 0, tFps = last, simT0 = sim.time;
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    const scale = $('slowmo').checked ? 0.1 : 1.0;
    simAcc += dt * scale;
    const t0 = performance.now(), n = Math.round(simAcc / sim.timestep);
    let k = 0;
    while (k < n && performance.now() - t0 < 14) { sim.step(10); k += 10; view.sampleTrails(sim); }
    simAcc -= k * sim.timestep; if (simAcc > 0.1) simAcc = 0;
    view.sync(sim); view.render();
    strip.push([T.wheelRpm(), T.current], [T.targetRpm()]);
    frames++;
    if (now - tFps > 500) {
      $('perf').textContent = `${(frames * 1000 / (now - tFps)).toFixed(0)} fps, sim x${((sim.time - simT0) / ((now - tFps) / 1000)).toFixed(2)}`;
      frames = 0; tFps = now; simT0 = sim.time; strip.draw(); live();
      $('status').textContent = `t ${sim.time.toFixed(1)} s   ${T.state}`;
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
}
main().catch((e) => { $('loadmsg').textContent = 'Error: ' + e.message; console.error(e); });
