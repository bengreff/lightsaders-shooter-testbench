// The shooter on the bench, port of sim/mechanism/shooter/bench.py (ShooterBench + ShotTracker) on top of Sim.
//
// Per physics step, in bench.py's order: the hub's velocity loop (on the motor's encoder), the motor's electrical model
// (battery sag, back-EMF), the feeder's motor, the servo slew (angle servo with the vendor's deadband), the air law,
// then mj_step; after the step the shot trackers. A shot (entry stage): a parked ball is placed AT REST between the
// side rollers (on the arm, rotated with it), the rollers opened to that ball's size, the feeder run at duty feeder_u;
// the feeder brakes when the FLYWHEEL touches the ball. The exit is the ball's state at its LAST contact with the
// shooter. JAM: not taken by the tread within take_s, still touching after 1.0 s, or leaving under 2 m/s.
// Older exports (no feeder) inject at the hood's mouth or on the feed column, moving.
// The Lightsaders v2 feed (manifest.controller.v10, shooter/v10.py V10Bench): the ball is seated in the carousel cell's
// bowl, the servo crank swings the lift arm's stop from retracted to engaged (V10Lift), the lift motor's electrical model
// drives the rubber lift wheel, which rolls the ball up the spine into the throat wheel and the flywheel.

import { Hub, Motor } from './hub.js';
import { Feeder } from './feeder.js';
import { Air } from './air.js';
import * as DS from './design.js';
import { NumpyRNG } from './nprandom.js';

const deg = (r) => r * 180 / Math.PI, rad = (d) => d * Math.PI / 180;
/** Quaternions (w, x, y, z), MuJoCo's order: product a b, and v rotated by q. */
const qmul = (a, b) => [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
const qrot = (q, v) => { const p = qmul(qmul(q, [0, v[0], v[1], v[2]]), [q[0], -q[1], -q[2], -q[3]]); return [p[1], p[2], p[3]]; };

/** bench.py BALL_SPREAD (older exports carry none in manifest.controller.ball_spread): per property (sigma, bound) as
 *  fractions of the nominal value; a truncated normal per shot. */
export const BALL_SPREAD = { diameter: [0.0053, 0.02], mass: [0.015, 0.045], stiffness: [0.10, 0.25], cor: [0.02, 0.05] };

/** bench.py trunc_normal: redraw until |x| <= bound. */
export function truncNormal(r, sigma, bound) {
  for (;;) { const x = r.normal(0.0, sigma); if (Math.abs(x) <= bound) return x; }
}

/** Seeded uniform PRNG (mulberry32): the JS jitter is reproducible, but NOT numpy's stream. */
export function rng(seed) {
  let a = seed >>> 0;
  const next = () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  return { uniform: (lo, hi) => lo + (hi - lo) * next() };
}

/** numpy's Generator.uniform(lo, hi) on default_rng(seed): lo + (hi - lo) * random(). */
export function numpyUniform(seed) {
  const r = new NumpyRNG(seed);
  return { uniform: (lo, hi) => lo + (hi - lo) * r.random() };
}

/** Servo slew at the vendor's no-load speed toward the target; a servo with a deadband ignores a target change smaller
 *  than it (bench.py servos(): the angle servo, 0.6 deg at the servo = deadband/ratio at the hood). */
export class Servos {
  constructor(spec) { this.spec = spec; }
  onLoad(sim) {
    this.S = {};
    for (const [k, s] of Object.entries(this.spec)) {
      this.S[k] = { aid: sim.actuatorId(s.actuator), qadr: sim.qadr(s.joint), vadr: sim.vadr(s.joint), speed: s.speed,
        deadband: s.deadband || 0, rigid: !!s.rigid, tgt: 0, cmd: 0, held: undefined };
    }
  }
  set(k, target) { this.S[k].tgt = target; }
  preStep(sim) {
    const dt = sim.timestep;
    for (const k in this.S) {
      const s = this.S[k];
      let tgt = s.tgt;
      if (s.deadband > 0) {
        if (s.held === undefined || Math.abs(tgt - s.held) > s.deadband) s.held = tgt;
        tgt = s.held;
      }
      const e = tgt - s.cmd, mx = s.speed * dt;
      const v = Math.max(-mx, Math.min(mx, e));
      s.cmd += v;
      sim.ctrlv[s.aid] = s.cmd;
      if (s.rigid) { sim.qpos[s.qadr] = s.cmd; sim.qvel[s.vadr] = v / dt; }   // held statically at the command: the ball never moves it
    }
  }
}

/** The robot's lift servo (bench.py lift_step): continuous rotation on the 6 V rail through its belt speed-up onto the
 *  lift wheel; the same straight torque-speed line as the motor, BRAKE when off. */
export class LiftServo {
  constructor(spec) { this.L = spec; this.u = 0; this.current = 0; }
  onLoad(sim) { this.v = sim.vadr(this.L.joint); this.aid = sim.actuatorId(this.L.actuator); }
  preStep(sim) {
    const L = this.L, om = sim.qvel[this.v] / L.up, band = 1e-6;
    const i = (this.u * L.V - L.Ke * om) / L.R; this.current = i;
    const tq = L.Kt * i - L.T_fric * (om > band ? 1 : om < -band ? -1 : 0);
    sim.ctrlv[this.aid] = tq / L.up;
  }
}

/** The v2 lift (shooter/v10.py V10Bench lift_step + set_arm_stop): the servo crank carries the arm's closing stop from
 *  retracted (q_ret) to engaged (0) as a cosine between its dead centres in t_engage; the flexure presses the arm onto
 *  that stop (the joint's spring, preloaded by `pre`). The lift motor (Yellow Jacket, 1:1 to the wheel) on the battery
 *  the flywheel motor sags: i = (u V - Ke w) / R, torque Kt i - T_fric sign(w) on the rotor joint. */
export class V10Lift {
  constructor(spec, shooter) { this.V = spec; this.B = shooter; this.u = 0; this.current = 0; this.engage_t0 = null; this.arm_stop = 0; this.press_n = 0; }
  onLoad(sim) {
    const V = this.V;
    this.jid = sim.jointId(V.arm_joint); this.j_larm = sim.qadr(V.arm_joint); this.v_larm = sim.vadr(V.arm_joint);
    this.j_lw = sim.qadr(V.wheel_joint); this.v_lw = sim.vadr(V.wheel_joint);
    this.j_rot = sim.qadr(V.motor.joint); this.v_rot = sim.vadr(V.motor.joint); this.aid = sim.actuatorId(V.motor.actuator);
    this.range0 = sim.m.jnt_range[2 * this.jid]; this.spring0 = sim.m.qpos_spring[this.j_larm];    // as compiled (engaged)
    // catch lift (9b): the motor's torque grounded (no reaction on the arm), a constant extra press down at the wheel, the
    // catch bracket drawn in / out
    this.gid = V.motor.ground_actuator ? sim.actuatorId(V.motor.ground_actuator) : -1;
    this.b_wheel = V.press_body ? sim.bodyId(V.press_body) : -1;
    this.j_catch = V.catch_joint ? sim.qadr(V.catch_joint) : -1; this.v_catch = V.catch_joint ? sim.vadr(V.catch_joint) : -1;
    this.u = 0; this.current = 0; this.engage_t0 = null; this.arm_stop = 0; this.caught = true;
  }
  /** set_arm_stop: the crank's link puts the arm's closing stop at q (rad); the flexure presses the arm onto it. */
  setArmStop(sim, q) {
    const m = sim.m; m.jnt_range[2 * this.jid] = q; m.qpos_spring[this.j_larm] = q - this.V.lift2.pre; this.arm_stop = q;
  }
  /** Back to the compiled stop (engaged), e.g. after a reset. */
  restore(sim) {
    const m = sim.m; m.jnt_range[2 * this.jid] = this.range0; m.qpos_spring[this.j_larm] = this.spring0;
    this.u = 0; this.current = 0; this.engage_t0 = null; this.arm_stop = 0; this.caught = true;
  }
  preStep(sim) {
    const V = this.V, L2 = V.lift2;
    if (L2 && L2.mode === 'catch' && !this.caught && sim.qpos[this.j_larm] > L2.q_ret + L2.rearm) {   // pushed above the catch: it engages
      this.setArmStop(sim, L2.q_ret); this.caught = true;
    }
    if (L2 && L2.mode === 'servo' && this.engage_t0 !== null) {       // the crank between its dead centres
      const f = Math.min(1.0, (sim.d.time - this.engage_t0) / L2.t_engage);
      this.setArmStop(sim, L2.q_ret * 0.5 * (1.0 + Math.cos(Math.PI * f)));
      if (f >= 1.0) this.engage_t0 = null;
    }
    const M = V.motor, om = sim.qvel[this.v_rot], band = 1e-6;          // = bench.py FRICTION_BAND
    const i = (this.u * this.B.v_batt - M.Ke * om) / M.R; this.current = i;
    const tq = M.Kt * i - M.T_fric * (om > band ? 1.0 : om < -band ? -1.0 : 0.0);
    sim.ctrlv[this.aid] = tq;
    if (this.gid >= 0) sim.ctrlv[this.gid] = tq;                      // grounded motor: the arm feels none of it
    if (this.b_wheel >= 0) sim.xfrc[6 * this.b_wheel + 2] = -this.press_n;
    if (this.j_catch >= 0) { sim.qpos[this.j_catch] = this.caught ? 0 : V.catch_travel; sim.qvel[this.v_catch] = 0; }
  }
}

export class Shooter {
  /** manifest: a variant's manifest.json. opts: {seed, jitter (true: the Python bench's arrival jitter),
   *  ballSpread (null | BALL_SPREAD keys to vary per shot: bench.py set_ball_spread)}. */
  constructor(sim, manifest, opts = {}) {
    this.sim = sim; this.M = manifest; this.D = manifest.design;
    this.jitter = opts.jitter ?? false;
    // arrival jitter: mulberry32 (not numpy's stream), except on the v2 feed, whose cell_start draws are numpy's
    // default_rng(seed).uniform, draw for draw (V10Bench.rng), so a seeded jittered run matches the Python bench shot for shot
    this.rand = manifest.controller.v10 ? numpyUniform(opts.seed ?? 0) : rng(opts.seed ?? 0);
    // ball-to-ball variation: numpy's default_rng(seed + 7919) stream, draw for draw with bench.py's spread_rng
    this.spreadRng = new NumpyRNG((opts.seed ?? 0) + 7919);
    this.ballSpread = null; this.lastBallDraw = null; this._ballNominal = null;
    this.setBallSpread(opts.ballSpread);
    const C = manifest.controller;
    this.hub = new Hub(C.hub, C.wheels, !C.motor);
    this.motor = C.motor ? new Motor(C.motor, this.hub) : null;
    this.feeder = C.feeder ? new Feeder({ ...C.feeder, battery_v: C.motor?.battery_v ?? 12 }, this.motor) : null;
    this.servos = new Servos(C.servos);
    this.lift = C.lift ? new LiftServo(C.lift) : C.v10 ? new V10Lift(C.v10, this) : null;
    this.air = new Air(manifest.air, manifest.balls);
    this.trackers = new Set();
    this.onShot = null;                  // callback(result) when a tracker finishes
    this.tracker = { postStep: () => this._track() };
    this.hooks = [this.hub, this.motor, this.feeder, this.lift, this.servos, this.air, this.tracker].filter(Boolean);
    for (const h of this.hooks) sim.addHook(h);
    this._init();
  }

  /** Take this shooter's hooks out of the sim (before loading another variant into the same Sim). */
  detach() {
    for (const h of this.hooks) this.sim.removeHook(h);
    if (this._scratch) { this._scratch.delete(); this._scratch = null; }
  }

  _init() {
    const sim = this.sim, M = this.M;
    this.balls = M.balls.map((b) => ({ ...b, body: sim.bodyId(b.name), geom: sim.geomId(b.name + '_geom'),
      qadr: sim.qadr(b.name + '_free'), vadr: sim.vadr(b.name + '_free') }));
    if (!this._ballNominal) this._captureNominal();
    const set = (k) => new Set((M.geom_sets[k] || []).map((g) => sim.geomId(g)));
    this.rig = set('rig'); this.wheelG = set('wheel');
    this.hoodG = new Set((M.geom_sets.hood || M.geom_sets.rig.filter((g) => g.startsWith('hood_s'))).map((g) => sim.geomId(g)));
    this.j_arm = sim.qadr('arm'); this.v_arm = sim.vadr('arm'); this.j_gap = sim.qadr('gap');
    // bench.py ShooterBench.__init__: angle 0, gap at the POLLEN dwell, the shell placed there
    const S = this.servos.S;
    S.angle.tgt = S.angle.cmd = 0; S.angle.held = undefined;
    S.gap.tgt = S.gap.cmd = DS.offset(this.D, 'pollen'); S.gap.held = undefined;
    sim.qpos[this.j_gap] = S.gap.cmd;
    sim.ctrlv[S.gap.aid] = S.gap.cmd;
    sim.forward();
    this.ball_type = 'pollen';
    this.next_ball = 0;
    this.inject_speed = this.D.inject_speed;
    this.feeder_u = this.M.controller.feeder?.u ?? 0;
    const cell = this.M.controller.cell;
    if (cell) {                                     // bench.py __init__: the robot (cell + lift) under the turret's axis
      this.b_robot = sim.m.body_mocapid[sim.bodyId(cell.robot)];
      this.j_lslide = sim.qadr(cell.slide); this.v_lslide = sim.vadr(cell.slide);
      sim.mocap_pos.set([cell.origin[0], cell.origin[1], cell.origin[2] + this.D.height_m], 3 * this.b_robot);
      if (this.lift) this.lift.u = 0;
      sim.forward();
    }
    const V10 = this.M.controller.v10;
    if (V10) {                                      // V10Bench.__init__: the robot under the turret, the spine's friction
      this.b_robot = sim.m.body_mocapid[sim.bodyId(V10.robot)];
      sim.mocap_pos.set([V10.origin[0], V10.origin[1], V10.origin[2] + this.D.height_m], 3 * this.b_robot);
      if (V10.spine_mu !== null && V10.spine_mu !== undefined) {
        const m = sim.m, p1 = m.pair_geom1, p2 = m.pair_geom2;
        for (const gn of V10.spine_geoms) {
          const gid = sim.geomId(gn);
          for (let i = 0; i < m.npair; i++) if (p1[i] === gid || p2[i] === gid) { m.pair_friction[5 * i] = V10.spine_mu; m.pair_friction[5 * i + 1] = V10.spine_mu; }
          m.geom_friction[3 * gid] = V10.spine_mu;
        }
      }
      sim.forward();
    }
    this.trackers.clear();
  }

  /** Back to t = 0 (mj_resetData) with the controllers re-initialised; every ball back to its nominal properties. */
  reset() {
    this.sim.reset();
    if (this.lift && this.lift.restore) this.lift.restore(this.sim);
    this._restoreNominal(); if (this.M.controller.v10?.mass_fix) this._setConst();
    this._init();
  }

  /** v12b+ mass_fix (V10Bench.draw_ball): re-derive the model's constants after a ball draw (mj_setConst) so the mass and
   *  inertia draw take effect; on a scratch MjData, because mj_setConst resets the data it is given to qpos0. */
  _setConst() {
    const s = this.sim;
    if (!this._scratch) this._scratch = new s.mj.MjData(s.m);
    s.mj.mj_setConst(s.m, this._scratch);
    s.views();                                      // a new MjData can grow the heap: re-fetch the live views
  }

  // ------------------------------------------------------------------ ball-to-ball variation (bench.py)
  /** Each ball's nominal model values (bench.py __init__ _ball_nominal), read once from the freshly loaded model. */
  _captureNominal() {
    const m = this.sim.m; this._ballNominal = new Map();
    const p1 = m.pair_geom1, p2 = m.pair_geom2;
    for (const b of this.balls) {
      const g = b.geom, bd = m.geom_bodyid[g], pairs = [];
      for (let i = 0; i < m.npair; i++) if (p1[i] === g || p2[i] === g) pairs.push(i);
      this._ballNominal.set(g, { bd, r: m.geom_size[3 * g], rb: m.geom_rbound[g], mass: m.body_mass[bd],
        inertia: Array.from(m.body_inertia.subarray(3 * bd, 3 * bd + 3)), solref: [m.geom_solref[2 * g], m.geom_solref[2 * g + 1]],
        pairs, pair_solref: pairs.map((i) => [m.pair_solref[2 * i], m.pair_solref[2 * i + 1]]) });
    }
  }

  _restoreNominal() { if (this._ballNominal) for (const b of this.balls) this._setBall(b, { diameter: 0, mass: 0, stiffness: 0, cor: 0 }); }

  /** bench.py set_ball_spread: null (nominal balls) | an iterable of BALL_SPREAD keys to vary (a fresh ball per shot). */
  setBallSpread(props) { const l = props ? Array.from(props) : []; this.ballSpread = l.length ? l : null; }

  /** bench.py draw_ball: this ball nominal, then (if ballSpread) a fresh draw in BALL_SPREAD's order. -> the draw. */
  drawBall(ball) {
    const SP = this.M.controller.ball_spread || BALL_SPREAD, dr = {};
    for (const k of Object.keys(SP))
      dr[k] = this.ballSpread && this.ballSpread.includes(k) ? truncNormal(this.spreadRng, SP[k][0], SP[k][1]) : 0.0;
    this._setBall(ball, dr);
    this.lastBallDraw = { ...dr };
    if (this.M.controller.v10?.mass_fix) this._setConst();
    return this.lastBallDraw;
  }

  /** The model values for a ball at draw dr (fractions), in bench.py draw_ball's arithmetic. */
  _setBall(ball, dr) {
    const m = this.sim.m, g = ball.geom, N = this._ballNominal.get(g), bd = N.bd;
    const fd = 1 + dr.diameter, fm = 1 + dr.mass, fk = 1 + dr.stiffness;
    m.geom_size[3 * g] = N.r * fd; m.geom_rbound[g] = N.rb * fd;
    m.body_mass[bd] = N.mass * fm;
    for (let i = 0; i < 3; i++) m.body_inertia[3 * bd + i] = N.inertia[i] * fm * fd * fd;
    const k0 = -N.solref[0], c0 = -N.solref[1];
    // COR -> damping: measured 2026-10-06, damping ~ COR^-2.24 (bench.py)
    const fc = Math.pow(1 + dr.cor, -2.24), c1 = c0 * fc;
    m.geom_solref[2 * g] = -k0 * fk; m.geom_solref[2 * g + 1] = -c1;
    N.pairs.forEach((i, j) => { m.pair_solref[2 * i] = N.pair_solref[j][0] * fk; m.pair_solref[2 * i + 1] = N.pair_solref[j][1] * fc; });
  }

  // ------------------------------------------------------------------ commands (bench.py)
  setRpm(rpm) { this.hub.setRpm(rpm); }
  targetRpm() { return this.hub.targetRpm(); }
  setAngle(arm_deg) { const lim = this.D.angle_travel_deg; this.servos.set('angle', rad(Math.max(-lim, Math.min(lim, arm_deg)))); }
  setBall(bt, squeeze_mm) { this.ball_type = bt; this.servos.set('gap', DS.offset(this.D, bt, squeeze_mm)); }
  armDeg() { return deg(this.sim.qpos[this.j_arm]); }
  armTargetDeg() { return deg(this.servos.S.angle.tgt); }
  gapMm() { return 1000 * this.sim.qpos[this.j_gap]; }
  gapTargetMm() { return 1000 * this.servos.S.gap.tgt; }
  wheelRpm() { return this.hub.wheelRpm(this.sim); }
  get current() { return this.motor ? this.motor.current : 0; }
  get v_batt() { return this.motor ? this.motor.v_batt : 0; }
  get feed_current() { return this.feeder ? this.feeder.current : 0; }
  get feed_u() { return this.feeder ? this.feeder.u : 0; }
  set feed_u(u) { if (this.feeder) this.feeder.u = u; }

  settled(tol) {
    tol = tol ?? this.D.gate_tol;
    const s = this.sim, S = this.servos.S;
    return this.hub.ready(tol)
      && Math.abs(s.qpos[this.j_arm] - S.angle.tgt) < rad(this.D.settle_arm_deg ?? 0.25)
      && Math.abs(s.qvel[this.v_arm]) < rad(5.0)
      && Math.abs(s.qpos[this.j_gap] - S.gap.tgt) < (this.D.settle_gap_mm ?? 0.3) / 1000;
  }

  waitReady(limit_s = 3.0, tol) {
    const t0 = this.sim.time;
    while (!this.settled(tol)) { this.sim.step(10); if (this.sim.time - t0 > limit_s) return false; }
    return true;
  }

  spinUp(rpm, limit_s) {
    limit_s = limit_s ?? this.M.controller.spin_up_limit_s ?? 3.0;
    this.setRpm(rpm);
    const t0 = this.sim.time;
    while (this.sim.time - t0 < 0.3 || !this.settled()) { this.sim.step(20); if (this.sim.time - t0 > limit_s) break; }
    return this.sim.time - t0;
  }

  /** Turn the drawn-only pinion and cam (mocap bodies) to follow the arm angle and the gap (render.py pose_mocaps).
   *  Visual only. Call once per frame before stepping. */
  poseMocaps() {
    const s = this.sim, mc = this.M.mocaps || {}, m = s.m, h = this.D.height_m;
    const arm = s.qpos[this.j_arm], gap = s.qpos[this.j_gap];
    const quatX = (a) => [Math.cos(a / 2), Math.sin(a / 2), 0, 0];
    const K = this.M.mocap_kin || {};
    if (K.v2lift) {   // the v2 lift's servo crank and its link, from the crank's stop angle (tools/v2_visuals.py table)
      const T = K.v2lift, q = this.lift ? this.lift.arm_stop : 0, Q = T.q, n = Q.length;
      let k = 0; while (k < n - 2 && q > Q[k + 1]) k++;
      const f = Math.min(1, Math.max(0, (q - Q[k]) / (Q[k + 1] - Q[k])));
      const rb = m.body_mocapid[s.bodyId(T.robot)], rp = s.mocap_pos.subarray(3 * rb, 3 * rb + 3), rq = Array.from(s.mocap_quat.subarray(4 * rb, 4 * rb + 4));
      for (const [name, tab] of [['v2_crank', T.crank], ['v2_link', T.link]]) {
        if (!tab || !s.names.body.map.has(name)) continue;
        const a = tab[k].map((x, i) => x + f * (tab[k + 1][i] - x));
        const t = qrot(rq, [a[1], a[2], a[3]]), b = m.body_mocapid[s.bodyId(name)];
        s.mocap_pos.set([rp[0] + t[0], rp[1] + t[1], rp[2] + t[2]], 3 * b);
        s.mocap_quat.set(qmul(rq, [Math.cos(a[0] / 2), Math.sin(a[0] / 2), 0, 0]), 4 * b);
      }
      return;
    }
    if (K.v9) {   // render.py v9_mocaps: pinion on the sliding hood, the slide's crank-slider (exact kinematics)
      const quatZ = (a) => [Math.cos(a / 2), 0, 0, Math.sin(a / 2)];
      const set = (name, pos, q) => { const b = m.body_mocapid[s.bodyId(name)]; s.mocap_pos.set(pos, 3 * b); s.mocap_quat.set(q, 4 * b); };
      if (mc.pinion) { const o = mc.pinion.origin; set('pinion', [o[0], o[1] - gap, o[2] + h], quatX(K.pinion_ratio * arm)); }
      if (mc.crank && mc.link) {
        const o = mc.crank.origin, r = K.crank_r, L = K.link_L, dy = (K.pin_y - gap) - o[1];
        const th = Math.acos(Math.min(1, Math.max(-1, (r * r + dy * dy - L * L) / (2 * r * dy))));
        set('crank', [o[0], o[1], o[2] + h], quatZ(th));
        const pc = [o[0] - r * Math.sin(th), o[1] + r * Math.cos(th), o[2] + h];
        set('link', pc, quatZ(Math.atan2(-r * Math.sin(th), dy - r * Math.cos(th))));
      }
      return;
    }
    if (mc.pinion) {
      const b = m.body_mocapid[s.bodyId('pinion')];
      s.mocap_quat.set(quatX(-this.D.angle_ratio * arm), 4 * b);
    }
    if (mc.cam) {
      const b = m.body_mocapid[s.bodyId('cam')], o = mc.cam.origin;
      const dP = DS.offset(this.D, 'pollen'), dN = DS.offset(this.D, 'nectar');
      const frac = Math.min(1, Math.max(0, (dN - gap) / (dN - dP)));
      const c = Math.cos(arm), sn = Math.sin(arm);
      s.mocap_pos.set([o[0], c * o[1] - sn * o[2], sn * o[1] + c * o[2] + h], 3 * b);
      s.mocap_quat.set(quatX(arm - rad(90) * frac), 4 * b);
    }
  }

  // ------------------------------------------------------------------ balls
  freeBall(bt) {
    const cand = this.balls.filter((b) => b.type === bt);
    const b = cand[this.next_ball % cand.length]; this.next_ball += 1;
    return b;
  }

  place(ball, pos, vel, angvel = [0, 0, 0]) {
    const q = this.sim.qpos, v = this.sim.qvel;
    q.set(pos, ball.qadr); q.set([1, 0, 0, 0], ball.qadr + 3);
    v.set(vel, ball.vadr); v.set(angvel, ball.vadr + 3);
  }

  park(ball, k) { const p = this.M.controller.park; this.place(ball, [p.x + p.dx * k, 0, p.z], [0, 0, 0]); }

  /** Park balls whose flight is over (on the floor, or far to the side) so a burst reuses them. */
  parkAllFlown() {   // bench.py park_all_flown: also a ball coming back DOWN onto the shooter (a near-vertical shot)
    const X = this.sim.xpos, V = this.sim.qvel, h = this.D.height_m;
    this.balls.forEach((b, k) => {
      const o = 3 * b.body;
      if (X[o] < 2.0 && (X[o + 2] < 0.06 || Math.abs(X[o + 1]) > 2.5 || (Math.abs(X[o + 1]) < 0.6 && X[o + 2] < h + 0.3 && V[b.vadr + 2] < -0.5))) this.park(b, k);
    });
  }

  /** Deepest penetration (m) of the ball into the shooter at its current pose (runs mj_forward, as bench.py). */
  overlap(ball) {
    const s = this.sim; s.forward();
    let deep = 0;
    s.forEachContact((i, g1, g2, dist) => {
      if ((g1 === ball.geom || g2 === ball.geom) && (this.rig.has(g1) || this.rig.has(g2))) deep = Math.max(deep, -dist);
    });
    return deep;
  }

  _jitter() {
    const J = this.M.controller.inject_jitter;
    return this.jitter ? J.pos.map((x) => this.rand.uniform(-1, 1) * x) : [0, 0, 0];
  }

  inject(ball) {
    this.drawBall(ball);
    const C = this.M.controller;
    if (C.feeder) return this.stage(ball);
    if (C.v10) return this.v10CellStart(ball);
    if (C.cell) return this.cellStart(ball);
    const h = this.D.height_m;
    const { pos, vdir } = C.inject === 'feed_column' ? DS.feedStart(this.D, ball.type)
      : DS.mouth(this.D, ball.type, this.armDeg(), this.sim.qpos[this.j_gap]);
    const j = this._jitter();
    if (this.jitter && this.D.flags && this.D.flags.bore_feed && this.D.flags.bore_play) {   // anywhere in the free disc
      const rr = Math.sqrt(this.rand.uniform(0, 1)) * DS.borePlay(this.D, ball.type), th = this.rand.uniform(0, 2 * Math.PI);
      j[0] = rr * Math.cos(th); j[1] = rr * Math.sin(th);
    }
    if (this.D.flags && this.D.flags.feed_front) {          // v9 (bench.py inject): on the bore's front wall, never inside it
      j[1] = -Math.abs(j[1]);
      let cx = j[0], cy = pos[0] + j[1] + this.D.feed_offset_mm / 1000;
      const lim = this.D.bore_d_mm / 2000 - DS.ballD(this.D, ball.type) / 2 - 0.0005, n = Math.hypot(cx, cy);
      if (n > lim) { cx *= lim / n; cy *= lim / n; j[0] = cx; j[1] = cy - this.D.feed_offset_mm / 1000 - pos[0]; }
    }
    const f = this.jitter ? this.rand.uniform(1 - C.inject_jitter.speed, 1 + C.inject_jitter.speed) : 1;
    const sp = this.inject_speed * f;
    this.place(ball, [0 + j[0], pos[0] + j[1], pos[1] + h + j[2]], [0, vdir[0] * sp, vdir[1] * sp]);
    return this.overlap(ball);
  }

  /** v9 boost (bench.py cell_start): the ball in the robot's carousel cell, pressed on its far wall by the sprung lift
   *  wheel; the robot at a random angle under the turret; small indexing roll; lift servo at 90-100 %. */
  cellStart(ball) {
    const C = this.M.controller.cell, s = this.sim, h = this.D.height_m, jit = this.jitter;
    const r = s.m.geom_size[3 * ball.geom];          // the drawn ball's radius (bench.py cell_start reads the model)
    this.balls.forEach((bb, k) => {                 // a ball left in the cell (a stalled shot) is cleared (bench.py cell_start)
      if (bb === ball) return;
      const X = s.xpos, q = 3 * bb.body;
      if (Math.hypot(X[q] - C.origin[0], X[q + 1] - C.origin[1], X[q + 2] - C.origin[2] - h) < 0.30) this.park(bb, k);
    });
    const U = (lo, hi) => this.rand.uniform(lo, hi);
    const phi = jit ? U(0, 2 * Math.PI) : 0;
    const off = jit ? U(0, 0.001) : 0;
    let xb = -(C.r - r) + off, yb = jit ? U(-0.002, 0.002) : 0;
    const lim = C.r - r - 0.0005, n = Math.hypot(xb, yb);
    if (n > lim) { xb *= lim / n; yb *= lim / n; }
    const zb = C.floor + r + 0.0005, c = Math.cos(phi), sn = Math.sin(phi), o = C.origin;
    s.mocap_quat.set([Math.cos(phi / 2), 0, 0, Math.sin(phi / 2)], 4 * this.b_robot);
    s.qpos[this.j_lslide] = Math.max(0, xb + r - C.rim); s.qvel[this.v_lslide] = 0;
    const vel = jit ? [U(-0.03, 0.03), U(-0.03, 0.03), 0] : [0, 0, 0];
    const w = jit ? [U(-3, 3), U(-3, 3), U(-3, 3)] : [0, 0, 0];
    this.place(ball, [o[0] + c * xb - sn * yb, o[1] + sn * xb + c * yb, zb + h], vel, w);
    if (this.lift) this.lift.u = jit ? U(0.9, 1.0) : 1.0;
    return this.overlap(ball);
  }

  /** The v2 feed (shooter/v10.py V10Bench.cell_start, the same draws in the same order): the robot at a random angle
   *  under the turret, the ball seated in the cell's bowl (a quarter of the 1 mm / +-2 mm offsets, the bowl centres it),
   *  a small indexing roll, the arm retracted and its crank starting to engage, the lift motor at 90-100 % duty. Without
   *  jitter: robot angle 0, the ball centred, still, full duty. */
  v10CellStart(ball) {
    const V = this.M.controller.v10, s = this.sim, h = this.D.height_m, jit = this.jitter, L = this.lift;
    const r = s.m.geom_size[3 * ball.geom];          // the drawn ball's radius
    const o0 = [V.origin[0], V.origin[1], V.origin[2] + h];
    this.balls.forEach((bb, k) => {                 // a ball left in the robot (a stalled shot) is parked
      if (bb === ball) return;
      const X = s.xpos, q = 3 * bb.body;
      if (Math.hypot(X[q] - o0[0], X[q + 1] - o0[1], X[q + 2] - o0[2]) < 0.30 && X[q + 2] < h + 0.1) this.park(bb, k);
    });
    const U = (lo, hi) => (jit ? this.rand.uniform(lo, hi) : (lo !== -hi ? 0.5 * (lo + hi) : 0.0));
    let phi = jit ? this.rand.uniform(0.0, 2 * Math.PI) : 0.0;
    if (this.phiFixed !== undefined && this.phiFixed !== null) phi = this.phiFixed;
    const off = jit ? U(0.0, 0.001) : 0.0;
    let yb = V.y_spine + r + off;                  // off the spine (robot y, from the turret axis)
    let xb = U(-0.002, 0.002);                     // across
    let zb = V.floor + r + 0.0005;
    const c = Math.cos(phi), sn = Math.sin(phi);
    s.mocap_quat.set([Math.cos(phi / 2), 0.0, 0.0, Math.sin(phi / 2)], 4 * this.b_robot);
    if (V.bowl) {                                   // seated in the cell's bowl (it centres the ball)
      const bw = V.bowl;
      yb = (bw.y - V.yT_mm) / 1000.0 + 0.25 * (off - 0.0005);
      xb *= 0.25;
      zb = (bw.bottom_z - V.zfly_mm) / 1000.0 + r + 0.0003;
    }
    const L2 = V.lift2; let ca = 1.0;
    if (L2) {                                       // the arm retracted; its crank engages it now
      ca = -L2.sign;
      if (L2.mode === 'servo') { L.setArmStop(s, L2.q_ret); L.engage_t0 = s.time; }
      if (L2.mode === 'catch') { L.setArmStop(s, L2.q_low); L.caught = false; }   // the catch retracts: the arm falls onto the ball
      s.qpos[L.j_larm] = L2.q_ret; s.qvel[L.v_larm] = 0.0;
    }
    s.qvel[L.v_lw] = 0.0; s.qvel[L.v_rot] = 0.0;
    const T = V.tie || [1.0, ca];                                     // the belts' tie (rotor = T0 wheel + T1 arm)
    s.qpos[L.j_rot] = T[0] * s.qpos[L.j_lw] + T[1] * s.qpos[L.j_larm];
    const o = V.origin;
    const pos = [o[0] + c * xb - sn * yb, o[1] + sn * xb + c * yb, zb + h];
    const vel = [U(-0.03, 0.03), U(-0.03, 0.03), 0.0], w = [U(-3, 3), U(-3, 3), U(-3, 3)];
    this.place(ball, pos, vel, w);
    L.u = jit ? U(0.9, 1.0) : 1.0;
    return this.overlap(ball);
  }

  /** The entry stage (bench.py stage()): the ball at rest between the rollers, which open to its size; feeder on. */
  stage(ball) {
    const F = this.M.controller.feeder, st = F.stage[ball.type], h = this.D.height_m, s = this.sim;
    let S = st.S;
    if (F.on_arm) {
      const a = s.qpos[this.j_arm], c = Math.cos(a), sn = Math.sin(a);
      S = [c * S[0] - sn * S[1], sn * S[0] + c * S[1]];
    }
    for (const J of this.feeder.R) { s.qpos[J.qs] = st.q; s.qvel[J.vs] = 0; }
    const j = this._jitter();
    this.place(ball, [0 + j[0], S[0] + j[1], S[1] + h + j[2]], [0, 0, 0]);
    this.feed_u = this.feeder_u;
    return this.overlap(ball);
  }

  /** (touching the shooter, total normal force N, touching the flywheel) for one ball, from this step's contacts. */
  touching(ball) {
    let any = false, wh = false, tot = 0;
    const s = this.sim, rig = this.rig, wg = this.wheelG, g = ball.geom;
    s.forEachContact((i, g1, g2, dist, excl, efc) => {
      if ((g1 === g || g2 === g) && (rig.has(g1) || rig.has(g2))) {
        any = true;
        wh = wh || wg.has(g1) || wg.has(g2);
        tot += Math.abs(s.contactNormal(i, efc));
      }
    });
    return [any, tot, wh];
  }

  /** Start a shot now (non-blocking): the tracker is advanced after every step; onShot(result) when done. */
  shoot(bt, target) {
    const tr = new ShotTracker(this, bt ?? this.ball_type, target);
    this.trackers.add(tr);
    return tr;
  }

  _track() {
    if (!this.trackers.size) return;
    for (const tr of this.trackers) {
      tr.update();
      if (tr.done) { this.trackers.delete(tr); if (this.onShot) this.onShot(tr.result); }
    }
  }

  /** Blocking shot (bench.py fire): inject now and step until its exit. */
  fire(bt, target) {
    const tr = this.shoot(bt, target);
    while (!tr.done) this.sim.step();
    return tr.result;
  }
}

export class ShotTracker {
  constructor(B, bt, target) {
    this.B = B; this.bt = bt; this.target = target;
    this.ball = B.freeBall(bt);
    this.deep = B.inject(this.ball);
    this.draw = B.ballSpread ? B.lastBallDraw : null;     // this ball's variation draw (fractions), if varied
    const s = B.sim;
    this.t0 = s.time; this.last = null; this.first = null; this.taken = false; this.peak = 0;
    this.rpm0 = B.wheelRpm(); this.rpm_min = this.rpm0;
    this.target_rpm = B.targetRpm(); this.arm0 = B.armDeg(); this.gap0 = B.gapMm();
    this.done = false; this.result = null;
    this.depth = { tread: 0, hood: 0 }; this.force = { tread: 0, hood: 0 };
    this.i_peak = 0; this.v_min = B.v_batt; this.feed_i_peak = 0;
    this.meta = {};                      // caller's notes, copied into the result
  }

  /** squeeze (deepest overlap) and summed normal force into the TREAD and the HOOD shell this step (bench.py _contacts). */
  _contacts() {
    const B = this.B, s = B.sim, g = this.ball.geom, wg = B.wheelG, hg = B.hoodG;
    let ft = 0, fh = 0;
    const D = this.depth;
    s.forEachContact((i, g1, g2, dist, excl, efc) => {
      if (g1 !== g && g2 !== g) return;
      const o = g1 === g ? g2 : g1;
      const kind = wg.has(o) ? 'tread' : hg.has(o) ? 'hood' : null;
      if (!kind) return;
      const f = Math.abs(s.contactNormal(i, efc));
      D[kind] = Math.max(D[kind], -dist);
      if (kind === 'tread') ft += f; else fh += f;
    });
    this.force.tread = Math.max(this.force.tread, ft); this.force.hood = Math.max(this.force.hood, fh);
  }

  update() {
    const B = this.B, s = B.sim, ball = this.ball, S = B.M.controller.shot;
    const t = s.time;
    const [on, fN, wh] = B.touching(ball);
    this.taken = this.taken || wh;
    if (on) this._contacts();
    this.i_peak = Math.max(this.i_peak, Math.abs(B.current)); this.v_min = Math.min(this.v_min, B.v_batt);
    this.feed_i_peak = Math.max(this.feed_i_peak, Math.abs(B.feed_current));
    if (B.feeder && B.feed_u && wh) B.feed_u = 0;          // the flywheel has it: the feeder brakes
    this.rpm_min = Math.min(this.rpm_min, B.wheelRpm());
    if (on) {
      if (this.first === null) this.first = t;
      this.peak = Math.max(this.peak, fN);
      const o = 3 * ball.body, v = ball.vadr, R = s.xmat.subarray(9 * ball.body, 9 * ball.body + 9), q = s.qvel;
      const wl = [q[v + 3], q[v + 4], q[v + 5]];
      this.last = [t, [s.xpos[o], s.xpos[o + 1], s.xpos[o + 2]], [q[v], q[v + 1], q[v + 2]],
        [R[0] * wl[0] + R[1] * wl[1] + R[2] * wl[2], R[3] * wl[0] + R[4] * wl[1] + R[5] * wl[2], R[6] * wl[0] + R[7] * wl[1] + R[8] * wl[2]]];
    }
    if ((!this.taken && t - this.t0 > S.take_s) || t - this.t0 >= S.max_s ||
        (this.last !== null && this.taken && !on && t - this.last[0] > S.gap_s)) this.finish();
  }

  finish() {
    const B = this.B, last = this.last, S = B.M.controller.shot;
    B.feed_u = 0; if (B.lift) B.lift.u = 0;
    const jam = (!this.taken) || last === null || B.sim.time - this.t0 >= S.max_s;
    const out = { type: this.bt, t: this.t0, jam, taken: this.taken, arm_deg: this.arm0, gap_mm: this.gap0,
      rpm_target: this.target_rpm, rpm_at_feed: this.rpm0, inject_overlap_mm: 1000 * this.deep, ball: this.ball.name, ...this.meta };
    if (this.draw) out.ball_draw = this.draw;
    if (!jam) {
      const [t, p, v, w] = last;
      const sp = Math.hypot(v[0], v[1], v[2]);
      if (sp < S.min_speed) { out.jam = true; out.dud = true; }
      const mass = (B.M.ball_mass || {})[this.bt] ?? 0;
      const rw = B.M.controller.wheels[0].radius;
      Object.assign(out, { speed: sp, elevation: deg(Math.atan2(v[2], v[1])), azimuth: deg(Math.atan2(v[0], v[1])),
        spin: w[0], contact_ms: 1000 * (t - this.first), peak_force_n: this.peak,
        droop: 1 - this.rpm_min / Math.max(this.rpm0, 1e-9), release: p, vel: v, angvel: w, t_exit: t,
        squeeze_tread_mm: 1000 * this.depth.tread, squeeze_hood_mm: 1000 * this.depth.hood,
        force_tread_n: this.force.tread, force_hood_n: this.force.hood,
        motor_current_peak_a: this.i_peak, battery_min_v: this.v_min, feeder_current_peak_a: this.feed_i_peak,
        energy_j: 0.5 * mass * sp * sp,
        slip: 1 - sp / Math.max(1e-6, 0.5 * B.wheelRpm() * 2 * Math.PI / 60 * rw) });
      if (this.target) {
        out.target = { ...this.target };
        out.speed_err = sp / this.target.speed - 1;
        out.elev_err = out.elevation - this.target.elevation;
      }
    }
    this.done = true; this.result = out;
  }
}
