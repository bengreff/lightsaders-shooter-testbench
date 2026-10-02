// The shooter on the bench, port of sim/mechanism/shooter/bench.py (ShooterBench + ShotTracker) on top of Sim.
//
// Per physics step, in bench.py's order: the hub's velocity loop (on the motor's encoder), the motor's electrical model
// (battery sag, back-EMF), the feeder's motor, the servo slew (angle servo with the vendor's deadband), the air law,
// then mj_step; after the step the shot trackers. A shot (entry stage): a parked ball is placed AT REST between the
// side rollers (on the arm, rotated with it), the rollers opened to that ball's size, the feeder run at duty feeder_u;
// the feeder brakes when the FLYWHEEL touches the ball. The exit is the ball's state at its LAST contact with the
// shooter. JAM: not taken by the tread within take_s, still touching after 1.0 s, or leaving under 2 m/s.
// Older exports (no feeder) inject at the hood's mouth or on the feed column, moving.

import { Hub, Motor } from './hub.js';
import { Feeder } from './feeder.js';
import { Air } from './air.js';
import * as DS from './design.js';

const deg = (r) => r * 180 / Math.PI, rad = (d) => d * Math.PI / 180;

/** Seeded uniform PRNG (mulberry32): the JS jitter is reproducible, but NOT numpy's stream. */
export function rng(seed) {
  let a = seed >>> 0;
  const next = () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  return { uniform: (lo, hi) => lo + (hi - lo) * next() };
}

/** Servo slew at the vendor's no-load speed toward the target; a servo with a deadband ignores a target change smaller
 *  than it (bench.py servos(): the angle servo, 0.6 deg at the servo = deadband/ratio at the hood). */
export class Servos {
  constructor(spec) { this.spec = spec; }
  onLoad(sim) {
    this.S = {};
    for (const [k, s] of Object.entries(this.spec)) {
      this.S[k] = { aid: sim.actuatorId(s.actuator), qadr: sim.qadr(s.joint), vadr: sim.vadr(s.joint), speed: s.speed,
        deadband: s.deadband || 0, tgt: 0, cmd: 0, held: undefined };
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
      s.cmd += Math.max(-mx, Math.min(mx, e));
      sim.ctrlv[s.aid] = s.cmd;
    }
  }
}

export class Shooter {
  /** manifest: a variant's manifest.json. opts: {seed, jitter (true: the Python bench's arrival jitter)}. */
  constructor(sim, manifest, opts = {}) {
    this.sim = sim; this.M = manifest; this.D = manifest.design;
    this.jitter = opts.jitter ?? false;
    this.rand = rng(opts.seed ?? 0);
    const C = manifest.controller;
    this.hub = new Hub(C.hub, C.wheels, !C.motor);
    this.motor = C.motor ? new Motor(C.motor, this.hub) : null;
    this.feeder = C.feeder ? new Feeder({ ...C.feeder, battery_v: C.motor?.battery_v ?? 12 }, this.motor) : null;
    this.servos = new Servos(C.servos);
    this.air = new Air(manifest.air, manifest.balls);
    this.trackers = new Set();
    this.onShot = null;                  // callback(result) when a tracker finishes
    this.tracker = { postStep: () => this._track() };
    this.hooks = [this.hub, this.motor, this.feeder, this.servos, this.air, this.tracker].filter(Boolean);
    for (const h of this.hooks) sim.addHook(h);
    this._init();
  }

  /** Take this shooter's hooks out of the sim (before loading another variant into the same Sim). */
  detach() { for (const h of this.hooks) this.sim.removeHook(h); }

  _init() {
    const sim = this.sim, M = this.M;
    this.balls = M.balls.map((b) => ({ ...b, body: sim.bodyId(b.name), geom: sim.geomId(b.name + '_geom'),
      qadr: sim.qadr(b.name + '_free'), vadr: sim.vadr(b.name + '_free') }));
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
    this.trackers.clear();
  }

  /** Back to t = 0 (mj_resetData) with the controllers re-initialised. */
  reset() { this.sim.reset(); this._init(); }

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
      && Math.abs(s.qpos[this.j_arm] - S.angle.tgt) < rad(0.25)
      && Math.abs(s.qvel[this.v_arm]) < rad(5.0)
      && Math.abs(s.qpos[this.j_gap] - S.gap.tgt) < 0.0003;
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
  parkAllFlown() {
    const X = this.sim.xpos;
    this.balls.forEach((b, k) => {
      const o = 3 * b.body;
      if (X[o] < 2.0 && (X[o + 2] < 0.06 || Math.abs(X[o + 1]) > 2.5)) this.park(b, k);
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
    const C = this.M.controller;
    if (C.feeder) return this.stage(ball);
    const h = this.D.height_m;
    const { pos, vdir } = C.inject === 'feed_column' ? DS.feedStart(this.D, ball.type)
      : DS.mouth(this.D, ball.type, this.armDeg(), this.sim.qpos[this.j_gap]);
    const j = this._jitter();
    const f = this.jitter ? this.rand.uniform(1 - C.inject_jitter.speed, 1 + C.inject_jitter.speed) : 1;
    const sp = this.inject_speed * f;
    this.place(ball, [0 + j[0], pos[0] + j[1], pos[1] + h + j[2]], [0, vdir[0] * sp, vdir[1] * sp]);
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
    B.feed_u = 0;
    const jam = (!this.taken) || last === null || B.sim.time - this.t0 >= S.max_s;
    const out = { type: this.bt, t: this.t0, jam, taken: this.taken, arm_deg: this.arm0, gap_mm: this.gap0,
      rpm_target: this.target_rpm, rpm_at_feed: this.rpm0, inject_overlap_mm: 1000 * this.deep, ball: this.ball.name, ...this.meta };
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
