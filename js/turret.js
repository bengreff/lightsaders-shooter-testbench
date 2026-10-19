// The Lightsaders turret CONTACT bench (sim/mechanism/turret/bench.py builds the scene and manifest): the ONE
// implementation of its controller, used by the page and by the node tools (tools/turret_*.mjs).
// Only the ball-contact surfaces exist; every carrier is abstracted:
//   hood + extension  MOCAP bodies: placed (hood slide, extension angle about C), never pushed by the ball. They move
//                     only between shots, at the toggle / extension servo's no-load speed.
//   lift arm          an invisible hinge: an ideal hold keeps it at rest; released, the spring law + the lift motor's
//                     reaction drop the wheel onto the ball (what the latch did)
//
// What runs every physics step (hooks, in order):
//   Hub + Motor   the flywheel's velocity loop on the motor encoder and the motor's electrical model (hub.js, unchanged)
//   DCMotor       the lift motor (electrical model, duty u; u = 0 shorts the windings: a brake)
//   Placer        the hood / extension poses (mocap) and the arm hold
//   Sequencer     the feed: release the arm once the cell's ball is seated, the wheel lifts the ball; hold the arm
//                 again once the ball is taken; in a burst the carousel indexes 90 deg and the next ball is fired
//   Air           drag + Magnus on the balls (air.js, unchanged)
//   tracker       each shot's ShotTracker
import { Hub, Motor } from './hub.js';
import { Air } from './air.js';

const rad = (d) => d * Math.PI / 180, deg = (r) => r * 180 / Math.PI;

class Placer {
  constructor(H, hold) { this.H = H; this.hold = hold; this.slide = 0; this.ext = 0; this.slideT = 0; this.extT = 0; this.holding = true; }
  onLoad(sim) {
    const mid = (n) => sim.m.body_mocapid[sim.bodyId(n)];
    this.mh = mid('hood'); this.me = mid('ext'); this.qa = sim.qadr('larm'); this.va = sim.vadr('larm'); this.aid = sim.actuatorId('arm_hold');
    this.place(sim);
  }
  at() { return this.slide === this.slideT && this.ext === this.extT; }
  place(sim) {
    const P = sim.mocap_pos, Q = sim.mocap_quat, C = this.H.C;
    P[3 * this.mh] = 0; P[3 * this.mh + 1] = -this.slide; P[3 * this.mh + 2] = 0;
    P[3 * this.me] = C[0]; P[3 * this.me + 1] = C[1] - this.slide; P[3 * this.me + 2] = C[2];
    Q[4 * this.me] = Math.cos(this.ext / 2); Q[4 * this.me + 1] = -Math.sin(this.ext / 2); Q[4 * this.me + 2] = 0; Q[4 * this.me + 3] = 0;
  }
  preStep(sim) {
    const dt = sim.timestep, H = this.H;
    const mv = (x, t, v) => x + Math.max(-v * dt, Math.min(v * dt, t - x));
    if (!this.at()) { this.slide = mv(this.slide, this.slideT, H.slide_speed); this.ext = mv(this.ext, this.extT, H.ext_speed); this.place(sim); }
    const h = this.hold, q = sim.qpos[this.qa], w = sim.qvel[this.va];
    sim.ctrlv[this.aid] = this.holding ? Math.max(-h.limit, Math.min(h.limit, -h.kp * q - h.kd * w)) : 0;
  }
}

class DCMotor {
  /** spec.react_actuator: the same torque also on the arm (the motor sits on the pivot and drives the wheel through a
   *  belt, so the arm + wheel feel the motor's torque about the pivot; the wheel joint's actuator alone would cancel it). */
  constructor(spec) { this.M = spec; this.u = 0; this.current = 0; this.v_batt = spec.battery_v; }
  onLoad(sim) {
    this.vadr = sim.vadr(this.M.joint); this.aid = sim.actuatorId(this.M.actuator);
    this.rid = this.M.react_actuator ? sim.actuatorId(this.M.react_actuator) : -1;
  }
  preStep(sim) {
    const M = this.M, g = M.ratio || 1, om = M.spin * sim.qvel[this.vadr] / g;          // ratio: wheel speed / motor output speed
    this.v_batt = M.battery_v - M.battery_r * Math.abs(this.current);
    this.current = (this.u * this.v_batt - M.Ke * om) / M.R;
    const tq = M.spin * (M.Kt * this.current - M.T_fric * Math.sign(om)) / g;
    sim.ctrlv[this.aid] = tq;
    if (this.rid >= 0) sim.ctrlv[this.rid] = M.react_sign * tq;
  }
}

export class Turret {
  /** sim: a Sim with the scene loaded; manifest: the scene's manifest.json. */
  constructor(sim, manifest, opts = {}) {
    this.sim = sim; this.M = manifest; this.C = manifest.controller; this.G = manifest.geom;
    const C = this.C;
    this.hub = new Hub(C.hub, C.wheels, false);
    this.motor = new Motor(C.motor, this.hub);
    this.lift = new DCMotor(C.lift);
    this.placer = new Placer(C.hood, C.arm_hold);
    this.air = new Air(manifest.air, manifest.balls);
    this.trackers = new Set(); this.onShot = null; this.onEvent = null;
    this.seq = { preStep: () => this._sequence() };
    this.tracker = { postStep: () => this._track() };
    this.hooks = [this.hub, this.motor, this.lift, this.placer, this.seq, this.air, this.tracker];
    for (const h of this.hooks) sim.addHook(h);
    this._init();
  }

  detach() { for (const h of this.hooks) this.sim.removeHook(h); }

  _init() {
    const s = this.sim, M = this.M;
    this.balls = M.balls.map((b) => ({ ...b, body: s.bodyId(b.name), geom: s.geomId(b.name + '_geom'), qadr: s.qadr(b.name + '_free'), vadr: s.vadr(b.name + '_free') }));
    const set = (k) => new Set((M.geom_sets[k] || []).map((g) => s.geomId(g)));
    this.treadG = set('tread'); this.liftG = set('lift_tread'); this.hoodG = set('hood');
    this.rig = new Set([...set('guide'), ...this.treadG, ...this.liftG]);
    this.j = {}; for (const n of ['carousel', 'larm', 'fly']) this.j[n] = s.qadr(n);
    this.a_carousel = s.actuatorId('carousel_servo');
    this.carouselTarget = 0; s.ctrlv[this.a_carousel] = 0;
    const P = this.placer; P.slide = P.slideT = 0; P.ext = P.extT = 0; P.holding = true; P.place(s);
    this.ball_type = 'pollen';
    this.state = 'idle'; this.queue = []; this.lift.u = 0; this.trackers.clear();
    this.parkAll();
    s.forward();
  }

  reset() { this.sim.reset(); this._init(); }

  // ------------------------------------------------------------------ commands
  setRpm(rpm) { this.hub.setRpm(rpm); }
  targetRpm() { return this.hub.targetRpm(); }
  wheelRpm() { return this.hub.wheelRpm(this.sim); }
  /** POLLEN or NECTAR: the hood is placed forward (POLLEN) or slid back (NECTAR). */
  setBall(bt) { this.ball_type = bt; this.placer.slideT = bt === 'nectar' ? this.C.hood.slide : 0; }
  /** Extension angle, deg about the hood's centre (+ = flatter shots). */
  setAngle(d) { const T = this.C.hood.ext_travel_deg; this.placer.extT = rad(Math.max(-T, Math.min(T, d))); }
  armDeg() { return deg(this.placer.ext); }
  gapMm() { return 1000 * this.placer.slide; }
  liftArmDeg() { return deg(this.sim.qpos[this.j.larm]); }
  get current() { return this.motor.current; }
  get v_batt() { return this.motor.v_batt; }

  settled(tol = 0.02) {
    return this.hub.ready(tol) && this.placer.at();
  }

  spinUp(rpm, limit_s = 3.0) {
    this.setRpm(rpm); const t0 = this.sim.time;
    while (this.sim.time - t0 < 0.3 || !this.settled()) { this.sim.step(20); if (this.sim.time - t0 > limit_s) break; }
  }

  // ------------------------------------------------------------------ balls
  place(ball, p, v = [0, 0, 0]) {
    const s = this.sim, q = ball.qadr, d = ball.vadr;
    s.qpos[q] = p[0]; s.qpos[q + 1] = p[1]; s.qpos[q + 2] = p[2]; s.qpos[q + 3] = 1; s.qpos[q + 4] = 0; s.qpos[q + 5] = 0; s.qpos[q + 6] = 0;
    for (let i = 0; i < 6; i++) s.qvel[d + i] = i < 3 ? v[i] : 0;
  }
  parkAll() { const P = this.C.park; this.balls.forEach((b, k) => this.place(b, [P.x + P.dx * (k % 4), -0.25 + 0.11 * Math.floor(k / 4), b.radius + 0.001])); }
  ballPos(b) { const o = 3 * b.body, s = this.sim; return [s.xpos[o], s.xpos[o + 1], s.xpos[o + 2]]; }
  /** Pocket k's centre (world, m) at the carousel's present angle. */
  pocket(k) {
    const c = this.C.cell.carousel_c, p = this.C.cell.pockets[k], a = this.sim.qpos[this.j.carousel];
    const dx = p[0] - c[0], dy = p[1] - c[1];
    return [c[0] + Math.cos(a) * dx - Math.sin(a) * dy, c[1] + Math.sin(a) * dx + Math.cos(a) * dy];
  }
  /** Load balls into the pockets, the first into the cell (front pocket) and on round the carousel in firing order. */
  load(types) {
    this.parkAll();
    const used = new Set(), z0 = this.C.cell.centre[2];
    const order = [0, 1, 2, 3];                             // the carousel indexes +90 deg (CCW): pocket 1 comes to the front next
    types.forEach((bt, i) => {
      const b = this.balls.find((x) => x.type === bt && !used.has(x)); used.add(b);
      const p = this.pocket(order[i % 4]); this.place(b, [p[0], p[1], z0 + b.radius + 0.0005]);
    });
    this.sim.forward();
  }
  /** The ball sitting in the cell (front pocket), or null. */
  cellBall() {
    for (const b of this.balls) {
      const p = this.ballPos(b);
      if (Math.hypot(p[0], p[1]) < 0.02 && p[2] < 0.16) return b;
    }
    return null;
  }
  seated(b, tol = 0.004) {
    const p = this.ballPos(b), s = this.sim, v = b.vadr;
    return Math.hypot(p[0], p[1]) < tol && Math.hypot(s.qvel[v], s.qvel[v + 1], s.qvel[v + 2]) < 0.05;
  }

  // ------------------------------------------------------------------ the feed
  /** Fire the ball in the cell (non-blocking); burst: keep indexing and firing until the cell comes up empty. */
  fire(burst = false) { this.burst = burst; if (this.state === 'idle') this._go('wait_seat'); }
  _go(st) { this.state = st; this.t_state = this.sim.time; if (this.onEvent) this.onEvent(st, this.sim.time); }
  _sequence() {
    const s = this.sim, t = s.time, C = this.C;
    switch (this.state) {
      case 'wait_seat': {                             // the next ball must be sitting still in the cell, the wheel up to speed
        const b = this.cellBall();
        if (b && this.seated(b) && this.settled() && this.armAtRest()) {
          this.firing = b; this.placer.holding = false; this.lift.u = 1.0;
          this.trackers.add(this.tr = new ShotTracker(this, b)); this._go('lift');
        } else if (!b && t - this.t_state > 0.6) this._go('idle');
        break;
      }
      case 'lift':                                    // arm released: the spring drops it, the wheel flicks the ball up
        if (this.tr.taken || this.tr.done || t - this.t_state > 0.6) {
          this.placer.holding = true; this.lift.u = 0; this._go('reset');      // the hold brings the arm back
        }
        break;
      case 'reset':
        if (!this.trackers.size && this.armAtRest()) {
          if (this.burst) { this.carouselTarget += Math.PI / 2; s.ctrlv[this.a_carousel] = this.carouselTarget; this._go('index'); }
          else this._go('idle');
        } else if (t - this.t_state > 1.5) this._go('idle');
        break;
      case 'index':
        if (Math.abs(s.qpos[this.j.carousel] - this.carouselTarget) < rad(0.6) && t - this.t_state > 0.05) this._go('wait_seat');
        break;
    }
  }
  armAtRest() { return this.placer.holding && Math.abs(this.sim.qpos[this.j.larm]) < rad(2.5); }
  _track() {
    for (const tr of this.trackers) {
      tr.update();
      if (tr.done) { this.trackers.delete(tr); if (this.onShot) this.onShot(tr.result); }
    }
  }

  touching(ball) {
    let any = false, wh = false, tot = 0;
    const s = this.sim, g = ball.geom;
    s.forEachContact((i, g1, g2, dist, excl, efc) => {
      if (g1 !== g && g2 !== g) return;
      const o = g1 === g ? g2 : g1;
      if (!this.rig.has(o)) return;
      any = true; wh = wh || this.treadG.has(o); tot += Math.abs(s.contactNormal(i, efc));
    });
    return [any, tot, wh];
  }
}

export class ShotTracker {
  constructor(B, ball) {
    this.B = B; this.ball = ball; this.bt = ball.type;
    const s = B.sim;
    this.t0 = s.time; this.first = null; this.last = null; this.taken = false; this.peak = 0;
    this.rpm0 = B.wheelRpm(); this.rpm_min = this.rpm0; this.target_rpm = B.targetRpm(); this.arm0 = B.armDeg(); this.gap0 = B.gapMm();
    this.turret0 = 0; this.done = false; this.result = null; this.squeeze = 0; this.i_peak = 0;
  }
  update() {
    const B = this.B, s = B.sim, S = B.C.shot, t = s.time, b = this.ball;
    const [on, fN, wh] = B.touching(b);
    this.taken = this.taken || wh;
    this.rpm_min = Math.min(this.rpm_min, B.wheelRpm()); this.i_peak = Math.max(this.i_peak, Math.abs(B.current));
    if (on && this.taken) {
      if (this.first === null) this.first = t;
      this.peak = Math.max(this.peak, fN);
      const o = 3 * b.body, v = b.vadr, q = s.qvel, R = s.xmat.subarray(9 * b.body, 9 * b.body + 9);
      const wl = [q[v + 3], q[v + 4], q[v + 5]];
      this.last = [t, [s.xpos[o], s.xpos[o + 1], s.xpos[o + 2]], [q[v], q[v + 1], q[v + 2]],
        [R[0] * wl[0] + R[1] * wl[1] + R[2] * wl[2], R[3] * wl[0] + R[4] * wl[1] + R[5] * wl[2], R[6] * wl[0] + R[7] * wl[1] + R[8] * wl[2]]];
    }
    if ((!this.taken && t - this.t0 > S.take_s) || t - this.t0 >= S.max_s || (this.last && this.taken && !on && t - this.last[0] > S.gap_s)) this.finish();
  }
  finish() {
    const B = this.B, S = B.C.shot, last = this.last;
    const jam = !this.taken || !last || B.sim.time - this.t0 >= S.max_s;
    const out = { type: this.bt, t: this.t0, jam, taken: this.taken, arm_deg: this.arm0, gap_mm: this.gap0, rpm_target: this.target_rpm,
      rpm_at_feed: this.rpm0, ball: this.ball.name, turret_deg: deg(this.turret0) };
    if (!jam) {
      const [t, p, v, w] = last, ps = this.turret0;
      const fwd = -Math.sin(ps) * v[0] + Math.cos(ps) * v[1], side = Math.cos(ps) * v[0] + Math.sin(ps) * v[1];
      const sp = Math.hypot(v[0], v[1], v[2]);
      if (sp < S.min_speed) { out.jam = true; out.dud = true; }
      Object.assign(out, { speed: sp, elevation: deg(Math.atan2(v[2], fwd)), azimuth: deg(Math.atan2(side, fwd)),
        spin: Math.cos(ps) * w[0] + Math.sin(ps) * w[1], time_to_exit: t - this.t0, contact_ms: 1000 * (t - this.first),
        peak_force_n: this.peak, droop: 1 - this.rpm_min / Math.max(this.rpm0, 1e-9), release: p, vel: v, motor_current_peak_a: this.i_peak });
    }
    this.done = true; this.result = out;
  }
}
