// The FTC hub's velocity loop (RUN_USING_ENCODER), port of sim/mechanism/launcher/bench.py Bench.hub():
// every sample_s (10 ms) the encoder is read in ticks into a ring of the last `ring` (5) samples, the velocity is the
// ring's end-to-end slope (ticks/s), and the motor power is (F*target + P*err + I*sum(err)) / 32767 clipped to +/-1.
// Targets are in ticks per second. One wheel entry per manifest.controller.wheels.
// With an electrical motor (manifest.controller.motor) the encoder is the motor's joint (encoder_joint), wheel rpm =
// motor rpm * ratio, and the output u is the H-bridge duty the Motor hook turns into torque; without one the output
// goes straight to the wheel's actuator (the older model: a <general> actuator with the motor curve built in).

export class Hub {
  constructor(hub, wheels, direct = true) {
    this.sample_s = hub.sample_s; this.ringN = hub.ring; this.quantize = !!hub.quantize;
    this.spec = wheels;
    this.direct = direct;              // false: an electrical Motor hook reads w.u
  }

  onLoad(sim) {
    this.W = this.spec.map((w) => ({
      ...w, ratio: w.ratio ?? 1, qadr: sim.qadr(w.encoder_joint ?? w.joint), vadr: sim.vadr(w.joint),
      aid: sim.actuatorId(w.actuator), target: Hub.ticks(w, w.target_rpm / (w.ratio ?? 1) / 60 * w.tpr), ring: [], vel: 0, integ: 0, u: 0,
    }));
    this.next_sample = 0;
  }

  /** The target the hub gets (ticks/s). int_ticks (the team's robot code, exact): DcMotorEx.setVelocity(ticks/s) as the FTC SDK sends
   *  it to the hub (LynxDcMotorController.setMotorVelocity: Math.round, clipped to +-32767 whole ticks per second). */
  static ticks(w, x) { return w.int_ticks ? Math.max(-32767, Math.min(32767, Math.round(x))) : x; }

  /** Switch every wheel to another loop in place (the page's loop control; controller.loops[name].wheel): its law and gains; the loop's
   *  own state (integral, observer) starts fresh, the hub's encoder ring is kept. */
  setLaw(spec) {
    for (const w of this.W) {
      const rpm = this.targetRpm(this.W.indexOf(w));
      for (const k of ['law', 'P', 'I', 'D', 'F', 'model', 'int_ticks']) { if (k in spec) w[k] = spec[k]; else delete w[k]; }
      w.integ = 0; w.vprev = undefined; w.th = undefined; w.boosting = false; w.okSince = null;
      if (w.ring.length > 1) { const a = w.ring[0], b = w.ring[w.ring.length - 1]; w.vel = (b[1] - a[1]) / (b[0] - a[0]); }
      w.ring = w.law === 'pro' ? [] : w.ring;
      w.target = Hub.ticks(w, rpm / w.ratio / 60 * w.tpr);
    }
  }

  onReset(sim) { this.onLoad(sim); }

  preStep(sim) {
    const t = sim.d.time;
    if (t + 1e-12 < this.next_sample) return;
    this.next_sample = t + this.sample_s; this.t = t;
    for (const w of this.W) {
      const ticks = w.spin * sim.qpos[w.qadr] * w.tpr / (2 * Math.PI);
      if (w.law === 'pro') { this.pro(w, this.quantize ? Math.floor(ticks) : ticks, t, sim); continue; }
      w.ring.push([t, this.quantize ? Math.floor(ticks) : ticks]);          // quantize: whole encoder ticks, as the hub counts them
      if (w.ring.length > this.ringN) w.ring.shift();
      if (w.ring.length > 1) {
        const a = w.ring[0], b = w.ring[w.ring.length - 1];
        w.vel = (b[1] - a[1]) / (b[0] - a[0]);
      }
      const err = w.target - w.vel;
      // D on the measurement (no kick on a target change); the integral only accumulates while the output is not
      // saturated (anti-windup). D = 0 and I = 0 (the older designs) give the plain loop unchanged.
      const dv = w.vprev === undefined ? 0 : w.vel - w.vprev; w.vprev = w.vel;
      const raw = (w.F * w.target + w.P * err + w.I * (w.integ + err) - (w.D || 0) * dv) / 32767.0;
      if (Math.abs(raw) < 1 || Math.sign(err) !== Math.sign(raw)) w.integ += err;
      w.u = Math.max(-1, Math.min(1, (w.F * w.target + w.P * err + w.I * w.integ - (w.D || 0) * dv) / 32767.0));
      if (this.direct) sim.ctrlv[w.aid] = w.u;
    }
  }

  /** law "pro" (FLY_LAW 2, ours; v3 2026-11-17 after Ben saw v1 overshoot after shots and bounce about the target): what the
   *  robot's own loop can run every sample_s from whole encoder ticks, the hub's battery voltage and its own feed command, on a
   *  deliberately imperfect copy of the motor model (w.model: Kt, Ke, R, friction, drag, J off by a few %):
   *  1. observer (alpha-beta, critically damped from one factor): the model predicts angle and speed over the sample from the last
   *     duty LESS the integral's volts (the integral is exactly what the model misses at steady speed, so the prediction has no bias),
   *     the count (its tick's middle) corrects them; smooth gains (lam) while steady, so the 28-tick count's beat against the 10 ms
   *     loop cannot move the reading; fast gains (lamf) for fast_s after a feed, at full power, or when the count leaves the
   *     prediction by trig ticks;
   *  2. volts = feedforward (back-EMF + friction + drag at the target) + Kp x error + the integral, which learns only while steady
   *     and within iband of the target (a shot never winds it up); full duty while more than `boost` below; duty = volts / battery;
   *  3. ready = |w^ - w*| <= tol w* continuously for hold_s. */
  pro(w, ticks, t, sim) {
    const M = w.model, dt = this.sample_s, Vb = this.motor ? this.motor.v_batt : M.Vn, q = 2 * Math.PI / w.tpr;
    const z = ticks * q + (this.quantize ? 0.5 * q : 0);                          // the count's best angle: its tick's middle
    const wt = w.target * 2 * Math.PI / w.tpr;                                   // target motor speed (rad/s)
    if (w.th === undefined) { w.th = z; w.om = 0; w.ui = 0; w.u = 0; w.okSince = null; w.fastUntil = -1; }
    // predict with the model, its error removed: the integral ui IS the volts the model misses at steady speed
    const um = w.u - w.ui / Vb;
    const a = (M.Kt * (um * Vb - M.Ke * w.om) / M.R - M.Tf * Math.sign(w.om) - M.b * w.om) / M.J;
    const thp = w.th + w.om * dt + 0.5 * a * dt * dt, omp = w.om + a * dt;
    const r = z - thp;
    if (Math.abs(r) > M.trig * q || w.u >= 1 || (this.fedAt !== undefined && t - this.fedAt < M.fast_s)) w.fastUntil = t + M.fast_s;
    const L = t < w.fastUntil ? M.lamf : M.lam, al = 1 - L * L, be = (1 - L) * (1 - L);
    w.th = thp + al * r; w.om = omp + be * r / dt;
    w.vel = w.om * w.tpr / (2 * Math.PI);                                         // the estimate, as ticks/s (measuredRpm / ready read it)
    // recovery (v4, 2026-11-17 "get recovery even faster"): full duty from the feed for kick_s (the ball is coming: do not wait to see
    // the drop), and once below `boost`, full duty until the speed predicted lead_s ahead (at full duty, from the model) reaches
    // within bend of the target: time-optimal, no slow proportional tail, and no overshoot from the loop's own lag
    const aFull = (M.Kt * ((1 - w.ui / Vb) * Vb - M.Ke * w.om) / M.R - M.Tf * Math.sign(w.om) - M.b * w.om) / M.J;
    if (w.om < (1 - M.boost) * wt) w.boosting = true;
    if (w.boosting && w.om + aFull * (M.lead_s ?? 0) >= (1 - (M.bend ?? M.boost)) * wt) w.boosting = false;
    const kick = (M.kick_s ?? 0) > 0 && this.fedAt !== undefined && t - this.fedAt < M.kick_s;
    const e = wt - w.om, boostOn = w.boosting || kick;
    if (!boostOn && t >= w.fastUntil && Math.abs(e) <= M.iband * Math.abs(wt)) w.ui += M.Ki * e * dt;   // learns only when steady
    w.ui = Math.max(-0.25 * Vb, Math.min(0.25 * Vb, w.ui));
    w.dl = w.ui;                                                                  // (traces: the learned volts)
    const volts = M.Ke * wt + M.R / M.Kt * (M.Tf * Math.sign(wt) + M.b * wt) + M.Kp * e + w.ui;
    w.u = boostOn ? 1 : Math.max(-1, Math.min(1, volts / Vb));
    const ok = Math.abs(e) <= M.tol * Math.abs(wt);
    w.okSince = ok ? (w.okSince ?? t) : null;
    if (this.direct) sim.ctrlv[w.aid] = w.u;
  }

  /** A ball was fed at time t (the robot commands the feed, so its loop knows): law pro switches to its fast gains. */
  fed(t) { this.fedAt = t; }

  /** Wheel rpm wanted (the loop runs on its encoder: motor rpm = wheel rpm / ratio). */
  setRpm(rpm) { for (const w of this.W) w.target = Hub.ticks(w, rpm / w.ratio / 60 * w.tpr); }
  targetRpm(i = 0) { const w = this.W[i]; return w.target * 60 / w.tpr * w.ratio; }
  /** The hub's own (sampled) velocity, as wheel rpm. */
  measuredRpm(i = 0) { const w = this.W[i]; return w.vel * 60 / w.tpr * w.ratio; }
  /** The true wheel speed now (rpm, positive in the shooting direction). */
  wheelRpm(sim, i = 0) { const w = this.W[i]; return w.spin * sim.qvel[w.vadr] * 60 / (2 * Math.PI); }
  ready(tol) {
    return this.W.every((w) => w.law === 'pro' ? (w.okSince !== null && w.okSince !== undefined && this.t - w.okSince >= w.model.hold_s - 1e-9
                                                    && Math.abs(w.vel - w.target) <= tol * w.target)
                                               : Math.abs(w.vel - w.target) <= tol * w.target);
  }
}

/** The motor's electrical model (shooter/bench.py motor_step), every physics step: battery sag, back-EMF, winding
 *  resistance, internal friction; writes the torque to the motor joint's plain torque actuator. */
export class Motor {
  constructor(spec, hub) { this.spec = spec; this.hub = hub; }
  onLoad(sim) {
    const s = this.spec;
    this.vadr = sim.vadr(s.joint); this.aid = sim.actuatorId(s.actuator);
    this.current = 0; this.v_batt = s.battery_v; this.torque = 0;
  }
  onReset(sim) { this.onLoad(sim); }
  preStep(sim) {
    const M = this.spec, u = this.hub.W[0].u;
    const om = M.spin * sim.qvel[this.vadr];
    this.v_batt = M.battery_v - M.battery_r * Math.abs(this.current);
    this.current = (u * this.v_batt - M.Ke * om) / M.R;
    const sg = om > 1e-6 ? 1 : om < -1e-6 ? -1 : 0;                                   // = bench.py FRICTION_BAND
    const tq = M.Kt * this.current - M.T_fric * sg - (M.drag_l1 || 0) * om;           // drag_l1: the fitted viscous part (FLY_LAW 1)
    this.torque = tq;
    sim.ctrlv[this.aid] = M.spin * tq;
  }
}
