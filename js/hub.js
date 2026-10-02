// The FTC hub's velocity loop (RUN_USING_ENCODER), port of sim/mechanism/launcher/bench.py Bench.hub():
// every sample_s (10 ms) the encoder is read in ticks into a ring of the last `ring` (5) samples, the velocity is the
// ring's end-to-end slope (ticks/s), and the motor power is (F*target + P*err + I*sum(err)) / 32767 clipped to +/-1.
// Targets are in ticks per second. One wheel entry per manifest.controller.wheels.
// With an electrical motor (manifest.controller.motor) the encoder is the motor's joint (encoder_joint), wheel rpm =
// motor rpm * ratio, and the output u is the H-bridge duty the Motor hook turns into torque; without one the output
// goes straight to the wheel's actuator (the older model: a <general> actuator with the motor curve built in).

export class Hub {
  constructor(hub, wheels, direct = true) {
    this.sample_s = hub.sample_s; this.ringN = hub.ring;
    this.spec = wheels;
    this.direct = direct;              // false: an electrical Motor hook reads w.u
  }

  onLoad(sim) {
    this.W = this.spec.map((w) => ({
      ...w, ratio: w.ratio ?? 1, qadr: sim.qadr(w.encoder_joint ?? w.joint), vadr: sim.vadr(w.joint),
      aid: sim.actuatorId(w.actuator), target: w.target_rpm / (w.ratio ?? 1) / 60 * w.tpr, ring: [], vel: 0, integ: 0, u: 0,
    }));
    this.next_sample = 0;
  }

  onReset(sim) { this.onLoad(sim); }

  preStep(sim) {
    const t = sim.d.time;
    if (t + 1e-12 < this.next_sample) return;
    this.next_sample = t + this.sample_s;
    for (const w of this.W) {
      w.ring.push([t, w.spin * sim.qpos[w.qadr] * w.tpr / (2 * Math.PI)]);
      if (w.ring.length > this.ringN) w.ring.shift();
      if (w.ring.length > 1) {
        const a = w.ring[0], b = w.ring[w.ring.length - 1];
        w.vel = (b[1] - a[1]) / (b[0] - a[0]);
      }
      const err = w.target - w.vel;
      w.integ += err;
      w.u = Math.max(-1, Math.min(1, (w.F * w.target + w.P * err + w.I * w.integ) / 32767.0));
      if (this.direct) sim.ctrlv[w.aid] = w.u;
    }
  }

  /** Wheel rpm wanted (the loop runs on its encoder: motor rpm = wheel rpm / ratio). */
  setRpm(rpm) { for (const w of this.W) w.target = rpm / w.ratio / 60 * w.tpr; }
  targetRpm(i = 0) { const w = this.W[i]; return w.target * 60 / w.tpr * w.ratio; }
  /** The hub's own (sampled) velocity, as wheel rpm. */
  measuredRpm(i = 0) { const w = this.W[i]; return w.vel * 60 / w.tpr * w.ratio; }
  /** The true wheel speed now (rpm, positive in the shooting direction). */
  wheelRpm(sim, i = 0) { const w = this.W[i]; return w.spin * sim.qvel[w.vadr] * 60 / (2 * Math.PI); }
  ready(tol) { return this.W.every((w) => Math.abs(w.vel - w.target) <= tol * w.target); }
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
    const tq = M.Kt * this.current - M.T_fric * (om > 1e-6 ? 1 : om < -1e-6 ? -1 : 0);   // = bench.py FRICTION_BAND
    this.torque = tq;
    sim.ctrlv[this.aid] = M.spin * tq;
  }
}
