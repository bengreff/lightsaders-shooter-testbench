// The entry stage's motor, port of shooter/bench.py feeder_step(): the same electrical model as the flywheel's motor,
// open-loop duty `u` while feeding, BRAKE (u = 0: shorted terminals, back-EMF only) when idle. One motor drives both
// side rollers through one gear pair: it sees their mean speed and each roller gets half its torque, signed so the
// roller's contact face drives the ball along the entry line. Uses the battery voltage the flywheel's Motor computed
// this step (so it must run after it).

export class Feeder {
  constructor(spec, motor) {
    this.spec = spec; this.motor = motor; this.u = 0; this.current = 0;
    this.zeroBand = 1e-6;                // rad/s: |om| below this counts as stopped (= shooter/bench.py FRICTION_BAND)
  }

  onLoad(sim) {
    this.R = this.spec.rollers.map((r) => ({ ...r, qs: sim.qadr(r.slide), vs: sim.vadr(r.slide), v: sim.vadr(r.joint),
      a: sim.actuatorId(r.actuator) }));
    this.u = 0; this.current = 0;
  }
  onReset(sim) { this.onLoad(sim); }

  preStep(sim) {
    const F = this.spec, qv = sim.qvel, R = this.R;
    let om = 0;
    for (const J of R) om += J.spin * qv[J.v];
    om /= R.length;
    const vb = this.motor ? this.motor.v_batt : F.battery_v;
    this.current = (this.u * vb - F.Ke * om) / F.R;
    const zb = this.zeroBand;
    const tq = F.Kt * this.current - F.T_fric * (om > zb ? 1 : om < -zb ? -1 : 0);
    for (const J of R) sim.ctrlv[J.a] = J.spin * tq / 2;
  }
}
