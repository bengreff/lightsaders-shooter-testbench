// Air on the balls: drag + Magnus, the law of sim/mechanism/models/air.py, applied as xfrc_applied before each step.
//
//   drag    F = -rho * blunt * pi r^2 * |v| v
//   Magnus  F = magnus/2 * rho * (4/3 pi r^3) * (w x v)
//
// rho, blunt, magnus come from manifest.air (constants.yaml via the exporter). v is the free joint's linear velocity
// (world), w its angular velocity (the free joint stores it in the BODY frame: rotated to world with the ball's
// quaternion from qpos, i.e. the pose this step's forward pass will use -- what the Python passive callback reads
// from xmat). The force goes on the ball's centre of mass; a ball's COM is its body origin, so xfrc_applied there
// equals the Python hook's qfrc_passive on the three linear dofs and leaves the angular dofs untouched.

export class Air {
  constructor(air, balls) {
    this.rho = air.rho; this.blunt = air.blunt; this.magnus = air.magnus;
    this.ballsSpec = balls;              // [{name, radius}, ...]
    this.enabled = air.rho > 0;
  }

  onLoad(sim) {
    this.B = this.ballsSpec.map((b) => {
      const body = sim.bodyId(b.name), j = sim.jointId(b.name + '_free'), r = b.radius;
      return {
        body, q: sim.jnt_qposadr[j], v: sim.jnt_dofadr[j],
        kd: this.rho * this.blunt * Math.PI * r * r,
        km: 0.5 * this.magnus * this.rho * 4.0 / 3.0 * Math.PI * r * r * r,
      };
    });
  }

  preStep(sim) {
    if (!this.enabled) return;
    const qp = sim.qpos, qv = sim.qvel, X = sim.xfrc;
    for (const b of this.B) {
      const vx = qv[b.v], vy = qv[b.v + 1], vz = qv[b.v + 2];
      // body-frame angular velocity -> world, by the (normalised) quaternion
      let qw = qp[b.q + 3], qx = qp[b.q + 4], qy = qp[b.q + 5], qz = qp[b.q + 6];
      const qn = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) || 1;
      qw /= qn; qx /= qn; qy /= qn; qz /= qn;
      const lx = qv[b.v + 3], ly = qv[b.v + 4], lz = qv[b.v + 5];
      const r00 = qw * qw + qx * qx - qy * qy - qz * qz, r01 = 2 * (qx * qy - qw * qz), r02 = 2 * (qx * qz + qw * qy);
      const r10 = 2 * (qx * qy + qw * qz), r11 = qw * qw - qx * qx + qy * qy - qz * qz, r12 = 2 * (qy * qz - qw * qx);
      const r20 = 2 * (qx * qz - qw * qy), r21 = 2 * (qy * qz + qw * qx), r22 = qw * qw - qx * qx - qy * qy + qz * qz;
      const wx = r00 * lx + r01 * ly + r02 * lz, wy = r10 * lx + r11 * ly + r12 * lz, wz = r20 * lx + r21 * ly + r22 * lz;
      const sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
      const o = 6 * b.body;
      X[o] = b.km * (wy * vz - wz * vy) - b.kd * sp * vx;
      X[o + 1] = b.km * (wz * vx - wx * vz) - b.kd * sp * vy;
      X[o + 2] = b.km * (wx * vy - wy * vx) - b.kd * sp * vz;
    }
  }
}
