// The shooter's shared geometry, port of sim/mechanism/shooter/design.py (offset, mid_axis, mouth). Every number comes
// from manifest.design (the exporter writes the completed design and its derived values).

const rad = (x) => x * Math.PI / 180;

export function ballD(D, bt) { return bt === 'pollen' ? D.D_POLLEN : D.D_NECTAR; }

/** Radial shell offset (m) along the slide radial that gives squeeze_mm (default: the design's setting for bt). */
export function offset(D, bt, squeeze_mm) {
  const s = (squeeze_mm === undefined || squeeze_mm === null ? D[`squeeze_${bt}_mm`] : squeeze_mm) / 1000;
  return (D.R_WHEEL + ballD(D, bt) - s) - D.hood_radius;
}

export function midAxis(D) {
  const a = rad(D.lip_mid_deg + D.wrap_deg * D.slide_frac);
  return [Math.cos(a), Math.sin(a)];
}

/** Injection pose (launcher y, z) and unit direction for a ball at the hood's mouth (design.py mouth()). */
export function mouth(D, bt, arm_deg = 0, off = null) {
  const Rc = D.hood_radius;
  const a = rad(D.lip_mid_deg + D.wrap_deg + D.flare_deg * 0.85 + arm_deg);
  const o = off === null ? offset(D, bt) : off;
  const u = midAxis(D);
  let r_s = Rc + D.flare_mm / 1000 * 0.5 * (1 - Math.cos(Math.PI * 0.85));
  const c = [Math.cos(a), Math.sin(a)];
  const ang_u = Math.atan2(u[1], u[0]) + rad(arm_deg);
  r_s += o * Math.cos(a - ang_u);
  const rc = 0.5 * (D.R_WHEEL + r_s);
  return { pos: [rc * c[0], rc * c[1]], vdir: [Math.sin(a), -Math.cos(a)] };
}

/** Feed-column entry (design.py feed_start): below the shooter on the turret axis (y = -feed_offset), low enough to
 *  clear tread and shell at every hood angle, moving straight up. */
export function feedStart(D, bt) {
  const r = ballD(D, bt) / 2;
  const rmax = D.hood_radius + (D.flare_mm + D.gap_cam_mm + D.hood_thickness_mm) / 1000;
  return { pos: [-D.feed_offset_mm / 1000, -(rmax + r + 0.008)], vdir: [0, 1] };
}

export function lipTangentDeg(D, arm_deg) { return D.lip_mid_deg + arm_deg - 90; }
