// The calibration map, port of shooter/calibrate.py Map: exit (speed, elevation) per ball as a function of (hood arm
// angle deg, wheel rpm), and its inverse (solve: dense grid then Newton steps, as Python).
//
// Two surfaces:
//   'spline'  the SAME bicubic B-splines Python fitted (scipy RectBivariateSpline, with its smoothing), evaluated from
//             the knots/coefficients the exporter writes into calibration.json -> identical predictions to Python
//   'interp'  natural bicubic interpolation through the grid means (tensor product of natural cubic splines): used for
//             a grid measured in the browser (Recalibrate), where there is no scipy fit. NOT identical to Python's
//             smoothing spline on the same data: on v4_final's grid, within 0.28 % / 0.10 deg below 4000 rpm, up to
//             1.3 % between the 4000 and 4400 rpm columns where the wheel saturates (README).
// Plus measureGrid(): calibrate.py measure() in JS, for the in-browser recalibration.

export const RPMS = [1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3700, 4000, 4400];
export const ARMS_STEP = 4.0;

// ------------------------------------------------------------------ B-spline (FITPACK bispev semantics)
function span(t, k, n, x) {                 // knot interval l with t[l] <= x < t[l+1], k <= l < n - k - 1
  let l = k;
  while (l < n - k - 2 && x >= t[l + 1]) l++;
  return l;
}
function basis(t, k, l, x, out) {           // de Boor-Cox: the k+1 non-zero basis functions at x
  out[0] = 1;
  const left = new Float64Array(k + 1), right = new Float64Array(k + 1);
  for (let j = 1; j <= k; j++) {
    left[j] = x - t[l + 1 - j]; right[j] = t[l + j] - x;
    let saved = 0;
    for (let r = 0; r < j; r++) {
      const tmp = out[r] / (right[r + 1] + left[j - r]);
      out[r] = saved + right[r + 1] * tmp;
      saved = left[j - r] * tmp;
    }
    out[j] = saved;
  }
}
export function bispline(S) {
  const { tx, ty, c } = S, kx = S.kx ?? 3, ky = S.ky ?? 3;
  const nx = tx.length, ny = ty.length, my = ny - ky - 1;
  const bx = new Float64Array(kx + 1), by = new Float64Array(ky + 1);
  return (x, y) => {
    x = Math.min(Math.max(x, tx[kx]), tx[nx - kx - 1]);       // fpbisp clamps to the base interval
    y = Math.min(Math.max(y, ty[ky]), ty[ny - ky - 1]);
    const lx = span(tx, kx, nx, x), ly = span(ty, ky, ny, y);
    basis(tx, kx, lx, x, bx); basis(ty, ky, ly, y, by);
    let s = 0;
    for (let i = 0; i <= kx; i++) {
      const row = (lx - kx + i) * my;
      let r = 0;
      for (let j = 0; j <= ky; j++) r += c[row + ly - ky + j] * by[j];
      s += bx[i] * r;
    }
    return s;
  };
}

// ------------------------------------------------------------------ natural bicubic interpolation
function naturalSecond(x, y) {              // second derivatives of the natural cubic spline through (x, y)
  const n = x.length, M = new Float64Array(n), u = new Float64Array(n);
  for (let i = 1; i < n - 1; i++) {
    const sig = (x[i] - x[i - 1]) / (x[i + 1] - x[i - 1]), p = sig * M[i - 1] + 2;
    M[i] = (sig - 1) / p;
    u[i] = (6 * ((y[i + 1] - y[i]) / (x[i + 1] - x[i]) - (y[i] - y[i - 1]) / (x[i] - x[i - 1])) / (x[i + 1] - x[i - 1]) - sig * u[i - 1]) / p;
  }
  M[n - 1] = 0;
  for (let k = n - 2; k >= 0; k--) M[k] = M[k] * M[k + 1] + u[k];
  return M;
}
function splint(x, y, M, v) {
  const n = x.length;
  v = Math.min(Math.max(v, x[0]), x[n - 1]);
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) { const k = (hi + lo) >> 1; if (x[k] > v) hi = k; else lo = k; }
  const h = x[hi] - x[lo], a = (x[hi] - v) / h, b = (v - x[lo]) / h;
  return a * y[lo] + b * y[hi] + ((a * a * a - a) * M[lo] + (b * b * b - b) * M[hi]) * h * h / 6;
}
export function bicubicInterp(arms, rpms, Z) {
  const rows = Z.map((row) => ({ y: row, M: naturalSecond(rpms, row) }));
  return (a, r) => {
    const col = rows.map((R) => splint(rpms, R.y, R.M, r));
    return splint(arms, col, naturalSecond(arms, col), a);
  };
}

// ------------------------------------------------------------------ the map
export class CalMap {
  /** cal: calibration.json (from the exporter: grid + spline_v/spline_e per ball) or a browser-measured grid. */
  constructor(cal, mode) {
    this.cal = cal; this.T = +cal.travel;
    this.rpms = cal.rpms || RPMS;
    this.mode = mode || (cal.pollen.spline_v ? 'spline' : 'interp');
    this.f = {};
    for (const bt of ['pollen', 'nectar']) {
      const c = cal[bt], g = c.grid;
      this.f[bt] = this.mode === 'spline'
        ? [bispline(c.spline_v), bispline(c.spline_e)]
        : [bicubicInterp(g.arms, g.rpms, g.v), bicubicInterp(g.arms, g.rpms, g.e)];
    }
  }

  predict(bt, arm, rpm) { const [fv, fe] = this.f[bt]; return [fv(arm, rpm), fe(arm, rpm)]; }

  /** (arm deg, rpm, ok) giving (speed, elev): dense 81 x 113 grid, then 8 Newton steps (calibrate.py Map.solve). */
  solve(bt, speed, elev) {
    const T = this.T, R0 = this.rpms[0], R1 = this.rpms[this.rpms.length - 1];
    let best = Infinity, x0 = 0, x1 = R0;
    for (let i = 0; i < 113; i++) {                      // meshgrid(linspace(-T,T,81), linspace(R0,R1,113)): rows = rpm
      const r = R0 + (R1 - R0) * i / 112;
      for (let j = 0; j < 81; j++) {
        const a = -T + 2 * T * j / 80;
        const [v, e] = this.predict(bt, a, r);
        const err = ((v - speed) / speed * 100) ** 2 + (e - elev) ** 2;
        if (err < best) { best = err; x0 = a; x1 = r; }
      }
    }
    const h = [0.05, 2.0];
    for (let it = 0; it < 8; it++) {
      const [v0, e0] = this.predict(bt, x0, x1);
      const [va, ea] = this.predict(bt, x0 + h[0], x1), [vr, er] = this.predict(bt, x0, x1 + h[1]);
      const J00 = (va - v0) / h[0], J01 = (vr - v0) / h[1], J10 = (ea - e0) / h[0], J11 = (er - e0) / h[1];
      const det = J00 * J11 - J01 * J10;
      if (!det || !isFinite(det)) break;
      const b0 = v0 - speed, b1 = e0 - elev;
      x0 -= (J11 * b0 - J01 * b1) / det; x1 -= (-J10 * b0 + J00 * b1) / det;
      x0 = Math.min(Math.max(x0, -T), T); x1 = Math.min(Math.max(x1, R0), R1);
    }
    const [v0, e0] = this.predict(bt, x0, x1);
    return [x0, x1, Math.abs(v0 / speed - 1) < 0.005 && Math.abs(e0 - elev) < 0.25];
  }
}

// ------------------------------------------------------------------ measuring in the browser (calibrate.py measure)
/** Fire each ball over the grid (arm -T..T step 4 x RPMS, `per` shots per point, snake order), settled, with the
 *  arrival jitter on. shooter: a Shooter on its own Sim (the caller's live scene is not touched). onProgress(i, n).
 *  Async: yields to the event loop every shot so a page or worker stays responsive. -> calibration object ('interp'). */
export async function measureGrid(shooter, onProgress = () => {}, per = 2, rpms = RPMS, shouldStop = () => false) {
  const B = shooter, T = B.D.angle_travel_deg;
  const arms = []; for (let k = 0; -T + k * ARMS_STEP <= T + 1e-9; k++) arms.push(-T + k * ARMS_STEP);   // np.arange(-T, T, 4)
  const n = 2 * rpms.length * arms.length * per; let i = 0;
  const rows = [];
  B.setBall('pollen'); B.setAngle(arms[0]); B.spinUp(rpms[0]);
  for (const bt of ['pollen', 'nectar']) {
    B.setBall(bt);
    for (let ri = 0; ri < rpms.length; ri++) {
      B.setRpm(rpms[ri]);
      for (const a of (ri % 2 === 0 ? arms : [...arms].reverse())) {
        B.setAngle(a);
        for (let k = 0; k < per; k++) {
          if (shouldStop()) return null;
          B.waitReady(); const s = B.fire(bt); B.parkAllFlown();
          rows.push({ type: bt, arm: a, rpm: rpms[ri], jam: s.jam, speed: s.speed, elevation: s.elevation });
          onProgress(++i, n);
          await new Promise((r) => setTimeout(r, 0));
        }
      }
    }
  }
  return fitGrid(rows, T, rpms);
}

/** Grid means per ball (calibrate.py fit's grid; jams left out). */
export function fitGrid(rows, T, rpms = RPMS) {
  const cal = { travel: T, rpms, source: 'measured in the browser', rows };
  for (const bt of ['pollen', 'nectar']) {
    const R = rows.filter((r) => r.type === bt && !r.jam);
    const arms = [...new Set(R.map((r) => r.arm))].sort((a, b) => a - b);
    const rr = [...new Set(R.map((r) => r.rpm))].sort((a, b) => a - b);
    const mean = (a, q, k) => { const s = R.filter((r) => r.arm === a && r.rpm === q); return s.reduce((x, r) => x + r[k], 0) / s.length; };
    cal[bt] = { grid: { arms, rpms: rr, v: arms.map((a) => rr.map((q) => mean(a, q, 'speed'))), e: arms.map((a) => rr.map((q) => mean(a, q, 'elevation'))) },
      n: R.length, jams: rows.filter((r) => r.type === bt && r.jam).length };
  }
  return cal;
}
