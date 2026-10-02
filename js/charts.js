// Small live strip charts on one canvas: fixed ring buffers (no allocation per sample), redrawn a few times a second.

export class Strip {
  /** series: [{label, unit, color, fmt, min, max (fixed axis, or null = auto)}], one panel each, stacked. */
  constructor(canvas, series, n = 600) {
    this.c = canvas; this.ctx = canvas.getContext('2d');
    this.series = series.map((s) => ({ ...s, buf: new Float32Array(n), ref: s.ref ? new Float32Array(n) : null }));
    this.n = n; this.i = 0; this.count = 0;
  }

  /** values: one number per series (and series[k].ref values via refs[k]). */
  push(values, refs = []) {
    const i = this.i;
    this.series.forEach((s, k) => { s.buf[i] = values[k]; if (s.ref) s.ref[i] = refs[k] ?? NaN; });
    this.i = (i + 1) % this.n; this.count = Math.min(this.count + 1, this.n);
  }

  clear() { this.i = 0; this.count = 0; }

  draw() {
    const c = this.c, dpr = Math.min(window.devicePixelRatio || 1, 2);
    const W = Math.round(c.clientWidth * dpr), H = Math.round(c.clientHeight * dpr);
    if (c.width !== W || c.height !== H) { c.width = W; c.height = H; }
    const g = this.ctx; g.clearRect(0, 0, W, H);
    const S = this.series, ph = H / S.length, pad = 3 * dpr;
    g.font = `${11 * dpr}px ui-monospace, SFMono-Regular, Menlo, monospace`; g.textBaseline = 'top';
    S.forEach((s, k) => {
      const y0 = k * ph + pad, h = ph - 2 * pad;
      g.fillStyle = 'rgba(255,255,255,0.035)'; g.fillRect(0, y0, W, h);
      if (!this.count) return;
      let lo = s.min, hi = s.max;
      if (lo == null || hi == null) {
        let a = Infinity, b = -Infinity;
        for (let j = 0; j < this.count; j++) { const v = s.buf[j]; if (v < a) a = v; if (v > b) b = v; if (s.ref) { const r = s.ref[j]; if (r < a) a = r; if (r > b) b = r; } }
        const m = Math.max((b - a) * 0.12, s.span ?? 1e-6);
        lo = lo ?? a - m; hi = hi ?? b + m;
      }
      const X = (j) => (j / (this.n - 1)) * W, Y = (v) => y0 + h - (v - lo) / (hi - lo) * h;
      const line = (buf, color, w) => {
        g.strokeStyle = color; g.lineWidth = w * dpr; g.beginPath();
        const start = this.count < this.n ? 0 : this.i;
        for (let j = 0; j < this.count; j++) {
          const v = buf[(start + j) % this.n], x = X(j + this.n - this.count), y = Y(v);
          if (j === 0) g.moveTo(x, y); else g.lineTo(x, y);
        }
        g.stroke();
      };
      if (s.ref) line(s.ref, 'rgba(255,255,255,0.35)', 1);
      line(s.buf, s.color, 1.5);
      const last = s.buf[(this.i + this.n - 1) % this.n];
      g.fillStyle = 'rgba(152,162,179,1)'; g.fillText(s.label, 4 * dpr, y0 + 2 * dpr);
      g.fillStyle = '#e8ecf2'; g.textAlign = 'right'; g.fillText(`${s.fmt(last)} ${s.unit}`, W - 4 * dpr, y0 + 2 * dpr); g.textAlign = 'left';
    });
  }
}
