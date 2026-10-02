// MuJoCo (WebAssembly) wrapper: one scene, stepped with pre/post-step hooks, named access, fast contact iteration.
// No DOM, no three.js: runs the same in the browser and in node (tools/*.mjs).
//
//   const mj = await loadMujoco();                    // the @mujoco/mujoco module (CDN in the browser, npm in node)
//   const sim = new Sim(mj);
//   sim.loadScene(xmlText, {"meshes/a.stl": Uint8Array, ...});
//   sim.addHook({ preStep(sim) {...}, postStep(sim) {...} });
//   sim.step(100);
//
// Views (qpos, qvel, ctrl, xfrc_applied, xpos, ...) are live Float64Arrays into the WASM heap. They are re-fetched
// after a scene load and whenever the heap grows (a grown heap detaches old views: length 0).

const CONTACT_STRIDE = 584;      // sizeof(mjContact) in MuJoCo 3.13 wasm32: 66 mjtNum + 13 int + 4 pad (checkContactLayout() verifies)
const C_DIST = 0, C_POS = 8, C_FRAME = 32, C_DIM = 528, C_GEOM1 = 532, C_GEOM2 = 536, C_EXCLUDE = 572, C_EFC = 576;

export class Sim {
  constructor(mujoco) {
    this.mj = mujoco;
    this.m = null; this.d = null;
    this.hooks = [];
    this._dir = null; this._n = 0;
    this._f6 = null;
    this.rawContacts = true;       // false: fall back to the (slower, copying) data.contact vector
  }

  // ------------------------------------------------------------------ loading
  /** Write the files into MuJoCo's virtual filesystem and compile scene.xml from it. files: { path relative to the
   *  scene: Uint8Array | ArrayBuffer | string }; paths may climb out of the scene's folder ("../../meshes/a.stl"):
   *  the virtual layout mirrors the site's, so a scene whose meshdir is "../../meshes" works unchanged. Replaces any
   *  loaded scene (hooks are kept and get onLoad). */
  loadScene(xml, files = {}) {
    const FS = this.mj.FS;
    const up = Math.max(0, ...Object.keys(files).map((p) => (p.match(/^(\.\.\/)+/)?.[0].length ?? 0) / 3));
    const root = `/scene${this._n++}`;
    const dir = root + '/v'.repeat(up);
    const norm = (p) => { const out = []; for (const part of p.split('/')) { if (part === '..') out.pop(); else if (part && part !== '.') out.push(part); } return '/' + out.join('/'); };
    const mkdirs = (p) => { let cur = ''; for (const part of p.split('/').filter(Boolean)) { cur += '/' + part; try { FS.mkdir(cur); } catch (e) { /* exists */ } } };
    mkdirs(dir);
    for (const [path, data] of Object.entries(files)) {
      const full = norm(`${dir}/${path}`);
      mkdirs(full.slice(0, full.lastIndexOf('/')));
      FS.writeFile(full, data instanceof ArrayBuffer ? new Uint8Array(data) : data);
    }
    FS.writeFile(`${dir}/scene.xml`, xml);
    const m = this.mj.MjModel.from_xml_path(`${dir}/scene.xml`);
    const d = new this.mj.MjData(m);
    this._dispose();
    this.m = m; this.d = d; this._removeDir(this._dir); this._dir = root;
    this._f6 = this._f6 || new this.mj.DoubleBuffer(6);
    this._names();
    this._refresh();
    this.mj.mj_forward(m, d);
    this.ellipticCone = m.opt.cone === this.mj.mjtCone.mjCONE_ELLIPTIC.value;
    for (const h of this.hooks) h.onLoad && h.onLoad(this);
    return this;
  }

  _removeDir(dir) {
    if (!dir) return;
    const FS = this.mj.FS;
    const rm = (p) => {
      for (const e of FS.readdir(p)) {
        if (e === '.' || e === '..') continue;
        const q = `${p}/${e}`;
        if (FS.isDir(FS.stat(q).mode)) rm(q); else FS.unlink(q);
      }
      FS.rmdir(p);
    };
    try { rm(dir); } catch (e) { /* best effort */ }
  }

  _dispose() {
    if (this.d) this.d.delete();
    if (this.m) this.m.delete();
    this.m = this.d = null;
  }

  _names() {
    const mj = this.mj, m = this.m, O = mj.mjtObj;
    const table = (type, n) => {
      const map = new Map(), list = [];
      for (let i = 0; i < n; i++) { const nm = mj.mj_id2name(m, type, i); list.push(nm); if (nm) map.set(nm, i); }
      return { map, list };
    };
    this.names = {
      body: table(O.mjOBJ_BODY.value, m.nbody), joint: table(O.mjOBJ_JOINT.value, m.njnt),
      geom: table(O.mjOBJ_GEOM.value, m.ngeom), actuator: table(O.mjOBJ_ACTUATOR.value, m.nu),
      mesh: table(O.mjOBJ_MESH.value, m.nmesh), material: table(O.mjOBJ_MATERIAL.value, m.nmat),
    };
    // model arrays are constant: copy the ones the hooks use every step
    this.jnt_qposadr = Int32Array.from(m.jnt_qposadr);
    this.jnt_dofadr = Int32Array.from(m.jnt_dofadr);
    this.jnt_type = Int32Array.from(m.jnt_type);
    this.timestep = m.opt.timestep;
  }

  _refresh() {
    const d = this.d;
    this.qpos = d.qpos; this.qvel = d.qvel; this.ctrlv = d.ctrl; this.xfrc = d.xfrc_applied; this.qfrc = d.qfrc_applied;
    this.xpos = d.xpos; this.xquat = d.xquat; this.xmat = d.xmat;
    this.geom_xpos = d.geom_xpos; this.geom_xmat = d.geom_xmat; this.mocap_pos = d.mocap_pos; this.mocap_quat = d.mocap_quat;
    const ar = d.arena;
    this._arenaBuf = ar.buffer; this._arenaOff = ar.byteOffset;
    this._cf64 = new Float64Array(ar.buffer, ar.byteOffset, Math.floor(ar.length / 8));
    this._ci32 = new Int32Array(ar.buffer, ar.byteOffset, Math.floor(ar.length / 4));
  }

  /** Re-fetch views if the WASM heap grew (old views detach and read length 0). Cheap; called every step. */
  views() { if (this.qpos.length === 0) this._refresh(); return this; }

  // ------------------------------------------------------------------ stepping
  /** first: run before the hooks already added (e.g. a firing controller that must act before the motor models). */
  addHook(h, first = false) { if (first) this.hooks.unshift(h); else this.hooks.push(h); if (this.m && h.onLoad) h.onLoad(this); return h; }
  removeHook(h) { this.hooks = this.hooks.filter((x) => x !== h); }

  /** n physics steps; each: every hook's preStep, mj_step, every hook's postStep. */
  step(n = 1) {
    const mj = this.mj, m = this.m, d = this.d, H = this.hooks;
    for (let i = 0; i < n; i++) {
      this.views();
      for (let k = 0; k < H.length; k++) if (H[k].preStep) H[k].preStep(this);
      mj.mj_step(m, d);
      for (let k = 0; k < H.length; k++) if (H[k].postStep) H[k].postStep(this);
    }
  }

  forward() { this.mj.mj_forward(this.m, this.d); this.views(); }

  reset() {
    this.mj.mj_resetData(this.m, this.d);
    this._refresh();
    this.mj.mj_forward(this.m, this.d);
    for (const h of this.hooks) h.onReset && h.onReset(this);
  }

  get time() { return this.d.time; }
  get ncon() { return this.d.ncon; }

  // ------------------------------------------------------------------ names
  id(kind, name) {
    if (typeof name === 'number') return name;
    const i = this.names[kind].map.get(name);
    if (i === undefined) throw new Error(`no ${kind} named '${name}'`);
    return i;
  }
  bodyId(n) { return this.id('body', n); }
  geomId(n) { return this.id('geom', n); }
  jointId(n) { return this.id('joint', n); }
  actuatorId(n) { return this.id('actuator', n); }
  qadr(joint) { return this.jnt_qposadr[this.jointId(joint)]; }
  vadr(joint) { return this.jnt_dofadr[this.jointId(joint)]; }

  // ------------------------------------------------------------------ state accessors
  /** Joint position: a number for hinge/slide, a 7-array (pos, quat) for free, 4-array for ball. With value: set. */
  jointQpos(joint, value) {
    const j = this.jointId(joint), a = this.jnt_qposadr[j], n = [7, 4, 1, 1][this.jnt_type[j]];
    if (value !== undefined) { if (n === 1) this.qpos[a] = value; else this.qpos.set(value, a); }
    return n === 1 ? this.qpos[a] : Array.from(this.qpos.subarray(a, a + n));
  }
  jointQvel(joint, value) {
    const j = this.jointId(joint), a = this.jnt_dofadr[j], n = [6, 3, 1, 1][this.jnt_type[j]];
    if (value !== undefined) { if (n === 1) this.qvel[a] = value; else this.qvel.set(value, a); }
    return n === 1 ? this.qvel[a] : Array.from(this.qvel.subarray(a, a + n));
  }
  /** Actuator control: read, or set when value is given. */
  ctrl(actuator, value) {
    const i = this.actuatorId(actuator);
    if (value !== undefined) this.ctrlv[i] = value;
    return this.ctrlv[i];
  }
  /** World force (N) and torque (N.m) at the body's centre of mass; persists until changed (clearForce). */
  applyForce(body, f = [0, 0, 0], torque = [0, 0, 0]) {
    const o = 6 * this.bodyId(body);
    this.xfrc[o] = f[0]; this.xfrc[o + 1] = f[1]; this.xfrc[o + 2] = f[2];
    this.xfrc[o + 3] = torque[0]; this.xfrc[o + 4] = torque[1]; this.xfrc[o + 5] = torque[2];
  }
  clearForce(body) { this.applyForce(body); }
  bodyPos(body) { const o = 3 * this.bodyId(body); return [this.xpos[o], this.xpos[o + 1], this.xpos[o + 2]]; }
  bodyQuat(body) { const o = 4 * this.bodyId(body); return Array.from(this.xquat.subarray(o, o + 4)); }

  // ------------------------------------------------------------------ contacts
  /** Call fn(i, geom1, geom2, dist, exclude, efcAddress) for each active contact, read straight from the arena (no copy). */
  forEachContact(fn) {
    const n = this.d.ncon;
    if (!this.rawContacts) {
      const v = this.d.contact;
      for (let i = 0; i < n; i++) { const c = v.get(i); fn(i, c.geom1, c.geom2, c.dist, c.exclude, c.efc_address); c.delete(); }
      v.delete();
      return n;
    }
    const I = this._ci32, F = this._cf64;
    for (let i = 0; i < n; i++) {
      const b = i * CONTACT_STRIDE;
      fn(i, I[(b + C_GEOM1) >> 2], I[(b + C_GEOM2) >> 2], F[b >> 3], I[(b + C_EXCLUDE) >> 2], I[(b + C_EFC) >> 2]);
    }
    return n;
  }

  /** One contact in full (pos, frame: normal first), from the arena. */
  contact(i) {
    const b = i * CONTACT_STRIDE, F = this._cf64, I = this._ci32;
    return {
      geom1: I[(b + C_GEOM1) >> 2], geom2: I[(b + C_GEOM2) >> 2], dim: I[(b + C_DIM) >> 2], dist: F[b >> 3],
      pos: Array.from(F.subarray((b + C_POS) >> 3, ((b + C_POS) >> 3) + 3)),
      frame: Array.from(F.subarray((b + C_FRAME) >> 3, ((b + C_FRAME) >> 3) + 9)),
      exclude: I[(b + C_EXCLUDE) >> 2], efc_address: I[(b + C_EFC) >> 2],
    };
  }

  /** All contacts as objects (convenience; allocates). */
  contacts() { const out = []; for (let i = 0; i < this.d.ncon; i++) out.push(this.contact(i)); return out; }

  /** mj_contactForce: the contact's 6-vector (normal, tangent1, tangent2, torsion, roll1, roll2) in its frame. */
  contactForce(i, out = new Float64Array(6)) {
    this.mj.mj_contactForce(this.m, this.d, i, this._f6);
    out.set(this._f6.GetView());
    return out;
  }

  /** The contact's normal force, fast: an elliptic cone's efc_force row 0 is mj_contactForce's f[0] exactly. */
  contactNormal(i, efcAddress) {
    if (efcAddress < 0) return 0;
    if (this.ellipticCone) return this.d.efc_force[efcAddress];
    return this.contactForce(i)[0];
  }

  /** Compare the raw arena reading with the embind copy; switch to the copy if they disagree. Needs ncon >= 2. */
  checkContactLayout() {
    const n = this.d.ncon;
    if (n < 2) return null;
    const v = this.d.contact; let ok = true;
    for (let i = 0; i < Math.min(n, 4); i++) {
      const c = v.get(i), r = this.contact(i);
      ok = ok && c.geom1 === r.geom1 && c.geom2 === r.geom2 && c.dist === r.dist && c.efc_address === r.efc_address && c.dim === r.dim;
      c.delete();
    }
    v.delete();
    this.rawContacts = ok;
    return ok;
  }
}
