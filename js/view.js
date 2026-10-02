// three.js drawing of a MuJoCo scene. Every geom is built once from the compiled model (meshes straight from MuJoCo's
// own buffers: mesh_vert / mesh_face / mesh_normal / mesh_facenormal, so there is no second mesh file to keep in
// step), then each frame takes its world pose from data.geom_xpos / geom_xmat. MuJoCo is z-up: so is this scene.
//
// Groups: MuJoCo geom groups 0-2 are drawn; group 3 (collision shapes in the shooter export) is hidden unless
// showCollision is set, and then drawn in a flat debug colour (their own alpha is 0 when CAD visuals exist).
// Ball trails: one THREE.Line per ball over a fixed-size buffer updated in place (no per-step objects).

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const GEOM = { PLANE: 0, HFIELD: 1, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6, MESH: 7 };
const DEFAULT_RGBA = [0.5, 0.5, 0.5, 1];
const TRAIL_N = 400;                 // points per trail (at TRAIL_DT that is 1.6 s of flight)
const TRAIL_DT = 0.004;              // s of simulated time between trail points
const TRAIL_MAX_S = 2.5;             // a trail stops growing after this long (the Python testbench's 2.5 s)
const TRAIL_FADE_S = 1.5;            // ... and fades out over this long once its ball has landed

export class View {
  constructor(container) {
    this.container = container;
    const r = this.renderer = new THREE.WebGLRenderer({ antialias: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFShadowMap;
    r.outputColorSpace = THREE.SRGBColorSpace;
    r.toneMapping = THREE.ACESFilmicToneMapping; r.toneMappingExposure = 1.05;
    container.appendChild(r.domElement);
    const s = this.scene = new THREE.Scene();
    s.background = new THREE.Color(0x14171d);
    const c = this.camera = new THREE.PerspectiveCamera(40, 1, 0.005, 60);
    c.up.set(0, 0, 1);
    c.position.set(0.85, -0.75, 0.62);
    this.controls = new OrbitControls(c, r.domElement);
    this.controls.target.set(0, -0.05, 0.3);
    this.controls.enableDamping = true;
    this.controls.update();
    // light: soft sky/ground fill + one shadow-casting key light + a cool rim
    s.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a2622, 0.9));
    const key = this.key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(1.2, -1.6, 2.4); key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    const sc = key.shadow.camera; sc.left = -0.8; sc.right = 0.8; sc.top = 0.8; sc.bottom = -0.8; sc.near = 0.1; sc.far = 6;
    key.shadow.bias = -0.0004; key.shadow.normalBias = 0.002;
    s.add(key); s.add(key.target);
    const rim = new THREE.DirectionalLight(0x9fb8ff, 0.6); rim.position.set(-1.5, 1.2, 1.0); s.add(rim);
    this.showCollision = false;
    this.geoms = []; this.trails = [];
    this.root = new THREE.Group(); s.add(this.root);
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
  }

  resize() {
    const w = this.container.clientWidth || 1, h = this.container.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%'; this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }

  // ------------------------------------------------------------------ building
  /** (Re)build every drawable from sim.m. balls: [{body, type}] for trails and colours. */
  build(sim, balls = []) {
    for (const o of this.root.children.slice()) { this.root.remove(o); o.traverse?.((x) => { x.geometry?.dispose(); }); }
    this.geoms = []; this.trails = [];
    const m = sim.m;
    const type = m.geom_type, size = m.geom_size, rgbaA = m.geom_rgba, matid = m.geom_matid, group = m.geom_group,
      dataid = m.geom_dataid, bodyid = m.geom_bodyid, matRgba = m.mat_rgba;
    const meshCache = new Map();
    // bodies with no drawn geom of their own (e.g. parts the CAD does not cover yet): draw their collision shapes as
    // stand-ins, in a neutral colour, so nothing that moves is invisible
    const drawn = new Set();
    for (let g = 0; g < m.ngeom; g++) {
      const mid = matid[g], a = mid >= 0 && Math.abs(rgbaA[4 * g + 3] - 1) < 1e-6 ? matRgba[4 * mid + 3] : rgbaA[4 * g + 3];
      if (group[g] !== 3 && a > 0) drawn.add(bodyid[g]);
    }
    const ballColor = new Map(balls.map((b) => [b.geom, b.type === 'nectar' ? [0.47, 0.75, 1.0, 1] : [0.98, 0.84, 0.25, 1]]));
    for (let g = 0; g < m.ngeom; g++) {
      const t = type[g];
      let rgba = Array.from(rgbaA.subarray(4 * g, 4 * g + 4));
      const mid = matid[g];
      if (mid >= 0 && rgba.every((v, i) => Math.abs(v - DEFAULT_RGBA[i]) < 1e-6)) rgba = Array.from(matRgba.subarray(4 * mid, 4 * mid + 4));
      if (ballColor.has(g)) rgba = ballColor.get(g);        // the app's ball colours (POLLEN yellow, NECTAR blue)
      let collision = group[g] === 3;
      if (collision && !drawn.has(bodyid[g]) && bodyid[g] !== 0) { collision = false; rgba = [0.16, 0.17, 0.19, 1]; }
      else if (collision) rgba = [0.95, 0.35, 0.2, 0.35];
      else if (rgba[3] === 0) continue;
      let geo;
      const sz = [size[3 * g], size[3 * g + 1], size[3 * g + 2]];
      if (t === GEOM.PLANE) { this._floor(sz); continue; }
      else if (t === GEOM.SPHERE) geo = new THREE.SphereGeometry(sz[0], 32, 20);
      else if (t === GEOM.ELLIPSOID) { geo = new THREE.SphereGeometry(1, 32, 20); geo.scale(sz[0], sz[1], sz[2]); }
      else if (t === GEOM.CAPSULE) { geo = new THREE.CapsuleGeometry(sz[0], 2 * sz[1], 8, 24); geo.rotateX(Math.PI / 2); }
      else if (t === GEOM.CYLINDER) { geo = new THREE.CylinderGeometry(sz[0], sz[0], 2 * sz[1], 40); geo.rotateX(Math.PI / 2); }
      else if (t === GEOM.BOX) geo = new THREE.BoxGeometry(2 * sz[0], 2 * sz[1], 2 * sz[2]);
      else if (t === GEOM.MESH) {
        const id = dataid[g];
        if (!meshCache.has(id)) meshCache.set(id, meshGeometry(m, id));
        geo = meshCache.get(id);
      } else continue;
      const mat = new THREE.MeshStandardMaterial({
        color: new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.SRGBColorSpace),
        roughness: mid >= 0 ? 1 - 0.7 * m.mat_shininess[mid] : 0.6,
        metalness: mid >= 0 ? Math.min(0.6, 0.6 * m.mat_specular[mid]) : 0.05,
        transparent: rgba[3] < 1, opacity: rgba[3], depthWrite: rgba[3] >= 1, side: rgba[3] < 1 ? THREE.DoubleSide : THREE.FrontSide,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.matrixAutoUpdate = false;
      mesh.castShadow = rgba[3] > 0.5 && !collision; mesh.receiveShadow = !collision;
      mesh.visible = !collision || this.showCollision;
      mesh.userData = { geom: g, body: bodyid[g], collision };
      this.root.add(mesh);
      this.geoms.push(mesh);
    }
    for (const b of balls) this._trail(b);
  }

  _floor(sz) {
    const half = sz[0] > 0 ? Math.min(sz[0], 6) : 6;
    const geo = new THREE.PlaneGeometry(2 * half, 2 * half);
    const mat = new THREE.MeshStandardMaterial({ color: 0x2a2e36, roughness: 0.95, metalness: 0 });
    const floor = new THREE.Mesh(geo, mat); floor.receiveShadow = true; this.root.add(floor);
    const grid = new THREE.GridHelper(2 * half, 2 * half * 4, 0x4a5160, 0x363b45);   // 0.25 m cells
    grid.rotateX(Math.PI / 2); grid.position.z = 0.0005; this.root.add(grid);
  }

  _trail(b) {
    const pos = new Float32Array(TRAIL_N * 3), col = new Float32Array(TRAIL_N * 4);
    const c = new THREE.Color(b.type === 'nectar' ? 0x78beff : 0xfad640);
    for (let i = 0; i < TRAIL_N; i++) { col[4 * i] = c.r; col[4 * i + 1] = c.g; col[4 * i + 2] = c.b; col[4 * i + 3] = 1; }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 4).setUsage(THREE.DynamicDrawUsage));
    geo.setDrawRange(0, 0);
    const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9, depthWrite: false }));
    line.frustumCulled = false;
    this.root.add(line);
    this.trails.push({ body: b.body, line, pos, col, n: 0, tNext: 0, active: false, t0: 0, tEnd: null, nAlpha: -1 });
  }

  /** Start (clear) the trail of a ball body: call when a ball is injected. */
  startTrail(body, t) {
    const tr = this.trails.find((x) => x.body === body);
    if (tr) { tr.n = 0; tr.tNext = t; tr.active = true; tr.t0 = t; tr.tEnd = null; tr.nAlpha = -1; }
  }
  clearTrails() { for (const tr of this.trails) { tr.n = 0; tr.active = false; tr.tEnd = null; tr.line.geometry.setDrawRange(0, 0); } }

  /** Sample the active trails at the current sim time (call after each batch of steps, or each step for fine trails). */
  sampleTrails(sim) {
    const t = sim.time, X = sim.xpos;
    for (const tr of this.trails) {
      if (!tr.active || t < tr.tNext) continue;
      tr.tNext = t + TRAIL_DT;
      if (tr.n === TRAIL_N) { tr.pos.copyWithin(0, 3); tr.n--; }
      const o = 3 * tr.body, k = 3 * tr.n;
      tr.pos[k] = X[o]; tr.pos[k + 1] = X[o + 1]; tr.pos[k + 2] = X[o + 2];
      tr.n++;
      if (X[o + 2] < 0.05 || t - tr.t0 > TRAIL_MAX_S) { tr.active = false; tr.tEnd = t; }   // landed or long enough
    }
  }

  setShowCollision(on) { this.showCollision = on; for (const g of this.geoms) if (g.userData.collision) g.visible = on; }

  // ------------------------------------------------------------------ per frame
  sync(sim) {
    const P = sim.geom_xpos, R = sim.geom_xmat;
    for (const mesh of this.geoms) {
      const g = mesh.userData.geom, p = 3 * g, r = 9 * g;
      mesh.matrix.set(R[r], R[r + 1], R[r + 2], P[p], R[r + 3], R[r + 4], R[r + 5], P[p + 1], R[r + 6], R[r + 7], R[r + 8], P[p + 2], 0, 0, 0, 1);
      mesh.matrixWorldNeedsUpdate = true;
    }
    const t = sim.time;
    for (const tr of this.trails) {
      const geo = tr.line.geometry;
      if (tr.n !== tr.nAlpha) {                         // alpha ramps up along the trail: the old end fades
        for (let i = 0; i < tr.n; i++) tr.col[4 * i + 3] = 0.15 + 0.85 * Math.pow((i + 1) / tr.n, 0.6);
        geo.attributes.color.needsUpdate = true; geo.attributes.position.needsUpdate = true; tr.nAlpha = tr.n;
      }
      const age = tr.tEnd === null ? 0 : t - tr.tEnd;
      tr.line.material.opacity = 0.9 * Math.max(0, 1 - age / TRAIL_FADE_S);
      tr.line.visible = tr.n > 1 && tr.line.material.opacity > 0.01;
      geo.setDrawRange(0, tr.n);
    }
  }

  /** Camera presets as MuJoCo free-camera (azimuth, elevation deg, distance m, look-at). */
  setView(v) {
    const az = v.az * Math.PI / 180, el = v.el * Math.PI / 180, L = new THREE.Vector3(...v.lookat);
    const f = new THREE.Vector3(Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el));
    this.camera.position.copy(L).addScaledVector(f, -v.dist);
    this.controls.target.copy(L);
    this.controls.update();
  }

  render() { this.controls.update(); this.renderer.render(this.scene, this.camera); }
}

/** A non-indexed BufferGeometry of mesh `id` from the compiled model, with MuJoCo's own per-corner normals. */
function meshGeometry(m, id) {
  const va = m.mesh_vertadr[id], fa = m.mesh_faceadr[id], nf = m.mesh_facenum[id], na = m.mesh_normaladr[id];
  const V = m.mesh_vert, F = m.mesh_face, N = m.mesh_normal, FN = m.mesh_facenormal;
  const pos = new Float32Array(nf * 9), nor = new Float32Array(nf * 9);
  for (let f = 0; f < nf; f++) {
    for (let c = 0; c < 3; c++) {
      const vi = 3 * (va + F[3 * (fa + f) + c]), ni = 3 * (na + FN[3 * (fa + f) + c]), o = 9 * f + 3 * c;
      pos[o] = V[vi]; pos[o + 1] = V[vi + 1]; pos[o + 2] = V[vi + 2];
      nor[o] = N[ni]; nor[o + 1] = N[ni + 1]; nor[o + 2] = N[ni + 2];
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  geo.computeBoundingSphere();
  return geo;
}
