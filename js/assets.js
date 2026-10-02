// Fetch a variant (scene.xml + manifest.json + the files the manifest lists, + calibration.json if it has one).
// manifest.files: {virtual path in MuJoCo's filesystem: URL relative to the variant folder} (shared meshes live in
// assets/meshes/, named by content, so identical meshes are downloaded once and cached by the browser across variants).
// An older array form (virtual path = URL) is accepted too.
export async function fetchVariant(base, onProgress = () => {}) {
  if (!base.endsWith('/')) base += '/';
  const here = typeof document !== 'undefined' ? document.baseURI : self.location.href;
  const baseUrl = new URL(base, here);
  const get = async (rel, kind) => {
    const r = await fetch(new URL(rel, baseUrl), kind === 'json' || kind === 'text' ? { cache: 'no-cache' } : {});
    if (!r.ok) throw new Error(`${new URL(rel, baseUrl)}: HTTP ${r.status}`);
    return kind === 'json' ? r.json() : kind === 'text' ? r.text() : new Uint8Array(await r.arrayBuffer());
  };
  const manifest = await get('manifest.json', 'json');
  const xml = await get(manifest.scene, 'text');
  const entries = Array.isArray(manifest.files) ? manifest.files.map((f) => [f, f]) : Object.entries(manifest.files);
  const files = {};
  let done = 0;
  await Promise.all(entries.map(async ([vfs, url]) => {
    files[vfs] = await get(url, 'bin');
    onProgress(++done, entries.length);
  }));
  const calibration = manifest.calibration ? await get(manifest.calibration, 'json').catch(() => null) : null;
  return { manifest, xml, files, calibration, base: baseUrl.href };
}
