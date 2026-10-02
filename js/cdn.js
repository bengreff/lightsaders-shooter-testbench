// Pinned CDN locations (workers cannot see the page's import map, so the MuJoCo URL lives here too).
export const MUJOCO_URL = 'https://cdn.jsdelivr.net/npm/@mujoco/mujoco@3.13.0/mujoco.js';
export const loadMujoco = async () => (await import(MUJOCO_URL)).default();
