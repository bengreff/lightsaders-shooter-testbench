# Lightsaders BIOBUZZ turret testbench

Interactive simulations of our FTC team's turret shooters for the 2026-27 BIOBUZZ season, a fair comparison of them, and the build
pack of our v5 turret.

- **Testbench:** https://bengreff.github.io/lightsaders-shooter-testbench/ : three turrets (group "Compare"): the team's POLLEN turret on
  the robot today, our v5 for both balls (one motor, one servo), and a sprung passive turret built as a benchmark.
- **Turret comparison:** https://bengreff.github.io/lightsaders-shooter-testbench/compare/
- **v5 build pack:** https://bengreff.github.io/lightsaders-shooter-testbench/build/

Runs in the browser: MuJoCo (WebAssembly) for the physics, three.js for the drawing. Desktop Chrome or Edge recommended.
The first load downloads ~25 MB (the physics engine and the meshes). All numbers come from the simulator, an ideal environment
(no measurement error, no hardware precision limits): real life will likely be slightly lower.

This repository is generated: it is a published copy of the testbench, built from the team's private development
repository; changes are made there and re-published.
