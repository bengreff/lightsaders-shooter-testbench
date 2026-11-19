# Lightsaders BIOBUZZ turret testbench

FTC team 12928 Lightsaders, 2026-27 season. Three turrets in one simulator, picked from the menu (group "Compare"):
1. **Team turret**: the POLLEN turret on our robot today, decoded from the team's CAD, running the robot's own speed loop.
2. **v5 active**: our design for both balls (POLLEN and NECTAR), one motor and one servo.
3. **Passive sprung**: a benchmark only; v4's one hood held by an abstract spring, pushed open by a NECTAR.

- **[Turret comparison](compare/)**: the fair comparison of the three, every number and how it was measured.
- **[v5 build pack](build/)**: parts list, plates to cut, prints (sliced), assembly steps and the STEP.

All numbers come from this simulator, an ideal environment: no measurement error and no hardware precision limits, so real life will likely
be slightly lower.

## Turret v5 in detail

Our FTC team's turret shooter for the 2026-27 BIOBUZZ season, for both balls, simulated: the parts that touch a ball, the
mechanisms that move the hood pieces and the frame that holds them (drawn; the robot's goBILDA 3208 gear ring is shown underneath).

## The design
- **One motor:** a goBILDA Hogback flywheel (96 mm, steel discs) and, geared off the same motor (1 : 3.8), a squishy feed wheel
  that pushes each ball up the turret's bore against its front wall and into the flywheel.
- **One servo** (goBILDA 2000-0025-0003, 25-3 Speed, at 6 V through a REV Servo Hub or servo power module) turns a 40-tooth
  sector gear on the flywheel axle (1.333 : 1), so 213 deg of hood travel is 284 deg of servo.
- **The flywheel: 1 Hogback + 4 steel discs, and our own speed loop**, on the motor as fitted to our robot's logs. Every 10 ms, from
  the encoder's whole ticks, the battery voltage and the robot's own feed command: the motor model predicts the speed (corrected by
  what the loop has learned the model misses), the ticks correct it, smoothly while the wheel runs steady and quickly just after a
  ball is fed; the motor gets the feedforward for the target speed + a proportional term + an integral that learns only while the
  wheel is steady (a shot never winds it up); full power when more than 3 % low; "ready" = within 0.5 % for 50 ms. Its motor model
  is deliberately 3-30 % off, so the simulation does not flatter it. After a shot it stays at full power until the speed it
  predicts 20 ms ahead is within 0.3 % of the target (no slow tail, no overshoot from its own lag). After a NECTAR shot at 3000 rpm the
  wheel comes back within 0.5 % in 0.28 s with 5 rpm of overshoot and then holds within about 5 rpm (the team's hub loop,
  160 / 0 / 0 / 15.2, sits 15-20 rpm low). Full power is the limit: about 0.3 s to put back what a ball takes.
- **Every hood part turns about the flywheel axle** on ball bearings, with arms on both sides of the flywheel, and every
  ball-contact face is a surface of revolution about that axle:
  - **POLLEN hood** (yellow): face radius 118.3 mm, trough section radius 39.9 mm, 73 mm wide. Two parts: a lower half (the
    entry plus two side teeth) and a top half (one central tooth, on the gear).
  - **NECTAR hood** (blue): face radius 135.85 mm, section radius 50.6 mm, 86 mm wide. Two static side teeth on a solid
    support with feet down to the turret gear, and one central tooth that rests and slides on that support.
  - The POLLEN parts sit 0.5 mm under the NECTAR faces: the NECTAR hood backs the POLLEN hood.
- **One pin per side does three jobs:** it rides a slot in the lower half's hub (it lifts the lower half up and brings it
  back over the top), it works the lower half's latch, and it pushes the NECTAR arm (the hook).
- **The lower half is latched, positively (no springs, no magnets, no gravity):** a pawl on its hub has a closed track the
  pin runs in. While POLLEN shoots, the track is concentric with the axle, so the pin stops the pawl turning either way and its
  hook stays behind the end of a fixed ring; the balls' push goes through the hook into the ring and the pawl's pivot. To switch,
  the track bends: the pin turns the pawl and draws the hook in under the ring. While carried, the ring keeps the pawl turned, so
  the lower half is locked to the top half (about 1.5 deg of play) and cannot drop over the top. Built to last: the ring is acetal
  on steel standoffs, the lower half's rest stops are solid cheeks of the NECTAR support, the ring's end is fitted to the hook
  (no play), and the pin and the pawl's pivot are steel shoulder screws clamped to their plates.
- **One servo does every change:**
  - 0-31 deg: POLLEN exit angle (about 56-88 deg), only the central top tooth moves;
  - 31-38 deg: the pin turns the latch open; from 38 deg it lifts the lower half; both swing up over the flywheel;
  - from 182 deg they push NECTAR's central tooth out: NECTAR exit angle about 58-88 deg.
- **Switching POLLEN to NECTAR** takes about 0.64 s and back about 0.48 s, from the servo's catalogue torque and speed, the hood
  parts' weight and inertia, and guessed loop gains. On the way back the code stops the hood just above where the lower half lands
  and eases it down through the landing (about 150 instead of 590 deg/s). **MIXED** fires POLLEN, NECTAR, POLLEN, NECTAR in real
  time: the hood swings as soon as a ball has left it.

- **The frame:**
  - two 1/4 in acetal side plates, only over the bore, no tie: the two axles clamp them and they screw to the base;
  - ONE printed base piece: the bore ring, the static NECTAR teeth, the NECTAR support and the side plates' ears, screwed to the
    gear ring;
  - the flywheel axle is a goBILDA 12 mm REX shaft, with 12 x 21 mm bearings in every plate;
  - the motor sits behind the bore, 24T:24T HTD 5M on a stock 275 mm belt; the squishy is fixed, driven 1 : 3.8 by HTD 3M 25T:95T on
    a stock 444 mm belt (no idler);
  - the servo is screwed to its plate on 4 standoffs.
- **Mass as drawn:** about 3.25 kg (prints counted at 1.0 g/cm3), of which the 4 steel flywheel discs are 0.6 kg.

## Measured in this simulation
| | |
|---|---|
| Calibration grid (both balls, 1600-4400 rpm, every hood setting, 6 shots a point) | 1404 shots, 0 jams |
| TARGET check: 60 random targets per ball, each fired 4 times | the map's own error (rms): POLLEN 0.45 % speed / 0.77 deg, NECTAR 0.79 % / 0.48 deg; shot-to-shot spread of the balls themselves: POLLEN 0.69 % / 1.14 deg, NECTAR 0.34 % / 0.62 deg |
| Mechanisms + frame over the whole servo travel | no part touches another; every part at least 1.2 mm from the balls |
| Reach from the swivel axis (R105, 228.6 mm) | 173.9 mm at most |
| Six balls of one kind in a row (6.0 m/s at 65 deg) | POLLEN 1.65 s, NECTAR 1.58 s from the first exit to the last, 0 jams |
| MIXED P N P N (6.0 m/s at 65 deg, and 7.5 m/s at 58 deg; 9 bursts each) | 1.81-1.85 s from the first ball's exit to the last, 0 jams; speed error 0.74-1.01 % rms, elevation 0.77-1.44 deg |

## What is NOT verified
- Nothing here has been built. The frame's goBILDA parts are placed from their own CAD; the bearings' flanges, the 25T pulley and
  the hub bolt patterns are to check against the bought parts. The hood parts, the latch and the pin follow the servo angle in this model; their contacts
  are checked as solids (no overlaps over the whole servo travel), not simulated as forces.
- The squishy's stiffness is fitted to one hand measurement; its deepest squash (33 mm here) is a guess.
- The servo's loop gains and rotor inertia are guesses, and the hood is rigid against the balls here: how far a real servo
  gives under the ball's push is not measured. Our speed loop runs in this simulation; the robot's code does not run it yet.
- The ball, tread and air laws are the testbench's (calibrated against the goBILDA StarterBot launcher's shots and our
  ball measurements); this geometry has never met a real ball.

## How this page works
MuJoCo 3.13 compiled to WebAssembly runs the same model file as our Python bench; three.js draws it. TARGET mode solves
wheel speed and hood angle from the variant's calibration; DIRECT mode lets you set them yourself.
