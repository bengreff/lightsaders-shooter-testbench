# Lightsaders BIOBUZZ turret v5

Our FTC team's turret shooter for the 2026-27 BIOBUZZ season, for both balls, simulated: the parts that touch a ball and the
mechanisms that move the hood pieces. The side plates, fasteners and mounts come later.

## The design
- **One motor:** a goBILDA Hogback flywheel (96 mm, steel discs) and, geared off the same motor (1 : 3.8), a squishy feed wheel
  that pushes each ball up the turret's bore against its front wall and into the flywheel.
- **One servo** (goBILDA 2000-0025-0003, 25-3 Speed, at 6 V through a REV Servo Hub or servo power module) turns a 40-tooth
  sector gear on the flywheel axle (1.333 : 1), so 213 deg of hood travel is 284 deg of servo.
- **The flywheel's speed loop is the team's code** (the hub's own loop, gains 160 / 0 / 0 / 15.2), on the motor as fitted to our
  robot's logs: no overshoot, it settles about 25 rpm under its target and drops 500 rpm in about half a second.
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
  the lower half is locked to the top half (about 1.5 deg of play) and cannot drop over the top. Built to last: the ring and the
  lower half's rest stops are acetal on steel standoffs, the ring's end is fitted to the hook (no play), and the pin and the
  pawl's pivot are steel shoulder screws clamped to their plates.
- **One servo does every change:**
  - 0-31 deg: POLLEN exit angle (about 56-88 deg), only the central top tooth moves;
  - 31-38 deg: the pin turns the latch open; from 38 deg it lifts the lower half; both swing up over the flywheel;
  - from 182 deg they push NECTAR's central tooth out: NECTAR exit angle about 58-88 deg.
- **Switching POLLEN to NECTAR** takes about 0.64 s and back about 0.48 s, from the servo's catalogue torque and speed, the hood
  parts' weight and inertia, and guessed loop gains. On the way back the code stops the hood just above where the lower half lands
  and eases it down through the landing (about 150 instead of 590 deg/s). **MIXED** fires POLLEN, NECTAR, POLLEN, NECTAR in real
  time: the hood swings as soon as a ball has left it.

## Measured in this simulation
| | |
|---|---|
| Calibration grid (both balls, 1600-4400 rpm, every hood setting) | 468 shots, 0 jams |
| TARGET mode check, 30 random targets per ball | POLLEN 1.0 % speed / 1.3 deg rms error; NECTAR 1.2 % / 0.7 deg |
| Mechanisms over the whole servo travel | no part touches another; every part at least 1.2 mm from the balls |
| MIXED P N P N (6.0 m/s at 65 deg, and 7.5 m/s at 58 deg; 4 bursts) | 1.76-1.85 s from the first ball's exit to the last, 0 jams |

## What is NOT verified
- Nothing here has been built. The hood parts, the latch and the pin follow the servo angle in this model; their contacts
  are checked as solids (no overlaps over the whole servo travel), not simulated as forces.
- The squishy's stiffness is fitted to one hand measurement; its deepest squash (33 mm here) is a guess.
- The servo's loop gains and rotor inertia are guesses, and the hood is rigid against the balls here: how far a real servo
  gives under the ball's push is not measured. The calibration grid was shot with the old speed loop and re-keyed to the new one's
  commands; it has not been re-shot.
- The ball, tread and air laws are the testbench's (calibrated against the goBILDA StarterBot launcher's shots and our
  ball measurements); this geometry has never met a real ball.

## How this page works
MuJoCo 3.13 compiled to WebAssembly runs the same model file as our Python bench; three.js draws it. TARGET mode solves
wheel speed and hood angle from the variant's calibration; DIRECT mode lets you set them yourself.
