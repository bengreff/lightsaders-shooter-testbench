# Lightsaders BIOBUZZ turret v5

Our FTC team's turret shooter for the 2026-27 BIOBUZZ season, for both balls, simulated. This page shows the **contact
geometry only**: the parts that touch a ball, the flywheel, the feed wheel and the turret's bore. Structure, bearings and
fasteners come later.

## The design
- **One motor:** a goBILDA Hogback flywheel (96 mm, steel discs) and, geared off the same motor (1 : 3.8), a squishy feed
  wheel that pushes each ball up the turret's bore against its front wall and into the flywheel.
- **Every hood part turns about the flywheel axle**, and every ball-contact face is a surface of revolution about that
  axle, so each ball sees the same gap all along its path:
  - **POLLEN hood:** face radius 118.3 mm about the axle, trough section radius 39.9 mm. Two parts: a lower half (the
    entry plus two side teeth) that rests on a hard stop and continues the bore wall, and a top half (one central tooth).
  - **NECTAR hood:** face radius 135.85 mm, section radius 50.6 mm. Two static side teeth that are part of the bore ring,
    and one central tooth.
  - The POLLEN parts slide 1 mm under the NECTAR face: NECTAR fills the space behind POLLEN with no gap.
- **One servo does every change:**
  - 0-31 deg: POLLEN exit angle (56-87 deg), only the central top tooth moves;
  - at 31 deg the top tooth picks up the lower half; both swing up over the flywheel, out of NECTAR's way;
  - from 182 deg they pull NECTAR's central tooth out: NECTAR exit angle 58-88 deg.

## Measured in this simulation (40 shots per setting, about 7 m/s)
| | exit angle | speed spread | angle spread |
|---|---|---|---|
| POLLEN lowest / middle / highest | 56.4 / 71.0 / 86.7 deg | 0.32 / 0.35 / 0.34 % | 0.86 / 0.83 / 0.73 deg |
| NECTAR lowest / middle / highest | 57.7 / 72.0 / 87.9 deg | 0.30 / 0.26 / 0.24 % | 0.86 / 0.28 / 0.45 deg |

No jams in those 240 shots. TARGET mode's own check (30 random targets per ball): POLLEN 1.6 % / 1.4 deg rms error, NECTAR
1.7 % / 1.1 deg.

## What is NOT verified
- Nothing here has been built. The hood parts' couplings (the lower half's stop and pick-up, the NECTAR tooth's pull) are
  not modelled: those parts simply follow the servo angle. The servo's speed is assumed (300 deg/s).
- POLLEN can jam at low flywheel speeds (1600-2000 rpm) in the calibration's shot sequence; not yet explained.
- The ball, tread and air laws are the testbench's (calibrated against the goBILDA StarterBot launcher's shots and our
  ball measurements: POLLEN rebounds 40 cm from 1 m); this geometry has never met a real ball.

## How this page works
MuJoCo 3.13 compiled to WebAssembly runs the same model file as our Python bench; three.js draws it. TARGET mode solves
wheel speed and hood angle from the variant's calibration; DIRECT mode lets you set them yourself.
