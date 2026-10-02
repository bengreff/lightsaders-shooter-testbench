# Lightsaders BIOBUZZ prototype shooter

A design proposal for the turret's shooter, simulated with the physics we calibrated against the goBILDA StarterBot
launcher. Only the turret is modelled: the intake below it is abstracted (a ball arrives at rest in the entry stage).

## The design
- **Flywheel:** two goBILDA Hogback 96 mm 50A wheels with steel flywheels (3628-0032-0082) on an 8 mm REX shaft, driven by
  **one** Yellow Jacket 1:1 through a 5 mm HTD belt outside the right plate (16 T on the motor, 24 T on the shaft).
- **Hood:** one smooth polycarbonate arc whose radius sits halfway between the POLLEN and NECTAR ideals. Its arms ride on
  the flywheel shaft in bearings, so the hood is **concentric** with the wheel:
  - the **angle servo** (goBILDA 2000 Speed, 5:1 sector gear) swings it along that circle to set the exit angle;
  - the **gap servo** (2000 Torque) turns a two-dwell cam that moves the shell between the POLLEN and NECTAR settings.
    At a dwell the pinch load goes through the cam, not the servo.
- **Entry:** a ball waits in the hood's mouth between two sprung 1 in Stealth rollers that ride on the hood arms, each
  turned by a goBILDA Super Speed servo (continuous mode). FIRE spins them and they hand the ball into the pinch the same
  way at every hood angle. The path from the turret's bore up to the mouth (the intake/transfer) comes later; here the
  ball simply appears in the mouth.
- **Turret:** the shooter stands on a printed ring gear (192 T, module 1.25) on an AndyMark am-5039 turntable bearing,
  turned by a pinion from outside later.

## What is modelled
- **The same physics as our goBILDA StarterBot model:** the same contact, ball and air laws reproduce that robot's shot
  (5.40 m/s at 70.6 deg, against 5.48 m/s at 70.4 deg from the real robot's video).
- **Contact:** the ball-on-rubber pinch law from material numbers (Gent + Hertz + the ball's shell). Bounce and stiffness
  are calibrated to measurements: a POLLEN rebounds 40 cm from 1 m (e = 0.63), and squeezes ~2 mm at 130-220 N.
- **Motors:** the Yellow Jackets' published stall and free points, with winding resistance, back-EMF and internal friction.
  The battery sags with current (12.6 V, 0.08 ohm).
- **Drive and control:**
  - belt stretch, and bearing friction on the shaft;
  - the FTC hub's RUN_USING_ENCODER velocity loop on the motor's encoder;
  - the servos' speed, stall torque and 0.6 deg deadband.
- **Air:** drag and Magnus lift on the balls in flight.
- **Parts:** masses and inertias from the CAD's solids and the vendors' published masses.

## What is NOT verified
- Nothing here has been checked against a real launcher with these balls. The pinch law was validated on the goBILDA
  StarterBot's POLLEN shot only. Tread friction on polyethylene is one assumed number.
- The cam/arm stiffness, printed-part density and belt stiffness are estimates. The belt is modelled ~10x softer than a
  real one so the 0.2 ms step stays stable; it rings far faster than a shot either way.
- NECTAR's bounce and the balls' real masses (nominal 24.9 g / 41.3 g) have not been measured.
- The intake/transfer below the turret is not modelled; the balls appear in the hood's mouth.

## Numbers from the design's own check (fresh seeds, 20 shots per ball at random targets)
| | speed error rms / max | angle error rms / max |
|---|---|---|
| POLLEN | 0.17 % / 0.39 % | 0.04 deg / 0.11 deg |
| NECTAR | 0.17 % / 0.28 % | 0.26 deg / 0.55 deg |

## How this page works
MuJoCo 3.13 compiled to WebAssembly runs the same model file as our Python bench. The same control code is ported to
JavaScript, and a parity check shows the two agree to rounding. three.js draws it. TARGET mode solves wheel speed and
hood angle from each variant's calibration, measured in Python. "Recalibrate" measures a fresh one in your browser.
