# Climbing model (concept, 19 Nov 2026)

Not in this repo yet: `cubi_climb_parts.py` and its STLs. This page records the design.

- Same v2 body and hip servos. Each leg becomes 2-joint: 20 mm thigh pod holding a second GH-S37D as the knee, 26 mm shin, TPU foot cap.
- Knee axis parallel to the hip axis. Front knees point forward, rear knees back.
- ~105 mm leg-to-leg (v2 ~59), ~130 g total.
- Kinematics say she clears ~19 mm steps at 36 mm ride height (18 at 38, 16 at 40). Hip travel limits low ride, leg reach limits high ride. No torque, slip or servo slop modelled.

## Unsolved
- **Pins**: all 11 XIAO pins are used. Four more servos need a PCA9685 on I2C, or drop the amp and touch pad.
- **Power**: 8 servos on one 500 mAh 1S cell with no boost means brownouts are likely.
- **Sensing**: the ToF reads a step as a wall; firmware needs a step-climb behaviour.
- Not printed, not fit-checked.
