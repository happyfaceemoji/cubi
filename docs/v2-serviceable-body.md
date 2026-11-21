# Cubi v2: serviceable body (19 Nov 2026)

Files: `hardware/v2/` (STLs, print notes, fit-check results) and `cad/cubi_v2_parts.py`.
Regenerate: `python cad/cubi_v2_parts.py hardware/v1/cubi_shell_v7.stl` (needs `ponder_parts.py` beside it).

## Concept
- **Hood**: the v7 shell skin with every internal mount removed. U-slots from the hip holes and switch hole down to the bottom edge, so it lifts straight off after 4 M2x6 screws. No wires go to the hood.
- **Chassis**: base plate + front, mid and rear bulkheads + 4 servo frames + speaker rib + touch paddle. 0.3 mm clearance to the hood everywhere.
- **Servos** load from outside: tabs on inboard pads, screws from outside.
- **Camera + XIAO** drop into an open-top channel with a keeper clip and lift out as one unit. A 1 mm foam pad behind the XIAO keeps the B2B seated (fix for the 18 Nov fault).
- **Distance sensor**: swappable pod at 30/35/40 deg, one M2x4 countersunk screw. Fix path for the flat aim: try 30, then 35 or 40.
- **Battery**: through-pocket with a 1.6 mm door underneath (2x M2x4). Swap without opening the hood.

## Status
- Fit check 64/64 (static fit, hood lift path, servo pull-out, camera+XIAO lift, battery drop, pod removal, ToF beam and camera cones).
- Hip coupon v3 printed: slot came out 19.67 mm vs a 19.90 mm servo case. Filed to fit; the model is left at 20.00, so expect to file the chassis slots too.
- Weight +9 g vs v7. Grip was already marginal at ~100 g.
- Older docs' "left/right eye" labels are mirrored; v2 says "camera side" and "sensor side".
