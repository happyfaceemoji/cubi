"""
Cubi v2 - serviceable body: a HOOD that lifts straight off a CHASSIS.

  hood    = the built v7 shell skin (eyes, bezels, USB-C, grille, hip teardrops, corner screw bosses), with
            every internal mount removed and U-slots so it slides up past the servo outputs and the switch.
  chassis = base plate + front bulkhead (camera, distance-sensor pod) + mid bulkhead (XIAO backstop, amp)
            + rear bulkhead (switch, antenna) + four servo frames (servos screw in from OUTSIDE) + speaker rib
            + touch paddle. Every part and wire lives on the chassis, so taking the hood off disturbs nothing.
  extras  = ToF eye pod (30/38/45 deg), camera keeper clip, battery door (battery out from underneath).

Run:  pip install manifold3d trimesh numpy
      python cubi_v2_parts.py  path/to/cubi_shell_v7.stl
All dimensions mm. Robot axes: x = right, y = up, z = forward. Origin = body centre (hood is 48 cube).
"""
import os, sys, math
import numpy as np
import trimesh
from manifold3d import Manifold, Mesh, CrossSection
import ponder_parts as P
from ponder_parts import rr, box, prism_y, prism_x, cyl, union, SEG

# ---------------- carried over from v7 (measured off cubi_shell_v7.stl) ----------------
HX = HZ = 24.0; HY = 24.0
WALL = 1.6
IX = IZ = HX - WALL          # 22.4 hood inner faces
CEIL = HY - WALL             # 22.4 hood ceiling (underside)
HIP_Y, HIP_Z = -14.8, 12.5   # servo shaft centre
TD_R, TD_TIP = 4.62, -4.94   # hip teardrop: circle radius and top of the point
EYE_X, EYE_Y = 10.5, 13.3    # camera lens / sensor eye centres
OVAL_C = (10.5, 12.3)        # eye oval centre (x +-, y); opening 9 x 12
OVAL = (9.0, 12.0)
SW_Y, SW_R = -1.6, 3.33      # switch bushing hole centre / radius (back wall, x = 0)
SW_BODY = (8.4, 13.4)
USB_Y_V7, USB_Z_V7 = 13.3, 10.1
BOSS = 19.8                  # corner screw bosses (stay in the hood)

# ---------------- v2 parameters ----------------
GAP = 0.3                    # chassis-to-hood clearance everywhere
CI = IX - GAP                # 22.1 chassis outer limit (x and z)
CTOP = CEIL - GAP            # 22.1 chassis top
FLOOR = -HY                  # -24: top of the base plate = bottom edge of the hood

# Camera: the lens may no longer poke into the wall (the hood has to lift off), so the whole XIAO stack sits
# 1.9 mm further back than v7 and the front bulkhead is now the "wall" the camera module seats against.
LENS_PROUD = 1.6             # MEASURE: lens tip ahead of the camera module's front face (<= 1.6 fits)
BH_T = 1.6                   # bulkhead thickness
FB1 = CI                     # front bulkhead front face (22.1)
FB0 = FB1 - BH_T             # 20.5 = camera module front face
STACK_SHIFT = (IZ - FB0)     # 1.9 back vs v7
CAM_SQ = 8.2                 # drop-in channel (8.0 module measured; slides in from the top)
CAM_DEPTH = 4.6
CAM_WALL = 1.2
LENS_SLOT = 5.4              # lens barrel 5.0 + 0.4
XIAO_T = 15.5                # stack incl camera, front face at FB0
XIAO_Z0 = FB0 - XIAO_T       # 5.0 back face of the XIAO
FOAM = 1.0                   # foam pad between XIAO back and the mid bulkhead (keeps the B2B seated)
MB1 = XIAO_Z0 - FOAM         # 4.0 mid bulkhead front face
MB0 = MB1 - BH_T             # 2.4
RB0 = -CI                    # -22.1 rear bulkhead back face
RB1 = RB0 + BH_T             # -20.5
USB_Z = USB_Z_V7 - STACK_SHIFT   # 8.2 new USB-C centre on the left wall
USB_OVAL = (9.9, 4.6)        # (tall, wide): 0.4 wider than v7 for stack-position tolerance
USB_LIP = 0.6
USB_BEVEL_IN = (14.0, 8.0)

# Servo (GH-S37D, tabs trimmed to 3.8): case top face 0.3 inside the hood wall
SERVO_L, SERVO_T, SERVO_H = 20.0, 9.0, 17.5
SHAFT_FROM_END = 5.0
TAB_OVER, TAB_DEPTH, TAB_T = 3.8, 3.2, 1.5
S_TOP = CI                                  # case top face x (22.1)
S_BOT = S_TOP - SERVO_H                     # 4.6
TAB_FACE = S_TOP - TAB_DEPTH                # 18.9 tab outer face (screw goes in from here)
PAD1 = TAB_FACE - TAB_T                     # 17.4 pad face (tab sits on it from OUTSIDE)
PAD0 = PAD1 - 3.2                           # 14.2
CASE_Y0 = HIP_Y - SHAFT_FROM_END            # -19.8
CASE_Y1 = CASE_Y0 + SERVO_L                 # 0.2
COL_Z = (HIP_Z - SERVO_T / 2 - 0.2 - 1.6, HIP_Z - SERVO_T / 2 - 0.2)   # column beside the case (6.1..7.7)

# Battery: Liter 602035 (6 x 20 x 35) flat, now loaded from UNDERNEATH through a door
PLATE_T = 4.9
PLATE_Y0 = FLOOR - PLATE_T                  # -28.9
BATT_POCKET = (21.6, 36.6)                  # x, z (v7 flat-battery plate)
LEAD_NOTCH = (5.6, 2.3)                     # rear lead notch x width, z extra
DOOR_T = 1.6
DOOR = (25.6, 44.6)                         # door x, z
DOOR_Z0 = -22.6
DOOR_SCREWS = [(0.0, 20.9), (6.0, -20.9)]   # M2 x 4 countersunk
BATT_TOP = PLATE_Y0 + DOOR_T + 6.0          # -21.3

# Distance sensor GY-530 (measured 13.37 x 10.75), chip side forward, SCL/SDA pads toward the robot's middle
TOF_BOARD = (13.37, 10.75, 1.6)
CHIP_UV = (2.515, 2.625)                    # chip centre from board centre (u = +x outward, v = up the board)
CHIP_H = 1.0
TOF_TILTS = (30.0, 35.0, 40.0)              # print all three; 30 = original design (45 clips the eye)
TOF_FOV_HALF = 14.0                         # window half-angle (sensor FoV 25)

# Speaker 1511 (15 x 11 x 3.5) on the right, behind the v7 grille (y 13.5, z 2)
SPK_Y, SPK_Z = 13.5, 2.0
SPK = (15.0, 11.0, 3.5)
SPK_RIB = (CI - SPK[2] - 0.3 - 0.6, CI - SPK[2] - 0.3)   # rib behind the speaker (x)

M2_PILOT = 1.7
M2_CLEAR = 2.4


# ---------------- helpers ----------------
def tm_to_manifold(t):
    return Manifold(Mesh(vert_properties=np.asarray(t.vertices, np.float32),
                         tri_verts=np.asarray(t.faces, np.uint32)))


def manifold_to_tm(m):
    mm = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts), process=False)


def load_v7(path):
    """v7 STL is in print pose (top face down). Back to robot coordinates."""
    t = trimesh.load(path)
    v = t.vertices
    r = np.c_[v[:, 0], HY - v[:, 2], v[:, 1] + 0.7]
    t = trimesh.Trimesh(r, t.faces, process=True)
    assert t.is_watertight and t.volume > 0
    return tm_to_manifold(t)


def inside_hood(y0=FLOOR, y1=CTOP, g=GAP):
    """Everything the chassis may occupy above the plate."""
    return prism_y(rr(2 * (IX - g), 2 * (IZ - g), P.SHELL_R - WALL - g), y0, y1)


def boss_keepout():
    """Hood corner bosses + gussets, grown by GAP and extruded all the way up: the hood lifts straight off,
    so nothing on the chassis may sit above them."""
    k = BOSS - P.BOSS_R - GAP                     # 16.8
    return union([box(*sorted((sx * k, sx * 30)), FLOOR - 1, 30, *sorted((sz * k, sz * 30)))
                  for sx in (-1, 1) for sz in (-1, 1)])


# ---------------- ToF frame ----------------
def tof_pose(tilt):
    """Chip-top centre P for a tilt: as far forward as the pod allows, aimed through the eye-oval centre."""
    t = math.radians(tilt)
    v_top = TOF_BOARD[1] / 2 - CHIP_UV[1] + 1.0          # border above the chip
    pz = FB0 - (v_top * math.sin(t) + 0.6 * math.cos(t)) - 0.05
    py = OVAL_C[1] + (HZ - 0.1 - pz) * math.tan(t)
    return (EYE_X, py, pz)


def tof_frame(m, tilt):
    """Local frame: u = x, v = up the board, w = out of the chip (w = 0 at chip top). Tilt looks DOWN."""
    return m.rotate([tilt, 0, 0]).translate(list(tof_pose(tilt)))


def tof_cone(tilt, half=TOF_FOV_HALF, length=40.0):
    return tof_frame(Manifold.cylinder(length, 1.6, 1.6 + length * math.tan(math.radians(half)), SEG), tilt)


def tof_pod(tilt):
    """Swappable distance-sensor wedge. Board pushes in from behind into a pocket; the face touches only the
    board's 1 mm border. One M2 countersunk screw from the FRONT of the bulkhead holds it (hood off)."""
    bw, bh, bt = TOF_BOARD
    cu, cv = CHIP_UV
    bu0, bu1 = -bw / 2 - cu, bw / 2 - cu                # board in chip-centred coords
    bv0, bv1 = -bh / 2 - cv, bh / 2 - cv
    wf = -CHIP_H                                        # board front face
    face = box(bu0 - 1.25, bu1 + 1.25, bv0 - 1.25, bv1 + 1.0, wf - bt - 0.6, 0.6)
    face = tof_frame(face, tilt)
    pz = tof_pose(tilt)[2]
    # vertical front slab against the bulkhead, same footprint as the face's projection
    bb = face.bounding_box()
    x0, y0, _, x1, y1, _ = bb
    slab = box(x0, x1, y0, min(y1, CTOP - 0.1), FB0 - 0.6, FB0 - 0.05)
    body = Manifold.batch_hull([face, slab])
    body = body ^ box(-5, 30, FLOOR + 6, CTOP - 0.1, -10, FB0 - 0.05)
    cuts = [
        tof_frame(box(bu0 - 0.25, bu1 + 0.25, bv0 - 0.25, bv1 + 0.25, wf - bt - 0.1, wf), tilt),          # board
        tof_frame(box(bu0 - 0.25, bu1 + 0.25, bv0 - 0.25, bv1 + 0.25, wf - bt - 20, wf - bt), tilt),      # open behind
        tof_frame(box(bu0 + 1.0, bu1 - 1.0, bv0 + 1.0, bv1 - 0.5, wf - 0.1, 0.25), tilt),                 # parts recess
        tof_frame(box(bu0 - 3, bu0 + 0.5, bv0 + 1.5, bv1 - 1.5, wf - bt - 3, wf + 0.2), tilt),            # wire notch (middle side)
        tof_cone(tilt, TOF_FOV_HALF, 30),
    ]
    for c in cuts:
        body = body - c
    sx, sy = pod_screw(tilt)
    body = body - cyl('z', sx, sy, M2_PILOT / 2, FB0 - 4.0, FB0 + 1)
    return body


def pod_screw(tilt=None):
    # one spot that is solid in all three pods (searched), so the bulkhead needs a single hole
    return 3.5, 12.25


# ---------------- hood ----------------
def hood(v7):
    e = 0.003                                               # eats the coplanar inner faces, no slivers
    interior = prism_y(rr(2 * (IX + e), 2 * (IZ + e), P.SHELL_R - WALL + e), -HY - 1, CEIL + e)
    corners = union([box(*sorted((sx * 17.05, sx * 30)), -HY - 1, -9.0, *sorted((sz * 17.05, sz * 30)))
                     for sx in (-1, 1) for sz in (-1, 1)])
    h = (v7 - interior) + (v7 ^ interior ^ corners)          # skin + the corner screw bosses

    cuts = []
    for sx in (-1, 1):                                      # hip U-slots: teardrop down to the bottom edge
        for sz in (-1, 1):
            cuts.append(box(*sorted((sx * (IX - 0.5), sx * (HX + 1))), -HY - 1, HIP_Y,
                            sz * HIP_Z - TD_R, sz * HIP_Z + TD_R))
    cuts.append(box(-SW_R, SW_R, -HY - 1, SW_Y, -HZ - 1, -IZ + 0.5))      # switch U-slot
    for c in cuts:
        h = h - c

    # USB-C: fill the v7 opening, re-cut 1.9 mm further back (the XIAO moved back with the camera)
    h = h + (box(-HX, -IX + 0.01, USB_Y_V7 - 8.5, USB_Y_V7 + 8.5, USB_Z_V7 - 6.5, USB_Z_V7 + 6.5)
             ^ prism_y(rr(2 * HX, 2 * HZ, P.SHELL_R), -HY, HY))
    uy, uz = USB_Y_V7, USB_Z
    h = h - prism_x(rr(USB_OVAL[1], USB_OVAL[0], USB_OVAL[1] / 2, -uz, uy), -HX - 1, -IX + 1)
    d = HX - IX - USB_LIP
    th, tw = USB_BEVEL_IN

    def slab(x, grow):
        return prism_x(rr(tw + 2 * grow, th + 2 * grow, (tw + 2 * grow) / 2, -uz, uy), x - 0.01, x)
    h = h - Manifold.batch_hull([slab(-IX - USB_LIP, 0.0), slab(-HX - 0.6, d + 0.6)])
    return h


# ---------------- chassis ----------------
def base_plate():
    p = prism_y(rr(2 * HX, 2 * HZ, P.SHELL_R), PLATE_Y0, FLOOR)
    for sx in (-1, 1):
        for sz in (-1, 1):
            p = p - cyl('y', sx * BOSS, sz * BOSS, M2_CLEAR / 2, PLATE_Y0 - 1, FLOOR + 1)
            p = p - cyl('y', sx * BOSS, sz * BOSS, P.M2_HEAD_CB / 2, PLATE_Y0 - 1, FLOOR - P.PLATE_GRIP)
    bx, bz = BATT_POCKET
    p = p - box(-bx / 2, bx / 2, PLATE_Y0 - 1, FLOOR + 1, -bz / 2, bz / 2)                       # through pocket
    p = p - box(-LEAD_NOTCH[0] / 2, LEAD_NOTCH[0] / 2, PLATE_Y0 - 1, FLOOR + 1,
                -bz / 2 - LEAD_NOTCH[1], 0)                                                     # lead notch (rear)
    dx, dz = DOOR
    p = p - box(-dx / 2 - 0.2, dx / 2 + 0.2, PLATE_Y0 - 1, PLATE_Y0 + DOOR_T, DOOR_Z0 - 0.2, DOOR_Z0 + dz + 0.2)
    for (x, z) in DOOR_SCREWS:
        p = p - cyl('y', x, z, M2_PILOT / 2, PLATE_Y0, FLOOR - 0.6)
    return p


def battery_door():
    dx, dz = DOOR
    d = box(-dx / 2, dx / 2, PLATE_Y0, PLATE_Y0 + DOOR_T, DOOR_Z0, DOOR_Z0 + dz)
    for (x, z) in DOOR_SCREWS:
        d = d - cyl('y', x, z, M2_CLEAR / 2, PLATE_Y0 - 1, PLATE_Y0 + DOOR_T + 1)
        d = d - Manifold.cylinder(1.1, 3.9 / 2, 2.2 / 2, SEG).rotate([-90, 0, 0]).translate([x, PLATE_Y0, z])  # countersink
    d = d - box(-4, 4, PLATE_Y0 - 1, PLATE_Y0 + 0.5, DOOR_Z0 + dz - 3.5, DOOR_Z0 + dz - 1.5)   # pry notch
    return d


def front_bulkhead():
    b = box(-CI, CI, FLOOR, CTOP, FB0, FB1)
    ex, ey = -EYE_X, EYE_Y
    # camera channel behind it: open at the TOP so camera + XIAO lift out as one unit
    sq = CAM_SQ / 2
    ch = box(ex - sq - CAM_WALL, ex + sq + CAM_WALL, ey - sq - CAM_WALL, ey + sq + 0.2, FB0 - CAM_DEPTH, FB0 + 0.01)
    ch = ch - box(ex - sq, ex + sq, ey - sq, ey + 20, FB0 - CAM_DEPTH - 1, FB0 + 0.1)
    shelf = box(ex - sq - CAM_WALL, ex + sq + CAM_WALL, ey - sq - CAM_WALL, ey - sq, FB0 - CAM_DEPTH, FB0)
    foot = box(ex - sq - CAM_WALL, ex + sq + CAM_WALL, ey - sq - CAM_WALL - CAM_DEPTH, ey - sq - CAM_WALL - CAM_DEPTH + 0.01, FB0 - 0.01, FB0)
    b = b + ch + Manifold.batch_hull([shelf, foot])                                            # 45 deg gusset under the channel
    b = b - cyl('z', ex, ey, LENS_SLOT / 2, FB0 - 1, FB1 + 1)
    b = b - box(ex - LENS_SLOT / 2, ex + LENS_SLOT / 2, ey, CTOP + 1, FB0 - 1, FB1 + 1)        # lens slot to the top
    # distance sensor window: union of the pod variants' cones, only through the bulkhead
    win = union([tof_cone(t, TOF_FOV_HALF + 1.0, 30) for t in TOF_TILTS])
    b = b - (win ^ box(-30, 30, -30, 30, FB0 - 0.01, FB1 + 1))
    # pod screws: M2 countersunk from the front (heads sit flush, hood clears by 0.3)
    sx, sy = pod_screw()
    b = b - cyl('z', sx, sy, M2_CLEAR / 2, FB0 - 1, FB1 + 1)
    b = b - Manifold.cylinder(1.15, 2.2 / 2, 4.0 / 2, SEG).translate([sx, sy, FB1 - 1.15 + 0.01])
    b = b - box(-14.0, 14.0, -21.0, -3.0, FB0 - 1, FB1 + 1)                                       # wire / lightening window
    return b


def mid_bulkhead():
    b = box(-CI, CI, FLOOR, CTOP, MB0, MB1)
    b = b - box(-BATT_POCKET[0] / 2 - 0.5, BATT_POCKET[0] / 2 + 0.5, FLOOR - 1, BATT_TOP + 0.4, MB0 - 1, MB1 + 1)  # battery arch
    b = b - box(-8.5, 8.5, -16.0, 1.0, MB0 - 1, MB1 + 1)                                       # wire window
    b = b - box(-21.0, -12.0, -18.0, -2.0, MB0 - 1, MB1 + 1)
    b = b - box(12.0, SPK_RIB[0] - 0.6, -18.0, -2.0, MB0 - 1, MB1 + 1)
    b = b - box(SPK_RIB[1], 30, SPK_Y - SPK[1] / 2 - 1.0, SPK_Y + SPK[1] / 2 + 1.0, MB0 - 1, MB1 + 1)  # speaker
    b = b + box(0.3, 1.8, 6.0, 20.0, MB1 - 0.01, MB1 + 2.4)                                    # XIAO right-edge stop
    return b


def rear_bulkhead():
    b = box(-CI, CI, FLOOR, CTOP, RB0, RB1)
    b = b - cyl('z', 0, SW_Y, SW_R, RB0 - 1, RB1 + 1)                                           # switch bushing
    b = b - box(-SW_BODY[0] / 2, SW_BODY[0] / 2, SW_Y - SW_BODY[1] / 2, SW_Y + SW_BODY[1] / 2, RB1 - 0.6, RB1 + 1)
    # tongue that fills the hood's switch U-slot below the bushing (flush with the outside, locates the hood)
    b = b + box(-SW_R + 0.2, SW_R - 0.2, FLOOR, SW_Y - SW_R - 0.4, -HZ, RB0 + 0.01)
    b = b - box(-14, 14, 8.0, 20.0, RB0 - 1, RB0 + 0.4)                                        # antenna recess (back face)
    for s in (-1, 1):
        b = b - box(*sorted((s * 6.5, s * 15.5)), -20.0, -9.0, RB0 - 1, RB1 + 1)                 # lightening / wire windows
    for x in (-12.0, 12.0):                                                                     # zip-tie slots
        for y in (-12.0, 2.0):
            b = b - box(x - 1.5, x + 1.5, y, y + 1.4, RB0 - 1, RB1 + 1)
    return b


def servo_frames():
    parts = []
    for sx in (-1, 1):
        for sz in (-1, 1):
            xa = sorted((sx * PAD0, sx * PAD1))
            zc = sz * HIP_Z
            zt = SERVO_T / 2
            parts.append(box(*xa, FLOOR, CASE_Y0, zc - zt, zc + zt))                            # lower pad
            zb = sorted((sz * COL_Z[0], sz * (FB0 if sz > 0 else -RB1)))
            parts.append(box(*xa, CASE_Y1, CASE_Y1 + TAB_OVER + 0.2, *zb))                     # upper pad / bar to bulkhead
            parts.append(box(*xa, FLOOR, CASE_Y1 + TAB_OVER + 0.2, *sorted((sz * COL_Z[0], sz * COL_Z[1]))))   # column
    return union(parts)


def speaker_cradle():
    r0, r1 = SPK_RIB
    z0, z1 = SPK_Z - SPK[0] / 2 - 0.2, SPK_Z + SPK[0] / 2 + 0.2
    yb, yt = SPK_Y - SPK[1] / 2 - 0.2, SPK_Y + SPK[1] / 2 + 0.2
    zr0, zr1 = max(z0, -(HIP_Z - SERVO_T / 2 - 0.3)), min(z1, HIP_Z - SERVO_T / 2 - 0.3)    # rib clears the servos
    rib = box(r0, r1, FLOOR, yt + 0.8, zr0, zr1)
    # bottom ledge with a 45 deg underside (prints without support)
    sec = CrossSection([[(r1 - 0.01, yb - 1.0), (CI - 0.4, yb - 1.0), (CI - 0.4, yb), (r1 - 0.01, yb - 1.0 - (CI - 0.4 - r1))]])
    ledge = sec.extrude(zr1 - zr0).translate([0, 0, zr0])
    stop = box(r1 - 0.01, CI - 0.6, yb - 1.0, yt, zr0 - 0.8, zr0)                               # rear end stop
    c = rib + ledge + stop
    c = c - box(r0 - 1, r1 + 1, SPK_Y - 2, SPK_Y + 2, SPK_Z - 1.5, SPK_Z + 1.5)                # wire hole
    return c


def touch_paddle():
    return box(-9.0, 9.0, CTOP - BH_T, CTOP, RB1 - 0.01, MB0 + 0.01)


def chassis():
    upper = union([front_bulkhead(), mid_bulkhead(), rear_bulkhead(), servo_frames(), speaker_cradle(), touch_paddle()])
    upper = (upper ^ inside_hood(FLOOR - 0.01, CTOP)) + (rear_bulkhead() ^ box(-5, 5, FLOOR, 0, -HZ, -CI + 0.01))
    upper = upper - boss_keepout()
    return base_plate() + upper


def cam_keeper():
    """U-clip over the top of the camera channel: stops camera + XIAO lifting out. Pull it to service."""
    ex, ey = -EYE_X, EYE_Y
    sq = CAM_SQ / 2
    xo = sq + CAM_WALL
    top = ey + sq + 0.2
    k = box(ex - xo - 1.15, ex + xo + 1.15, top + 0.0, top + 1.2, FB0 - CAM_DEPTH + 0.3, FB0 - 0.3)
    for s in (-1, 1):
        k = k + box(*sorted((ex + s * (xo + 0.15), ex + s * (xo + 1.15))), top - 3.0, top + 0.01,
                    FB0 - CAM_DEPTH + 0.3, FB0 - 0.3)
    k = k + box(ex - sq + 0.3, ex + sq - 0.3, ey + 4.1, top + 0.01, FB0 - CAM_DEPTH + 0.3, FB0 - 1.0)   # rests on the module top
    return k


# ---------------- stand-ins for the bought parts (fit check) ----------------
def servo_model(sx, sz):
    zc = sz * HIP_Z
    a = sorted((sx * S_BOT, sx * S_TOP))
    case = box(*a, CASE_Y0, CASE_Y1, zc - SERVO_T / 2, zc + SERVO_T / 2)
    tabs = box(*sorted((sx * (TAB_FACE - TAB_T), sx * TAB_FACE)), CASE_Y0 - TAB_OVER, CASE_Y1 + TAB_OVER,
               zc - SERVO_T / 2, zc + SERVO_T / 2)
    xt = sx * S_TOP
    feat = [cyl('x', 0, 0, 1.0, *sorted((xt, xt + sx * 1.5))).scale([1, 2.5, 4.17]).translate([0, HIP_Y, zc]),   # 8.34 x 5 oval boss
            box(*sorted((xt, xt + sx * 1.0)), HIP_Y + 2.5, HIP_Y + 5.5, zc - 2.5, zc + 2.5),      # dome 5 x 3 at +4
            box(*sorted((xt, xt + sx * 0.8)), HIP_Y + 7.6, HIP_Y + 9.4, zc - 0.88, zc + 0.88),    # small dome at +8.48
            cyl('x', HIP_Y, zc, P.TOWER_R, *sorted((xt, xt + sx * P.TOWER_H)))]
    return case + tabs + union(feat)


def parts_models():
    m = {}
    m['servos'] = union([servo_model(sx, sz) for sx in (-1, 1) for sz in (-1, 1)])
    ex, ey = -EYE_X, EYE_Y
    m['xiao_stack'] = box(-21.0, 0.0, ey - 8.9, ey + 8.9, XIAO_Z0, FB0 - CAM_DEPTH) \
        + box(ex - 4.0, ex + 4.0, ey - 4.0, ey + 4.0, FB0 - CAM_DEPTH, FB0) \
        + cyl('z', ex, ey, 2.5, FB0, FB0 + LENS_PROUD)
    bx, bz = 20.0, 35.0
    m['battery'] = box(-bx / 2, bx / 2, PLATE_Y0 + DOOR_T, BATT_TOP, -bz / 2, bz / 2)
    m['speaker'] = box(CI - SPK[2], CI, SPK_Y - SPK[1] / 2, SPK_Y + SPK[1] / 2, SPK_Z - SPK[0] / 2, SPK_Z + SPK[0] / 2)
    m['switch'] = box(-SW_BODY[0] / 2 + 0.2, SW_BODY[0] / 2 - 0.2, SW_Y - SW_BODY[1] / 2 + 0.2, SW_Y + SW_BODY[1] / 2 - 0.2,
                      RB1 - 0.6, RB1 + 8.0) + cyl('z', 0, SW_Y, 3.175, -HZ - 3, RB1)
    m['amp'] = box(-9.0, 9.0, 1.0, 20.0, MB0 - 3.0, MB0)          # Adafruit 3006 on the mid bulkhead's back
    return m


def tof_board(tilt):
    bw, bh, bt = TOF_BOARD
    cu, cv = CHIP_UV
    b = box(-bw / 2 - cu, bw / 2 - cu, -bh / 2 - cv, bh / 2 - cv, -CHIP_H - bt, -CHIP_H) + box(-1.2, 1.2, -2.0, 2.0, -CHIP_H, 0)
    return tof_frame(b, tilt)


# ---------------- export ----------------
def export(m, name, rot=(0, 0, 0), outdir='stl_v2'):
    m = m.rotate(list(rot))
    t = manifold_to_tm(m)
    t.apply_translation([-t.bounds[0][0] - t.extents[0] / 2, -t.bounds[0][1] - t.extents[1] / 2, -t.bounds[0][2]])
    os.makedirs(outdir, exist_ok=True)
    t.export(os.path.join(outdir, name))
    e = t.extents
    print(f'{name:34s} {e[0]:6.1f} x {e[1]:6.1f} x {e[2]:6.1f} mm  {t.volume / 1000 * 1.24:5.1f} g  watertight={t.is_watertight}')
    return t


if __name__ == '__main__':
    v7 = load_v7(sys.argv[1] if len(sys.argv) > 1 else 'cubi_shell_v7.stl')
    H = hood(v7)
    C = chassis()
    export(H, 'cubi_v2_hood.stl', rot=(-90, 0, 0))                  # top face on the bed, like v7
    export(C, 'cubi_v2_chassis.stl', rot=(90, 0, 0))                # base plate on the bed
    for t in TOF_TILTS:
        export(tof_pod(t), f'cubi_v2_tof_pod_{int(t)}deg.stl', rot=(180, 0, 0))   # front face on the bed
    export(cam_keeper(), 'cubi_v2_camera_keeper.stl', rot=(-90, 0, 0))
    export(battery_door(), 'cubi_v2_battery_door.stl', rot=(-90, 0, 0))
    # coupons (rule: every new interface gets a coupon before the full part)
    eyes = C ^ box(-17.0, 17.5, 5.0, CTOP + 1, 9.5, HZ)
    export(eyes, 'cubi_v2_coupon_eyes.stl', rot=(90, 0, 0))
    export(H ^ box(-17.0, 17.5, 4.0, HY + 1, IZ - 0.01, HZ + 3), 'cubi_v2_coupon_eyes_hood.stl', rot=(0, 0, 0))
    hip = C ^ box(12.5, HX + 1, PLATE_Y0 - 1, CASE_Y1 + TAB_OVER + 0.3, 4.5, FB1 + 0.01)
    export(hip, 'cubi_v2_coupon_hip_v3.stl', rot=(90, 0, 0))
