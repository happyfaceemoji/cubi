"""
Ponder desk quadruped - printable parts generator.

Run:  pip install manifold3d trimesh numpy
      python ponder_parts.py
Writes one STL per part into ./stl, each already laid flat for printing.

All dimensions in mm. Robot axes while modelling: x = right, y = up, z = forward.
Change the parameters below to match the parts you actually receive, then re-run.
"""
import os, math
import numpy as np
import trimesh
from manifold3d import Manifold, CrossSection, JoinType

SEG = 64

# ---------------- parameters ----------------
# PONDER MINI: 48 x 48 mm footprint (40% smaller than the 80 mm v3, kept in v3_80mm/)
BODY_W, BODY_H, BODY_D = 48.0, 48.0, 48.0   # outer shell size (+ 2.4 mm bottom plate = 50.4 tall)
WALL = 1.6                                   # 4 lines at 0.4 mm
SHELL_R = 4.0                                # vertical edge radius

# Generic 3.7 g sub-micro servo (20 x 8.5 x 17.5 mm case). Mounted upright: shaft out the side wall
# near the BOTTOM of the case, case running up. Measure yours and adjust.
SERVO_L, SERVO_T, SERVO_H = 20.0, 9.0, 17.5   # case length (vertical), thickness (front-back), top face to bottom
SHAFT_FROM_END = 5.0      # shaft centre to the near (bottom) end of the case
TAB_OVER = 3.8            # each tab sticks out this far past the case end
TAB_DEPTH = 3.2           # case top face to the tab's top face
TAB_T = 1.5
TAB_HOLE_UP = SERVO_L - SHAFT_FROM_END + 1.9   # 16.9: tab hole above the shaft
TAB_HOLE_DOWN = SHAFT_FROM_END + 1.9           # 6.9: tab hole below the shaft
TOWER_R, TOWER_H = 2.9, 3.0     # round output boss on the top face
SPLINE_R = 1.9

# Hip servo placement in the shell
HIP_Y = -14.8             # shaft height from body centre (tabs just clear the bottom plate)
HIP_Z = 12.5              # shaft distance front/back from centre

# EYES (no screen): two big round eyes on the front.
#  Left eye  = the camera of a Seeed XIAO ESP32S3 Sense (21 x 17.8 x 15 mm stack incl. camera), board parallel
#              to the front wall, lens in the eye; its USB-C pokes out a hole in the LEFT wall.
#  Right eye = a VL53L0X time-of-flight distance sensor module (up to 13 x 18 mm) behind a matching opening.
# MEASURE YOURS: the camera's lens position on the stack sets XIAO_LENS_DX / DY.
EYE_X, EYE_Y = 10.5, 13.3        # eye centres (+-x)
EYE_OD, EYE_HOLE = 18.0, 10.0    # (round-eye version, kept for reference)
# Tall oval eyes (w, h): the opening stretches DOWN, which is where the distance sensor's beam leaves the right eye.
EYE_RING = (16.5, 20.0)          # bezel ring outside
EYE_OPEN = (9.0, 12.0)           # see-through opening
EYE_BAND = 2.4                   # ring width; inside it a shallow dish slopes to the opening
EYE_DY = -1.0                    # oval centre sits 1 mm below EYE_Y (camera lens / sensor stay put) so the ring
                                 # clears the top edge of the face
EYE_PROUD = 1.4                  # bezel stands this far out of the front face
XIAO_W, XIAO_H, XIAO_T = 21.0, 17.8, 15.0     # long side along x (USB-C at the left end), height, stack depth
XIAO_LENS_DX, XIAO_LENS_DY = 0.0, 0.0         # lens offset from the stack centre
CAM_SQ = 8.6                     # camera module socket: 8.5 mm module + 0.05/side = snug push fit. MEASURE YOURS and
                                 # print ponder_fit_coupon_camera.stl first: too tight -> 8.7, rattles -> 8.55
CAM_SOCKET_DEPTH = 4.6           # socket walls reach back this far from the inner wall face (module is 3.2 deep + lens)
CAM_LENS_D = 6.4                 # lens barrel diameter. MEASURE YOURS.
CAM_COLLAR = 0.2                 # radial gap between the lens barrel and the collar in the left eye
CAM_COLLAR_T = 0.8               # collar thickness (inside the wall, behind the oval opening)
TOF_W, TOF_H = 20.5, 15.5      # VL53L0X GY-530 board, mounted LANDSCAPE (long side left-right). MEASURE YOURS.
TOF_TILT = 30.0                 # sensor looks 30 deg DOWN: it sees desk edges ~10 cm ahead AND walls (physics-tested)
TOF_SENSOR = (EYE_X + 1.2, 14.8, 17.7)   # sensor centre on the board's front face (robot mm); the board sits on a
                                         # tilted wedge behind the right eye

# Back-wall openings (x, y centres). ESP32-S3-Zero and the charge/boost board hang under the top,
# component side down, their USB-C ports in these holes.
CEIL_FOAM = 1.5
_USB_Y = BODY_H / 2 - WALL - CEIL_FOAM - 3.2
BOOST_XZ = (-10.5, -12.0)   # (unused) old 5 V boost spot. Servos + amp now run straight off the switched 1S battery.
                            # the battery charges through the XIAO's own charger and its left-side USB-C
XIAO_USB_BACK = 3.2      # MEASURE: USB-C centre, mm in front of the back face of the XIAO stack
XIAO_USB_YZ = (EYE_Y - XIAO_LENS_DY, BODY_D / 2 - WALL - 0.5 - XIAO_T + XIAO_USB_BACK)   # left-wall USB-C (y, z)
SWITCH_SLOT = (0.0, -1.6)   # E-Switch 100SP1T1B4M2QE mini toggle in the battery + line (back wall, x/y centre)
# B4 = smooth (non-threaded) bushing, so there is no nut: the neck goes through a snug hole and the body sits in a
# shallow pocket on the inside of the wall (stops it turning), glued. MEASURE YOURS: bushing diameter and body size.
SWITCH_BUSH_D = 6.35        # 1/4 in bushing
SWITCH_BODY = (8.4, 13.4)   # body footprint + clearance (x, y), lever moves up/down
SWITCH_POCKET = 0.6         # pocket depth into the 1.6 mm wall
SPK = (13.5, 2.0)           # 1511 speaker (15 x 11 x 3.5) on the right wall: (y, z) centre

# Battery: 1S LiPo 602035 (6 x 20 x 35 mm, 400 mAh) standing on its long edge between the servos.
# The bay takes cells up to 7 x 21 x 36 mm. Amp is foam-taped to its front face.
BATT_L, BATT_W, BATT_H, BATT_X = 36.0, 7.0, 21.0, -1.0   # length (x), thickness (z), height (y)

# Fasteners
M2_PILOT = 1.7            # self-tapping M2 into PLA
M2_CLEAR = 2.4
BOSS_XZ, BOSS_R = 19.8, 2.7   # corner bosses for the bottom-plate screws (M2 x 6)
PLATE_GRIP = 2.4          # plate thickness the M2 x 6 screws clamp through (the rest is counterbored)
PLATE_T = PLATE_GRIP + 2.5   # 4.9 mm: the extra 2.5 hangs below so she can lie flat on the plate (see LIE DOWN)
M2_HEAD_CB = 4.4          # counterbore for the screw heads, from below, PLATE_T - PLATE_GRIP deep
FDM_SLOT = 0.4
USB_HOLE = (13.5, 8.0)    # (legacy) old plain rectangular hole size
# Left-wall USB-C: a stadium (oval) that outlines the port, in a 45-deg bevel. The bevel is deep enough to
# swallow a max-size cable overmold (12.35 x 6.5) down to a thin lip, so the plug still seats fully.
USB_OVAL = (9.9, 4.2)     # (tall, wide) through-opening around the 8.94 x 3.26 receptacle
USB_LIP = 0.6             # wall left at the port; the overmold stops this far from the inner face
USB_BEVEL_IN = (14.0, 7.6)   # bevel size at the bottom: clears a 12.4 x 6.5 overmold with 1.5 mm corners

# Legs: one rigid piece per leg, swung by the hip servo
LEG_T = 3.6               # leg thickness
LEG_LEN = 30.0            # hip axis to the bottom of the shoe
LEG_TOP_R = 5.0
LEG_BOT_R = 3.2
HORN_HUB_H = 2.6          # micro-servo horn hub height
HORN_HUB_R = 2.6
LEG_RECESS = 1.6          # horn recess depth in the leg (horn is glued in + centre screw)
SHOE_H = 5.5
SHOE_W, SHOE_L = 8.0, 9.0
SHOE_ROCK_R = 18.0
PEG_X, PEG_Z, PEG_L = 3.2, 4.0, 4.0

# LIE DOWN: front legs fold forward, rear legs back, until the body rests flat on the bottom plate. The legs are
# mounted ~30 deg off centre (front forward, rear back) so the servos' 90 deg travel covers walking (+-12) AND
# folding to 75 deg. With a 2.4 mm plate the body only touches at ~78 deg; the 4.9 mm plate makes it touch at ~74.
HORN_OFFSET_DEG = 30.0      # leg angle at servo centre: front +30 (forward), rear -30 (back)
FOLD_DEG = 75.0             # lie-down leg angle (servo at 45 deg, the end of its 90 deg travel)
# SHUTDOWN TUCK: after lying down, the legs keep folding up toward vertical beside her (180 deg = pointing straight
# up). Clearance is fine all the way to 180 (fitcheck); the limit is the servo's travel. MEASURE YOURS with the
# self-test 'r' command: the GH-S37D is sold as 90 deg, which stops the tuck at 75. If yours turns further, the horn
# offset can grow: HORN_OFFSET_DEG = SERVO_TRAVEL_DEG / 2 - 15 keeps walking and reaches up to that + travel / 2.
SERVO_TRAVEL_DEG = 90.0
TUCK_DEG = min(180.0, HORN_OFFSET_DEG + SERVO_TRAVEL_DEG / 2)


def leg_geometry():
    """Stack-up of one right-side leg along x, measured from the body centre (mm)."""
    ix = BODY_W / 2 - WALL
    g = {}
    g['hip_horn_x0'] = ix + TOWER_H                              # horn seats on the tower top
    g['leg_x0'] = g['hip_horn_x0'] + HORN_HUB_H - LEG_RECESS     # leg inner face
    g['shoe_x'] = g['leg_x0'] + LEG_T / 2
    g['shoe_top_y'] = -LEG_LEN + SHOE_H
    return g


# ---------------- helpers ----------------
def rr(w, h, r, cx=0.0, cy=0.0):
    r = min(r, w / 2 - 0.01, h / 2 - 0.01)
    return CrossSection.square([w - 2 * r, h - 2 * r], center=True).offset(
        r, JoinType.Round, circular_segments=SEG).translate([cx, cy])


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def prism_y(sec, y0, y1):   # section in (x, -z)
    return sec.extrude(y1 - y0).rotate([-90, 0, 0]).translate([0, y0, 0])


def prism_z(sec, z0, z1):   # section in (x, y)
    return sec.extrude(z1 - z0).translate([0, 0, z0])


def prism_x(sec, x0, x1):   # section in (-z, y)
    return sec.extrude(x1 - x0).rotate([0, 90, 0]).translate([x0, 0, 0])


def ocyl_z(cx, cy, w, h, z0, z1):
    """Elliptic cylinder along z, w x h, centred (cx, cy)."""
    return Manifold.cylinder(z1 - z0, 1.0, 1.0, SEG).scale([w / 2, h / 2, 1]).translate([cx, cy, z0])


def cyl(axis, a, b, r, h0, h1):
    """Cylinder along an axis from h0 to h1. (a, b) = the other two coordinates:
    axis x -> (y, z), axis y -> (x, z), axis z -> (x, y)."""
    m = Manifold.cylinder(h1 - h0, r, r, SEG).translate([0, 0, h0])
    if axis == 'x':
        return m.rotate([0, 90, 0]).translate([0, a, b])
    if axis == 'y':
        return m.rotate([-90, 0, 0]).translate([a, 0, b])
    return m.translate([a, b, 0])


def union(parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def export(m, name, rot=(0, 0, 0)):
    m = m.rotate(list(rot))
    mesh = m.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                         faces=np.asarray(mesh.tri_verts), process=False)
    tm.apply_translation([-tm.bounds[0][0] - tm.extents[0] / 2,
                          -tm.bounds[0][1] - tm.extents[1] / 2,
                          -tm.bounds[0][2]])
    os.makedirs('stl', exist_ok=True)
    path = os.path.join('stl', name)
    tm.export(path)
    e = tm.extents
    print(f'{name:28s} {e[0]:6.1f} x {e[1]:6.1f} x {e[2]:6.1f} mm  watertight={tm.is_watertight}')


def tof_frame(m):
    """Place something built in the ToF board's local frame (x along the board, y up the board, z out of its
    front face, origin = sensor centre on the front face) into robot coordinates."""
    return m.rotate([TOF_TILT, 0, 0]).translate(list(TOF_SENSOR))


def tof_mount():
    """A solid tilted wedge behind the right eye. The sensor board is pushed forward onto its tilted face and
    glued/taped: the face touches only the PCB's outer 1.2 mm, with a pocket for the parts, and a cone-shaped
    window lets the sensor see out through the eye at TOF_TILT degrees down."""
    w, h = TOF_W / 2, TOF_H / 2
    block = box(-w, w, -h, h, 0, 30) - box(-w + 1.2, w - 1.2, -h - 1.0, h - 1.0, -1, 1.7)   # pocket open at the bottom: board slides up into it
    cone = Manifold.cylinder(30, 2.6, 2.6 + 30 * math.tan(math.radians(14)), SEG)       # 28 deg window (sensor FoV 25)
    inside = box(-BODY_W / 2 + WALL, BODY_W / 2 - WALL, -BODY_H / 2, BODY_H / 2 - WALL + 0.8, -BODY_D / 2, BODY_D / 2 - WALL + 0.8)
    return (tof_frame(block) ^ inside) - tof_frame(cone)


def tof_window():
    return tof_frame(Manifold.cylinder(40, 2.6, 2.6 + 40 * math.tan(math.radians(14)), SEG))


# ---------------- shell (one piece, open bottom, print upside down) ----------------
def shell():
    hx, hz, hy = BODY_W / 2, BODY_D / 2, BODY_H / 2
    ix, iz = hx - WALL, hz - WALL
    ceil = hy - WALL
    s = prism_y(rr(BODY_W, BODY_D, SHELL_R), -hy, hy) - prism_y(rr(2 * ix, 2 * iz, SHELL_R - WALL), -hy - 1, ceil)

    adds, cuts = [], []
    # Corner bosses for the bottom-plate screws, with 45 deg gussets so they print without supports
    for sx in (-1, 1):
        for sz in (-1, 1):
            c = (sx * BOSS_XZ, sz * BOSS_XZ)
            boss = cyl('y', c[0], c[1], BOSS_R, -hy, -hy + 7)
            corner = box(*sorted((sx * (ix - 0.6), sx * ix)), -hy + 7 + BOSS_R + 2.4, -hy + 7 + BOSS_R + 2.6, *sorted((sz * (iz - 0.6), sz * iz)))
            adds.append(boss)
            adds.append((cyl('y', c[0], c[1], BOSS_R, -hy + 6.9, -hy + 7) + corner).hull())
            cuts.append(cyl('y', c[0], c[1], M2_PILOT / 2, -hy - 0.5, -hy + 6))

    # Camera socket behind the left eye (camera module glued in, lens in the opening) and a cradle for the
    # VL53L0X board behind the right eye.
    ex, ey = -EYE_X, EYE_Y
    sq = CAM_SQ / 2
    adds.append(box(ex - sq - 1.2, ex + sq + 1.2, ey - sq - 1.2, ey + sq + 1.2, iz - CAM_SOCKET_DEPTH, iz + 0.1) -
                box(ex - sq, ex + sq, ey - sq, ey + sq, iz - CAM_SOCKET_DEPTH - 0.1, iz + 0.2))
    adds.append(tof_mount())                                                         # tilted wedge for the distance sensor

    # Hip servo tab ledges: a solid pad under each mounting tab. The tab screw self-taps into the pad wherever
    # the hole lands, so any 3.7 g servo's hole spacing works (GH-S37D has no published tab drawing).
    for sx in (-1, 1):
        for sz in (-1, 1):
            x0, x1 = ix - TAB_DEPTH, ix + 0.3
            a, b = (x0, x1) if sx > 0 else (-x1, -x0)
            yb = HIP_Y - SHAFT_FROM_END
            yt = yb + SERVO_L
            t = SERVO_T / 2
            adds.append(box(a, b, max(yb - TAB_OVER - 0.2, -hy), yb, sz * HIP_Z - t, sz * HIP_Z + t))   # lower pad
            adds.append(box(a, b, yt, yt + TAB_OVER + 0.2, sz * HIP_Z - t, sz * HIP_Z + t))           # upper pad

    # Speaker pocket on the inside of the right wall (1511: 15 long, 11 tall, 3.5 thick)
    sy, sz_ = SPK
    # two ledges, speaker glued between them. The top one hangs from the ceiling; the bottom one is only
    # 2 mm deep so it prints as a short overhang (the shell prints upside down).
    adds.append(box(ix - 4.3, ix + 0.3, sy + 5.7, hy - WALL + 0.3, sz_ - 7.6, sz_ + 7.6))
    adds.append(box(ix - 2.0, ix + 0.3, sy - 7.3, sy - 5.7, sz_ - 7.6, sz_ + 1.0))     # short: the sensor board passes in front of it

    for sx in (-1, 1):                                                                 # eye bezels, standing proud of the face
        cx, cy, (rw, rh) = sx * EYE_X, EYE_Y + EYE_DY, EYE_RING
        ring = ocyl_z(cx, cy, rw, rh, hz - 0.2, hz + EYE_PROUD) - ocyl_z(cx, cy, rw - 2 * EYE_BAND, rh - 2 * EYE_BAND, hz - 1, hz + EYE_PROUD + 1)
        adds.append(ring + ocyl_z(cx, cy, rw - 2 * EYE_BAND, rh - 2 * EYE_BAND, hz - 0.2, hz + 0.4))   # oval ring + a shallow iris dish
    s = union([s] + adds)

    # Openings
    uy, uz = XIAO_USB_YZ                                                               # XIAO USB-C, left wall: oval + bevel
    cuts.append(prism_x(rr(USB_OVAL[1], USB_OVAL[0], USB_OVAL[1] / 2, -uz, uy), -hx - 1, -ix + 1))
    d = hx - ix - USB_LIP                                                              # bevel depth (1.0 mm)
    th, tw = USB_BEVEL_IN
    def slab(x, grow):
        return prism_x(rr(tw + 2 * grow, th + 2 * grow, (tw + 2 * grow) / 2, -uz, uy), x - 0.01, x)
    cuts.append(Manifold.batch_hull([slab(-ix - USB_LIP, 0.0), slab(-hx - 0.6, d + 0.6)]))
    sw_x, sw_y = SWITCH_SLOT
    cuts.append(cyl('z', sw_x, sw_y, SWITCH_BUSH_D / 2 + 0.15, -hz - 1, -iz + 1))      # snug hole for the smooth bushing
    cuts.append(box(sw_x - SWITCH_BODY[0] / 2, sw_x + SWITCH_BODY[0] / 2, sw_y - SWITCH_BODY[1] / 2, sw_y + SWITCH_BODY[1] / 2,
                    -iz - SWITCH_POCKET, -iz + 0.5))                                    # anti-rotation pocket for the body
    cuts.append(tof_window() ^ box(-hx, hx, -hy, hy, -hz, iz - 0.01))                 # sensor's view: only INSIDE the wall,
                                                                                       # so the face shows a clean oval
    for sx in (-1, 1):                                                                 # the eyes: openings
        z0 = iz + CAM_COLLAR_T if sx < 0 else iz - 1                                   # left eye keeps a collar at the back
        cuts.append(ocyl_z(sx * EYE_X, EYE_Y + EYE_DY, *EYE_OPEN, z0, hz + EYE_PROUD + 1))
    cuts.append(cyl('z', -EYE_X, EYE_Y, CAM_LENS_D / 2 + CAM_COLLAR, iz - 1, hz + 1))   # collar hole: hugs the lens barrel
    for sx in (-1, 1):
        for sz in (-1, 1):
            a, b = (ix - 1, hx + 1) if sx > 0 else (-hx - 1, -ix + 1)
            cuts.append(cyl('x', HIP_Y, sz * HIP_Z, TOWER_R + 0.5, a, b))            # hip shaft hole
    for i in range(-2, 3):                                                             # speaker grille
        for j in range(-1, 2):
            u, v = i * 2.9 + (1.45 if j % 2 else 0.0), j * 2.6
            if abs(u) <= 6.2 and abs(v) <= 4.0:
                cuts.append(cyl('x', sy + v, sz_ + u, 0.8, ix - 1, hx + 1))
    for c in cuts:
        s = s - c
    return s


# ---------------- bottom plate ----------------
def bottom_plate():
    hy = BODY_H / 2
    p = prism_y(rr(BODY_W, BODY_D, SHELL_R), -hy - PLATE_T, -hy)
    for sx in (-1, 1):
        for sz in (-1, 1):
            p = p - cyl('y', sx * BOSS_XZ, sz * BOSS_XZ, M2_CLEAR / 2, -hy - PLATE_T - 1, -hy + 1)
            p = p - cyl('y', sx * BOSS_XZ, sz * BOSS_XZ, M2_HEAD_CB / 2, -hy - PLATE_T - 1, -hy - PLATE_GRIP)   # head counterbore
    # battery stands on its long edge: end stops + side fences, strap slots either side
    hb, wb, bx = BATT_L / 2, BATT_W / 2, BATT_X
    p = p + box(bx - hb - 1.6, bx - hb, -hy, -hy + 4, -wb, wb)                       # left end stop
    for sz in (-1, 1):
        for x in (-10.0, 10.0):
            p = p + box(x, x + 4, -hy, -hy + 4, *sorted((sz * wb, sz * (wb + 1.6))))   # side fences
        p = p - box(-15.5, -11.5, -hy - PLATE_T - 1, -hy + 1, *sorted((sz * (wb + 0.6), sz * (wb + 2.6))))   # strap slot (left end; amp sits mid-front)
    return p



# ---------------- leg (x 4, all identical) ----------------
def leg():
    """Rigid leg. Inner face (x=0) takes the micro-servo horn (glued into the recess, arm down the leg).
    Origin on the hip axis. A peg at the bottom takes the rocker shoe."""
    g = leg_geometry()
    bot_c = g['shoe_top_y'] + LEG_BOT_R
    sec = (CrossSection.circle(LEG_TOP_R, SEG) + CrossSection.circle(LEG_BOT_R, SEG).translate([0, bot_c])).hull()
    l = prism_x(sec, 0, LEG_T)
    l = l + box(0, PEG_X, g['shoe_top_y'] - PEG_L, g['shoe_top_y'] + 0.5, -PEG_Z / 2, PEG_Z / 2)
    l = l - cyl('x', 0, 0, HORN_HUB_R + 0.4, -0.1, LEG_RECESS) - box(-0.1, LEG_RECESS, -15.5, 0, -2.2, 2.2)   # horn recess
    l = l - cyl('x', 0, 0, 1.6, -1, LEG_T + 1)                     # horn screw access
    return l


# ---------------- PLA rocker shoe (x 4) ----------------
def shoe():
    body = box(-SHOE_W / 2, SHOE_W / 2, -SHOE_H, 0, -SHOE_L / 2, SHOE_L / 2)
    rock = Manifold.cylinder(2 * SHOE_W, SHOE_ROCK_R, SHOE_ROCK_R, 720).rotate([0, 90, 0]).translate([-SHOE_W, -SHOE_H + SHOE_ROCK_R, 0])
    s = body ^ rock
    return s - box(-PEG_X / 2 - 0.25, PEG_X / 2 + 0.25, -PEG_L - 0.3, 0.5, -PEG_Z / 2 - 0.25, PEG_Z / 2 + 0.25)


if __name__ == '__main__':
    export(shell(), 'ponder_shell.stl', rot=(-90, 0, 0))          # top face on the bed
    export(bottom_plate(), 'ponder_bottom_plate.stl', rot=(90, 0, 0))
    hx, hy, hz = BODY_W / 2, BODY_H / 2, BODY_D / 2                    # 10-minute test print: the left eye + camera socket,
    coupon = shell() ^ box(-EYE_X - 9.5, -EYE_X + 9.5, EYE_Y - 11, hy, hz - WALL - CAM_SOCKET_DEPTH - 0.5, hz + 3)   # cut from the real shell
    export(coupon, 'ponder_fit_coupon_camera.stl', rot=(-90, 0, 0))   # prints the same way up as the shell
    export(leg(), 'ponder_leg_x4.stl', rot=(0, 90, 0))               # flat, horn pocket facing UP (no bridging)
    export(shoe(), 'ponder_shoe_x4.stl', rot=(0, -90, 0))            # on its side: the rocker curve prints smooth
