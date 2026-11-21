"""Virtual assembly check for CUBI (48 mm, 4 micro servos).

Builds every printed part plus generous envelopes of the real components, then checks:
  1. STATIC     - nothing overlaps once assembled
  2. ASSEMBLY   - every part can actually be put in, in build order, along its real path
  3. CABLES     - a max-size USB-C plug fits both back-wall ports
  4. MOTION     - legs swing without hitting the body or each other
Run:  python fitcheck.py
"""
from manifold3d import Manifold
import ponder_parts as P

box, cyl = P.box, P.cyl
IX = P.BODY_W / 2 - P.WALL     # inner wall
HY = P.BODY_H / 2
CEIL = HY - P.WALL             # underside of the top
TOP = CEIL - P.CEIL_FOAM       # ceiling boards hang from here


# ---------- real-component envelopes (generous, mm) ----------
def micro_servo(top_x, shaft_y, shaft_z, side):
    """3.7 g servo, shaft along +x*side, case top face at x=top_x, case running up (+y) from the shaft."""
    def X(a, b):
        return (top_x + side * a, top_x + side * b) if side > 0 else (top_x + side * b, top_x + side * a)
    t = P.SERVO_T / 2
    y0, y1 = shaft_y - P.SHAFT_FROM_END, shaft_y - P.SHAFT_FROM_END + P.SERVO_L
    p = {}
    p['case'] = box(*X(-P.SERVO_H, 0), y0, y1, shaft_z - t, shaft_z + t)
    p['tabs'] = box(*X(-P.TAB_DEPTH - P.TAB_T, -P.TAB_DEPTH), y0 - P.TAB_OVER, y1 + P.TAB_OVER, shaft_z - t, shaft_z + t)
    p['tower'] = cyl('x', shaft_y, shaft_z, P.TOWER_R, *X(0, P.TOWER_H))
    p['spline'] = cyl('x', shaft_y, shaft_z, P.SPLINE_R, *X(P.TOWER_H, P.TOWER_H + 2.5))
    p['cable'] = box(*X(-P.SERVO_H + 0.5, -P.SERVO_H + 3.5), y1, y1 + 4.0, shaft_z - 1.5, shaft_z + 1.5)
    return p


def glued_parts():
    """Parts fixed inside the shell before the servos. Each value is a list of convex pieces."""
    g = {}
    # XIAO ESP32S3 Sense stack (main board + expansion board + camera) behind the left eye
    ex, ey = -P.EYE_X + P.XIAO_LENS_DX, P.EYE_Y - P.XIAO_LENS_DY
    zf = IX - 0.5
    uy, uz = P.XIAO_USB_YZ
    g['xiao'] = [box(ex - P.XIAO_W / 2, ex + P.XIAO_W / 2, ey - P.XIAO_H / 2, ey + P.XIAO_H / 2, zf - P.XIAO_T, zf - 4.5),
                 box(-P.EYE_X - 4.25, -P.EYE_X + 4.25, P.EYE_Y - 4.25, P.EYE_Y + 4.25, zf - 4.5, zf - 1.3),     # camera module
                 cyl('z', -P.EYE_X, P.EYE_Y, P.CAM_LENS_D / 2, zf - 1.3, IX + 0.6),                                          # lens barrel
                 box(-IX, -IX + 3.5, uy - 4.5, uy + 4.5, uz - 1.7, uz + 1.7),                                    # USB-C receptacle
                 box(ex - P.XIAO_W / 2 + 2, ex + P.XIAO_W / 2 - 1, ey - P.XIAO_H / 2 + 0.5, ey + P.XIAO_H / 2 - 0.5, zf - P.XIAO_T - 3.0, zf - P.XIAO_T)]   # soldered wire tails on the back
    # VL53L0X board in its tilted rails behind the right eye (local frame: x along, y up the board, z out the face)
    w, h = P.TOF_W / 2, P.TOF_H / 2
    g['tof'] = [P.tof_frame(box(-w, w, -h, h, -1.6, 0)),                              # PCB
                P.tof_frame(box(-w + 1.2, w - 1.2, -h + 1.0, h - 1.0, 0, 1.5)),       # sensor + parts
                P.tof_frame(box(-6, 6, -h, -h + 4.5, -4.8, -1.6))]                    # wire joints on the back, bottom edge
    sy, sz = P.SPK
    g['speaker'] = [box(IX - 3.6, IX, sy - 5.6, sy + 5.6, sz - 7.6, sz + 7.6)]
    g['cap2'] = [cyl('z', 13.5, TOP - 4.3, 4.1, -17.0, -4.5)]                             # battery bulk cap at the switch output, 1000 uF 6.3 V, 8 x 10.5
    g['cap1'] = [cyl('x', 3.6, -3.0, 3.3, -15.5, -3.5)]                                   # servo-rail cap at the splice, 470 uF 6.3 x 11
    wx, wy = P.SWITCH_SLOT
    # E-Switch 100SP1T1B4M2QE: smooth bushing (no nut), body seated in a 0.6 mm pocket in the back wall, PCB pins
    g['switch'] = [cyl('z', wx, wy, 3.2, -IX - 6.0, -IX - 0.6),                          # smooth bushing
                   box(wx - 4.05, wx + 4.05, wy - 6.6, wy + 6.6, -IX - 0.6, -IX + 9.9),  # body
                   box(wx - 3.0, wx + 3.0, wy - 5.5, wy + 5.5, -IX + 9.9, -IX + 16),     # PCB pins + soldered wires
                   cyl('z', wx, wy, 1.6, -IX - 13, -IX - 6.0)]                           # lever
    return g


def battery_parts():
    """Battery on its long edge on the bottom plate; amp foam-taped to its front face."""
    y0 = -HY + 0.3
    x0, x1 = P.BATT_X - P.BATT_L / 2, P.BATT_X + P.BATT_L / 2
    w = P.BATT_W / 2 - 0.5
    return {'battery': [box(x0 + 0.5, x1 - 0.5, y0, y0 + 20.5, -w, w)],
            'battery_leads': [box(x1 - 0.5, x1 + 4.0, y0 + 2, y0 + 12, -2.5, 2.5)],
            'strap': [box(-15.5, -11.5, -HY, y0 + 21.5, -w - 1.3, w + 1.3)],
            'amp': [box(0.5, 18.5, -HY + 16, -HY + 34.5, w + 0.5, w + 4.0)]}               # MAX98357A, no screw terminal, right half


def servo_list():
    return [((sx, sz), micro_servo(sx * IX, P.HIP_Y, sz * P.HIP_Z, sx)) for sx in (-1, 1) for sz in (-1, 1)]


def leg_parts(sx, sz, hip_deg=0.0):
    G = P.leg_geometry()
    loc = {'leg': P.leg().translate([G['leg_x0'], 0, 0]),
           'shoe': P.shoe().translate([G['shoe_x'], G['shoe_top_y'], 0]),
           'hip_horn': cyl('x', 0, 0, P.HORN_HUB_R, G['hip_horn_x0'], G['hip_horn_x0'] + P.HORN_HUB_H) +
                       box(G['hip_horn_x0'] + P.HORN_HUB_H - 1.3, G['hip_horn_x0'] + P.HORN_HUB_H, -14, 0, -1.8, 1.8)}
    out = {}
    for k, v in loc.items():
        v = v.rotate([hip_deg, 0, 0])
        if sx < 0:
            v = v.mirror([1, 0, 0])
        out[k] = v.translate([0, P.HIP_Y, sz * P.HIP_Z])
    return out


def U(pieces):
    out = pieces[0]
    for p in pieces[1:]:
        out = out + p
    return out


def ov(a, b):
    return (a ^ b).volume()


def sweep(pieces, v):
    return U([(p + p.translate(v)).hull() for p in pieces])


def body_parts():
    d = {'shell': P.shell(), 'bottom_plate': P.bottom_plate()}
    for k, v in glued_parts().items():
        d[k] = U(v)
    for k, v in battery_parts().items():
        d[k] = U(v)
    for (sx, sz), s in servo_list():
        for k, v in s.items():
            d[f'hip_servo_{"R" if sx > 0 else "L"}{"F" if sz > 0 else "B"}_{k}'] = v
    return d


if __name__ == '__main__':
    TOL = 0.3
    problems = []
    shell, plate = P.shell(), P.bottom_plate()
    glued, batt, servos = glued_parts(), battery_parts(), servo_list()

    # ---- 1. STATIC ----
    named = [('shell', shell), ('bottom_plate', plate)] + [(k, U(v)) for k, v in glued.items()] + \
            [(k, U(v)) for k, v in batt.items()] + \
            [(f'servo {"R" if sx > 0 else "L"}{"F" if sz > 0 else "B"}', U(list(s.values()))) for (sx, sz), s in servos]
    for sx in (-1, 1):
        for sz in (-1, 1):
            lp = leg_parts(sx, sz)
            named.append((f'leg {"R" if sx > 0 else "L"}{"F" if sz > 0 else "B"}', lp['leg'] + lp['shoe'] + lp['hip_horn']))
    ok_pairs = {('strap', 'battery'), ('strap', 'bottom_plate'), ('battery', 'strap')}
    for i in range(len(named)):
        for j in range(i + 1, len(named)):
            a, b = named[i], named[j]
            if (a[0], b[0]) in ok_pairs or (b[0], a[0]) in ok_pairs:
                continue
            if a[0].startswith('servo') and b[0].startswith('leg') and a[0][-2:] == b[0][-2:]:
                c = ov(a[1], b[1]) - ov(U([s for (sx, sz), s2 in servos for s in [s2['spline']] if f'{"R" if sx > 0 else "L"}{"F" if sz > 0 else "B"}' == a[0][-2:]]), b[1])
                v = c
            else:
                v = ov(a[1], b[1])
            if v > TOL:
                problems.append(f'STATIC   {a[0]} overlaps {b[0]} ({v:.1f} mm3)')

    # ---- 2. ASSEMBLY ----
    installed = shell
    UP = [0, -60, 0]
    # waypoints (offsets from the final spot): lifted up from the open bottom to the first, then moved through the rest
    PATHS = [('tof', [[-2.8, -3.1, -12], [-2.8, -3.1, -6], [0, -3.1, -6], [0, -3.1, -1.8]]), ('xiao', [[0, 0, -14.5]]), ('speaker', [[-4.5, 0, 0]]), ('cap2', [[0, 0, 0]])]
    LATE = [('switch', [[0, 0, 14]]), ('cap1', [[0, 0, 0]])]
    def place(k, off):
        global installed
        pieces = glued[k]
        wps = off + [[0, 0, 0]]
        v = ov(sweep([p.translate(wps[0]) for p in pieces], UP), installed)
        for a, b in zip(wps, wps[1:]):
            v += ov(sweep([p.translate(a) for p in pieces], [b[i] - a[i] for i in range(3)]), installed)
        if v > TOL:
            problems.append(f'ASSEMBLY {k} can\'t be put in ({v:.1f} mm3)')
        installed = installed + U(pieces)
    for k, off in PATHS:
        place(k, off)
    SHIFT = 9.0
    for (sx, sz), s in servos:
        tag = f'servo {"R" if sx > 0 else "L"}{"F" if sz > 0 else "B"}'
        pieces = list(s.values())
        shifted = [p.translate([-sx * SHIFT, 0, 0]) for p in pieces]
        v1 = ov(sweep(shifted, UP), installed)
        v2 = ov(sweep(shifted, [sx * SHIFT, 0, 0]), installed)
        if v1 > TOL:
            problems.append(f'ASSEMBLY {tag} can\'t be lifted in ({v1:.1f} mm3)')
        if v2 > TOL:
            problems.append(f'ASSEMBLY {tag} can\'t slide out into place ({v2:.1f} mm3)')
        installed = installed + U(pieces)
    for k, off in LATE:
        place(k, off)
    stack = batt['battery'] + batt['battery_leads'] + batt['strap'] + batt['amp']
    v = ov(sweep(stack, UP), installed)
    if v > TOL:
        problems.append(f'ASSEMBLY battery + amp on the bottom plate can\'t be lifted in ({v:.1f} mm3)')
    v = ov(sweep([plate.hull()], UP) - plate.hull(), installed)
    if v > TOL:
        problems.append(f'ASSEMBLY bottom plate doesn\'t close ({v:.1f} mm3)')

    # ---- 3. CABLES ----
    uy, uz = P.XIAO_USB_YZ
    plug = U([P.prism_x(P.rr(6.5, 12.4, 1.5, -uz, uy), -80, -IX - P.USB_LIP),        # max-size overmold (rounded)
              box(-80, -IX + 0.05, uy - 4.2, uy + 4.2, uz - 1.3, uz + 1.3)])           # metal plug shell (8.4 x 2.6)
    if ov(plug, shell) > 0.1:
        problems.append('CABLES   USB-C cable won\'t seat in the XIAO port (left side)')

    # ---- 4. MOTION ----
    body = shell + plate
    free = [a for a in range(-90, 91, 5) if ov(U([leg_parts(1, 1, a)['leg'], leg_parts(1, 1, a)['shoe']]), body) <= 0.3]
    meet = None
    for a in range(0, 61):
        f, r = leg_parts(1, 1, a), leg_parts(1, -1, -a)
        if ov(f['leg'] + f['shoe'], r['leg'] + r['shoe']) > 0.3:
            meet = a
            break

    print('STATIC + ASSEMBLY + CABLES:', 'all clear' if not problems else f'{len(problems)} problem(s)')
    for p in problems:
        print('  ', p)
    G = P.leg_geometry()
    lowest = leg_parts(1, 1, 0)['shoe'].bounding_box()[1]
    print(f'\nMOTION   legs swing {min(free)} to {max(free)} deg without touching the body')
    print(f'         front/rear legs touch at {meet} deg each (gait limit: keep within +-15)')
    print(f'SIZE     width {2 * (G["shoe_x"] + P.SHOE_W / 2):.1f} mm, depth {P.BODY_D:.0f} mm, height {HY - lowest:.1f} mm, '
          f'ground clearance {-HY - P.PLATE_T - lowest:.1f} mm')
