"""Cubi v2 fit check: clearances, the hood's lift-off path, and every service path."""
import sys, math
from manifold3d import Manifold
import cubi_v2_parts as V
from ponder_parts import box, union

v7 = V.load_v7(sys.argv[1])
H, C = V.hood(v7), V.chassis()
M = V.parts_models()
pods = {t: V.tof_pod(t) for t in V.TOF_TILTS}
keeper = V.cam_keeper()
door = V.battery_door()
results = []


def check(name, a, b, tol=0.02):
    v = (a ^ b).volume()
    ok = v <= tol
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name:58s} overlap {v:8.3f} mm3")
    return v


def path_check(name, m, d, n, obstacle, start=None, tol=0.02):
    """Step m along d (after an optional start offset); report the worst overlap with obstacle."""
    worst, at = 0.0, 0.0
    base = m if start is None else m.translate(list(start))
    for i in range(1, n + 1):
        f = i / n
        v = (base.translate([d[0] * f, d[1] * f, d[2] * f]) ^ obstacle).volume()
        if v > worst:
            worst, at = v, f
    ok = worst <= tol
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name:58s} worst {worst:8.3f} mm3" + ('' if ok else f' at {at:.0%}'))


print('-- static fit (assembled) --')
check('hood vs chassis', H, C)
check('hood vs battery door', H, door)
check('chassis vs battery door', C, door)
for k, m in M.items():
    check(f'chassis vs {k}', C, m)
    check(f'hood vs {k}', H, m)
check('servos vs xiao_stack', M['servos'], M['xiao_stack'])
check('servos vs battery', M['servos'], M['battery'])
check('keeper vs chassis', keeper, C)
check('keeper vs hood', keeper, H)
check('keeper vs xiao_stack', keeper, M['xiao_stack'])
for t, p in pods.items():
    check(f'pod {t:.0f} vs chassis', p, C)
    check(f'pod {t:.0f} vs hood', p, H)
    for k in ('xiao_stack', 'servos', 'speaker'):
        check(f'pod {t:.0f} vs {k}', p, M[k])
    check(f'pod {t:.0f} vs its sensor board', p, V.tof_board(t))
    check(f'sensor board {t:.0f} vs chassis', V.tof_board(t), C)
    beam = V.tof_cone(t, 12.5, 25)
    check(f'ToF beam {t:.0f} deg (25 deg FoV) vs hood', beam, H)
    check(f'ToF beam {t:.0f} deg vs chassis', beam, C)

print('-- camera view --')
ex, ey = -V.EYE_X, V.EYE_Y
for half in (20, 26):
    cone = Manifold.cylinder(12, 1.0, 1.0 + 12 * math.tan(math.radians(half)), 64).translate([ex, ey, V.FB0 + V.LENS_PROUD])
    check(f'camera view cone {half} deg half-angle vs hood', cone, H, tol=0.5)
    check(f'camera view cone {half} deg half-angle vs chassis', cone, C, tol=0.5)

print('-- service paths --')
blk = C + M['servos'] + M['switch']
path_check('hood lifts straight up 60 mm (vs chassis+servos+switch)', H, (0, 60, 0), 240, blk)
for sx in (-1, 1):
    for sz in (-1, 1):
        s = V.servo_model(sx, sz)
        path_check(f'servo {"R" if sx > 0 else "L"}{"F" if sz > 0 else "R"} pulls out sideways 25 mm (hood off)', s, (sx * 25, 0, 0), 100, C)
path_check('camera + XIAO lift out 25 mm (keeper off)', M['xiao_stack'], (0, 25, 0), 100, C + M['servos'])
path_check('battery drops out 10 mm (door off)', M['battery'], (0, -10, 0), 40, C + M['servos'])
for t, p in pods.items():
    obs = C + M['xiao_stack'] + M['servos'] + M['speaker']
    path_check(f'pod {t:.0f} slides back 1.5 mm', p, (0, 0, -1.5), 6, obs)
    path_check(f'pod {t:.0f} then lifts out 25 mm', p, (0, 25, 0), 100, obs, start=(0, 0, -1.5))
print(f"\n{sum(results)}/{len(results)} checks pass")
