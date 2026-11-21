"""Lay the PONDER MINI print files out on one Bambu A1 mini plate (180 x 180 mm). Run after ponder_parts.py."""
import trimesh, os
L = lambda n: trimesh.load(os.path.join('stl', n))
# (file, x, y) = footprint centre on the bed, mm from the bed centre
LAYOUT = [('ponder_shell.stl', -32, 22), ('ponder_bottom_plate.stl', 30, 22),
          ('ponder_leg_x4.stl', -45, -32), ('ponder_leg_x4.stl', -30, -32), ('ponder_leg_x4.stl', -15, -32), ('ponder_leg_x4.stl', 0, -32),
          ('ponder_shoe_x4.stl', 22, -24), ('ponder_shoe_x4.stl', 36, -24), ('ponder_shoe_x4.stl', 22, -40), ('ponder_shoe_x4.stl', 36, -40)]
if __name__ == '__main__':
    parts = []
    for f, x, y in LAYOUT:
        m = L(f).copy()
        b = m.bounds
        m.apply_translation([x - (b[0][0] + b[1][0]) / 2, y - (b[0][1] + b[1][1]) / 2, -b[0][2]])
        parts.append(m)
    # minimum gap between footprints
    import itertools
    gap = min(max(max(a.bounds[0][0] - b.bounds[1][0], b.bounds[0][0] - a.bounds[1][0]),
                  max(a.bounds[0][1] - b.bounds[1][1], b.bounds[0][1] - a.bounds[1][1])) for a, b in itertools.combinations(parts, 2))
    allm = trimesh.util.concatenate(parts)
    lo, hi = allm.bounds
    os.makedirs('a1mini_plates', exist_ok=True)
    for f in os.listdir('a1mini_plates'):
        os.remove(os.path.join('a1mini_plates', f))
    out = 'a1mini_plates/ponder_mini_all_parts_PLA_Basic.stl'
    allm.export(out)
    print(f'{out}: {len(parts)} parts, spans x {lo[0]:.0f}..{hi[0]:.0f}, y {lo[1]:.0f}..{hi[1]:.0f} (bed +-90), min gap {gap:.1f} mm')
