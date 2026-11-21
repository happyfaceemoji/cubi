import json, base64, numpy as np, math
import ponder_parts as P, fitcheck as F
from manifold3d import Manifold

def mesh_of(m):
    me = m.to_mesh()
    v = np.asarray(me.vert_properties)[:, :3].astype(np.float32)
    f = np.asarray(me.tri_verts).astype(np.uint32)
    return v, f

def enc(m):
    v, f = mesh_of(m)
    return {'v': base64.b64encode(v.tobytes()).decode(), 'i': base64.b64encode(f.tobytes()).decode(), 'tris': int(len(f))}

def Rx(d):
    a = math.radians(d); c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], float)
def Ry(d):
    a = math.radians(d); c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], float)
def T(x,y,z):
    m = np.eye(4); m[:3,3] = [x,y,z]; return m
def S(x,y,z):
    return np.diag([x,y,z,1.0])

meshes = {
  'shell': P.shell(), 'bottom_plate': P.bottom_plate(),
  'leg': P.leg(), 'shoe': P.shoe(),
}
G = P.leg_geometry()
HIP = 0
asm = []
legs = []
for name in ('shell','bottom_plate'):
    asm.append({'mesh': name, 'm': np.eye(4)})
for sx in (-1,1):
    for sz in (-1,1):
        base = T(0, P.HIP_Y, sz*P.HIP_Z) @ S(sx,1,1) @ Rx(HIP)
        asm.append({'mesh':'leg', 'm': base @ T(G['leg_x0'],0,0)})
        asm.append({'mesh':'shoe', 'm': base @ T(G['shoe_x'], G['shoe_top_y'], 0)})
        legs.append({'sx': sx, 'sz': sz, 'base': base.T.flatten().round(5).tolist()})

# components as ghost envelopes (from the fit check)
body = F.body_parts()
comp = None
for n, m in body.items():
    if n in ('shell','bottom_plate','strap'): continue
    comp = m if comp is None else comp + m
for sx in (-1,1):
    for sz in (-1,1):
        lp = F.leg_parts(sx, sz, HIP)
        comp = comp + lp['hip_horn']
meshes['components'] = comp

# print orientations exactly as in the STL export
rot = {'shell':(-90,0,0),'bottom_plate':(90,0,0),'leg':(0,90,0),'shoe':(0,-90,0)}
pm = {}
for k, r in rot.items():
    m = meshes[k].rotate(list(r))
    b = m.bounding_box()  # (minx,miny,minz,maxx,maxy,maxz)
    m = m.translate([-(b[0]+b[3])/2, -(b[1]+b[4])/2, -b[2]])
    pm['print_'+k] = m
meshes.update(pm)

def ext(m):
    b = m.bounding_box(); return [round(b[3]-b[0],1), round(b[4]-b[1],1), round(b[5]-b[2],1)]

# print beds: (bed index, mesh, x, y in bed coords (mm), rotZ)
plates = [
  [0, 'print_shell', 0, 0, 0],
  [1, 'print_bottom_plate', 0, 32, 0],
  [2, 'print_femur', -60, 35, 0], [2, 'print_femur', -36, 35, 0], [2, 'print_femur', -60, -35, 0], [2, 'print_femur', -36, -35, 0],
  [2, 'print_tibia', 15, 30, 0], [2, 'print_tibia', 58, 30, 0], [2, 'print_tibia', 15, -30, 0], [2, 'print_tibia', 58, -30, 0],
  [3, 'print_foot', -14, 11, 0], [3, 'print_foot', 14, 11, 0], [3, 'print_foot', -14, -11, 0], [3, 'print_foot', 14, -11, 0],
]
info = {
  'shell': ['Shell', 1, 'PLA Basic', 'Two big eye bezels on the front: camera in the left, distance sensor in the right. 4 walls, 15% gyroid. Prints top-down, no supports.'],
  'bottom_plate': ['Bottom plate', 1, 'PLA Basic', 'The battery stands on edge in its fences with the amp taped to its front; unscrewing the plate takes them out. 4.9 mm thick so she can lie flat on it. 4 M2x6 screws, heads sunk in counterbores, self-tap into the corner bosses.'],
  'leg': ['Leg', 4, 'PLA Basic', 'One rigid piece. The micro-servo horn is glued into the recess on the inner face; the servo screw goes through the centre.'],
  'shoe': ['Rocker shoe', 4, 'PLA Basic', 'Curved bottom so the foot rocks and slides. Push onto the leg peg with a drop of glue.'],
  'components': ['Bought parts (envelopes)', 0, '', 'Micro servos, camera board, distance sensor, battery and speaker at spec-sheet size.'],
}
out = {
  'meshes': {k: enc(m) for k, m in meshes.items()},
  'ext': {k: ext(m) for k, m in meshes.items()},
  'assembly': [{'mesh': a['mesh'], 'm': a['m'].T.flatten().round(5).tolist()} for a in asm],
  'plates': plates, 'info': info, 'legs': legs, 'G': G,
  'C': {'HIP_Y': P.HIP_Y, 'HIP_Z': P.HIP_Z, 'IX': P.BODY_W / 2 - P.WALL, 'CEIL': P.BODY_H / 2 - P.WALL,
        'EYE_X': P.EYE_X, 'EYE_Y': P.EYE_Y, 'EYE_OD': P.EYE_OD, 'EYE_HOLE': P.EYE_HOLE, 'EYE_PROUD': P.EYE_PROUD, 'EYE_RING': P.EYE_RING, 'EYE_OPEN': P.EYE_OPEN, 'EYE_DY': P.EYE_DY, 'CAM_LENS_D': P.CAM_LENS_D,
        'XIAO_W': P.XIAO_W, 'XIAO_H': P.XIAO_H, 'XIAO_T': P.XIAO_T, 'XIAO_USB_YZ': P.XIAO_USB_YZ,
        'TOF_W': P.TOF_W, 'TOF_H': P.TOF_H, 'TOF_TILT': P.TOF_TILT, 'TOF_SENSOR': P.TOF_SENSOR, 'BOOST_XZ': P.BOOST_XZ, 'SWITCH': P.SWITCH_SLOT, 'SPK': P.SPK, 'SERVO_H': P.SERVO_H, 'SERVO_L': P.SERVO_L, 'SERVO_T': P.SERVO_T, 'SHAFT_FROM_END': P.SHAFT_FROM_END,
        'TOWER_R': P.TOWER_R, 'TOWER_H': P.TOWER_H, 'TAB_DEPTH': P.TAB_DEPTH, 'TAB_OVER': P.TAB_OVER,
        'BATT_X': P.BATT_X, 'BATT_L': P.BATT_L, 'BATT_W': P.BATT_W, 'BATT_H': P.BATT_H, 'HY': P.BODY_H / 2,
        'TOP': P.BODY_H / 2 - P.WALL - P.CEIL_FOAM},

}
s = json.dumps(out, separators=(',',':'))
open('viewer_data.js','w').write('window.CUBI=' + s + ';')
print('tris', {k: v['tris'] for k, v in out['meshes'].items()}, 'size KB', len(s)//1024)
