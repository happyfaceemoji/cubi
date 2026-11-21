"""CUBI - full physics test bench (pybullet).

Uses the real printed geometry (leg + rocker shoe meshes straight from ponder_parts.py), the real body
size, a mass layout built from every component's weight and position, GH-S37D servo limits (torque,
speed, 90 deg travel), and a ray-cast VL53L0X in the right eye. Runs a set of scenarios and prints a
pass/fail table.  Run:  pip install pybullet trimesh manifold3d && python ponder_sim.py [--gif]

Sim axes: X right, Y forward, Z up (metres, kg). Robot model axes (mm): x right, y up, z forward.
"""
import math, os, sys, json
import numpy as np
import pybullet as p
import trimesh
import ponder_parts as P

DT = 1 / 1000
OUT = 'sim_out'
os.makedirs(OUT, exist_ok=True)

# ---------------- robot description ----------------
G = P.leg_geometry()
HY = P.BODY_H / 2
mm = lambda x, y, z: [x / 1000, z / 1000, y / 1000]          # robot mm -> sim m

# component masses (g) and centres (robot mm)
PARTS = [
    ('shell (PLA)', 15.0, (0, 0, 0)), ('bottom plate (PLA)', 11.0, (0, -HY - P.PLATE_T / 2, 0)),
    ('4 servos', 4 * 3.7, (0, P.HIP_Y + 5, 0)), ('battery 602035', 10.0, (P.BATT_X, -HY + 10.5, 0)),
    ('XIAO Sense + camera', 6.0, (-P.EYE_X, P.EYE_Y, 14.0)),
    ('amp', 2.0, (9.5, -HY + 25, 5)), ('VL53L0X', 1.0, (P.EYE_X, 13, 21)), ('speaker', 2.0, (20, P.SPK[0], P.SPK[1])),
    ('toggle switch', 4.0, (0, P.SWITCH_SLOT[1], -17)), ('caps + diode', 2.0, (0, 6, -6)), ('wires + solder + glue', 6.0, (0, 2, 0)),
]
BODY_MASS = sum(m for _, m, _ in PARTS) / 1000
COM = np.sum([np.array(c) * m for _, m, c in PARTS], axis=0) / sum(m for _, m, _ in PARTS)
LEG_MASS = 1.4 / 1000                                            # leg + shoe + horn

# GH-S37D on the 1S battery (spec: 0.5 kg.cm / 0.12 s/60 deg @ 3.6 V, 0.7 / 0.10 @ 4.8 V, rated 3.6-4.8 V). Use the 3.6 V figures.
SERVO = {'torque': 0.049, 'speed': math.radians(60) / 0.12, 'limit': math.radians(45), 'stall_A': 0.45, 'idle_A': 0.03}


def leg_mesh(side):
    """Leg + shoe for one side as a sim-frame trimesh, origin on the hip axis."""
    leg = P.leg().translate([G['leg_x0'], 0, 0]) + P.shoe().translate([G['shoe_x'], G['shoe_top_y'], 0])
    if side < 0:
        leg = leg.mirror([1, 0, 0])
    m = leg.to_mesh()
    v = np.asarray(m.vert_properties)[:, :3]
    v = np.c_[v[:, 0], v[:, 2], v[:, 1]] / 1000                   # (x, z, y) -> sim
    tm = trimesh.Trimesh(v, np.asarray(m.tri_verts), process=True)
    tm.fix_normals()
    # one file per process, written atomically: parallel sim workers used to read each other's half-written legs
    path = os.path.join(OUT, f'leg_{"R" if side > 0 else "L"}_{os.getpid()}.obj')
    tmp = path + '.tmp.obj'
    tm.export(tmp)
    os.replace(tmp, path)
    return path, tm


def build(cl, mu=0.35, mass_scale=1.0, torque_scale=1.0, start=(0, 0), yaw=0.0):
    hb = [P.BODY_W / 2000, P.BODY_D / 2000, (P.BODY_H + P.PLATE_T) / 2000]       # shell + the 4.9 mm bottom plate
    zc = (P.BODY_H / 2 - P.PLATE_T / 2) / 1000                   # box centre above the robot origin... see below
    body_c = [0, 0, -P.PLATE_T / 2000]                   # box centre relative to robot origin (sim)
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=hb, collisionFramePosition=body_c, physicsClientId=cl)
    eye_q = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
    ex, ey, fz = P.EYE_X / 1000, P.EYE_Y / 1000, P.BODY_D / 2000
    vis = p.createVisualShapeArray([p.GEOM_BOX, p.GEOM_CYLINDER, p.GEOM_CYLINDER, p.GEOM_CYLINDER, p.GEOM_CYLINDER],
                                   halfExtents=[hb, [0] * 3, [0] * 3, [0] * 3, [0] * 3],
                                   radii=[0, P.EYE_OD / 2000, P.EYE_OD / 2000, P.EYE_HOLE / 2000, P.EYE_HOLE / 2000],
                                   lengths=[0, 0.0014, 0.0014, 0.0016, 0.0016],
                                   rgbaColors=[[0.93, 0.9, 0.84, 1], [0.98, 0.98, 0.98, 1], [0.98, 0.98, 0.98, 1], [0.05, 0.05, 0.07, 1], [0.05, 0.05, 0.07, 1]],
                                   visualFramePositions=[body_c, [-ex, fz + 0.0007, ey], [ex, fz + 0.0007, ey], [-ex, fz + 0.0015, ey], [ex, fz + 0.0015, ey]],
                                   visualFrameOrientations=[[0, 0, 0, 1], eye_q, eye_q, eye_q, eye_q], physicsClientId=cl)
    links = dict(m=[], col=[], vis=[], pos=[], orn=[], ipos=[], iorn=[], par=[], jt=[], ax=[])
    for sx in (-1, 1):
        path, tm = leg_mesh(sx)
        for sz in (1, -1):
            c = p.createCollisionShape(p.GEOM_MESH, fileName=path, physicsClientId=cl)
            v = p.createVisualShape(p.GEOM_MESH, fileName=path, rgbaColor=[0.93, 0.9, 0.84, 1], physicsClientId=cl)
            links['m'].append(LEG_MASS); links['col'].append(c); links['vis'].append(v)
            links['pos'].append(mm(0, P.HIP_Y, sz * P.HIP_Z)); links['orn'].append([0, 0, 0, 1])
            links['ipos'].append(list(tm.centroid)); links['iorn'].append([0, 0, 0, 1])
            links['par'].append(0); links['jt'].append(p.JOINT_REVOLUTE); links['ax'].append([1, 0, 0])
    # order: LF, LB, RF, RB
    lowest = -(P.LEG_LEN - P.HIP_Y * 0) / 1000
    z0 = (-P.HIP_Y + P.LEG_LEN) / 1000 + 0.0008
    robot = p.createMultiBody(BODY_MASS * mass_scale, col, vis, [start[0], start[1], z0], p.getQuaternionFromEuler([0, 0, yaw]),
                              baseInertialFramePosition=mm(*COM),
                              linkMasses=links['m'], linkCollisionShapeIndices=links['col'], linkVisualShapeIndices=links['vis'],
                              linkPositions=links['pos'], linkOrientations=links['orn'], linkInertialFramePositions=links['ipos'],
                              linkInertialFrameOrientations=links['iorn'], linkParentIndices=links['par'], linkJointTypes=links['jt'],
                              linkJointAxis=links['ax'], physicsClientId=cl)
    for j in range(4):
        p.changeDynamics(robot, j, lateralFriction=mu, rollingFriction=0.0, spinningFriction=0.0002, physicsClientId=cl)
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, force=0, physicsClientId=cl)
        for k in range(j + 1, 4):
            p.setCollisionFilterPair(robot, robot, j, k, 1, physicsClientId=cl)   # legs CAN hit each other
    p.changeDynamics(robot, -1, lateralFriction=mu, physicsClientId=cl)
    robot_torque = SERVO['torque'] * torque_scale
    return robot, robot_torque


LEGS = {'LF': 0, 'LB': 1, 'RF': 2, 'RB': 3}


def desk(cl, mu=0.35, edge_y=None, wall_y=None, slope_deg=0.0, cable_y=None, cable_r=0.002, cable_mass=0.0, desk_w=0.6):
    p.resetSimulation(physicsClientId=cl)
    p.setGravity(0, 0, -9.81, physicsClientId=cl)
    p.setTimeStep(DT, physicsClientId=cl)
    p.setPhysicsEngineParameter(numSolverIterations=80, physicsClientId=cl)
    ids = []
    floor = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE, physicsClientId=cl), basePosition=[0, 0, -0.75], physicsClientId=cl)
    ids.append(floor)
    far = 1.0 if edge_y is None else edge_y
    top = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[desk_w, (far + 1.0) / 2, 0.02], physicsClientId=cl),
                            p.createVisualShape(p.GEOM_BOX, halfExtents=[desk_w, (far + 1.0) / 2, 0.02], rgbaColor=[0.55, 0.42, 0.3, 1], physicsClientId=cl),
                            basePosition=[0, (far - 1.0) / 2, -0.02 + (far - 1.0) / 2 * math.tan(math.radians(slope_deg))],   # surface passes z=0 at y=0
                            baseOrientation=p.getQuaternionFromEuler([math.radians(slope_deg), 0, 0]), physicsClientId=cl)
    ids.append(top)
    if wall_y is not None:
        ids.append(p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.01, 0.1], physicsClientId=cl),
                                     p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.01, 0.1], rgbaColor=[0.3, 0.35, 0.45, 1], physicsClientId=cl),
                                     basePosition=[0, wall_y + 0.01, 0.1], physicsClientId=cl))
    if cable_y is not None:   # a 4 mm USB cable lying across the desk
        ids.append(p.createMultiBody(cable_mass, p.createCollisionShape(p.GEOM_CYLINDER, radius=cable_r, height=0.6, physicsClientId=cl),
                                     p.createVisualShape(p.GEOM_CYLINDER, radius=cable_r, length=0.6, rgbaColor=[0.1, 0.1, 0.1, 1], physicsClientId=cl),
                                     basePosition=[0, cable_y, cable_r], baseOrientation=p.getQuaternionFromEuler([0, math.pi / 2, 0]), physicsClientId=cl))
    # pybullet MULTIPLIES the two bodies' friction coefficients. The desk gets 1.0 so the robot's mu is the real
    # shoe-on-desk friction (before 2 Nov the desk also got mu, so "mu 0.35" was really 0.12: more slippery than labelled).
    for b in ids:
        p.changeDynamics(b, -1, lateralFriction=1.0, physicsClientId=cl)
    return ids


# ---------------- gaits (targets in radians, per leg) ----------------
A = math.radians(12)


def saw(ph, fast=0.2):
    ph %= 1
    return -A + 2 * A * ph / fast if ph < fast else A - 2 * A * (ph - fast) / (1 - fast)


def gait(name, t, f=1.5):
    a, b = saw(f * t), saw(f * t + 0.5)
    if name == 'forward':
        return {'LF': a, 'RB': a, 'RF': b, 'LB': b}
    if name == 'climb':            # front legs reach far forward (feet lift ~L(1-cos30) = 4 mm), rear legs push far back
        def sw(ph, lo, hi, fast=0.2):
            ph %= 1
            return lo + (hi - lo) * ph / fast if ph < fast else hi - (hi - lo) * (ph - fast) / (1 - fast)
        lo_f, hi_f, lo_r, hi_r = math.radians(-12), math.radians(32), math.radians(-32), math.radians(12)
        fa, fb = f * t * 0.6, f * t * 0.6 + 0.5
        return {'LF': sw(fa, lo_f, hi_f), 'RB': sw(fa, lo_r, hi_r), 'RF': sw(fb, lo_f, hi_f), 'LB': sw(fb, lo_r, hi_r)}
    if name == 'back':
        return {'LF': -a, 'RB': -a, 'RF': -b, 'LB': -b}
    if name == 'pivot_right':      # left side walks, right side holds
        return {'LF': a, 'LB': b, 'RF': 0, 'RB': 0}
    if name == 'spin_right':
        return {'LF': a, 'RB': -a, 'RF': -b, 'LB': b}
    return {k: 0.0 for k in LEGS}


# ---------------- sensors ----------------
def tof_reading(cl, robot, tilt_deg):
    """VL53L0X behind the right eye, pointing forward and tilted down by tilt_deg. Returns mm (8190 = no target).
    The ray starts just outside the eye so the shell itself isn't seen."""
    pos, orn = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
    R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    sx, sy, sz = P.TOF_SENSOR if tilt_deg == P.TOF_TILT else (P.EYE_X, P.EYE_Y, P.BODY_D / 2 - 2)
    tr = math.radians(tilt_deg)
    start = np.array([sx, sy - 8 * math.sin(tr), sz + 8 * math.cos(tr)])              # 8 mm along the beam = outside the shell
    origin = np.array(pos) + R @ np.array(mm(*start))
    t = math.radians(tilt_deg)
    d = R @ np.array([0, math.cos(t), -math.sin(t)])
    hit = p.rayTest(list(origin), list(origin + d * 1.2), physicsClientId=cl)[0]
    return 8190.0 if hit[0] < 0 else hit[2] * 1200.0


# ---------------- one run ----------------
def run(name, seconds=10.0, plan=None, env=None, robot_kw=None, tof_tilt=None, controller=None, frames=None, cl=None):
    env = env or {}
    desk(cl, mu=env.get('mu', 0.35), edge_y=env.get('edge_y'), wall_y=env.get('wall_y'), slope_deg=env.get('slope', 0.0), cable_y=env.get('cable_y'), cable_r=env.get('cable_r', 0.002), cable_mass=env.get('cable_mass', 0.0))
    robot, torque = build(cl, mu=env.get('mu', 0.35), **(robot_kw or {}))
    for _ in range(600):                                       # settle at neutral
        for j in range(4):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, 0, force=torque, maxVelocity=SERVO['speed'], physicsClientId=cl)
        p.stepSimulation(physicsClientId=cl)
    pos0, orn0 = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
    yaw0 = p.getEulerFromQuaternion(orn0)[2]
    worst_tilt, amps_sum, amps_peak, log, state = 0.0, 0.0, 0.0, [], {'mode': 'walk', 't_mode': 0.0, 'events': []}
    n = int(seconds / DT)
    for i in range(n):
        t = i * DT
        mode = plan(t) if plan else 'forward'
        if controller and i % 20 == 0:                          # 50 Hz control loop with the distance sensor
            r = tof_reading(cl, robot, tof_tilt)
            mode = controller(t, r, state) or mode
            state['last'] = mode
        elif controller:
            mode = state.get('last', mode)
        tgt = gait(mode, t)
        for k, j in LEGS.items():
            q = max(-SERVO['limit'], min(SERVO['limit'], tgt[k]))
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, q, force=torque, maxVelocity=SERVO['speed'], physicsClientId=cl)
        p.stepSimulation(physicsClientId=cl)
        pos, orn = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
        r_, pi_, _ = p.getEulerFromQuaternion(orn)
        if pos[2] > -0.2:
            worst_tilt = max(worst_tilt, abs(r_), abs(pi_))
        taus = [abs(p.getJointState(robot, j, physicsClientId=cl)[3]) for j in range(4)]
        amps = sum(SERVO['idle_A'] + (SERVO['stall_A'] - SERVO['idle_A']) * min(1, tq / torque) for tq in taus) if mode != 'stop' else 0.02
        amps_sum += amps; amps_peak = max(amps_peak, amps)
        if frames is not None and i % 100 == 0:
            frames.append((t, pos, orn))
        if env.get('edge_y') is not None and i % 10 == 0:
            ls = [p.getLinkState(robot, j, physicsClientId=cl)[0][1] for j in range(4)]
            state['max_front'] = max(state.get('max_front', -1), max(ls) + (P.SHOE_L / 2) / 1000, pos[1] + (P.BODY_D / 2) / 1000 * math.cos(p.getEulerFromQuaternion(orn)[2]))
        if i % 50 == 0:
            log.append([round(t, 2), round(pos[0] * 1000, 1), round(pos[1] * 1000, 1), round(pos[2] * 1000, 1), mode])
    pos1, orn1 = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
    fell_off = pos1[2] < -0.1
    return {'name': name, 'forward_mm': (pos1[1] - pos0[1]) * 1000, 'side_mm': (pos1[0] - pos0[0]) * 1000,
            'yaw_deg': math.degrees(p.getEulerFromQuaternion(orn1)[2] - yaw0), 'tilt_deg': math.degrees(worst_tilt),
            'tipped': math.degrees(worst_tilt) > 45 and not fell_off, 'fell_off': fell_off, 'seconds': seconds,
            'edge_gap_mm': (env['edge_y'] - state.get('max_front', 0)) * 1000 if env.get('edge_y') is not None else None, 'servo_avg_A': amps_sum / n, 'servo_peak_A': amps_peak, 'final_y_mm': pos1[1] * 1000, 'events': state['events'], 'log': log,
            'robot': robot}


# ---------------- controllers ----------------
def edge_controller(tilt):
    """Walk forward; if the down-looking ToF suddenly reads much farther than the flat-desk baseline, it's an
    edge: stop, back up 1.5 s, then stop. Also stops for anything closer than 60 mm (a wall)."""
    def ctl(t, r, s):
        if 'base' not in s and t > 0.3:
            s['base'] = r
        if s['mode'] == 'walk' and 'base' in s:
            if r > s['base'] * 1.35 or r > 1000:
                s['mode'], s['t_mode'] = 'backup', t; s['events'].append(f'edge seen at {t:.2f}s (ToF {r:.0f} mm, flat {s["base"]:.0f} mm)')
            elif r < s['base'] * 0.75:
                s['mode'], s['t_mode'] = 'stop', t; s['events'].append(f'obstacle seen at {t:.2f}s (ToF {r:.0f} mm, flat {s["base"]:.0f} mm)')
        if s['mode'] == 'backup' and t - s['t_mode'] > 1.5:
            s['mode'] = 'stop'
        return {'walk': 'forward', 'backup': 'back', 'stop': 'stop'}[s['mode']]
    return ctl


def wall_controller():
    def ctl(t, r, s):
        if s['mode'] == 'walk' and r < 60:
            s['mode'] = 'stop'; s['events'].append(f'wall at {r:.0f} mm, stopped at {t:.2f}s')
        return 'forward' if s['mode'] == 'walk' else 'stop'
    return ctl


def render_gif(cl, scen, path, cam):
    """Re-run a scenario and grab frames with pybullet's software renderer."""
    from PIL import Image
    imgs = []
    W, H = 360, 270
    env = scen.get('env', {})
    desk(cl, mu=env.get('mu', 0.35), edge_y=env.get('edge_y'), wall_y=env.get('wall_y'), slope_deg=env.get('slope', 0.0), cable_y=env.get('cable_y'), desk_w=0.1)
    robot, torque = build(cl, mu=env.get('mu', 0.35))
    state = {'mode': 'walk', 't_mode': 0.0, 'events': []}
    ctl, tilt = scen.get('controller'), scen.get('tof_tilt')
    for _ in range(600):
        for j in range(4):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, 0, force=torque, maxVelocity=SERVO['speed'], physicsClientId=cl)
        p.stepSimulation(physicsClientId=cl)
    mode = 'forward'
    for i in range(int(scen['seconds'] / DT)):
        t = i * DT
        mode = scen['plan'](t) if scen.get('plan') else 'forward'
        if ctl:
            if i % 20 == 0:
                state['last'] = ctl(t, tof_reading(cl, robot, tilt), state)
            mode = state.get('last', 'forward')
        tgt = gait(mode, t)
        for k, j in LEGS.items():
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, max(-SERVO['limit'], min(SERVO['limit'], tgt[k])), force=torque, maxVelocity=SERVO['speed'], physicsClientId=cl)
        p.stepSimulation(physicsClientId=cl)
        if i % 40 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
            tgt_c = [pos[0], pos[1], max(pos[2], 0.0) + 0.01] if cam.get('follow') else cam['target']
            view = p.computeViewMatrixFromYawPitchRoll(tgt_c, cam['dist'], cam['yaw'], cam['pitch'], 0, 2, physicsClientId=cl)
            proj = p.computeProjectionMatrixFOV(40, W / H, 0.01, 5, physicsClientId=cl)
            _, _, rgb, _, _ = p.getCameraImage(W, H, view, proj, renderer=p.ER_TINY_RENDERER, lightDirection=[0.4, -0.6, 1], shadow=1, physicsClientId=cl)
            im = Image.fromarray(np.reshape(rgb, (H, W, 4))[:, :, :3].astype(np.uint8))
            imgs.append(im)
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=40, loop=0)
    return len(imgs)


if __name__ == '__main__':
    cl = p.connect(p.DIRECT)
    print(f'Cubi: body {BODY_MASS * 1000:.0f} g + legs {4 * LEG_MASS * 1000:.0f} g; centre of mass (mm) x {COM[0]:.1f}, y {COM[1]:.1f}, z {COM[2]:.1f}')
    flat_tof = None
    S = [
        dict(name='Walk forward, wood desk (mu 0.35)', seconds=12),
        dict(name='Walk backward', seconds=12, plan=lambda t: 'back'),
        dict(name='Pivot turn', seconds=12, plan=lambda t: 'pivot_right'),
        dict(name='Spin in place', seconds=12, plan=lambda t: 'spin_right'),
        dict(name='Slippery desk (mu 0.2, glass/laminate)', seconds=12, env={'mu': 0.2}),
        dict(name='Grippy mat (mu 0.6)', seconds=12, env={'mu': 0.6}),
        dict(name='30% heavier (glue, wires)', seconds=12, robot_kw={'mass_scale': 1.3}),
        dict(name='Weak servos (half torque)', seconds=12, robot_kw={'torque_scale': 0.5}),
        dict(name='Uphill 5 deg', seconds=12, env={'slope': 5}),
        dict(name='Downhill 5 deg', seconds=12, env={'slope': -5}),
        dict(name='Taped-down 2 mm cable (can\'t move)', seconds=12, env={'cable_y': 0.08, 'cable_r': 0.001}),
        dict(name='OLD: desk edge, ToF looking straight ahead', seconds=16, env={'edge_y': 0.25}, tof_tilt=0, controller=wall_controller()),
        dict(name='Desk edge, ToF 30 deg down (new mount)', seconds=16, env={'edge_y': 0.25}, tof_tilt=30, controller=edge_controller(30)),
        dict(name='Desk edge 30 deg off-square, sensor side first', seconds=20, env={'edge_y': 0.25}, tof_tilt=30, controller=edge_controller(30), robot_kw={'yaw': math.radians(-30)}),
        dict(name='Desk edge 60 deg off-square, blind side first', seconds=24, env={'edge_y': 0.25}, tof_tilt=30, controller=edge_controller(30), robot_kw={'yaw': math.radians(60)}),
        dict(name='Desk edge on a slippery desk (mu 0.2)', seconds=16, env={'edge_y': 0.25, 'mu': 0.2}, tof_tilt=30, controller=edge_controller(30)),
        dict(name='Desk edge going downhill 5 deg', seconds=16, env={'edge_y': 0.25, 'slope': -5}, tof_tilt=30, controller=edge_controller(30)),
        dict(name='Wall ahead, ToF 30 deg down', seconds=12, env={'wall_y': 0.2}, tof_tilt=30, controller=edge_controller(30)),
        dict(name='Loose USB cable lying across its path', seconds=12, env={'cable_y': 0.08, 'cable_mass': 0.015}),
    ]
    results = []
    for sc in ([] if '--gif-only' in sys.argv else S):
        r = run(cl=cl, **sc)
        results.append(r)
        ev = ('; ' + '; '.join(r['events'])) if r['events'] else ''
        if r['edge_gap_mm'] is not None and not r['fell_off']:
            ev += f"; closest it got to the edge: {r['edge_gap_mm']:.0f} mm"
        status = 'FELL OFF THE DESK' if r['fell_off'] else ('TIPPED OVER' if r['tipped'] else 'ok')
        print(f"{r['name']:48s} fwd {r['forward_mm']:7.1f} mm ({r['forward_mm'] / r['seconds']:5.1f} mm/s) side {r['side_mm']:6.1f} "
              f"yaw {r['yaw_deg']:7.1f} deg  tilt {r['tilt_deg']:4.1f}  servo {r['servo_avg_A']:.2f} A avg / {r['servo_peak_A']:.2f} peak  {status}{ev}")
    json.dump([{k: v for k, v in r.items() if k != 'robot'} for r in results], open(os.path.join(OUT, 'results.json'), 'w'), indent=1)
    if '--gif' in sys.argv or '--gif-only' in sys.argv:
        print('rendering gifs...')
        print(render_gif(cl, S[0], os.path.join(OUT, 'walk.gif'), {'follow': True, 'dist': 0.22, 'yaw': 150, 'pitch': -18}))
        print(render_gif(cl, S[12], os.path.join(OUT, 'edge_tilted.gif'), {'follow': False, 'target': [0, 0.2, -0.08], 'dist': 0.55, 'yaw': 75, 'pitch': -12}))
        print(render_gif(cl, S[11], os.path.join(OUT, 'edge_straight.gif'), {'follow': False, 'target': [0, 0.2, -0.08], 'dist': 0.55, 'yaw': 75, 'pitch': -12}))
