"""CUBI - record physics-sim runs for the 3D sim viewer (viewer_sim/cubi-sim.html).

Runs real pybullet scenarios (the same robot, servos and controllers as cubi_train.py / cubi_play.py) and
writes viewer_sim/sim_data.js: per frame (25 fps) the body pose, each leg's pose, the ToF ray and a label.
  python cubi_record.py
"""
import json, math, os
import numpy as np
import pybullet as p
import trimesh
import ponder_sim as S
import ponder_parts as P
import cubi_train as C
import cubi_play as Q

FPS = 25
STEP_EVERY = int(round(1 / (FPS * S.DT)))       # sim steps per recorded frame
R4 = lambda v: [round(float(x), 4) for x in v]


class Rec:
    def __init__(self, cl, robot, world, name, note):
        self.cl, self.robot, self.frames, self.labels, self.i = cl, robot, [], [], 0
        self.label, self.tof, self.ray = '', None, None
        self.meta = {'name': name, 'note': note, 'world': world}

    def lab(self, s):
        if s not in self.labels:
            self.labels.append(s)
        self.label = s

    def sense(self):
        """ToF reading + its ray, like S.tof_reading."""
        cl, robot = self.cl, self.robot
        pos, orn = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
        Rm = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        sx, sy, sz = P.TOF_SENSOR
        tr = math.radians(P.TOF_TILT)
        start = np.array([sx, sy - 8 * math.sin(tr), sz + 8 * math.cos(tr)])
        o = np.array(pos) + Rm @ np.array(S.mm(*start))
        d = Rm @ np.array([0, math.cos(tr), -math.sin(tr)])
        hit = p.rayTest(list(o), list(o + d * 1.2), physicsClientId=cl)[0]
        r = 8190.0 if hit[0] < 0 else hit[2] * 1200.0
        end = o + d * (0.3 if hit[0] < 0 else hit[2] * 1.2)
        self.tof, self.ray = r, (o, end, hit[0] >= 0)
        return r

    def step(self, n=1):
        for _ in range(n):
            p.stepSimulation(physicsClientId=self.cl)
            self.i += 1
            if self.i % STEP_EVERY == 0:
                self.snap()

    def snap(self):
        cl, robot = self.cl, self.robot
        pos, orn = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
        legs = []
        for j in range(4):
            ls = p.getLinkState(robot, j, computeForwardKinematics=True, physicsClientId=cl)
            legs += R4(ls[4]) + R4(ls[5])
        if self.ray is None:
            self.sense()
        o, e, h = self.ray
        self.frames.append(R4(pos) + R4(orn) + legs + R4(o) + R4(e) + [1 if h else 0, round(self.tof or 0), self.labels.index(self.label) if self.label in self.labels else 0])

    def out(self):
        return dict(self.meta, labels=self.labels, frames=self.frames, fps=FPS)


def setup(cl, w, edge_y=None, wall_y=None):
    robot, torque, vmax = C.make_robot(cl, w, edge_y=edge_y, wall_y=wall_y)
    sv = C.Servos(cl, robot, torque, vmax, w['lag_ms'])
    return robot, sv


def drive(rec, sv, f, seconds, label, guard=None, fold=False):
    """Play f(t)->leg degrees (LF,LB,RF,RB keys) at 50 Hz. guard() True stops (edge reflex)."""
    rec.lab(label)
    for i in range(int(seconds * C.CTRL_HZ)):
        r = rec.sense()
        if guard and guard(r):
            return False
        q = Q.clamp(f(i / C.CTRL_HZ), None if fold else Q.LIM)
        sv.command([math.radians(q[k]) for k in S.LEGS])
        rec.step(C.STEPS_PER_CTRL)
    return True


WOOD = {'mu': 0.35, 'mass': 1.1, 'torque': 0.9, 'speed': 0.9, 'lag_ms': 15, 'slope': 0.0, 'yaw': 0.0}


def _ease(x):
    x = max(0.0, min(1.0, x)); return x * x * (3 - 2 * x)


def sc_walk(cl, gait, name, note, soft=False):
    """8 s walk. soft: stride eases in over 0.5 s and shrinks to nothing over 0.6 s at the end (firmware walk())."""
    robot, sv = setup(cl, WOOD)
    rec = Rec(cl, robot, {'mu': 0.35}, name, note)
    if not soft:
        drive(rec, sv, lambda t: {k: math.degrees(v) for k, v in C.gait_targets(gait, t).items()}, 8.0, 'walk')
    else:
        T = 6.9
        env = lambda t: _ease(t / 0.5) * (1 - _ease((t - T) / 0.6))
        drive(rec, sv, lambda t: {k: math.degrees(v) * env(t) for k, v in C.gait_targets(gait, t).items()}, T + 0.6, 'walk (soft start / stop)')
        drive(rec, sv, lambda t: {k: 0.0 for k in Q.LEGS}, 0.5, 'standing')
    return rec


def sc_edge(cl, yaw_deg, name, note):
    w = dict(WOOD, yaw=math.radians(yaw_deg))
    robot, sv = setup(cl, w, edge_y=0.16)
    rec = Rec(cl, robot, {'mu': 0.35, 'edge_y': 0.16}, name, note)
    base = np.mean([rec.sense() for _ in range(10)])
    guard = lambda r: r > base * Q.EDGE['ratio'] or r > 1000
    if not drive(rec, sv, lambda t: Q.walk_deg(t), 14.0, 'walk (edge reflex on)', guard):
        rec.lab('EDGE: back up')
        drive(rec, sv, lambda t: Q.walk_deg(t, -1.0), Q.EDGE['backup_s'], 'EDGE: back up')
        drive(rec, sv, lambda t: Q.zoomies_spin(t), 1.6, 'turn away')
        drive(rec, sv, lambda t: {k: 0.0 for k in Q.LEGS}, 1.0, 'stop')
    return rec


def sc_wall(cl):
    robot, sv = setup(cl, WOOD, wall_y=0.16)
    rec = Rec(cl, robot, {'mu': 0.35, 'wall_y': 0.16}, 'Wall ahead', 'ToF sees the wall closer than 0.75 x the desk reading and stops. In puppy mode, this is also how she sees your hand.')
    base = np.mean([rec.sense() for _ in range(10)])
    if not drive(rec, sv, lambda t: Q.walk_deg(t), 12.0, 'walk', lambda r: r < base * Q.EDGE['wall_ratio']):
        drive(rec, sv, Q.paw_pat, 1.2, 'wall/hand: paw pat')
        drive(rec, sv, lambda t: {k: 0.0 for k in Q.LEGS}, 1.2, 'stop')
    return rec


def sc_tricks(cl):
    robot, sv = setup(cl, WOOD)
    rec = Rec(cl, robot, {'mu': 0.35}, 'All puppy tricks', 'Each trick in turn, as puppy mode plays them (keys 1-9 in the Serial Monitor).')
    for name in ['wiggle', 'paw_pat', 'bounce', 'chase', 'zoomies', 'sniff', 'startle', 'stretch', 'settle', 'flop', 'getup']:
        dur, f, _, sound = Q.TRICKS[name]
        lab = {'flop': 'lie down', 'getup': 'get up'}.get(name, name.replace('_', ' '))
        drive(rec, sv, f, dur, lab + (f'  ({sound})' if sound else ''), fold=name in Q.WIDE)
        if name == 'flop':
            drive(rec, sv, lambda t: Q.fold_to(1.0), 1.5, 'resting on her belly', fold=True)
        drive(rec, sv, lambda t: {k: 0.0 for k in Q.LEGS}, 0.6, 'pause')
    return rec


def sc_liedown(cl):
    robot, sv = setup(cl, WOOD)
    rec = Rec(cl, robot, {'mu': 0.35}, 'Lie down and get up',
              'Front legs fold 75 deg forward, rear legs 75 deg back, until she rests flat on her bottom plate; then she pushes back up. Keys L and U, and what puppy mode does when she falls asleep.')
    drive(rec, sv, lambda t: {k: 0.0 for k in Q.LEGS}, 0.8, 'standing')
    drive(rec, sv, Q.flop, 1.8, 'lie down  (sigh)', fold=True)
    drive(rec, sv, lambda t: Q.fold_to(1.0), 2.5, 'resting on her belly (servos off)', fold=True)
    drive(rec, sv, Q.getup, 1.8, 'get up  (yip)', fold=True)
    drive(rec, sv, Q.wiggle, 1.0, 'wiggle  (trill)')
    drive(rec, sv, lambda t: {k: 0.0 for k in Q.LEGS}, 0.8, 'standing')
    return rec


def sc_shutdown(cl, tuck_deg, name, note):
    saved_reach, saved_tuck = dict(Q.REACH), Q.TUCK
    Q.TUCK = tuck_deg
    Q.REACH.update({'LF': (-15.0, 180.0), 'RF': (-15.0, 180.0), 'LB': (-180.0, 15.0), 'RB': (-180.0, 15.0)})
    robot, sv = setup(cl, WOOD)
    rec = Rec(cl, robot, {'mu': 0.35}, name, note)
    drive(rec, sv, lambda t: {k: 0.0 for k in Q.LEGS}, 0.6, 'goodnight (Z)')
    drive(rec, sv, Q.settle, 2.4, 'settle  (sigh)')
    drive(rec, sv, Q.flop, 1.8, 'lie down', fold=True)
    if tuck_deg > P.FOLD_DEG + 1:
        drive(rec, sv, Q.tuck, 1.8, f'tuck legs up to {tuck_deg:.0f} deg', fold=True)
    drive(rec, sv, lambda t: Q.tuck_to(tuck_deg), 2.0, 'off: servos relaxed, flip the switch', fold=True)
    if tuck_deg > P.FOLD_DEG + 1:
        drive(rec, sv, Q.untuck, 1.8, 'wake: legs down', fold=True)
    drive(rec, sv, Q.getup, 1.8, 'get up  (yip)', fold=True)
    drive(rec, sv, lambda t: {k: 0.0 for k in Q.LEGS}, 0.6, 'standing')
    Q.REACH.clear(); Q.REACH.update(saved_reach); Q.TUCK = saved_tuck
    return rec


def sc_puppy(cl, seed=4):
    """40 s of the real mood engine near a desk edge, with random taps, claps and hands."""
    rng = np.random.default_rng(seed)
    w = dict(WOOD, yaw=math.radians(25))
    robot, sv = setup(cl, w, edge_y=0.22)
    rec = Rec(cl, robot, {'mu': 0.35, 'edge_y': 0.22}, 'Puppy mode, 40 s near an edge',
              'The mood engine choosing tricks on its own while random taps, claps and hands arrive. The edge reflex overrides every forward trick.')
    mood = Q.Mood(rng)
    base = np.mean([rec.sense() for _ in range(10)])
    guard = lambda r: r > base * Q.EDGE['ratio'] or r > 1000
    t = 0.0
    while t < 40.0:
        hand, stim = None, ''
        if rng.random() < 0.15:
            mood.stimulus('tap'); stim = 'tap! '
        if rng.random() < 0.05:
            mood.stimulus('clap'); stim += 'clap! '
            act = 'startle' if mood.excite < 0.4 else 'bounce'
        else:
            if rng.random() < 0.15:
                hand = 'close' if rng.random() < 0.5 else 'away'
                mood.stimulus('hand'); stim += f'hand {hand}! '
            act = mood.pick(t, hand)
        lbl = f'{stim}{act}  (excite {mood.excite:.2f})'
        if act in ('idle', 'sleep'):
            drive(rec, sv, lambda tt: {k: 0.0 for k in Q.LEGS}, 1.0, lbl); dt = 1.0
        else:
            name = 'chase' if act == 'explore' else act
            dur, f, fwd, _ = Q.TRICKS[name]
            ok = drive(rec, sv, f, dur, lbl, guard if fwd else None)
            if not ok:
                drive(rec, sv, lambda tt: Q.walk_deg(tt, -1.0), Q.EDGE['backup_s'], 'EDGE: back up')
                drive(rec, sv, lambda tt: Q.zoomies_spin(tt), 1.6, 'turn away')
                base = 0.7 * base + 0.3 * np.mean([rec.sense() for _ in range(5)])
            dt = dur
        mood.tick(dt, act not in ('idle', 'sleep', 'settle'))
        t += dt
    return rec


def leg_mesh(side):
    m = trimesh.load(os.path.join(S.OUT, f'leg_{side}_{os.getpid()}.obj'))
    return {'v': [round(float(x), 5) for x in m.vertices.flatten()], 'f': [int(x) for x in m.faces.flatten()]}


def sc_mind(cl, persona='golden', seed=41, seconds=150):
    """Mind mode on a finite 50 x 36 cm desk, with a simulated person who comes and goes."""
    import cubi_mind as M
    rng = np.random.default_rng(seed)
    w = dict(WOOD)
    body = M.SimBody(cl, w, rng)
    rec = Rec(cl, body.robot, {'mu': w['mu'], 'desk': [body.desk[0], body.desk[1], body.desk_c[0], body.desk_c[1]]},
              f'Mind mode: a {"golden retriever" if persona == "golden" else persona}, {seconds // 60} min',
              f"No commands: everything here is her own idea. A '{persona}' personality{' (retriever habits: greets you, never startles, wags while sitting, yips instead of whining)' if persona in M.BREEDS else ''} (" + ', '.join(f'{k} {v}' for k, v in M.PERSONALITIES[persona].items()) +
              "). Curiosity, loneliness, boredom and tiredness build up and she picks what to do; while exploring she glances left and right "
              "every 2 s so she can't sidle off an edge. A simulated person comes and goes and taps her when she does what they like. Naps are time-lapsed.")
    n = [0]
    def cb(label):
        rec.lab(label)
        n[0] += 1
        if n[0] % 2 == 0:
            rec.sense(); rec.snap()
    body.rec = cb
    mind = M.Mind(np.random.default_rng(seed + 1), M.PERSONALITIES[persona], None, M.USE_MEMORY, persona if persona in M.BREEDS else None)
    mind.energy, mind.curiosity, mind.boredom = 0.45, 0.8, 0.5        # start tired-ish and curious so a nap shows up
    M.live(mind, body, M.Person(np.random.default_rng(seed + 2)), seconds)
    return rec


def main():
    cl = p.connect(p.DIRECT)
    b = C.load_brain()
    gait = b.get('gait', dict(C.HAND))
    runs = [
        sc_walk(cl, gait, 'Trained walk', 'The gait trained in simulation (cubi_gait.h): 12 deg, 2.2 steps/s, 35% fast swing. About 27 mm/s. Now with a soft start and stop.', soft=True),
        sc_walk(cl, b['gait_calm'], 'Calm walk (smoothest)', 'Trained for smoothness with at least 18 mm/s: 1.9 steps/s, 10 deg strides, softened reversals. 18.6 mm/s, 30% less jolting and 32% less bob than the trained walk.', soft=True),
        sc_walk(cl, dict(C.HAND), 'Hand-tuned walk', 'The original gait: 12 deg, 1.5 steps/s, 20% fast swing. About 17 mm/s on the same desk.'),
        sc_edge(cl, 0, 'Desk edge, square on', 'The ToF looks 30 deg down at the desk ~10 cm ahead. Past the edge it reads nothing: stop, back up, turn away.'),
        sc_edge(cl, 40, 'Desk edge at 40 deg', 'Approaching at an angle, sensor side first.'),
        sc_wall(cl),
        sc_liedown(cl),
        sc_shutdown(cl, P.TUCK_DEG, 'Shutdown, your servos (90 deg)',
                    'Goodnight: settle, lie down, servos off. With 90 deg servos the legs stop at 75 deg, so this is as far as the tuck goes.'),
        sc_shutdown(cl, 180.0, 'Shutdown, 180 deg preview',
                    'PREVIEW: what the tuck looks like if your servos turn ~210 deg (measure with the self-test r command). Legs fold all the way up, vertical beside her. Clearance checked to 180 deg.'),
        sc_tricks(cl),
        sc_puppy(cl),
        sc_mind(cl),
    ]
    # body: box relative to the base COM frame (pybullet reports the base pose at the inertial frame)
    com = S.mm(*S.COM)
    data = {
        'body': {'half': [P.BODY_W / 2000, P.BODY_D / 2000, (P.BODY_H + P.PLATE_T) / 2000],
                 'offset': [0 - com[0], 0 - com[1], -P.PLATE_T / 2000 - com[2]],
                 'eye_x': P.EYE_X / 1000, 'eye_y': P.EYE_Y / 1000, 'eye_od': P.EYE_OD / 1000, 'eye_hole': P.EYE_HOLE / 1000, 'eye_ring': [v / 1000 for v in P.EYE_RING], 'eye_open': [v / 1000 for v in P.EYE_OPEN], 'eye_dy': P.EYE_DY / 1000,
                 'front': P.BODY_D / 2000},
        'legs': {'L': leg_mesh('L'), 'R': leg_mesh('R')}, 'leg_side': ['L', 'L', 'R', 'R'],
        'runs': [r.out() for r in runs],
    }
    os.makedirs('viewer_sim', exist_ok=True)
    s = json.dumps(data, separators=(',', ':'))
    open('viewer_sim/sim_data.js', 'w').write('window.CUBI_SIM=' + s + ';')
    print('wrote viewer_sim/sim_data.js', len(s) // 1024, 'KB')
    for r in runs:
        print(f"  {r.meta['name']:32s} {len(r.frames) / FPS:5.1f} s  labels {len(r.labels)}")


if __name__ == '__main__':
    main()
