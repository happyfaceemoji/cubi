"""CUBI - puppy behaviours, tested in the physics sim.

Each trick is a timeline of the four hip angles (deg, + = leg swings forward), exactly what the firmware plays
(firmware/ponder_selftest/cubi_play.h mirrors these numbers). A hip-only leg can't make a pose you'd notice
(12 deg of swing lowers a 30 mm leg by 0.7 mm), so every trick is MOTION: shimmies, pats, bounces, bursts.

  python cubi_play.py tricks     each trick on its own: tilt, drift, leg limits, current
  python cubi_play.py soak       the mood engine for 60 s at a time, near a desk edge, with random taps / hands /
                                 claps, 16 runs. Pass = never falls, never tips.
  python cubi_play.py sounds     writes sim_out/sounds/*.wav (the same synth the firmware uses)
"""
import math, os, sys, json, wave, struct
from multiprocessing import Pool
import numpy as np
import pybullet as p
import ponder_sim as S
import ponder_parts as P
import cubi_train as C

LIM = 12.0
brain = C.load_brain()
G = brain.get('gait', dict(C.HAND))
EDGE = brain.get('edge', {'ratio': 1.35, 'backup_s': 1.5, 'wall_ratio': 0.75})
LEGS = ['LF', 'LB', 'RF', 'RB']


# Each leg's reach is set by its 90 deg servo and the ~30 deg horn offset: front legs -15..+75, rear -75..+15.
_h, _r = P.HORN_OFFSET_DEG, P.SERVO_TRAVEL_DEG / 2
REACH = {'LF': (_h - _r, _h + _r), 'RF': (_h - _r, _h + _r), 'LB': (-_h - _r, -_h + _r), 'RB': (-_h - _r, -_h + _r)}
TUCK = P.TUCK_DEG                 # how far the legs fold up at shutdown (180 = vertical, pointing up beside her)


def clamp(d, lim=LIM):
    """Walking and play stay within +-12 deg (front and rear legs touch at 17 deg). Folding (lim=None) may use the
    whole servo reach."""
    if lim is None:
        return {k: max(REACH[k][0], min(REACH[k][1], v)) for k, v in d.items()}
    return {k: max(-lim, min(lim, v)) for k, v in d.items()}


def env(t, dur, ramp=0.12):
    """Soft start / stop so no trick jerks the servos."""
    return max(0.0, min(1.0, t / ramp, (dur - t) / ramp))


def walk_deg(t, direction=1.0, side=(1.0, 1.0)):
    """Trained gait in degrees; side=(left, right) scales each side (pivots and spins)."""
    q = C.gait_targets(G, t, 1.0)
    out = {}
    for k in LEGS:
        s = side[0] if k[0] == 'L' else side[1]
        out[k] = math.degrees(q[k]) * direction * s
    return out


# ---- the tricks: name -> (duration s, f(t) -> leg degrees, moves_forward, sound) ----
def wiggle(t):          # tail-wag stand-in: left and right sides swing opposite ways, 6 Hz -> body shimmies
    a = 6 * env(t, 1.0) * math.sin(2 * math.pi * 6 * t)
    return {'LF': a, 'LB': a, 'RF': -a, 'RB': -a}


def paw_pat(t):         # play-bow stand-in: front paws pat the desk in turn, rear legs braced back
    a = 10 * env(t, 1.2) * math.sin(2 * math.pi * 5 * t)
    return {'LF': a, 'RF': -a, 'LB': -6 * env(t, 1.2), 'RB': -6 * env(t, 1.2)}


def bounce(t):          # excited bob: all four legs together, fast -> little surges
    a = 6 * env(t, 1.0) * math.sin(2 * math.pi * 4 * t)   # 8 deg @ 5 Hz rocked her 17 deg; 6 Hz resonates
    return {k: a for k in LEGS}


def chase(t):           # burst toward your hand
    return walk_deg(t)


def zoomies_run(t):     # 2 s flat out ...
    return walk_deg(t)


def zoomies_spin(t):    # ... then a spin: left legs forward, right legs backward
    return walk_deg(t, side=(1.0, -1.0))


def sniff(t):           # look left, pause, look right, pause (pivots: one side steps, other holds)
    if t < 0.9:
        return walk_deg(t, side=(0.0, 1.0))
    if t < 1.3:
        return {k: 0.0 for k in LEGS}
    if t < 2.2:
        return walk_deg(t, side=(1.0, 0.0))
    return {k: 0.0 for k in LEGS}


def startle(t):         # hop back ~1 cm, freeze
    return walk_deg(t, -1.0) if t < 0.45 else {k: 0.0 for k in LEGS}


BOW_DEG, KICK_DEG = 72.0, 60.0    # uses the folding reach: front legs out to 72 forward, each back leg 60 back


def stretch(t):         # wake-up stretch, like a real puppy: play bow (front paws way out, bum up, little bob),
    q = {'LF': 0.0, 'RF': 0.0, 'LB': 0.0, 'RB': 0.0}          # then each back leg stretched out behind, one at a time
    if t < 2.2:
        e = smooth(t / 0.8) * (1 - smooth((t - 1.6) / 0.6))
        q['LF'] = q['RF'] = BOW_DEG * e
        if 0.8 < t < 1.6:                                       # bum bob: rear legs IN phase (opposite phase twists her)
            q['LB'] = q['RB'] = 3 * math.sin(2 * math.pi * 3 * (t - 0.8))
    else:
        for leg, t0 in (('LB', 2.3), ('RB', 3.4)):
            u = t - t0
            q[leg] -= KICK_DEG * smooth(u / 0.35) * (1 - smooth((u - 0.65) / 0.35))
    return q


def settle(t):          # slow sway that winds down, then legs straight (firmware then cuts the pulses)
    a = 3 * max(0.0, 1 - t / 2.0) * math.sin(2 * math.pi * 0.8 * t)
    return {'LF': a, 'LB': a, 'RF': a, 'RB': a}


def fold_to(e):
    a = P.FOLD_DEG * e
    return {'LF': a, 'RF': a, 'LB': -a, 'RB': -a}


def smooth(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def flop(t):            # lie down: legs fold out front and back until she rests flat on her bottom plate
    return fold_to(smooth(t / 1.4))


def getup(t):           # push back up onto her feet
    return fold_to(1 - smooth(t / 1.4))


def tuck_to(a):
    return {'LF': a, 'RF': a, 'LB': -a, 'RB': -a}


def tuck(t):            # shutdown: already lying down, keep folding the legs up beside her
    return tuck_to(P.FOLD_DEG + (TUCK - P.FOLD_DEG) * smooth(t / 1.4))


def untuck(t):          # wake from shutdown: legs back down to the lying-down fold
    return tuck_to(TUCK + (P.FOLD_DEG - TUCK) * smooth(t / 1.4))


TRICKS = {
    'wiggle':  (1.0, wiggle, False, 'trill'),
    'paw_pat': (1.2, paw_pat, False, 'yip2'),
    'bounce':  (1.0, bounce, False, 'yip'),
    'chase':   (1.2, chase, True, 'yip'),
    'zoomies': (3.5, lambda t: zoomies_run(t) if t < 2.0 else zoomies_spin(t), True, 'ruff'),
    'sniff':   (2.6, sniff, False, None),
    'startle': (0.9, startle, False, 'whine'),
    'stretch': (4.5, stretch, True, 'sigh'),      # the bow lunges ~2 cm: edge-guarded like a step
    'settle':  (2.4, settle, False, 'sigh'),
    'flop':    (1.8, flop, False, 'sigh'),
    'getup':   (1.8, getup, False, 'yip'),
    'tuck':    (1.8, tuck, False, None),
    'untuck':  (1.8, untuck, False, None),
}
FOLDING = {'flop', 'getup', 'tuck', 'untuck'}
WIDE = FOLDING | {'stretch'}      # tricks allowed the whole servo reach (the rest stay within +-12)


# ---- the mood engine (same rules as the firmware) ----
class Mood:
    """energy: 0..1, drains with activity, refills at rest (and is capped by battery in firmware).
    excite: 0..1, jumps on a tap / clap / hand, decays with a 20 s time constant."""
    def __init__(self, rng):
        self.rng, self.energy, self.excite, self.idle = rng, 0.8, 0.3, 0.0
        self.last_zoom = -1e9

    def stimulus(self, kind):
        self.excite = min(1.0, self.excite + {'tap': 0.5, 'clap': 0.3, 'hand': 0.25, 'name': 0.35}[kind])
        self.idle = 0.0

    def tick(self, dt, active):
        self.excite *= math.exp(-dt / 20.0)
        self.energy = min(1.0, max(0.0, self.energy + (-0.02 if active else 0.01) * dt))
        self.idle += dt

    def pick(self, now, hand):
        act = self._pick(now, hand)
        if act == getattr(self, 'last', None) and hand is None and act in ('bounce', 'wiggle', 'paw_pat', 'sniff'):
            act = self._pick(now, hand)          # a puppy that does the same thing twice in a row reads as a machine
        self.last = act
        return act

    def _pick(self, now, hand):
        r = self.rng.random()
        if self.idle > 300:
            return 'sleep'
        if hand == 'close':
            return 'paw_pat'
        if hand == 'away' and self.excite > 0.3:
            return 'chase'
        e = self.excite
        if e > 0.7:
            if self.energy > 0.5 and now - self.last_zoom > 60 and r < 0.35:
                self.last_zoom = now
                return 'zoomies'
            return 'bounce' if r < 0.7 else 'wiggle'
        if e > 0.4:
            return ['wiggle', 'paw_pat', 'bounce'][int(r * 3)]
        if e > 0.15:
            return 'sniff' if r < 0.6 else 'explore'
        return 'settle' if r < 0.5 else 'idle'


# ---- sim helpers ----
def run_trick(cl, robot, sv, name, t_scale=1.0, edge_guard=None):
    """Play one trick at 50 Hz. edge_guard(r) -> True stops any forward motion (edge reflex wins)."""
    dur, f, fwd, _ = TRICKS[name] if name in TRICKS else (1.6, chase, True, None)
    dur *= t_scale
    worst, peak_leg, stopped = 0.0, 0.0, False
    n = int(dur * C.CTRL_HZ)
    for i in range(n):
        t = i / C.CTRL_HZ / t_scale
        q = clamp(f(t), None if name in WIDE else LIM)
        peak_leg = max(peak_leg, max(abs(v) for v in q.values()))
        if fwd and edge_guard and edge_guard():
            stopped = True
            break
        sv.command([math.radians(q[k]) for k in S.LEGS])
        for _ in range(C.STEPS_PER_CTRL):
            p.stepSimulation(physicsClientId=cl)
        worst = max(worst, C.tilt_of(cl, robot))
    return math.degrees(worst), peak_leg, stopped


def trick_check(name):
    cl = p.connect(p.DIRECT)
    rows = []
    for w in C.test_worlds(4, seed=55):
        robot, torque, vmax = C.make_robot(cl, w)
        sv = C.Servos(cl, robot, torque, vmax, w['lag_ms'])
        pos0, orn0 = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
        tilt, peak, _ = run_trick(cl, robot, sv, name)
        for _ in range(300):
            p.stepSimulation(physicsClientId=cl)
        pos1, orn1 = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
        d = (np.array(pos1[:2]) - np.array(pos0[:2])) * 1000
        yaw = math.degrees(p.getEulerFromQuaternion(orn1)[2] - p.getEulerFromQuaternion(orn0)[2])
        rows.append((tilt, peak, float(np.hypot(*d)), abs(yaw), pos1[2] < -0.1))
    p.disconnect(cl)
    a = np.array([r[:4] for r in rows])
    return {'trick': name, 'tilt_max': round(a[:, 0].max(), 1), 'leg_max_deg': round(a[:, 1].max(), 1),
            'moves_mm': round(a[:, 2].mean(), 1), 'turns_deg': round(a[:, 3].mean(), 1), 'falls': sum(r[4] for r in rows)}


def soak(seed):
    """60 s of puppy life on a desk with an edge 12-30 cm ahead at a random angle. Random taps, claps, hands.
    The edge reflex guards every forward step; 'startle' backs up, so it's limited to 1 cm per 10 s."""
    rng = np.random.default_rng(seed)
    cl = p.connect(p.DIRECT)
    w = C.sample_world(rng)
    w['yaw'] = math.radians(rng.uniform(-50, 50))
    edge_y = rng.uniform(0.12, 0.30)
    robot, torque, vmax = C.make_robot(cl, w, edge_y=edge_y)
    sv = C.Servos(cl, robot, torque, vmax, w['lag_ms'])
    mood = Mood(rng)
    base = None
    for _ in range(15):
        base = S.tof_reading(cl, robot, P.TOF_TILT)
    def guard():
        r = S.tof_reading(cl, robot, P.TOF_TILT) + rng.normal(0, 3)
        return r > base * EDGE['ratio'] or r > 1000
    t, log, worst, edge_stops, back_budget = 0.0, [], 0.0, 0, 0.0
    while t < 60.0:
        hand = None
        if rng.random() < 0.15:
            mood.stimulus('tap')
        if rng.random() < 0.05:
            mood.stimulus('clap')
            act = 'startle' if mood.excite < 0.4 and back_budget <= 0 else 'bounce'
        else:
            if rng.random() < 0.15:
                hand = 'close' if rng.random() < 0.5 else 'away'
                mood.stimulus('hand')
            act = mood.pick(t, hand)
        if act == 'startle':
            back_budget = 10.0
        if act in ('idle', 'sleep'):
            for _ in range(int(1.0 / S.DT)):
                p.stepSimulation(physicsClientId=cl)
            dt = 1.0
        else:
            name = 'chase' if act == 'explore' else act
            tilt, _, stopped = run_trick(cl, robot, sv, name, t_scale=rng.uniform(0.85, 1.15), edge_guard=guard)
            worst = max(worst, tilt)
            if stopped:   # edge: back off like the firmware does, then turn away
                edge_stops += 1
                for i in range(int(EDGE['backup_s'] * C.CTRL_HZ)):
                    q = walk_deg(i / C.CTRL_HZ, -1.0)
                    sv.command([math.radians(q[k]) for k in S.LEGS])
                    for _ in range(C.STEPS_PER_CTRL):
                        p.stepSimulation(physicsClientId=cl)
                for i in range(int(1.6 * C.CTRL_HZ)):          # turn away ~100 deg before playing on
                    q = clamp(zoomies_spin(i / C.CTRL_HZ))
                    sv.command([math.radians(q[k]) for k in S.LEGS])
                    for _ in range(C.STEPS_PER_CTRL):
                        p.stepSimulation(physicsClientId=cl)
                for _ in range(15):
                    base = 0.7 * base + 0.3 * S.tof_reading(cl, robot, P.TOF_TILT)
            dt = TRICKS.get(name, (1.6,))[0]
        back_budget -= dt
        mood.tick(dt, act not in ('idle', 'sleep', 'settle'))
        t += dt
        log.append(act)
        pos = p.getBasePositionAndOrientation(robot, physicsClientId=cl)[0]
        if pos[2] < -0.1:
            p.disconnect(cl)
            return {'seed': seed, 'fell': True, 'at_s': round(t, 1), 'last': log[-3:], 'tilt_max': round(worst, 1)}
    p.disconnect(cl)
    counts = {k: log.count(k) for k in set(log)}
    return {'seed': seed, 'fell': False, 'tipped': worst > 30, 'tilt_max': round(worst, 1),
            'edge_stops': edge_stops, 'acts': counts}


# ---- sounds (identical maths in firmware/ponder_selftest/cubi_play.h) ----
SR = 16000


def synth(name):
    out = []
    def chirp(f0, f1, ms, vol=0.5, vib=0.0, noise=0.0):
        n = int(SR * ms / 1000); ph = 0.0
        for i in range(n):
            x = i / n
            f = f0 + (f1 - f0) * x + vib * math.sin(2 * math.pi * 7 * i / SR) * f0 * 0.04
            ph += 2 * math.pi * f / SR
            env_ = min(1.0, x * 12) * (1 - x) ** 1.5
            s = math.sin(ph) * (1 - noise) + noise * (np.random.random() * 2 - 1)
            out.append(vol * env_ * s)
    def rest(ms):
        out.extend([0.0] * int(SR * ms / 1000))
    if name == 'yip':
        chirp(900, 1700, 90)
    elif name == 'yip2':
        chirp(900, 1700, 80); rest(60); chirp(1000, 1900, 80)
    elif name == 'whine':
        chirp(1300, 700, 450, vol=0.4, vib=1.0)
    elif name == 'sigh':
        chirp(420, 260, 650, vol=0.35, noise=0.6)
    elif name == 'trill':
        for i in range(8):
            chirp(1300 if i % 2 else 1700, 1500 if i % 2 else 1900, 32, vol=0.4)
    elif name == 'ruff':
        chirp(520, 380, 110, vol=0.6, noise=0.35); rest(70); chirp(560, 400, 90, vol=0.55, noise=0.35)
    return np.array(out)


def write_sounds():
    d = os.path.join(S.OUT, 'sounds'); os.makedirs(d, exist_ok=True)
    for n in ('yip', 'yip2', 'whine', 'sigh', 'trill', 'ruff'):
        x = synth(n); x = np.int16(np.clip(x, -1, 1) * 0.7 * 32767)
        with wave.open(os.path.join(d, f'{n}.wav'), 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR); w.writeframes(x.tobytes())
        print(f'{n:6s} {len(x) / SR * 1000:5.0f} ms -> {d}/{n}.wav')


if __name__ == '__main__':
    what = sys.argv[1] if len(sys.argv) > 1 else 'tricks'
    if what == 'tricks':
        with Pool(os.cpu_count()) as pool:
            res = pool.map(trick_check, list(TRICKS))
        print(f"{'trick':10s} {'tilt max':>8s} {'leg max':>8s} {'moves':>8s} {'turns':>7s} {'falls':>6s}")
        for r in res:
            print(f"{r['trick']:10s} {r['tilt_max']:7.1f}d {r['leg_max_deg']:7.1f}d {r['moves_mm']:6.1f}mm {r['turns_deg']:6.1f}d {r['falls']:6d}")
        json.dump(res, open(os.path.join(S.OUT, 'tricks.json'), 'w'), indent=1)
    elif what == 'soak':
        with Pool(os.cpu_count()) as pool:
            res = pool.map(soak, range(16))
        for r in res:
            print(r)
        falls = sum(r['fell'] for r in res)
        print(f'\n{len(res)} runs x 60 s near a desk edge: {falls} falls, '
              f'{sum(r.get("edge_stops", 0) for r in res)} edge stops, worst tilt {max(r["tilt_max"] for r in res):.1f} deg')
        json.dump(res, open(os.path.join(S.OUT, 'soak.json'), 'w'), indent=1)
    elif what == 'sounds':
        write_sounds()
