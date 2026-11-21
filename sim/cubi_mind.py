"""CUBI - a mind of her own. Tested in the physics sim; firmware/ponder_selftest/cubi_mind.h mirrors every number.

Puppy mode only REACTS (tap -> wiggle, clap -> startle). The mind gives her her own reasons to act:

  traits    bold, social, playful, lazy (0..1). Rolled once on first boot, saved forever: every Cubi is different.
  drives    energy, curiosity, social, boredom (0..1) build up and drain over time. Excitement spikes on a tap.
  choice    each drive scores the behaviour that satisfies it (explore, call for you, play, nap, look around,
            idle), weighted by her traits. She picks with a little randomness (softmax), so she's consistent
            but never a loop.
  learning  after she plays a trick, a tap within 4 s is praise: that trick gets more likely. Ignored ones fade.
            Saved, so she keeps what she learned.
  memory    dead-reckoned map of the desk (2 cm cells): where she's been and where she found edges. Explore
            heads for places she hasn't been and away from remembered edges. No gyro, so it fades.

The reflex layer from puppy mode stays on top: the edge reflex guards every step, taps / claps / hands
interrupt whatever she's doing.

  python cubi_mind.py day        6-min days on a finite desk, 3 personalities x 3 seeds: falls, time budget
  python cubi_mind.py memory     same days with the desk memory on vs off: edge hits, area covered
  python cubi_mind.py learn      two days in a row with someone who loves 'wiggle': does she learn it?
"""
import math, sys, json
from multiprocessing import Pool
import numpy as np
import pybullet as p
import ponder_sim as S
import ponder_parts as P
import cubi_train as C
import cubi_play as Q

# ---------------- the mind (pure logic, mirrored in firmware) ----------------
TRAITS = ('bold', 'social', 'playful', 'lazy')
PLAY_TRICKS = ['wiggle', 'paw_pat', 'bounce', 'sniff', 'stretch', 'zoomies']
WALK_MM_S, BACK_MM_S, SPIN_DEG_S = 29.0, 30.0, 22.6       # measured in the sim (6 worlds); firmware uses the same
CELL = 20.0                                               # memory grid, mm
PRAISE_S = 4.0                                            # a tap this soon after a trick = praise
TEMP = 0.12                                               # choice randomness (softmax temperature)
GLANCE_DEG, GLANCE_EVERY = 25.0, 2.0                      # while exploring: look 25 deg left and right every 2 s of
                                                          # walking. The sensor only looks ahead; walking within ~20 deg of
                                                          # parallel to an edge, the side legs go over before it sees it.
USE_MEMORY = False                                        # desk memory OFF: without a gyro it drifts and didn't reduce edge hits (python cubi_mind.py memory)
PERSONALITIES = {'explorer': dict(bold=0.9, social=0.3, playful=0.4, lazy=0.2),
                 'cuddler':  dict(bold=0.2, social=0.9, playful=0.5, lazy=0.6),
                 'clown':    dict(bold=0.5, social=0.5, playful=0.95, lazy=0.3),
                 'golden':   dict(bold=0.6, social=0.95, playful=0.85, lazy=0.35)}
# Breed habits on top of the traits. Default = the plain mind. Golden retriever: lives for people, learns fast
# from praise and holds on to it, a clap is a party not a threat, greets you like you've been gone a year,
# wags while sitting, and calls you with a happy yip-bounce instead of a whine.
BREEDS = {None:     dict(praise=0.35, fade=0.10, startles=True,  greet=False, wag=False, call='whine'),
          'golden': dict(praise=0.60, fade=0.05, startles=False, greet=True,  wag=True,  call='yip')}
GREET_AFTER_S = 60.0                                      # no contact for this long -> the next touch is a reunion


def lerp_trait(v, lo=0.4, hi=1.6):
    return lo + (hi - lo) * v


class DeskMap:
    """Dead-reckoned memory. Pose in mm / deg, origin where she woke up. visits and edges decay (drift)."""
    def __init__(self):
        self.x = self.y = 0.0
        self.h = 90.0                    # heading, deg (90 = the way she faced at boot)
        self.visits, self.edges = {}, {}

    def cell(self, x, y):
        return (int(math.floor(x / CELL)), int(math.floor(y / CELL)))

    def move(self, mm):
        n = max(1, int(abs(mm) / (CELL / 2)))
        for _ in range(n):
            self.x += mm / n * math.cos(math.radians(self.h))
            self.y += mm / n * math.sin(math.radians(self.h))
            c = self.cell(self.x, self.y)
            self.visits[c] = self.visits.get(c, 0.0) + 1.0

    def turn(self, deg):
        self.h = (self.h + deg) % 360

    def edge_ahead(self):
        """Edge reflex fired: the edge is ~70-110 mm ahead of her (where the sensor beam lands)."""
        for d in (70, 90, 110):
            for side in (-30, 0, 30):
                a = math.radians(self.h + side * 0.6)
                c = self.cell(self.x + d * math.cos(a) + side * math.cos(a + math.pi / 2),
                              self.y + d * math.sin(a) + side * math.sin(a + math.pi / 2))
                self.edges[c] = self.edges.get(c, 0.0) + 1.0

    def decay(self, dt):
        k = math.exp(-dt / 240.0)        # ~4 min memory: dead reckoning without a gyro drifts ~1 deg/s
        for d in (self.visits, self.edges):
            for c in list(d):
                d[c] *= k
                if d[c] < 0.05:
                    del d[c]

    def score_heading(self, rel):
        """How good is turning by rel deg and walking ~12 cm: new ground good, remembered edges bad."""
        a = math.radians(self.h + rel)
        s = 0.0
        for d in range(20, 181, 20):
            c = self.cell(self.x + d * math.cos(a), self.y + d * math.sin(a))
            s -= 3.0 * min(self.edges.get(c, 0.0), 2.0) * (1.4 if d < 120 else 0.7)
            s -= 0.15 * min(self.visits.get(c, 0.0), 6.0)
        return s - 0.004 * abs(rel)      # small preference for not turning much

    def best_heading(self, rng, options):
        sc = np.array([self.score_heading(r) for r in options]) + rng.normal(0, 0.25, len(options))
        return options[int(np.argmax(sc))]


class Mind:
    def __init__(self, rng, traits=None, prefs=None, memory=True, breed=None):
        self.rng, self.breed, self.habits = rng, breed, BREEDS[breed]
        self.last_contact = 0.0
        self.traits = dict(traits) if traits else {k: float(rng.uniform(0.1, 0.9)) for k in TRAITS}
        self.prefs = dict(prefs) if prefs else {k: 0.0 for k in PLAY_TRICKS}
        self.energy, self.curiosity, self.social, self.boredom, self.excite = 0.8, 0.4, 0.3, 0.3, 0.2
        self.map = DeskMap() if memory else None
        self.last_choice, self.praise_for, self.praise_until = None, None, -1.0

    # drives
    def tick(self, dt, activity):
        T = self.traits
        active = activity in ('walk', 'trick', 'call')
        if activity == 'rest':
            self.energy = min(1.0, self.energy + 0.012 * dt)
        else:
            self.energy = max(0.0, self.energy - (0.0028 * lerp_trait(T['lazy'], 0.7, 1.3) if active else 0.0004) * dt)
        self.curiosity = min(1.0, self.curiosity + dt / 150.0 * lerp_trait(T['bold'])) if activity != 'walk' else \
            max(0.0, self.curiosity - 0.025 * dt)
        self.social = min(1.0, self.social + dt / 200.0 * lerp_trait(T['social']))
        self.boredom = min(1.0, self.boredom + dt / 90.0 * lerp_trait(T['playful'])) if activity != 'trick' else self.boredom
        self.excite *= math.exp(-dt / 20.0)
        if self.map:
            self.map.decay(dt)

    def stimulus(self, kind, now):
        self.excite = min(1.0, self.excite + {'tap': 0.5, 'clap': 0.3, 'hand': 0.25}[kind])
        self.social *= {'tap': 0.15, 'clap': 0.6, 'hand': 0.5}[kind]
        if kind == 'tap' and self.praise_for and now <= self.praise_until:
            self.prefs[self.praise_for] = min(2.0, self.prefs[self.praise_for] + self.habits['praise'])   # praise: do that again
            self.praise_for = None

    def played(self, trick, now):
        if trick in self.prefs:
            if self.praise_for:                                   # the last one went unnoticed: fade it a little
                self.prefs[self.praise_for] = max(-1.0, self.prefs[self.praise_for] - self.habits['fade'])
            self.praise_for, self.praise_until = trick, now + PRAISE_S
        self.boredom = max(0.0, self.boredom - 0.15)

    def utilities(self):
        T, e = self.traits, self.energy
        return {
            'explore': self.curiosity * (0.4 + 0.6 * e) * lerp_trait(T['bold']),
            'call':    self.social ** 2 * lerp_trait(T['social']) * 1.3,
            'play':    (self.boredom * (0.3 + 0.7 * e) + 0.5 * self.excite) * lerp_trait(T['playful']),
            'rest':    (max(0.0, 0.55 - e) / 0.55) ** 1.5 * lerp_trait(T['lazy']) * 2.0,   # only once she's actually tired
            'look':    0.18,
            'idle':    0.10 + 0.15 * T['lazy'],
        }

    def choose(self):
        u = self.utilities()
        if self.energy < 0.12:
            return 'rest'                                          # too tired to argue with
        names = list(u)
        v = np.array([u[k] for k in names])
        if self.last_choice in ('look', 'idle', 'call'):          # don't repeat filler back-to-back
            v[names.index(self.last_choice)] *= 0.5
        w = np.exp((v - v.max()) / TEMP)
        c = names[int(self.rng.choice(len(names), p=w / w.sum()))]
        self.last_choice = c
        return c

    def pick_trick(self):
        ks = [k for k in PLAY_TRICKS if k != 'zoomies' or (self.energy > 0.5 and self.excite > 0.5)]
        v = np.array([self.prefs[k] for k in ks]) / 0.5          # softmax: favourites most of the time, still tries others
        w = np.exp(v - v.max())
        return ks[int(self.rng.choice(len(ks), p=w / w.sum()))]

    def explore_turn(self):
        opts = [-135, -90, -45, 0, 45, 90, 135, 180]
        if self.map is None:
            return float(self.rng.choice(opts))
        return self.map.best_heading(self.rng, opts)

    def escape_turn(self):
        opts = [-160, -130, -100, 100, 130, 160]
        if self.map is None:
            return float(self.rng.choice(opts))
        return self.map.best_heading(self.rng, opts)

    def walk_seconds(self):
        return 2.5 + 4.0 * self.traits['bold'] + self.rng.uniform(0, 1.5)


# ---------------- the body: the same behaviours on the simulated robot ----------------
def finite_desk(cl, mu, w=0.50, d=0.36, x0=0.0, y0=0.0):
    p.resetSimulation(physicsClientId=cl)
    p.setGravity(0, 0, -9.81, physicsClientId=cl)
    p.setTimeStep(S.DT, physicsClientId=cl)
    p.setPhysicsEngineParameter(numSolverIterations=80, physicsClientId=cl)
    floor = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE, physicsClientId=cl), basePosition=[0, 0, -0.75], physicsClientId=cl)
    top = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[w / 2, d / 2, 0.02], physicsClientId=cl),
                            p.createVisualShape(p.GEOM_BOX, halfExtents=[w / 2, d / 2, 0.02], rgbaColor=[0.55, 0.42, 0.3, 1], physicsClientId=cl),
                            basePosition=[x0, y0, -0.02], physicsClientId=cl)
    for b in (floor, top):
        p.changeDynamics(b, -1, lateralFriction=1.0, physicsClientId=cl)


class SimBody:
    def __init__(self, cl, w, rng, desk=(0.50, 0.36), rec=None):
        self.cl, self.rng, self.rec = cl, rng, rec
        # start somewhere on the desk, facing a random way, at least 9 cm from every edge
        self.desk = desk
        sx = rng.uniform(-desk[0] / 2 + 0.09, desk[0] / 2 - 0.09)
        sy = rng.uniform(-desk[1] / 2 + 0.09, desk[1] / 2 - 0.09)
        finite_desk(cl, w["mu"], desk[0], desk[1], -sx, -sy); self.desk_c = (-sx, -sy)       # robot spawns at the origin; move the desk instead
        self.robot, torque = S.build(cl, mu=w['mu'], mass_scale=w['mass'], torque_scale=w['torque'], yaw=rng.uniform(-math.pi, math.pi))
        vmax = S.SERVO['speed'] * w['speed']
        for _ in range(500):
            for j in range(4):
                p.setJointMotorControl2(self.robot, j, p.POSITION_CONTROL, 0, force=torque, maxVelocity=vmax, physicsClientId=cl)
            p.stepSimulation(physicsClientId=cl)
        self.sv = C.Servos(cl, self.robot, torque, vmax, w['lag_ms'])
        self.base = np.mean([S.tof_reading(cl, self.robot, P.TOF_TILT) for _ in range(10)])
        self.worst_tilt, self.t = 0.0, 0.0
        self.true_cells = set()
        self.flight = []                 # last ~12 s: (t, label, x, y mm from desk centre, yaw deg, tilt)

    def fell(self):
        return p.getBasePositionAndOrientation(self.robot, physicsClientId=self.cl)[0][2] < -0.1

    def guard(self):
        r = S.tof_reading(self.cl, self.robot, P.TOF_TILT) + self.rng.normal(0, 3)
        return r > self.base * Q.EDGE['ratio'] or r > 1000

    def _drive(self, f, secs, guarded=False, fold=False, label=None):
        n = int(secs * C.CTRL_HZ)
        for i in range(n):
            if guarded and self.guard():
                return i / C.CTRL_HZ, True
            q = Q.clamp(f(i / C.CTRL_HZ), None if fold else Q.LIM)
            self.sv.command([math.radians(q[k]) for k in S.LEGS])
            for _ in range(C.STEPS_PER_CTRL):
                p.stepSimulation(physicsClientId=self.cl)
            if i % 5 == 0:
                self.worst_tilt = max(self.worst_tilt, math.degrees(C.tilt_of(self.cl, self.robot)))
                pos, orn = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.cl)
                self.flight.append((round(self.t, 1), label, round((pos[0] - self.desk_c[0]) * 1000), round((pos[1] - self.desk_c[1]) * 1000),
                                    round(math.degrees(p.getEulerFromQuaternion(orn)[2])), round(math.degrees(C.tilt_of(self.cl, self.robot)))))
                self.flight = self.flight[-400:]
                if pos[2] < -0.01:                       # fell off: stop right here so the flight log ends at the fall
                    return i / C.CTRL_HZ, False
                self.true_cells.add((int(pos[0] / 0.04), int(pos[1] / 0.04)))
            if self.rec:
                self.rec(label)
            self.t += 1 / C.CTRL_HZ
        return secs, False

    def walk(self, secs, label='explore'):
        return self._drive(lambda t: Q.walk_deg(t), secs, guarded=True, label=label)

    def back(self, secs, label='edge! backing up'):
        return self._drive(lambda t: Q.walk_deg(t, -1.0), secs, label=label)

    def turn(self, deg, label='turning', guarded=False):
        secs = abs(deg) / SPIN_DEG_S
        side = (1.0, -1.0) if deg < 0 else (-1.0, 1.0)            # (1,-1) spins clockwise (right)
        done, stopped = self._drive(lambda t: Q.walk_deg(t, 1.0, side), secs, guarded=guarded, label=label)
        return (done, stopped) if guarded else done

    def trick(self, name, label=None):
        dur, f, fwd, _ = Q.TRICKS[name]
        return self._drive(f, dur, guarded=fwd, fold=name in Q.WIDE, label=label or name)

    def still(self, secs, label='idle'):
        """Nothing moves: step a little physics (settling), then skip the rest of the time."""
        self._drive(lambda t: {k: 0.0 for k in Q.LEGS}, min(secs, 0.4), label=label)
        rest = max(0.0, secs - 0.4)
        if self.rec:                                  # recordings show 1 s of stillness, then skip (time-lapse)
            for _ in range(int(min(rest, 1.0) * C.CTRL_HZ)):
                self.rec(label + (' (time-lapse)' if rest > 1.0 else ''))
        self.t += rest
        return secs


class Person:
    """Someone at the desk now and then. likes = the trick that makes them tap her."""
    def __init__(self, rng, likes='wiggle'):
        self.rng, self.likes = rng, likes
        self.windows, t = [], rng.uniform(0, 60)
        while t < 3600:
            stay = rng.uniform(90, 240)
            self.windows.append((t, t + stay))
            t += stay + rng.uniform(60, 180)

    def here(self, t):
        return any(a <= t < b for a, b in self.windows)

    def react(self, t, dt, act, trick=None):
        """Stimuli during the last dt seconds. Returns a list of 'tap' / 'clap' / 'hand-close' / 'hand-away'."""
        out = []
        if not self.here(t):
            return out
        r = self.rng.random
        if act == 'trick' and trick:
            if r() < (0.7 if trick == self.likes else 0.1):
                out.append('tap')
        elif act == 'call' and r() < 0.6:
            out.append('tap')
        if r() < 1 - math.exp(-dt * 0.004):
            out.append('tap')
        if r() < 1 - math.exp(-dt * 0.002):
            out.append('clap')
        if r() < 1 - math.exp(-dt * 0.004):
            out.append('hand-close' if r() < 0.5 else 'hand-away')
        return out


def live(mind, body, person, seconds, log=None):
    """One stretch of Cubi's life. Same loop as the firmware's mindMode()."""
    stats = {k: 0.0 for k in ('explore', 'call', 'play', 'rest', 'look', 'idle', 'react')}
    events = {'edge_stops': 0, 'naps': 0, 'calls': 0, 'answered': 0, 'taps': 0, 'praised': 0, 'min_energy': 1.0}
    play_counts = {k: 0 for k in PLAY_TRICKS}
    pending = []

    def say(s):
        if log is not None:
            log.append((round(body.t, 1), s))

    def handle(stims):
        nonlocal pending
        for s in stims:
            if mind.habits['greet'] and body.t - mind.last_contact > GREET_AFTER_S and s != 'clap':
                events['greetings'] = events.get('greetings', 0) + 1      # you're back!
                mind.excite, mind.social = 1.0, 0.0
                t0 = body.t
                body.trick('wiggle', label='you\'re back! (wag)'); body.trick('bounce', label='you\'re back! (bounce)')
                body.trick('wiggle', label='you\'re back! (wag)')
                stats['react'] += body.t - t0
            mind.last_contact = body.t
            if s == 'tap':
                events['taps'] += 1
                was = mind.prefs.get(mind.praise_for, None) if mind.praise_for and body.t <= mind.praise_until else None
                mind.stimulus('tap', body.t)
                if was is not None:
                    events['praised'] += 1
                t0 = body.t; body.trick('wiggle', label='tapped: wiggle'); stats['react'] += body.t - t0
            elif s == 'clap':
                mind.stimulus('clap', body.t)
                calm = mind.excite <= 0.4 and mind.habits['startles']
                t0 = body.t; body.trick('startle' if calm else 'bounce', label='clap!'); stats['react'] += body.t - t0
            else:
                mind.stimulus('hand', body.t)
                t0 = body.t
                stopped = body.trick('paw_pat' if s == 'hand-close' else 'chase', label='hand: ' + ('paw pat' if s == 'hand-close' else 'chase'))[1]
                if stopped:
                    edge_escape()
                stats['react'] += body.t - t0

    def edge_escape():
        events['edge_stops'] += 1
        if mind.map:
            mind.map.edge_ahead()
        body.back(Q.EDGE['backup_s'])
        if mind.map:
            mind.map.move(-BACK_MM_S * Q.EDGE['backup_s'])
        deg = mind.escape_turn()
        body.turn(deg, label='edge: turning away')
        if mind.map:
            mind.map.turn(deg)
        body.base = 0.7 * body.base + 0.3 * np.mean([S.tof_reading(body.cl, body.robot, P.TOF_TILT) for _ in range(5)])

    end = body.t + seconds
    while body.t < end and not body.fell():
        t0 = body.t
        c = mind.choose()
        trick = None
        if c == 'explore':
            deg = mind.explore_turn()
            if abs(deg) > 1:
                body.turn(deg, label='curious: picking a way')
                if mind.map:
                    mind.map.turn(deg)
            left, stopped = mind.walk_seconds(), False
            while left > 0 and not stopped and not body.fell():
                for g in ((-GLANCE_DEG, 2 * GLANCE_DEG, -GLANCE_DEG) if GLANCE_DEG else ()):   # glance: sweep the beam
                    done, stopped = body.turn(g, label='curious: glancing around', guarded=True)
                    if mind.map:
                        mind.map.turn(math.copysign(done * SPIN_DEG_S, g))
                    if stopped:
                        break
                if stopped:
                    break
                secs, stopped = body.walk(min(GLANCE_EVERY, left), label='curious: exploring')
                left -= GLANCE_EVERY
                if mind.map:
                    mind.map.move(WALK_MM_S * secs)
            mind.tick(body.t - t0, 'walk')
            if stopped:
                edge_escape()
            act = 'walk'
        elif c == 'call':
            events['calls'] += 1
            body.turn(float(mind.rng.choice([-60, 60])), label='lonely: looking for you')
            if mind.habits['call'] == 'yip':
                body.trick('bounce', label='where are you? (happy yip)'); body.trick('wiggle', label='where are you? (wag)')
            else:
                body.trick('wiggle', label='lonely: calling (whine)')
            act = 'call'
            mind.tick(body.t - t0, 'call')
        elif c == 'play':
            trick = mind.pick_trick()
            stopped = body.trick(trick, label='playful: ' + trick)[1]
            if stopped:
                edge_escape()
            play_counts[trick] += 1
            mind.played(trick, body.t)
            act = 'trick'
            mind.tick(body.t - t0, 'trick')
        elif c == 'rest':
            events['naps'] += 1
            body.trick('settle', label='sleepy: settling'); body.trick('flop', label='sleepy: lying down')
            nap_t = 0.0
            while mind.energy < 0.85 and body.t < end:
                body.still(5.0, label='napping (servos off)')
                mind.tick(5.0, 'rest'); nap_t += 5
                st = person.react(body.t, 5.0, 'rest')
                if 'tap' in st or ('clap' in st and mind.energy > 0.4):
                    mind.stimulus('tap' if 'tap' in st else 'clap', body.t)
                    events['taps'] += 'tap' in st
                    break
            body.trick('getup', label='waking up'); body.trick('stretch', label='waking up: stretch')
            act = 'rest'
        elif c == 'look':
            body.trick('sniff', label='looking around')
            act = 'idle'
            mind.tick(body.t - t0, 'idle')
        else:
            if mind.habits['wag']:
                body.trick('wiggle', label='content: sitting, wagging')
            body.still(mind.rng.uniform(3, 8), label='content: sitting')
            act = 'idle'
            mind.tick(body.t - t0, 'idle')
        stats[c] += body.t - t0
        events['min_energy'] = min(events['min_energy'], mind.energy)
        st = person.react(body.t, body.t - t0, 'call' if c == 'call' else act, trick)
        if c == 'call' and 'tap' in st:
            events['answered'] += 1
        handle(st)
    tot = sum(stats.values()) or 1
    return {'fell': body.fell(), 'flight': body.flight[-60:] if body.fell() else None, 'share': {k: round(v / tot, 3) for k, v in stats.items()}, 'events': events,
            'play_counts': play_counts, 'area_cells': len(body.true_cells), 'worst_tilt': round(body.worst_tilt, 1)}


# ---------------- tests ----------------
def _day(args):
    seed, persona, memory, minutes, prefs = args[:5]
    rng = np.random.default_rng(seed)
    cl = p.connect(p.DIRECT)
    w = C.sample_world(rng)
    w['slope'] = 0.0
    body = SimBody(cl, w, rng)
    mind = Mind(np.random.default_rng(seed + 1000), PERSONALITIES[persona], prefs, memory, 'golden' if persona == 'golden' else None)
    person = Person(np.random.default_rng(seed + 2000))
    r = live(mind, body, person, minutes * 60)
    p.disconnect(cl)
    r.update({'seed': seed, 'persona': persona, 'memory': memory, 'prefs': {k: round(v, 2) for k, v in mind.prefs.items()},
              'explore_s': r['share']['explore'] * minutes * 60})
    return r


def run(jobs):
    with Pool(2) as pool:
        return pool.map(_day, jobs)


if __name__ == '__main__':
    what = sys.argv[1] if len(sys.argv) > 1 else 'day'
    out = {}
    if what == 'day':
        rs = run([(s, k, True, 6, None) for k in PERSONALITIES for s in (1, 2, 3)])
        for r in rs:
            print(r['persona'], r['seed'], 'fell' if r['fell'] else 'ok', 'tilt', r['worst_tilt'], r['share'], r['events'])
        for k in PERSONALITIES:
            sub = [r for r in rs if r['persona'] == k]
            print('%-9s' % k, {b: round(np.mean([r['share'][b] for r in sub]), 2) for b in sub[0]['share']},
                  'edge stops', sum(r['events']['edge_stops'] for r in sub), 'naps', sum(r['events']['naps'] for r in sub),
                  'min energy %.2f' % min(r['events']['min_energy'] for r in sub))
        out['day'] = rs
    if what == 'memory':
        jobs = [(s, 'explorer', m, 6, None) for s in range(10, 22) for m in (True, False)]
        rs = run(jobs)
        for r in rs:
            if r['fell']:
                print('FELL seed', r['seed'], 'memory', r['memory']); [print('   ', f) for f in r['flight'][-40:]]
        for m in (True, False):
            sub = [r for r in rs if r['memory'] == m]
            ex = sum(r['explore_s'] for r in sub) / 60
            print('memory %-5s falls %d  edge stops %.1f per explore-min  area %.0f cells (4 cm) per explore-min' % (
                m, sum(r['fell'] for r in sub), sum(r['events']['edge_stops'] for r in sub) / ex,
                sum(r['area_cells'] for r in sub) / ex))
        out['memory'] = rs
    if what == 'golden':
        rs = run([(s, k, False, 6, None) for k in ('golden', 'clown', 'cuddler') for s in (51, 52, 53, 54)])
        for k in ('golden', 'clown', 'cuddler'):
            sub = [r for r in rs if r['persona'] == k]
            print('%-8s falls %d' % (k, sum(r['fell'] for r in sub)), {b: round(float(np.mean([r['share'][b] for r in sub])), 2) for b in sub[0]['share']},
                  'greetings', sum(r['events'].get('greetings', 0) for r in sub), 'calls', sum(r['events']['calls'] for r in sub),
                  'answered', sum(r['events']['answered'] for r in sub), 'taps', sum(r['events']['taps'] for r in sub),
                  'wiggle share of play %.0f%%' % (100 * np.mean([r['play_counts']['wiggle'] / max(1, sum(r['play_counts'].values())) for r in sub])))
        out['golden'] = rs
    if what == 'learn':
        d1 = run([(s, 'clown', True, 6, None) for s in (31, 32, 33, 34)])
        d2 = run([(s + 100, 'clown', True, 6, r['prefs']) for s, r in zip((31, 32, 33, 34), d1)])
        for a, b in zip(d1, d2):
            fa = a['play_counts']['wiggle'] / max(1, sum(a['play_counts'].values()))
            fb = b['play_counts']['wiggle'] / max(1, sum(b['play_counts'].values()))
            print('day 1 wiggle %.0f%% of play -> day 2 %.0f%%   prefs after day 2 %s' % (100 * fa, 100 * fb, b['prefs']))
        out['learn'] = [d1, d2]
    json.dump(out, open(f'sim_out/mind_{what}.json', 'w'), indent=1, default=str)
