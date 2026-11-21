"""CUBI - train her in simulation.

Uses the full physics bench in ponder_sim.py (real leg + shoe meshes, real mass layout, GH-S37D servo limits,
ray-cast VL53L0X) and trains three things:

  1. gait   - CMA-ES searches the stick-slip walk (amplitude, step rate, swing fraction, leg phase offsets, leg
              trims) for distance per second, averaged over randomized desks, weights, servo strengths and lag.
  2. edge   - grid-searches the edge/wall reflex thresholds on random desks and approach angles: never fall off,
              stop as close as is safe.
  3. ppo    - (optional) a small neural net that adds corrections on top of the trained gait, trained with PPO
              (stable-baselines3). It must beat the plain gait on the same test worlds to be worth flashing.

Everything ends up in cubi_brain.json, and `export` writes firmware/ponder_selftest/cubi_gait.h, which the
self-test firmware's walk() uses.

  pip install pybullet trimesh manifold3d cma gymnasium        (+ stable-baselines3 torch for ppo)
  python cubi_train.py gait   [--gens 30]
  python cubi_train.py edge
  python cubi_train.py ppo    [--steps 300000]
  python cubi_train.py test   # trained vs hand-tuned gait on 12 fixed test worlds
  python cubi_train.py export
"""
import argparse, json, math, os, sys, time
from multiprocessing import Pool
import numpy as np
import pybullet as p
import ponder_sim as S
import ponder_parts as P

BRAIN = 'cubi_brain.json'
CTRL_HZ = 50                     # firmware runs servos + ToF at 50 Hz
STEPS_PER_CTRL = int(1 / (S.DT * CTRL_HZ))
HIP_LIMIT = math.radians(12)     # front and rear legs touch at 17 deg: never exceed 12

# ---------------- gait parameters ----------------
# name, low, high, hand-tuned value (the gait every check so far has used)
GAIT_SPACE = [
    ('amp_deg',   4.0, 12.0, 12.0),   # swing amplitude (+-)
    ('freq_hz',   0.6, 2.2,  1.5),    # step cycles per second. Capped at 2.2: above that the sim's ideal servos
                                      # (instant acceleration, no deadband) flatter fast vibrating gaits
    ('fast',      0.08, 0.5, 0.2),    # fraction of the cycle spent swinging forward fast
    ('ph_RB',    -0.5, 0.5,  0.0),    # phase of each leg relative to LF (cycles)
    ('ph_RF',    -0.5, 0.5,  0.5),
    ('ph_LB',    -0.5, 0.5,  0.5),
    ('trim_F',   -4.0, 4.0,  0.0),    # centre offset of the front legs (deg, + = forward)
    ('trim_B',   -4.0, 4.0,  0.0),    # centre offset of the back legs
    ('smooth',    0.0, 1.0,  0.0),    # rounds off each leg's reversals (0 = sharp sawtooth, 1 = minimum-jerk)
]
SMOOTH_W = 0.0                        # how much jolting / rocking costs in the score (0 = speed only, as first trained)
SPEED_CAP = None                      # mm/s: if set, speed above this earns nothing, so the search spends it on smoothness
NAMES = [g[0] for g in GAIT_SPACE]
LO = np.array([g[1] for g in GAIT_SPACE]); HI = np.array([g[2] for g in GAIT_SPACE])
HAND = {g[0]: g[3] for g in GAIT_SPACE}


def to_params(u):
    """CMA works in [0,1]^n; map to real ranges."""
    u = np.clip(u, 0, 1)
    return dict(zip(NAMES, LO + u * (HI - LO)))


def to_unit(d):
    return (np.array([d.get(n, HAND[n]) for n in NAMES]) - LO) / (HI - LO)


def _ease(u, k):
    """Blend of a straight ramp (k=0) and a minimum-jerk quintic (k=1): same end points, softer turnarounds."""
    return (1 - k) * u + k * u * u * u * (10 - 15 * u + 6 * u * u)


def saw(ph, fast, k=0.0):
    """Leg sweep, -1..1. Forward for `fast` of the cycle, back for the rest. k rounds off the reversals
    (k=0: sharp sawtooth corners, k=1: velocity and acceleration both reach zero at each end)."""
    ph %= 1.0
    if ph < fast:
        return -1 + 2 * _ease(ph / fast, k)
    return 1 - 2 * _ease((ph - fast) / (1 - fast), k)


def gait_targets(g, t, direction=1.0):
    """Hip targets (rad) for LF, LB, RF, RB at time t. Clamped to +-12 deg."""
    a, f, fast = math.radians(g['amp_deg']), g['freq_hz'], g['fast']
    ph = {'LF': 0.0, 'RB': g['ph_RB'], 'RF': g['ph_RF'], 'LB': g['ph_LB']}
    trim = {'LF': g['trim_F'], 'RF': g['trim_F'], 'LB': g['trim_B'], 'RB': g['trim_B']}
    out = {}
    for k in S.LEGS:
        q = direction * a * saw(f * t + ph[k], fast, g.get('smooth', 0.0)) + math.radians(trim[k])
        out[k] = max(-HIP_LIMIT, min(HIP_LIMIT, q))
    return out


# ---------------- randomized worlds ----------------
def sample_world(rng):
    """One random desk + robot. Ranges cover glass to rubber mat, a 30% heavier build, tired/weak servos,
    servo lag and a slightly tilted desk."""
    return {'mu': rng.uniform(0.2, 0.6), 'mass': rng.uniform(0.95, 1.3), 'torque': rng.uniform(0.5, 1.0),
            'speed': rng.uniform(0.7, 1.1), 'lag_ms': rng.uniform(0, 40), 'slope': rng.uniform(-3, 3),
            'yaw': rng.uniform(-0.2, 0.2)}


def make_robot(cl, w, edge_y=None, wall_y=None):
    S.desk(cl, mu=w['mu'], edge_y=edge_y, wall_y=wall_y, slope_deg=w['slope'])
    robot, torque = S.build(cl, mu=w['mu'], mass_scale=w['mass'], torque_scale=w['torque'], yaw=w.get('yaw', 0.0))
    vmax = S.SERVO['speed'] * w['speed']
    for _ in range(500):
        for j in range(4):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, 0, force=torque, maxVelocity=vmax, physicsClientId=cl)
        p.stepSimulation(physicsClientId=cl)
    return robot, torque, vmax


class Servos:
    """50 Hz command updates with a transport/servo lag, like the real PWM loop."""
    def __init__(self, cl, robot, torque, vmax, lag_ms):
        self.cl, self.robot, self.torque, self.vmax = cl, robot, torque, vmax
        self.queue = [[0.0] * 4] * (1 + int(round(lag_ms / (1000 / CTRL_HZ))))

    def command(self, q4):
        self.queue.append(list(q4)); q = self.queue.pop(0)
        for j in range(4):
            p.setJointMotorControl2(self.robot, j, p.POSITION_CONTROL, q[j], force=self.torque, maxVelocity=self.vmax, physicsClientId=self.cl)


def tilt_of(cl, robot):
    r, pi, _ = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot, physicsClientId=cl)[1])
    return max(abs(r), abs(pi))


def walk_episode(g, w, seconds=3.0, cl=None):
    """Distance forward (mm), worst tilt (deg), mean |joint torque| share (energy proxy)."""
    robot, torque, vmax = make_robot(cl, w)
    sv = Servos(cl, robot, torque, vmax, w['lag_ms'])
    pos0, orn0 = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
    yaw0 = p.getEulerFromQuaternion(orn0)[2]
    worst, effort, n = 0.0, 0.0, int(seconds / S.DT)
    acc, rate, zs, v_prev = [], [], [], None
    for i in range(n):
        if i % STEPS_PER_CTRL == 0:
            tg = gait_targets(g, i * S.DT)
            sv.command([tg[k] for k in S.LEGS])
        p.stepSimulation(physicsClientId=cl)
        if i % 20 == 0:
            worst = max(worst, tilt_of(cl, robot))
            effort += sum(abs(p.getJointState(robot, j, physicsClientId=cl)[3]) for j in range(4)) / (4 * torque)
        if i % 10 == 0 and i * S.DT > 0.5:                 # smoothness, after the first half second
            lv, av = p.getBaseVelocity(robot, physicsClientId=cl)
            orn = p.getBasePositionAndOrientation(robot, physicsClientId=cl)[1]
            Rm = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            w_body = Rm.T @ np.array(av)
            rate.append(math.degrees(math.hypot(w_body[0], w_body[1])))    # roll + pitch rate (yaw is turning)
            if v_prev is not None:
                acc.append(np.linalg.norm((np.array(lv) - v_prev) / (10 * S.DT)))
            v_prev = np.array(lv)
            zs.append(p.getBasePositionAndOrientation(robot, physicsClientId=cl)[0][2])
    pos1, orn1 = p.getBasePositionAndOrientation(robot, physicsClientId=cl)
    # distance along her own heading (start yaw), sideways drift and turning count against her
    h = np.array([-math.sin(yaw0), math.cos(yaw0)])
    d = np.array(pos1[:2]) - np.array(pos0[:2])
    fwd = float(d @ h) * 1000
    side = abs(float(d @ np.array([h[1], -h[0]]))) * 1000
    yaw = abs(math.degrees(p.getEulerFromQuaternion(orn1)[2] - yaw0))
    return {'fwd_mm': fwd, 'side_mm': side, 'yaw_deg': yaw, 'tilt_deg': math.degrees(worst),
            'effort': effort / (n / 20), 'fell': pos1[2] < -0.1,
            'acc_rms': float(np.sqrt(np.mean(np.square(acc)))) if acc else 0.0,          # m/s^2: how much she jolts
            'rock_rms': float(np.sqrt(np.mean(np.square(rate)))) if rate else 0.0,       # deg/s: how much she rocks
            'bob_mm': float(np.std(zs) * 1000) if zs else 0.0}


def gait_score(r, seconds):
    """What CMA maximises: mm/s forward, minus drift, tilt and effort penalties."""
    if r['fell'] or r['tilt_deg'] > 25:
        return -50.0
    v = r['fwd_mm'] / seconds
    if SPEED_CAP is not None:
        v = min(v, SPEED_CAP) * 3.0       # below the target speed is punished hard, above it is free
    return (v - 0.3 * r['side_mm'] / seconds - 0.2 * r['yaw_deg'] - 0.5 * max(0, r['tilt_deg'] - 5) - 4.0 * r['effort']
            - SMOOTH_W * (10.0 * r['acc_rms'] + r['rock_rms']))    # trained gait: 10*1.17 + 13.2 = ~25 'jolt points'


# ---- worker pool (each process owns one pybullet client) ----
_CL = None
def _init():
    global _CL
    _CL = p.connect(p.DIRECT)


def _eval(args):
    g, w, seconds = args
    r = walk_episode(g, w, seconds, cl=_CL)
    return gait_score(r, seconds), r


def evaluate(pool, gaits, worlds, seconds):
    jobs = [(g, w, seconds) for g in gaits for w in worlds]
    res = pool.map(_eval, jobs)
    k = len(worlds)
    return [(np.mean([s for s, _ in res[i * k:(i + 1) * k]]), [r for _, r in res[i * k:(i + 1) * k]]) for i in range(len(gaits))]


def load_brain():
    return json.load(open(BRAIN)) if os.path.exists(BRAIN) else {}


def save_brain(b):
    json.dump(b, open(BRAIN, 'w'), indent=1)


def test_worlds(n=12, seed=1234):
    rng = np.random.default_rng(seed)
    return [sample_world(rng) for _ in range(n)]


# ---------------- 1. gait ----------------
def train_gait(gens=30, pop=10, worlds_per_gen=3, seconds=3.0, procs=None):
    import cma
    rng = np.random.default_rng(7)
    es = cma.CMAEvolutionStrategy(list(to_unit(HAND)), 0.18, {'bounds': [0, 1], 'popsize': pop, 'seed': 7, 'verbose': -9})
    hist, gen_best = [], []
    with Pool(procs or os.cpu_count(), initializer=_init) as pool:
        t0 = time.time()
        for gen in range(gens):
            worlds = [sample_world(rng) for _ in range(worlds_per_gen)]       # fresh worlds every generation
            us = es.ask()
            res = evaluate(pool, [to_params(u) for u in us], worlds, seconds)
            es.tell(us, [-s for s, _ in res])
            gb = int(np.argmax([s for s, _ in res]))
            hist.append(round(float(res[gb][0]), 2)); gen_best.append(to_params(us[gb]))
            print(f'gen {gen + 1:3d}/{gens}  best {res[gb][0]:6.1f}  mean {np.mean([s for s, _ in res]):6.1f}  '
                  f'({time.time() - t0:5.0f} s)', flush=True)
        # the CMA mean is the robust answer; check it and the hand gait on the same held-out worlds
        # candidates: the CMA mean, the best of each of the last 5 generations, and the current brain's gait.
        # A 3-world training score is noisy, so they are all judged on the same 12 held-out test worlds.
        pool_c = {'cma mean': to_params(es.mean)}
        pool_c.update({f'gen {gens - i} best': g for i, g in enumerate(reversed(gen_best[-5:]))})
        prev = load_brain().get('gait')
        if prev:
            pool_c['previous'] = prev
        tw = test_worlds()
        sc = {k: evaluate(pool, [g], tw, seconds)[0] for k, g in pool_c.items()}
        win = max(sc, key=lambda k: sc[k][0])
        print('candidates on the test worlds:', {k: round(float(v[0]), 1) for k, v in sc.items()}, '-> picked', win)
        cand = {'trained': pool_c[win], 'hand-tuned': dict(HAND)}
        scored = {'trained': sc[win], 'hand-tuned': evaluate(pool, [dict(HAND)], tw, seconds)[0]}
    b = load_brain()
    b['gait'] = {k: round(float(v), 4) for k, v in cand['trained'].items()}
    b['gait_history'] = hist
    b['gait_test'] = {k: summarize(v[1], seconds) | {'score': round(float(v[0]), 2)} for k, v in scored.items()}
    if scored['trained'][0] < scored['hand-tuned'][0]:
        print('trained gait did not beat the hand-tuned one on the test worlds: keeping hand-tuned')
        b['gait'] = dict(HAND)
    save_brain(b)
    report(b['gait_test'])
    return b


def summarize(rs, seconds):
    f = [r['fwd_mm'] / seconds for r in rs]
    return {'mm_per_s_mean': round(float(np.mean(f)), 1), 'mm_per_s_worst': round(float(np.min(f)), 1),
            'drift_mm_per_s': round(float(np.mean([r['side_mm'] for r in rs]) / seconds), 1),
            'turn_deg': round(float(np.mean([r['yaw_deg'] for r in rs])), 1),
            'tilt_deg_worst': round(float(np.max([r['tilt_deg'] for r in rs])), 1),
            'effort': round(float(np.mean([r['effort'] for r in rs])), 3), 'falls': int(sum(r['fell'] for r in rs)),
            'acc_rms': round(float(np.mean([r['acc_rms'] for r in rs])), 2), 'rock_rms': round(float(np.mean([r['rock_rms'] for r in rs])), 1),
            'bob_mm': round(float(np.mean([r['bob_mm'] for r in rs])), 2)}


def report(t):
    print(f"\n{'gait':12s} {'mm/s avg':>9s} {'worst':>6s} {'drift':>6s} {'turn':>6s} {'tilt':>6s} {'effort':>7s}")
    for k, v in t.items():
        print(f"{k:12s} {v['mm_per_s_mean']:9.1f} {v['mm_per_s_worst']:6.1f} {v['drift_mm_per_s']:6.1f} {v['turn_deg']:6.1f} "
              f"{v['tilt_deg_worst']:6.1f} {v['effort']:7.3f}")


# ---------------- 2. edge / wall reflex ----------------
def edge_episode(g, w, ratio, backup_s, approach_deg, kind, cl):
    """Walk at a desk edge (or a wall) from 0.12-0.20 m away at an angle. Returns (fell, gap_mm, stopped)."""
    dist = 0.16
    edge_y = dist if kind == 'edge' else None
    wall_y = dist if kind == 'wall' else None
    w = dict(w, yaw=math.radians(approach_deg))
    robot, torque, vmax = make_robot(cl, w, edge_y=edge_y, wall_y=wall_y)
    sv = Servos(cl, robot, torque, vmax, w['lag_ms'])
    base, mode, t_mode, max_front = None, 'walk', 0.0, -1.0
    n = int(12.0 / S.DT)
    for i in range(n):
        t = i * S.DT
        if i % STEPS_PER_CTRL == 0:
            r = S.tof_reading(cl, robot, P.TOF_TILT) + np.random.normal(0, 3)       # ~3 mm sensor noise
            if base is None and t > 0.3:
                base = r
            if mode == 'walk' and base:
                if r > base * ratio or r > 1000:
                    mode, t_mode = 'back', t
                elif r < base * 0.75:
                    mode, t_mode = 'stop', t
            if mode == 'back' and t - t_mode > backup_s:
                mode = 'stop'
            if mode == 'stop':
                break
            d = {'walk': 1.0, 'back': -1.0}[mode]
            tg = gait_targets(g, t, d)
            sv.command([tg[k] for k in S.LEGS])
        p.stepSimulation(physicsClientId=cl)
        if edge_y is not None and i % 10 == 0:
            ys = [p.getLinkState(robot, j, physicsClientId=cl)[0][1] for j in range(4)]
            pos = p.getBasePositionAndOrientation(robot, physicsClientId=cl)[0]
            max_front = max(max_front, max(ys) + P.SHOE_L / 2000, pos[1] + P.BODY_D / 2000 * 0.9)
            if pos[2] < -0.1:
                return True, -1.0, False
    pos = p.getBasePositionAndOrientation(robot, physicsClientId=cl)[0]
    fell = pos[2] < -0.1
    gap = (edge_y - max_front) * 1000 if edge_y is not None else None
    return fell, gap, mode == 'stop'


def _edge_job(args):
    return edge_episode(*args, cl=_CL)


def train_edge(procs=None):
    b = load_brain()
    g = b.get('gait', dict(HAND))
    rng = np.random.default_rng(99)
    worlds = [sample_world(rng) for _ in range(6)]
    angles = [0, 30, -30, 55]
    grid = [(ratio, back) for ratio in (1.15, 1.25, 1.35, 1.5) for back in (1.0, 1.5)]
    rows = []
    with Pool(procs or os.cpu_count(), initializer=_init) as pool:
        for ratio, back in grid:
            jobs = [(g, w, ratio, back, a, 'edge') for w in worlds for a in angles]
            res = pool.map(_edge_job, jobs)
            falls = sum(r[0] for r in res)
            gaps = [r[1] for r in res if not r[0] and r[1] is not None]
            # false alarms: on a plain desk with no edge she must keep walking
            fa = pool.map(_edge_job, [(g, w, ratio, back, 0, 'none') for w in worlds])
            false_stops = sum(r[2] for r in fa)
            rows.append({'ratio': ratio, 'backup_s': back, 'falls': falls, 'false_stops': false_stops,
                         'gap_min_mm': round(min(gaps), 1) if gaps else None, 'gap_mean_mm': round(float(np.mean(gaps)), 1) if gaps else None})
            print(rows[-1], flush=True)
    safe = [r for r in rows if r['falls'] == 0 and r['false_stops'] == 0 and r['gap_min_mm'] and r['gap_min_mm'] > 15]
    # Real desk edges drop past the sensor's range, so every ratio sees them at the same moment and the gaps tie.
    # Tie-break toward the middle of the grid (1.35): low ratios false-alarm on textured or dark desks, high ones
    # miss small drops like the edge of a book.
    pick = (min(safe, key=lambda r: (round(r['gap_mean_mm'] / 5), abs(r['ratio'] - 1.35), -r['backup_s'])) if safe
            else max(rows, key=lambda r: (r['gap_min_mm'] or -1)))
    b['edge'] = {'ratio': pick['ratio'], 'backup_s': pick['backup_s'], 'wall_ratio': 0.75}
    b['edge_grid'] = rows
    save_brain(b)
    print('picked', b['edge'])
    return b


# ---------------- 3. PPO (optional) ----------------
def make_env_class():
    import gymnasium as gym
    from gymnasium import spaces

    class CubiEnv(gym.Env):
        """Residual policy: the trained gait runs underneath, the net adds up to +-4 deg per leg.
        Observation is only what the real Cubi has: gait phase, her last commands, and the ToF reading
        (no IMU, no servo position feedback)."""
        metadata = {'render_modes': []}

        def __init__(self, seed=0, seconds=4.0):
            super().__init__()
            self.cl = p.connect(p.DIRECT)
            self.rng = np.random.default_rng(seed)
            self.g = load_brain().get('gait', dict(HAND))
            self.seconds = seconds
            self.action_space = spaces.Box(-1, 1, (4,), np.float32)
            self.observation_space = spaces.Box(-5, 5, (10,), np.float32)

        def _obs(self):
            ph = 2 * math.pi * self.g['freq_hz'] * self.t
            tof = S.tof_reading(self.cl, self.robot, P.TOF_TILT)
            return np.array([math.sin(ph), math.cos(ph), *[q / HIP_LIMIT for q in self.last],
                             min(tof, 1000) / 200.0, self.prev_tof / 200.0, self.t / self.seconds, 1.0], np.float32)

        def reset(self, seed=None, options=None):
            super().reset(seed=seed)
            if seed is not None:
                self.rng = np.random.default_rng(seed)
            self.w = sample_world(self.rng)
            self.robot, torque, vmax = make_robot(self.cl, self.w)
            self.sv = Servos(self.cl, self.robot, torque, vmax, self.w['lag_ms'])
            self.t, self.last, self.prev_tof = 0.0, [0.0] * 4, 100.0
            pos, orn = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.cl)
            self.y0, self.yaw0 = pos[1], p.getEulerFromQuaternion(orn)[2]
            return self._obs(), {}

        def step(self, action):
            tg = gait_targets(self.g, self.t)
            q = [max(-HIP_LIMIT, min(HIP_LIMIT, tg[k] + math.radians(4) * float(a))) for k, a in zip(S.LEGS, action)]
            self.sv.command(q); self.last = q
            y_before = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.cl)[0][1]
            for _ in range(STEPS_PER_CTRL):
                p.stepSimulation(physicsClientId=self.cl)
            self.t += 1 / CTRL_HZ
            pos, orn = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.cl)
            tilt = tilt_of(self.cl, self.robot)
            yaw = abs(p.getEulerFromQuaternion(orn)[2] - self.yaw0)
            reward = (pos[1] - y_before) * 1000 - 0.02 * yaw - 0.05 * max(0, math.degrees(tilt) - 5) - 0.01 * float(np.sum(np.square(action)))
            obs = self._obs(); self.prev_tof = float(obs[6] * 200)
            done = math.degrees(tilt) > 30 or pos[2] < -0.1
            return obs, float(reward - (20 if done else 0)), bool(done), self.t >= self.seconds, {}

    return CubiEnv


def train_ppo(steps=300000):
    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import SubprocVecEnv
    except ImportError:
        sys.exit('PPO needs: pip install stable-baselines3 torch')
    CubiEnv = make_env_class()
    n = max(1, os.cpu_count())
    env = SubprocVecEnv([lambda i=i: CubiEnv(seed=i) for i in range(n)])
    model = PPO('MlpPolicy', env, policy_kwargs={'net_arch': [32, 32]}, n_steps=512, batch_size=256, learning_rate=3e-4,
                gamma=0.98, verbose=1, seed=0)
    model.learn(total_timesteps=steps)
    model.save('cubi_ppo')
    # judge it against the plain trained gait on the same held-out worlds
    plain, ppo = [], []
    env1 = CubiEnv(seed=1234)
    for k in range(8):
        for use in (False, True):
            env1.rng = np.random.default_rng(1000 + k)
            o, _ = env1.reset(); y0 = env1.y0; done = trunc = False
            while not (done or trunc):
                a = model.predict(o, deterministic=True)[0] if use else np.zeros(4, np.float32)
                o, _, done, trunc, _ = env1.step(a)
            y = p.getBasePositionAndOrientation(env1.robot, physicsClientId=env1.cl)[0][1]
            (ppo if use else plain).append((y - y0) * 1000 / env1.seconds)
    b = load_brain()
    b['ppo_test'] = {'plain_gait_mm_s': round(float(np.mean(plain)), 1), 'ppo_mm_s': round(float(np.mean(ppo)), 1), 'steps': steps}
    b['ppo_better'] = bool(np.mean(ppo) > np.mean(plain) * 1.05)
    save_brain(b)
    print(b['ppo_test'], 'worth flashing' if b['ppo_better'] else 'not worth flashing: keep the plain gait')
    if b['ppo_better']:
        export_policy(model)


def export_policy(model):
    """Write the 2x32 net as C arrays for the ESP32 (tanh MLP, ~1.5k multiply-adds per step)."""
    sd = model.policy.state_dict()
    lines = ['// Cubi PPO residual policy (generated by cubi_train.py). obs[10] -> 4 x +-4 deg corrections.', '#pragma once']
    keys = ['mlp_extractor.policy_net.0', 'mlp_extractor.policy_net.2', 'action_net']
    for i, k in enumerate(keys):
        W, B = sd[k + '.weight'].cpu().numpy(), sd[k + '.bias'].cpu().numpy()
        lines.append(f'static const float W{i}[{W.shape[0]}][{W.shape[1]}] = {{' + ','.join('{' + ','.join(f'{x:.5f}f' for x in r) + '}' for r in W) + '};')
        lines.append(f'static const float B{i}[{B.shape[0]}] = {{' + ','.join(f'{x:.5f}f' for x in B) + '};')
    open('firmware/ponder_selftest/cubi_policy.h', 'w').write('\n'.join(lines) + '\n')
    print('wrote firmware/ponder_selftest/cubi_policy.h')


# ---------------- test + export ----------------
def test(seconds=3.0):
    b = load_brain()
    cand = {'hand-tuned': dict(HAND)}
    if 'gait' in b:
        cand['trained'] = b['gait']
    with Pool(os.cpu_count(), initializer=_init) as pool:
        tw = test_worlds()
        t = {k: summarize(evaluate(pool, [g], tw, seconds)[0][1], seconds) for k, g in cand.items()}
    report(t)


def export():
    b = load_brain()
    g = b.get('gait', dict(HAND)); e = b.get('edge', {'ratio': 1.35, 'backup_s': 1.5, 'wall_ratio': 0.75})
    h = ['// Cubi gait + reflex, trained in simulation by cubi_train.py. Re-run `python cubi_train.py export` after training.',
         '#pragma once',
         f'#define CUBI_AMP_DEG   {g["amp_deg"]:.3f}f   // swing amplitude (+-)',
         f'#define CUBI_FREQ_HZ   {g["freq_hz"]:.3f}f   // step cycles per second',
         f'#define CUBI_FAST      {g["fast"]:.3f}f   // fraction of the cycle swinging forward fast',
         f'#define CUBI_PH_LF     0.000f',
         f'#define CUBI_PH_LB     {g["ph_LB"]:.3f}f   // leg phase offsets (cycles)',
         f'#define CUBI_PH_RF     {g["ph_RF"]:.3f}f',
         f'#define CUBI_PH_RB     {g["ph_RB"]:.3f}f',
         f'#define CUBI_TRIM_F    {g["trim_F"]:.3f}f   // front / back leg centre offsets (deg)',
         f'#define CUBI_TRIM_B    {g["trim_B"]:.3f}f',
         f'#define CUBI_SMOOTH    {g.get("smooth", 0.0):.3f}f   // rounds off leg reversals (0 = sharp sawtooth, 1 = minimum-jerk)',
         f'#define CUBI_EDGE_RATIO {e["ratio"]:.2f}f  // ToF > ratio x flat-desk reading = edge',
         f'#define CUBI_WALL_RATIO {e["wall_ratio"]:.2f}f',
         f'#define CUBI_BACKUP_S   {e["backup_s"]:.2f}f']
    open('firmware/ponder_selftest/cubi_gait.h', 'w').write('\n'.join(h) + '\n')
    print('\n'.join(h))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('what', choices=['gait', 'edge', 'ppo', 'test', 'export'])
    ap.add_argument('--gens', type=int, default=30)
    ap.add_argument('--pop', type=int, default=10)
    ap.add_argument('--steps', type=int, default=300000)
    a = ap.parse_args()
    {'gait': lambda: train_gait(a.gens, a.pop), 'edge': train_edge, 'ppo': lambda: train_ppo(a.steps),
     'test': test, 'export': export}[a.what]()
