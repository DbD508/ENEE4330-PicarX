#!/usr/bin/env python3
"""
ENEE 4330 - Fuzzy Line Tracking for the PiCar-X, version 2 (sharp corners)
Mamdani Type-1 FIS  in Python.

What is new in v2
  1. Corner detection from the sensor PATTERN
       Wide    : tape lies across the bar (total darkness > 1.6)       -> 90 degree corner
       TwoLegs : both outer sensors dark, middle light                 -> V or acute corner
  2. Side memory from the sensor pattern (which outer sensor saw tape last),
     not from the sign of the last error.
  3. Second fuzzy system: SPEED from |E| and |dE| (slow down when the line is far off or moving fast).
  4. Corner and search routine instead of "stop after 1.5 s lost":
       TRACK -> TURN (14 cm forward, full lock toward the remembered side, slow)
             -> REVERSE (10 cm back, opposite lock) -> TURN ...   a multi point turn near the corner
       The line is accepted again only when it is centered under the bar and the car has
       turned at least 40 degrees (dead reckoning), so it does not lock back onto the incoming leg.
  5. Simulator sensor model: each sensor reads its own distance to the tape polyline,
     so tape across the bar and two legs under the bar are modeled.
  6. Test tracks that match the classroom floor: 90, 55 and 25 degree corners.

Inputs (steering FIS, unchanged from v1)
  Line error  E   (-1 to +1)       Left / Center / Right               (+ = line to the right)
  Error rate  dE  (-3 to +3 1/s)   MovingLeft / Steady / MovingRight
Output
  Steering angle  (-30 to +30 deg) HardLeft / Left / Straight / Right / HardRight

Inputs (speed FIS, new)
  |E|   (0 to 1)       Small / Large
  |dE|  (0 to 3 1/s)   Slow / Fast
Output
  Speed (0 to 100 % of the --power setting)   Slow / Medium / Full

Usage
  python3 fuzzy_line_fis_v2.py                        # laptop: worked examples, corner tests, figures
  sudo python3 fuzzy_line_fis_v2.py --calibrate       # PiCar-X: record white floor and black line readings
  sudo python3 fuzzy_line_fis_v2.py --robot           # PiCar-X: v2 fuzzy tracking with corner handling
  sudo python3 fuzzy_line_fis_v2.py --robot --v1      # PiCar-X: v1 behaviour for comparison
  sudo python3 fuzzy_line_fis_v2.py --robot --crisp   # PiCar-X: stock style crisp tracking
  sudo python3 fuzzy_line_fis_v2.py --robot --power 35
  python3 fuzzy_line_fis_v2.py --plot-log line_log.csv
"""
import argparse
import csv
import json
import os
import time

import numpy as np


# ================================================================ Step 1
def trapmf(x, p):
    """Trapezoidal membership function, p = [a, b, c, d]."""
    a, b, c, d = p
    x = np.asarray(x, float)
    y = np.zeros_like(x)
    if b > a:                                   # rising edge
        m = (x > a) & (x < b)
        y[m] = (x[m] - a) / (b - a)
    y[(x >= b) & (x <= c)] = 1.0                # plateau
    if d > c:                                   # falling edge
        m = (x > c) & (x < d)
        y[m] = (d - x[m]) / (d - c)
    return y


def fuzzify(x, sets):
    return {k: float(trapmf([x], v)[0]) for k, v in sets.items()}


def mamdani(inputs, input_sets, rules, out_sets, y):
    """Generic two input Mamdani FIS: AND = min, implication = min, aggregation = max, centroid.
    rules = [(name, set_in_1, set_in_2, set_out), ...]"""
    mus = [fuzzify(v, s) for v, s in zip(inputs, input_sets)]                    # Step 4
    w = {n: min(mus[0][a], mus[1][b]) for n, a, b, _ in rules}                    # Step 5
    clipped = [np.minimum(w[n], trapmf(y, out_sets[o])) for n, _, _, o in rules]  # Step 6
    agg = np.max(np.vstack(clipped), axis=0)                                      # Step 7
    out = float((y * agg).sum() / agg.sum()) if agg.sum() > 0 else None           # Step 8
    return out, {"mu": mus, "w": w, "clipped": clipped, "agg": agg}


# ================================================================ Step 2: steering FIS (as v1)
E_RANGE = (-1.0, 1.0)
DE_RANGE = (-3.0, 3.0)
A_RANGE = (-30.0, 30.0)

ERROR = {
    "Left":   [-1, -1, -0.8, 0],
    "Center": [-0.7, 0, 0, 0.7],
    "Right":  [0, 0.8, 1, 1],
}
RATE = {                         # 1/s
    "MovingLeft":  [-3, -3, -2, -0.5],
    "Steady":      [-2, 0, 0, 2],
    "MovingRight": [0.5, 2, 3, 3],
}
ANGLE = {                        # degrees, + = right
    "HardLeft":  [-30, -30, -24, -12],
    "Left":      [-20, -10, -10, 0],
    "Straight":  [-6, 0, 0, 6],
    "Right":     [0, 10, 10, 20],
    "HardRight": [12, 24, 30, 30],
}
y_angle = np.linspace(A_RANGE[0], A_RANGE[1], 121)   # 0.5 deg steps

# ================================================================ Step 3: steering rules (as v1)
TABLE = {
    "Left":   {"MovingLeft": "HardLeft", "Steady": "HardLeft", "MovingRight": "Left"},
    "Center": {"MovingLeft": "Left",     "Steady": "Straight", "MovingRight": "Right"},
    "Right":  {"MovingLeft": "Right",    "Steady": "HardRight", "MovingRight": "HardRight"},
}
RULES = []
for e_set in ["Left", "Center", "Right"]:
    for r_set in ["MovingLeft", "Steady", "MovingRight"]:
        RULES.append(("R%d" % (len(RULES) + 1), e_set, r_set, TABLE[e_set][r_set]))


def evaluate(error, rate, details=False):
    """Steering angle in degrees (+ = right)."""
    error = float(np.clip(error, *E_RANGE))
    rate = float(np.clip(rate, *DE_RANGE))
    a, info = mamdani([error, rate], [ERROR, RATE], RULES, ANGLE, y_angle)
    a = 0.0 if a is None else a
    if details:
        return a, {"mu_e": info["mu"][0], "mu_r": info["mu"][1], "w": info["w"],
                   "clipped": info["clipped"], "agg": info["agg"]}
    return a


# ================================================================ NEW: speed FIS
ABS_E = {"Small": [0, 0, 0.2, 0.6], "Large": [0.2, 0.6, 1, 1]}
ABS_DE = {"Slow": [0, 0, 0.5, 2.0], "Fast": [0.5, 2.0, 3, 3]}          # 1/s
SPEED = {"Slow": [30, 30, 40, 60], "Medium": [40, 60, 60, 80], "Full": [60, 80, 100, 100]}  # % of --power
y_speed = np.linspace(0, 100, 101)
SPEED_RULES = [
    ("S1", "Small", "Slow", "Full"),
    ("S2", "Small", "Fast", "Medium"),
    ("S3", "Large", "Slow", "Medium"),
    ("S4", "Large", "Fast", "Slow"),
]


def evaluate_speed(error, rate, details=False):
    """Speed as a percent of the base power setting."""
    ae = float(np.clip(abs(error), 0, 1))
    ar = float(np.clip(abs(rate), 0, 3))
    s, info = mamdani([ae, ar], [ABS_E, ABS_DE], SPEED_RULES, SPEED, y_speed)
    s = 100.0 if s is None else s
    return (s, info) if details else s


def print_steps(e, r):
    a, info = evaluate(e, r, details=True)
    print("\n=== Steering worked example: Error = %g, Error rate = %g 1/s ===" % (e, r))
    for k, v in info["mu_e"].items():
        print("   mu_%-11s(%+.2f)     = %.3f" % (k, e, v))
    for k, v in info["mu_r"].items():
        print("   mu_%-11s(%+.2f 1/s) = %.3f" % (k, r, v))
    for rr in RULES:
        if info["w"][rr[0]] > 0:
            print("   %s  w = %.3f  (IF Error is %s AND Rate is %s THEN Steer %s)"
                  % (rr[0], info["w"][rr[0]], rr[1], rr[2], rr[3]))
    print("   Steering angle = %+.1f deg" % a)
    s, si = evaluate_speed(e, r, details=True)
    print("=== Speed worked example: |E| = %.2f, |dE| = %.2f 1/s ===" % (abs(e), abs(r)))
    for k, v in si["mu"][0].items():
        print("   mu_%-6s(|E|)  = %.3f" % (k, v))
    for k, v in si["mu"][1].items():
        print("   mu_%-6s(|dE|) = %.3f" % (k, v))
    for n, a1, b1, o in SPEED_RULES:
        if si["w"][n] > 0:
            print("   %s  w = %.3f  (IF |E| is %s AND |dE| is %s THEN Speed %s)" % (n, si["w"][n], a1, b1, o))
    print("   Speed = %.1f %% of power" % s)
    return a, s


# ================================================================ Sensors
SENSOR_X = np.array([-2.0, 0.0, 2.0])   # cm, left, middle, right sensor across the bar
TAPE_HALF = 0.95                        # cm, half width of 19 mm electrical tape
FOOT = 0.6                              # cm, sensor footprint radius
MIN_TOTAL = 0.15                        # total darkness below this = line not seen
WIDE_TOTAL = 1.6                        # one tape gives at most 1.0, so above 1.6 = tape across the bar


def line_error(dark, last_error=0.0, min_total=MIN_TOTAL):
    """Continuous line position from -1 (left sensor) to +1 (right sensor)."""
    total = dark.sum()
    if total < min_total:
        return float(np.sign(last_error)) if last_error != 0 else 0.0, False
    return float(np.clip((dark * SENSOR_X).sum() / total / SENSOR_X[2], -1, 1)), True


def crisp_angle(dark, last_angle=0.0):
    """Stock style controller: 0/1 status, fixed 20 degree turns, hold angle if lost."""
    s = dark > 0.5
    if s[1]:
        return 0.0
    if s[0]:
        return -20.0
    if s[2]:
        return 20.0
    return last_angle


# ================================================================ NEW: controller with corner states
CTRL_DT_V1 = 0.05      # s, v1 control loop
CTRL_DT_V2 = 0.02      # s, v2 control loop (sees a 19 mm tape crossing at least once)
ALPHA_R = 0.15         # EMA on the error rate
LOST_STOP_V1 = 1.5     # s, v1 stops after this long without the line

TURN_POWER = 15.0      # % power while turning at a corner or searching
FWD_ARC = 14.0         # cm forward at full lock per turn segment
REV_ARC = 10.0         # cm backward at the opposite lock per reverse segment
MIN_ROT = 40.0         # deg the car must turn (dead reckoning) after a CORNER PATTERN before it accepts the line
MAX_TRIES = 6          # forward/reverse cycles (about 450 deg of rotation) before giving up
WIDE_COOLDOWN = 0.5    # s, ignore corner patterns right after a turn (crossing the tape steeply looks wide)
SETTLE = 1.0           # s, stay at turning speed after a corner while the car lines up with the new leg


class Follower:
    """One control step per call. mode = 'v2', 'v1' or 'crisp'."""

    def __init__(self, mode="v2", power=25.0, k_speed=0.9):
        self.mode, self.base, self.k_speed = mode, float(power), k_speed
        self.dt = CTRL_DT_V2 if mode == "v2" else CTRL_DT_V1
        self.e_prev, self.rate, self.angle = 0.0, 0.0, 0.0
        self.side = 0                 # -1 = line went left, +1 = right, 0 = unknown
        self.state, self.t_state, self.lost, self.tries = "TRACK", 0.0, 0.0, 0
        self.t_last_turn = -10.0
        self.rot, self.min_rot = 0.0, MIN_ROT
        self.events = []

    def _go(self, state, t):
        if state != self.state:
            self.events.append((round(t, 2), self.state, state, self.side))
        self.state, self.t_state = state, t

    def step(self, dark, t, dt):
        """Returns (angle_deg, power_percent, seen). Negative power = reverse. Power 0 + state STOP = done."""
        dark = np.asarray(dark, float)
        if self.mode in ("v1", "crisp"):
            return self._step_v1(dark, dt)
        return self._step_v2(dark, t, dt)

    # ---------- v1 behaviour, kept for comparison
    def _step_v1(self, dark, dt):
        e, seen = line_error(dark, self.e_prev)
        self.rate += ALPHA_R * ((e - self.e_prev) / dt - self.rate)
        self.angle = crisp_angle(dark, self.angle) if self.mode == "crisp" else evaluate(e, self.rate)
        self.e_prev = e
        self.lost = 0.0 if seen else self.lost + dt
        if self.lost > LOST_STOP_V1:
            self.state = "STOP"
            return self.angle, 0.0, seen
        return self.angle, self.base, seen

    # ---------- v2 behaviour
    def _step_v2(self, d, t, dt):
        total = d.sum()
        seen = total >= MIN_TOTAL
        wide = total > WIDE_TOTAL
        two_legs = d[0] > 0.5 and d[2] > 0.5 and d[1] < 0.3
        lock = A_RANGE[1]

        if self.state == "TRACK":
            # side memory from the pattern: which outer sensor sees tape on its own
            if d[0] > 0.5 and d[2] < 0.3:
                self.side = -1
            elif d[2] > 0.5 and d[0] < 0.3:
                self.side = +1
            if (wide or two_legs) and t - self.t_last_turn > WIDE_COOLDOWN:
                if two_legs:            # the leg we came in on is on the side of the last error
                    self.side = -int(np.sign(self.e_prev)) or self.side or -1
                else:                   # the new leg is on the darker outer side
                    self.side = -1 if d[0] >= d[2] else +1
                self.tries, self.rot, self.min_rot = 0, 0.0, MIN_ROT
                self._go("TURN", t)
            elif not seen:
                if self.side == 0:
                    self.side = int(np.sign(self.e_prev)) or -1
                self.tries, self.rot, self.min_rot = 0, 0.0, 0.0     # plain loss: take the line back at once
                self._go("TURN", t)
            else:
                e, _ = line_error(d, self.e_prev)
                self.rate += ALPHA_R * ((e - self.e_prev) / dt - self.rate)
                self.e_prev = e
                self.angle = evaluate(e, self.rate)
                pw = self.base * evaluate_speed(e, self.rate) / 100.0
                if t - self.t_last_turn < SETTLE:
                    pw = min(pw, TURN_POWER)
                return self.angle, pw, seen

        if self.state in ("TURN", "REVERSE"):
            # dead reckoning of how far the car has rotated since the corner started (deg)
            v = self.k_speed * (TURN_POWER if self.state == "TURN" else -TURN_POWER)
            self.rot += abs(np.degrees(v / WHEELBASE * np.tan(np.radians(self.angle)) * dt))
            centered = seen and d[1] > 0.5 and not wide and not two_legs
            if centered and self.rot >= self.min_rot:
                e, _ = line_error(d, 0.0)
                self.e_prev, self.rate, self.t_last_turn = e, 0.0, t
                self._go("TRACK", t)
                self.angle = evaluate(e, 0.0)
                return self.angle, min(TURN_POWER, self.base * evaluate_speed(e, 0.0) / 100.0), seen
            if self.state == "TURN" and t - self.t_state > FWD_ARC / (self.k_speed * TURN_POWER):
                self.tries += 1
                self._go("STOP" if self.tries > MAX_TRIES else "REVERSE", t)
            elif self.state == "REVERSE" and t - self.t_state > REV_ARC / (self.k_speed * TURN_POWER):
                self._go("TURN", t)
            if self.state == "TURN":
                self.angle = lock * self.side
                return self.angle, TURN_POWER, seen
            if self.state == "REVERSE":             # opposite lock while backing up: a multi point turn
                self.angle = -lock * self.side
                return self.angle, -TURN_POWER, seen

        self.angle = 0.0
        return 0.0, 0.0, seen                        # STOP


# ================================================================ NEW: track geometry and sensor model
def arc_track(segs, step=0.5):
    """Smooth track from straights and arcs: ("S", length) or ("L"/"R", radius, degrees)."""
    x, y, th = 0.0, 0.0, 0.0
    pts = [(x, y)]
    for s in segs:
        if s[0] == "S":
            for _ in range(int(s[1] / step)):
                x += step * np.cos(th); y += step * np.sin(th); pts.append((x, y))
        else:
            r, deg = s[1], s[2]
            sign = 1 if s[0] == "L" else -1
            n = int(np.radians(deg) * r / step)
            for _ in range(n):
                th += sign * np.radians(deg) / n
                x += step * np.cos(th); y += step * np.sin(th); pts.append((x, y))
    return np.array(pts)


def corner_track(interior_deg, turn="L", lead=60.0, tail=70.0):
    """Straight lead in, one sharp corner with the given interior angle, straight tail.
    Interior 90 = right angle, 55 = acute corner, 25 = hairpin V (photos IMG_4816, 4817, 4818)."""
    heading = np.radians(180.0 - interior_deg) * (1 if turn == "L" else -1)
    v = np.array([lead, 0.0])
    end = v + tail * np.array([np.cos(heading), np.sin(heading)])
    return np.array([[0.0, 0.0], v, end])


TRACKS = {
    "S track (v1)":        arc_track([("S", 50), ("L", 30, 90), ("S", 30), ("R", 25, 180),
                                      ("S", 30), ("L", 30, 90), ("S", 60)]),
    "90 deg corner":       corner_track(90),
    "55 deg acute corner": corner_track(55),
    "25 deg hairpin V":    corner_track(25),
}


def poly_dist(P, poly):
    """Distance from each point in P (k x 2) to the polyline."""
    A, B = poly[:-1], poly[1:]
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    AP = P[:, None, :] - A[None]
    t = np.clip((AP * AB).sum(2) / L2, 0, 1)
    proj = A[None] + t[..., None] * AB[None]
    return np.sqrt(((P[:, None, :] - proj) ** 2).sum(2)).min(1)


def sensor_points(x, y, th):
    bx, by = x + SENSOR_AHEAD * np.cos(th), y + SENSOR_AHEAD * np.sin(th)
    rx, ry = np.sin(th), -np.cos(th)                  # unit vector to the car's right
    return np.column_stack([bx + SENSOR_X * rx, by + SENSOR_X * ry]), np.array([bx, by])


def darkness(poly, x, y, th):
    """Each sensor sees darkness from its own distance to the nearest tape."""
    pts, bar = sensor_points(x, y, th)
    d = poly_dist(pts, poly)
    return np.clip((TAPE_HALF + FOOT - d) / (2 * FOOT), 0, 1), bar


# ================================================================ Car model and simulation
WHEELBASE = 9.5        # cm
SENSOR_AHEAD = 12.0    # cm from rear axle to the sensor bar
K_SPEED = 0.9          # cm/s per % power
SERVO_TAU = 0.08       # s, steering servo lag
SUB = 10               # integration substeps per control step


def simulate(track, mode="v2", power=25.0, t_max=40.0, noise=0.0, seed=0):
    poly = TRACKS[track] if isinstance(track, str) else track
    end = poly[-1]
    ctl = Follower(mode, power, K_SPEED)
    x, y, th, delta, t = -SENSOR_AHEAD, 0.0, 0.0, 0.0, 0.0
    log, lost = [], 0.0
    result = "timeout"
    rng = np.random.default_rng(seed)
    while t < t_max:
        dark, bar = darkness(poly, x, y, th)
        if noise > 0:
            dark = np.clip(dark + rng.uniform(-noise, noise, 3), 0, 1)
        angle, pw, seen = ctl.step(dark, t, ctl.dt)
        off = float(poly_dist(bar[None], poly)[0])
        lost += 0.0 if seen else ctl.dt
        log.append({"t": round(t, 3), "x": x, "y": y, "bx": bar[0], "by": bar[1], "offset": off,
                    "angle": angle, "power": pw, "state": ctl.state, "seen": seen})
        if np.hypot(*(bar - end)) < 4.0:
            result = "finished"; break
        if ctl.state == "STOP":
            result = "stopped (line lost)" if mode != "v2" else "stopped (search failed)"; break
        if off > 40:
            result = "left the track"; break
        h = ctl.dt / SUB
        v = K_SPEED * pw
        for _ in range(SUB):
            delta += (np.radians(angle) - delta) * h / SERVO_TAU
            x += v * np.cos(th) * h
            y += v * np.sin(th) * h
            th -= v / WHEELBASE * np.tan(delta) * h      # + delta turns right
        t += ctl.dt
    return log, {"result": result, "lost": lost, "events": ctl.events}


def metrics(log, info):
    off = np.array([r["offset"] for r in log])
    return {"result": info["result"], "time": log[-1]["t"],
            "rms_cm": float(np.sqrt(np.mean(off ** 2))), "max_cm": float(off.max()),
            "lost_s": info["lost"], "reverses": sum(1 for e in info["events"] if e[2] == "REVERSE")}


# ================================================================ Figures
def make_figures(out="figs_v2"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(out, exist_ok=True)
    plt.rcParams.update({"font.size": 12, "axes.grid": True, "grid.alpha": 0.3})
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]

    # Speed FIS membership functions
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    for ax, (title, sets, rng) in zip(axes, [("|E|", ABS_E, (0, 1)), ("|dE| (1/s)", ABS_DE, (0, 3)),
                                             ("Speed (% of power)", SPEED, (0, 100))]):
        xx = np.linspace(rng[0], rng[1], 601)
        for (k, v), c in zip(sets.items(), colors):
            ax.plot(xx, trapmf(xx, v), lw=2.5, color=c, label="%s %s" % (k, v))
        ax.set_title(title, fontweight="bold"); ax.set_ylim(0, 1.35); ax.set_xlim(*rng)
        ax.legend(fontsize=9, loc="upper center", ncol=2)
    axes[0].set_ylabel(r"$\mu$")
    fig.tight_layout(); fig.savefig(f"{out}/speed_mf.png", dpi=160); plt.close(fig)

    # Speed surface
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    E, R = np.meshgrid(np.linspace(0, 1, 41), np.linspace(0, 3, 41))
    Z = np.vectorize(evaluate_speed)(E, R)
    fig = plt.figure(figsize=(7.5, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(E, R, Z, cmap="viridis", edgecolor="none")
    ax.set_xlabel("|E|"); ax.set_ylabel("|dE| (1/s)"); ax.set_zlabel("Speed (%)")
    ax.view_init(elev=28, azim=-130)
    fig.savefig(f"{out}/speed_surface.png", dpi=160, bbox_inches="tight", pad_inches=0.25); plt.close(fig)

    # Sensor patterns at the three corners (bar just reaching the corner)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    for ax, name in zip(axes, ["90 deg corner", "55 deg acute corner", "25 deg hairpin V"]):
        poly = TRACKS[name]
        vx = poly[1, 0]
        ax.plot(poly[:, 0], poly[:, 1], color="k", lw=11, alpha=0.85, solid_capstyle="butt")
        for xb, c in [(vx - 6, "#2ca02c"), (vx - 1.0, "#d62728")]:
            dark, _ = darkness(poly, xb - SENSOR_AHEAD, 0.0, 0.0)
            ax.plot([xb, xb], [-2.6, 2.6], color=c, lw=2)
            for sx, dk in zip(SENSOR_X, dark):
                ax.scatter([xb], [-sx], s=120, color=c, edgecolor="k", zorder=5)
                ax.annotate("%.1f" % dk, (xb, -sx), textcoords="offset points", xytext=(6, -4),
                            fontsize=10, color=c, fontweight="bold")
        ax.set_xlim(vx - 14, vx + 6); ax.set_ylim(-6, 14); ax.set_aspect("equal")
        ax.set_title(name, fontweight="bold"); ax.set_xlabel("x (cm), car drives to the right")
    fig.tight_layout(); fig.savefig(f"{out}/corner_patterns.png", dpi=160); plt.close(fig)

    # Corner runs: v1 vs v2
    names = ["90 deg corner", "55 deg acute corner", "25 deg hairpin V"]
    fig, axes = plt.subplots(1, 3, figsize=(16, 6))
    for ax, name in zip(axes, names):
        poly = TRACKS[name]
        ax.plot(poly[:, 0], poly[:, 1], color="k", lw=8, alpha=0.2, label="Tape")
        for mode, c in [("v1", "#d62728"), ("v2", "#1f77b4")]:
            log, info = simulate(name, mode, 25)
            ax.plot([r["bx"] for r in log], [r["by"] for r in log], color=c, lw=2,
                    label="%s: %s" % ("v1 fuzzy" if mode == "v1" else "v2 fuzzy + corners", info["result"]))
            ax.plot(log[-1]["bx"], log[-1]["by"], "o", color=c, ms=8)
        ax.set_aspect("equal"); ax.set_title(name, fontweight="bold")
        ax.set_xlabel("x (cm)"); ax.legend(loc="lower left", fontsize=9.5)
        ax.set_xlim(-5, 95); ax.set_ylim(-35, 75)
    axes[0].set_ylabel("y (cm), path of the sensor bar")
    fig.tight_layout(); fig.savefig(f"{out}/corner_runs.png", dpi=160, bbox_inches="tight"); plt.close(fig)

    # Time trace for the hairpin with states
    log, info = simulate("25 deg hairpin V", "v2", 25)
    t = [r["t"] for r in log]
    fig, axes = plt.subplots(3, 1, figsize=(12, 6.6), sharex=True)
    axes[0].plot(t, [r["offset"] for r in log], lw=2); axes[0].set_ylabel("Offset (cm)")
    axes[1].plot(t, [r["angle"] for r in log], lw=2, color="#d62728"); axes[1].set_ylabel("Steer (deg)")
    axes[2].plot(t, [r["power"] for r in log], lw=2, color="#2ca02c"); axes[2].set_ylabel("Power (%)")
    axes[2].set_xlabel("Time (s)")
    shade = {"TURN": "#ffe8cc", "REVERSE": "#f6d2d2"}
    for ax in axes:
        start = None
        for i, r in enumerate(log + [{"state": "END", "t": t[-1]}]):
            if start is not None and r["state"] != log[start]["state"]:
                st = log[start]["state"]
                if st in shade:
                    ax.axvspan(log[start]["t"], r["t"], color=shade[st], alpha=0.9, lw=0)
                start = None
            if start is None and r["state"] in shade:
                start = i
    axes[0].set_title("25 deg hairpin, v2 at 25% power (orange = TURN, red = REVERSE)", fontweight="bold")
    fig.tight_layout(); fig.savefig(f"{out}/hairpin_trace.png", dpi=160); plt.close(fig)

    print("Figures saved in ./%s/" % out)


# ================================================================ Robot
CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grayscale_cal.json")


def calibrate():
    from picarx import Picarx
    px = Picarx()

    def avg(n=40):
        vals = np.array([px.get_grayscale_data() for _ in range(n)], float)
        return vals.mean(axis=0).tolist()

    input("Place ALL THREE sensors over the white floor, then press Enter ")
    white = avg()
    input("Place ALL THREE sensors over the black tape (use a corner where the tape overlaps), then press Enter ")
    black = avg()
    json.dump({"white": white, "black": black}, open(CAL_FILE, "w"), indent=2)
    print("white =", [round(v) for v in white], " black =", [round(v) for v in black])
    print("saved", CAL_FILE)
    check_calibration(white, black)


def check_calibration(white, black):
    ok = True
    for name, w, b in zip(["left", "middle", "right"], white, black):
        if w - b < 100:
            print("WARNING: %s sensor: white %.0f, black %.0f. Too little contrast. "
                  "Check the tape and recalibrate." % (name, w, b))
            ok = False
    return ok


def load_calibration():
    if not os.path.exists(CAL_FILE):
        print("No grayscale calibration found (%s)." % CAL_FILE)
        ans = input("Calibrate now? [Y/n] ").strip().lower()
        if ans not in ("", "y", "yes"):
            raise SystemExit("Run first:  sudo python3 fuzzy_line_fis_v2.py --calibrate")
        calibrate()
    cal = json.load(open(CAL_FILE))
    white, black = np.array(cal["white"], float), np.array(cal["black"], float)
    if not check_calibration(white, black):
        raise SystemExit("Calibration is not usable. Run:  sudo python3 fuzzy_line_fis_v2.py --calibrate")
    return white, black


def run_robot(mode="v2", power=25.0, log_file="line_log.csv"):
    from picarx import Picarx
    px = Picarx()
    white, black = load_calibration()
    ctl = Follower(mode, power)

    def read_dark():
        raw = np.array(px.get_grayscale_data(), float)
        return np.clip((white - raw) / (white - black), 0, 1)

    t0 = last = time.time()
    with open(log_file, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["t", "dL", "dM", "dR", "angle", "power", "state", "controller"])
        try:
            while True:
                dark = read_dark()
                now = time.time()
                dt = max(now - last, 1e-3)
                last = now
                t = now - t0
                angle, pw, _ = ctl.step(dark, t, dt)
                if ctl.state == "STOP":
                    print("line not found, stopping")
                    break
                px.set_dir_servo_angle(angle)
                if pw >= 0:
                    px.forward(pw)
                else:
                    px.backward(-pw)
                wr.writerow([round(t, 3)] + [round(v, 3) for v in dark] +
                            [round(angle, 1), round(pw, 1), ctl.state, mode])
                print("t=%5.2f  dark=%s  %-7s angle=%+5.1f  power=%5.1f"
                      % (t, np.round(dark, 2), ctl.state, angle, pw))
                time.sleep(ctl.dt)
        except KeyboardInterrupt:
            pass
        finally:
            px.stop()
            px.set_dir_servo_angle(0)
            print("log saved to %s" % log_file)


def plot_log(path, out="figs_v2"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = list(csv.DictReader(open(path)))
    t = [float(r["t"]) for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    for k, lbl in zip(["dL", "dM", "dR"], ["left", "middle", "right"]):
        axes[0].plot(t, [float(r[k]) for r in rows], lw=1.8, label=lbl)
    axes[0].set_ylabel("Darkness"); axes[0].legend(loc="upper right")
    axes[1].plot(t, [float(r["angle"]) for r in rows], lw=2, color="#d62728"); axes[1].set_ylabel("Angle (deg)")
    axes[2].plot(t, [float(r["power"]) for r in rows], lw=2, color="#2ca02c"); axes[2].set_ylabel("Power (%)")
    axes[2].set_xlabel("Time (s)")
    os.makedirs(out, exist_ok=True)
    fig.tight_layout(); fig.savefig(f"{out}/robot_run.png", dpi=160)
    print("saved %s/robot_run.png" % out)


# ================================================================ main
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--robot", action="store_true")
    ap.add_argument("--v1", action="store_true", help="v1 fuzzy behaviour (stop after 1.5 s lost)")
    ap.add_argument("--crisp", action="store_true")
    ap.add_argument("--power", type=float, default=25.0)
    ap.add_argument("--plot-log")
    ap.add_argument("--no-figs", action="store_true")
    args = ap.parse_args()
    mode = "crisp" if args.crisp else "v1" if args.v1 else "v2"

    if args.calibrate:
        calibrate()
    elif args.robot:
        run_robot(mode, args.power)
    elif args.plot_log:
        plot_log(args.plot_log)
    else:
        print_steps(0.2, -0.8)
        print_steps(0.7, 2.0)
        print("\nCorner tests (19 mm tape, simulated PiCar-X)")
        print("  %-20s %5s  %-6s %-24s %6s %7s %6s %4s"
              % ("track", "power", "ctrl", "result", "time_s", "rms_cm", "lost_s", "rev"))
        for name in TRACKS:
            for pw in (25, 40):
                for mode_ in ("crisp", "v1", "v2"):
                    log, info = simulate(name, mode_, pw)
                    m = metrics(log, info)
                    print("  %-20s %5d  %-6s %-24s %6.1f %7.2f %6.2f %4d"
                          % (name, pw, mode_, m["result"], m["time"], m["rms_cm"], m["lost_s"], m["reverses"]))
        if not args.no_figs:
            make_figures()
