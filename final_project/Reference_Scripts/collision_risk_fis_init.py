#!/usr/bin/env python3
"""
ENEE 4330 - Fuzzy Collision Risk for the PiCar-X
Mamdani Type-1 FIS

Inputs
  Distance  (0-100 cm)  ultrasonic reading      Near / Medium / Far
  Power     (0-100 %)   commanded motor power   Slow / Fast
Output
  Risk      (0-100 %)   collision risk          Low / Normal / High

Same operators as the Project Risk example (MATLAB defaults):
  AND = min, OR = max, implication = min, aggregation = max,
  defuzzification = centroid over 101 samples.

Usage
  python3 collision_risk_fis.py                                 # Run this only on PC/Mac: worked example, test cases, all figures
  python3 collision_risk_fis.py --rules refined
  sudo python3 collision_risk_fis.py --robot                    # PiCar-X, refined rules
  sudo python3 collision_risk_fis.py --robot --rules baseline   # PiCar-X, show the slow down trap
  python3 collision_risk_fis.py --plot-log run_log.csv
"""
import argparse
import csv
import os
import threading
import time

import numpy as np

# ---------------------------------------------------------------- Step 1
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


# ---------------------------------------------------------------- Step 2
DISTANCE = {                    # cm
    "Near":   [0, 0, 15, 30],
    "Medium": [20, 35, 45, 60],
    "Far":    [50, 70, 100, 100],
}
POWER = {                       # % motor power
    "Slow": [0, 0, 20, 45],
    "Fast": [30, 60, 100, 100],
}
RISK = {                        # % collision risk
    "Low":    [0, 0, 20, 35],
    "Normal": [25, 45, 55, 75],
    "High":   [65, 80, 100, 100],
}
y_risk = np.linspace(0, 100, 101)


# ---------------------------------------------------------------- Step 3
# Each rule: (name, text, firing function, consequent)
RULES_BASELINE = [
    ("R1", "IF Distance is Far OR Power is Slow THEN Risk is Low",
     lambda d, p: max(d["Far"], p["Slow"]), "Low"),
    ("R2", "IF Distance is Medium AND Power is Fast THEN Risk is Normal",
     lambda d, p: min(d["Medium"], p["Fast"]), "Normal"),
    ("R3", "IF Distance is Near THEN Risk is High",
     lambda d, p: d["Near"], "High"),
]

# Refined base: R1 no longer lets "Slow" alone fire Low risk, so a car that
# slows down near a wall cannot talk itself out of the danger zone.
RULES_REFINED = [
    ("R1", "IF Distance is Far THEN Risk is Low",
     lambda d, p: d["Far"], "Low"),
    ("R2", "IF Distance is Medium AND Power is Slow THEN Risk is Low",
     lambda d, p: min(d["Medium"], p["Slow"]), "Low"),
    ("R3", "IF Distance is Medium AND Power is Fast THEN Risk is Normal",
     lambda d, p: min(d["Medium"], p["Fast"]), "Normal"),
    ("R4", "IF Distance is Near THEN Risk is High",
     lambda d, p: d["Near"], "High"),
]
RULE_SETS = {"baseline": RULES_BASELINE, "refined": RULES_REFINED}


# ---------------------------------------------------------------- Steps 4-8
def fuzzify(x, sets):
    return {k: float(trapmf([x], v)[0]) for k, v in sets.items()}


def evaluate(distance, power, rules=RULES_BASELINE, details=False):
    distance = float(np.clip(distance, 0, 100))
    power = float(np.clip(power, 0, 100))
    mu_d = fuzzify(distance, DISTANCE)                      # Step 4
    mu_p = fuzzify(power, POWER)
    w = {name: f(mu_d, mu_p) for name, _, f, _ in rules}    # Step 5
    clipped = [np.minimum(w[name], trapmf(y_risk, RISK[cons]))   # Step 6
               for name, _, _, cons in rules]
    agg = np.max(np.vstack(clipped), axis=0)                # Step 7
    risk = float((y_risk * agg).sum() / agg.sum()) if agg.sum() > 0 else 50.0  # Step 8
    if details:
        return risk, {"mu_d": mu_d, "mu_p": mu_p, "w": w, "clipped": clipped, "agg": agg}
    return risk


def print_steps(distance, power, rules):
    risk, info = evaluate(distance, power, rules, details=True)
    print("\n=== Worked example: Distance = %.1f cm, Power = %.1f %% ===" % (distance, power))
    print("Step 4  fuzzification")
    for k, v in info["mu_d"].items():
        print("   mu_%-6s(%5.1f cm) = %.3f" % (k, distance, v))
    for k, v in info["mu_p"].items():
        print("   mu_%-6s(%5.1f %%)  = %.3f" % (k, power, v))
    print("Step 5  rule firing")
    for name, text, _, cons in rules:
        print("   %s  w = %.3f -> %-6s  (%s)" % (name, info["w"][name], cons, text))
    print("Steps 6-8  clip, aggregate (max), centroid")
    print("   Risk = %.1f %%" % risk)
    return risk


# ---------------------------------------------------------------- Figures
def make_figures(rules, out="figs", ex=(25, 55)):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    os.makedirs(out, exist_ok=True)
    plt.rcParams.update({"font.size": 12, "axes.grid": True, "grid.alpha": 0.3})
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]

    # MF plots
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    for ax, (title, sets, unit) in zip(axes, [("Distance (cm)", DISTANCE, "cm"),
                                              ("Motor power (%)", POWER, "%"),
                                              ("Collision risk (%)", RISK, "%")]):
        x = np.linspace(0, 100, 501)
        for (k, v), c in zip(sets.items(), colors):
            ax.plot(x, trapmf(x, v), lw=2.5, color=c, label="%s %s" % (k, v))
        ax.set_title(title, fontweight="bold"); ax.set_xlabel(unit)
        ax.set_ylim(0, 1.08); ax.legend(fontsize=10, loc="center right")
    axes[0].set_ylabel(r"$\mu$")
    fig.tight_layout(); fig.savefig(f"{out}/mf_plots.png", dpi=160); plt.close(fig)

    # Rule viewer (worked example)
    d0, p0 = ex
    risk, info = evaluate(d0, p0, rules, details=True)
    n = len(rules)
    fig, axes = plt.subplots(n, 3, figsize=(13, 2.3 * n), sharex=True)
    axes = np.atleast_2d(axes)
    x = np.linspace(0, 100, 501)
    for i, (name, text, _, cons) in enumerate(rules):
        used_d = [k for k in DISTANCE if k in text.split("THEN")[0]]
        used_p = [k for k in POWER if k in text.split("THEN")[0]]
        for ax, used, sets, val, mu in [(axes[i, 0], used_d, DISTANCE, d0, info["mu_d"]),
                                         (axes[i, 1], used_p, POWER, p0, info["mu_p"])]:
            for k in used:
                ax.plot(x, trapmf(x, sets[k]), "r", lw=2)
                ax.fill_between(x, np.minimum(trapmf(x, sets[k]), mu[k]), color="r", alpha=0.18)
                ax.text(2, 0.82, "%s\nmu=%.3f" % (k, mu[k]), fontsize=10)
            ax.axvline(val, color="k", lw=1.2); ax.set_ylim(0, 1.05)
        ax = axes[i, 2]
        ax.plot(y_risk, trapmf(y_risk, RISK[cons]), "--", color="#1f77b4", lw=1.2)
        ax.fill_between(y_risk, info["clipped"][i], color="#1f77b4", alpha=0.55)
        ax.text(2, 0.82, "%s: %s, w=%.3f" % (name, cons, info["w"][name]), fontsize=10)
        ax.set_ylim(0, 1.05)
    axes[0, 0].set_title("Distance = %g cm" % d0, fontweight="bold")
    axes[0, 1].set_title("Power = %g %%" % p0, fontweight="bold")
    axes[0, 2].set_title("Risk consequent (clipped)", fontweight="bold")
    fig.tight_layout(); fig.savefig(f"{out}/rule_viewer.png", dpi=160); plt.close(fig)

    # Aggregation + centroid
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.fill_between(y_risk, info["agg"], color="#1f77b4", alpha=0.5, label="Aggregated output")
    ax.plot(y_risk, info["agg"], color="#1f4e79")
    ax.axvline(risk, color="r", lw=2.5, label="Centroid = %.1f %%" % risk)
    ax.set_xlabel("Collision risk (%)"); ax.set_ylabel(r"$\mu$"); ax.set_ylim(0, 1.05)
    ax.set_xlim(0, 100); ax.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(f"{out}/aggregation.png", dpi=160); plt.close(fig)

    # Control surfaces (baseline and refined)
    D, P = np.meshgrid(np.linspace(0, 100, 41), np.linspace(0, 100, 41))
    for tag, rs in RULE_SETS.items():
        Z = np.vectorize(lambda d, p: evaluate(d, p, rs))(D, P)
        fig = plt.figure(figsize=(8, 6.5))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot_surface(D, P, Z, cmap="viridis", edgecolor="none")
        ax.set_xlabel("Distance (cm)"); ax.set_ylabel("Motor power (%)"); ax.set_zlabel("Risk (%)")
        ax.view_init(elev=28, azim=-130)
        ax.set_zlim(0, 100)
        fig.savefig(f"{out}/surface_{tag}.png", dpi=160, bbox_inches="tight", pad_inches=0.2); plt.close(fig)

    # Crisp vs fuzzy at fixed power
    dd = np.linspace(0, 100, 401)
    fig, ax = plt.subplots(figsize=(9, 4))
    for pw, ls in [(30, "-"), (60, "--")]:
        ax.plot(dd, [evaluate(v, pw, RULES_REFINED) for v in dd], ls, lw=2.5,
                label="Fuzzy, power %d %%" % pw)
    crisp = np.where(dd < 20, 90, np.where(dd < 40, 50, 10))
    ax.step(dd, crisp, where="post", color="#d62728", lw=2.5, label="Crisp zones (20 / 40 cm)")
    ax.set_xlabel("Distance (cm)"); ax.set_ylabel("Risk (%)"); ax.set_ylim(0, 100); ax.set_xlim(0, 100)
    ax.legend()
    fig.tight_layout(); fig.savefig(f"{out}/crisp_vs_fuzzy.png", dpi=160); plt.close(fig)

    # Simulated approach to a wall: baseline vs refined rule base
    fig, axes = plt.subplots(1, 2, figsize=(15, 4.6), sharey=True)
    for ax, (tag, rs) in zip(axes, RULE_SETS.items()):
        log = simulate_approach(rules=rs)
        t = [r["t"] for r in log]
        ax.plot(t, [r["distance"] for r in log], color="#1f77b4", lw=2.5, label="Distance (cm)")
        ax.plot(t, [r["power"] for r in log], color="#2ca02c", lw=2.5, label="Motor power (%)")
        ax.plot(t, [r["risk"] for r in log], color="#d62728", lw=2.5, label="Risk (%)")
        for thr, lab in [(CAUTION_RISK, "Caution"), (DANGER_RISK, "Danger")]:
            ax.axhline(thr, color="gray", ls=":", lw=1.5)
            ax.text(t[-1], thr + 1.5, lab, ha="right", color="gray")
        final = log[-1]["distance"]
        ax.set_title("%s rules: final distance %.0f cm" % (tag.capitalize(), final), fontweight="bold")
        ax.set_xlabel("Time (s)"); ax.set_ylim(0, 112)
    axes[0].legend(loc="upper right")
    fig.tight_layout(); fig.savefig(f"{out}/simulated_run.png", dpi=160); plt.close(fig)

    print("Figures saved in ./%s/" % out)


# ---------------------------------------------------------------- Robot behavior
CRUISE_POWER = 40       # % power when risk is low
CAUTION_POWER = 20      # % power when risk is in the caution band
CAUTION_RISK = 45       # % risk that triggers "Caution"
DANGER_RISK = 65        # % risk that triggers "Danger, stopping"
RELEASE_RISK = 35       # % risk needed to leave the danger state (hysteresis)
LOOP_DT = 0.1           # s


def decide(risk, state=None):
    """Map the fuzzy risk to a behavior state and a commanded power.
    Once in danger the car stays stopped until risk falls below RELEASE_RISK,
    so it does not stop and go when the stopped power lowers the risk."""
    if state == "danger" and risk > RELEASE_RISK:
        return "danger", 0
    if risk >= DANGER_RISK:
        return "danger", 0
    if risk >= CAUTION_RISK:
        return "caution", CAUTION_POWER
    return "clear", CRUISE_POWER


def simulate_approach(start_cm=110, rules=RULES_REFINED, k_cm_per_pct=0.9, t_end=6.0):
    """Kinematic stand in for the car: speed (cm/s) = k * power."""
    d, power, t, state, log = start_cm, CRUISE_POWER, 0.0, None, []
    while t <= t_end + 1e-9:
        risk = evaluate(min(d, 100), power, rules)
        state, power_cmd = decide(risk, state)
        log.append({"t": round(t, 2), "distance": d, "power": power, "risk": risk, "state": state})
        power = power_cmd
        d = max(d - k_cm_per_pct * power * LOOP_DT, 0)
        t += LOOP_DT
    return log


def run_robot(rules, log_file="run_log.csv"):
    from picarx import Picarx
    try:
        from picarx.tts import Espeak as Voice
    except ImportError:
        from robot_hat import TTS as Voice
    px = Picarx()
    voice = Voice()

    def say(text):                      # speak without blocking the control loop
        threading.Thread(target=voice.say, args=(text,), daemon=True).start()

    def read_cm():
        vals = [px.ultrasonic.read() for _ in range(3)]
        vals = sorted(v for v in vals if v is not None and v > 0)
        return vals[len(vals) // 2] if vals else None

    say("Fuzzy collision guard ready")
    time.sleep(1.5)
    state, power, t0 = None, CRUISE_POWER, time.time()
    with open(log_file, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "distance", "power", "risk", "state"] + [r[0] for r in rules])
        try:
            while True:
                d = read_cm()
                if d is None:
                    continue
                risk, info = evaluate(min(d, 100), power, rules, details=True)
                new_state, power_cmd = decide(risk, state)
                if new_state != state:
                    if new_state == "danger":
                        px.stop()               # stop first, then talk
                        say("Danger, stopping")
                    elif new_state == "caution":
                        say("Caution")
                    else:
                        say("Path clear")
                    state = new_state
                power = power_cmd
                px.set_dir_servo_angle(0)
                px.forward(power) if power > 0 else px.stop()
                t = time.time() - t0
                writer.writerow([round(t, 2), round(d, 1), power, round(risk, 1), state]
                                + [round(info["w"][r[0]], 3) for r in rules])
                print("t=%5.1f  d=%5.1f cm  P=%2d%%  risk=%5.1f%%  %s  w=%s"
                      % (t, d, power, risk, state,
                         " ".join("%.2f" % info["w"][r[0]] for r in rules)))
                time.sleep(LOOP_DT)
        except KeyboardInterrupt:
            pass
        finally:
            px.stop()
            px.set_dir_servo_angle(0)
            print("log saved to %s" % log_file)


def plot_log(path, out="figs"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = list(csv.DictReader(open(path)))
    t = [float(r["t"]) for r in rows]
    fig, ax = plt.subplots(figsize=(10, 4.2))
    for key, c in [("distance", "#1f77b4"), ("power", "#2ca02c"), ("risk", "#d62728")]:
        ax.plot(t, [float(r[key]) for r in rows], color=c, lw=2.5, label=key)
    ax.set_xlabel("Time (s)"); ax.legend(); ax.set_ylim(0, 105)
    os.makedirs(out, exist_ok=True)
    fig.tight_layout(); fig.savefig(f"{out}/robot_run.png", dpi=160)
    print("saved %s/robot_run.png" % out)


# ---------------------------------------------------------------- main
TEST_CASES = [(25, 55), (10, 80), (10, 10), (40, 70), (40, 15), (80, 80), (90, 10)]

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--rules", choices=RULE_SETS, default=None,
                    help="baseline for the slides demo, refined for the robot (defaults)")
    ap.add_argument("--robot", action="store_true")
    ap.add_argument("--plot-log")
    ap.add_argument("--no-figs", action="store_true")
    args = ap.parse_args()
    if args.robot:
        run_robot(RULE_SETS[args.rules or "refined"])
    elif args.plot_log:
        plot_log(args.plot_log)
    else:
        rules = RULE_SETS[args.rules or "baseline"]
        print_steps(25, 55, rules)
        for tag, rs in RULE_SETS.items():
            print("\nTest cases (%s rules)" % tag)
            print("  Dist  Power   " + "  ".join("%5s" % r[0] for r in rs) + "   Risk")
            for d, p in TEST_CASES:
                risk, info = evaluate(d, p, rs, details=True)
                print("  %4g  %5g   " % (d, p)
                      + "  ".join("%5.3f" % info["w"][r[0]] for r in rs) + "  %5.1f" % risk)
        if not args.no_figs:
            make_figures(rules)
