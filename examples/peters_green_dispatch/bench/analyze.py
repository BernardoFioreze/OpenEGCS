#!/usr/bin/env python3
"""Analysis for peters_bench (Peters 1998, Green Strategy No. 2): reads what
peters_bench wrote.

    python3 -I analyze.py <outDir> baseline [--kinematics tese]
    python3 -I analyze.py <outDir> effect [--kinematics tese] [--rho 1.0]
    python3 -I analyze.py <outDir> single [--kinematics tese]

`baseline` only reads plain runs and scores every (demand, mix) cell against the
"normal" waiting curve of the thesis's Fig. 9.4, which is how the primary cell is
fixed before the rule is switched on. The score is the mean absolute gap,
in points of percentage, at the instants of FIG94. The curve is "calls answered",
so a passenger's wait stops when the doors begin to open: the 1.8 s the engine adds
before an ENTER is taken off.

`effect` pairs each green run with the plain run of its seed: same passengers
(checked), so the change is the rule's. It stops if a passenger did not arrive or
if a run breaks the energy balance E = dPE + losses.

Standard library only.
"""

import csv
import json
import math
import os
import sys
from collections import defaultdict

# The "normal" curve of Fig. 9.4, read by hand: seconds -> % of calls answered.
FIG94 = {10: 52, 20: 73, 30: 80, 40: 87, 60: 95, 100: 97}
DOOR_OPEN = 1.8


def mean(xs):
    return sum(xs) / len(xs)


def sd(xs):
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1)) if len(xs) > 1 else 0.0


def read_run(out_dir, row):
    with open(os.path.join(out_dir, row["tag"] + ".json")) as f:
        snap = json.load(f)
    arrival = {int(k): v["arrivalTime"] for k, v in snap["customers"].items()}
    entered, left = {}, {}
    departs = 0
    for e in snap["events"]:
        if e["type"] == "ENTER" and e["customerId"] not in entered:
            entered[e["customerId"]] = e["time"]
        elif e["type"] == "LEAVE":
            left[e["customerId"]] = e["time"]
        elif e["type"] == "DEPART":
            departs += 1
    if set(left) != set(arrival) or set(entered) != set(arrival):
        raise SystemExit(f'{row["tag"]}: a passenger did not arrive')
    energy = float(row["energyRegenJ"])
    losses = energy - float(row["potentialJ"])
    if losses <= 0:
        raise SystemExit(f'{row["tag"]}: losses {losses} J, the energy balance does not close')
    waits = sorted(entered[c] - arrival[c] for c in arrival)
    return {
        "waits": waits,
        "awt": mean(waits),
        "att": mean([left[c] - entered[c] for c in arrival]),
        "e": energy,
        "ec": float(row["energyClampedJ"]),
        "pe": float(row["potentialJ"]),
        "loss": losses,
        "starts": int(row["starts"]),
        "departs": departs,
        "km": float(row["distanceM"]) / 1000.0,
        "decisions": int(row["decisions"]),
        "forced": int(row["forced"]),
        "passengers": sorted((v["arrivalTime"], v["atFloor"], v["toFloor"]) for v in snap["customers"].values()),
    }


def load(out_dir, kinematics):
    runs = {}
    with open(os.path.join(out_dir, "runs.tsv")) as f:
        for row in csv.DictReader(f, delimiter="\t"):
            if row["kinematics"] != kinematics:
                continue
            key = (row["baseline"], row["rule"], float(row["percent"]), float(row["outgoing"]), int(row["seed"]))
            runs[key] = read_run(out_dir, row)
    if not runs:
        raise SystemExit("no runs match")
    return runs


def baseline_report(runs):
    cells = sorted({k[0:1] + k[2:4] for k in runs})
    print("Plain runs against the 'normal' curve of Fig. 9.4 (lower is closer)")
    print("baseline  demand  down | gap | % answered by " + " ".join(f"{t:>3d}s" for t in FIG94) + " | AWT")
    print(" " * 39 + " ".join(f"{v:4d}" for v in FIG94.values()) + "   <- Fig. 9.4")
    for cell in cells:
        rs = [r for k, r in runs.items() if (k[0], k[2], k[3]) == cell and k[1] == "plain"]
        if not rs:
            continue
        waits = sorted(w - DOOR_OPEN for r in rs for w in r["waits"])
        curve = {t: 100.0 * sum(1 for w in waits if w <= t) / len(waits) for t in FIG94}
        gap = mean([abs(curve[t] - FIG94[t]) for t in FIG94])
        print(f"{cell[0]:8s} {cell[1]:5.0f}% {cell[2]:5.0%} | {gap:4.1f} |               "
              + " ".join(f"{curve[t]:4.0f}" for t in FIG94) + f" | {mean([r['awt'] for r in rs]):5.1f}")


def effect_report(runs, rho):
    def energy(r):
        return (1.0 - rho) * r["ec"] + rho * r["e"]

    pairs = defaultdict(list)
    for (base, rule, pct, out, seed), r in sorted(runs.items()):
        if rule != "green" or (base, "plain", pct, out, seed) not in runs:
            continue
        plain = runs[(base, "plain", pct, out, seed)]
        if plain["passengers"] != r["passengers"]:
            raise SystemExit(f"{base} {pct} {out} seed {seed}: different passengers with and without the rule")
        pairs[(base, pct, out)].append((plain, r))
    print(f"{sum(len(v) for v in pairs.values())} pairs; same passengers in each; every energy balance closes; rho={rho}")
    print("baseline demand down | forced/calls | starts  (seeds) | energy  (seeds) | E/starts | losses   dPE MJ | AWT      (seeds) | ATT | distance | standard errors")
    for (base, pct, out), ps in sorted(pairs.items()):
        ds = [100.0 * (r["starts"] / p["starts"] - 1.0) for p, r in ps]
        de = [100.0 * (energy(r) / energy(p) - 1.0) for p, r in ps]
        mean_ds, mean_de = mean(ds), mean(de)
        elastic = mean_de / mean_ds if mean_ds else float("nan")
        dl = mean([100.0 * (r["loss"] / p["loss"] - 1.0) for p, r in ps])
        dpe = mean([(r["pe"] - p["pe"]) / 1e6 for p, r in ps])
        dw = 100.0 * (mean([r["awt"] for _, r in ps]) / mean([p["awt"] for p, _ in ps]) - 1.0)
        dt = 100.0 * (mean([r["att"] for _, r in ps]) / mean([p["att"] for p, _ in ps]) - 1.0)
        forced = 100.0 * sum(r["forced"] for _, r in ps) / sum(r["decisions"] for _, r in ps)
        dkm = 100.0 * (mean([r["km"] for _, r in ps]) / mean([p["km"] for p, _ in ps]) - 1.0)
        se_s, se_e = sd(ds) / math.sqrt(len(ds)), sd(de) / math.sqrt(len(de))
        print(f"{base:8s} {pct:4.0f}% {out:4.0%} | {forced:5.1f}%       | {mean_ds:+5.2f}% ({sum(1 for x in ds if x < 0):2d}/{len(ds)}) | "
              f"{mean_de:+5.2f}% ({sum(1 for x in de if x < 0):2d}/{len(de)}) | {elastic:5.2f}   | {dl:+5.2f}% {dpe:+6.2f} | "
              f"{dw:+5.2f}% ({sum(1 for p, r in ps if r['awt'] > p['awt']):2d}/{len(ps)} up) | {dt:+5.2f}% | "
              f"km {dkm:+5.2f}% | SE starts {se_s:.2f} energy {se_e:.2f}")


def single_run_report(runs):
    """How far one run can land from the mean: the thesis reports what looks like a
    single simulation, so its figures are one draw of this."""
    groups = defaultdict(list)
    for (base, rule, pct, out, seed), r in sorted(runs.items()):
        if rule == "green" and (base, "plain", pct, out, seed) in runs:
            groups[(base, pct, out)].append((runs[(base, "plain", pct, out, seed)], r))
    print("Single runs: the spread a one-run result is drawn from")
    for (base, pct, out), ps in sorted(groups.items()):
        ds = [100.0 * (r["starts"] / p["starts"] - 1.0) for p, r in ps]
        de = [100.0 * (r["e"] / p["e"] - 1.0) for p, r in ps]
        both = sum(1 for s, e in zip(ds, de) if s <= -3.2 and e <= -6.2)
        ratio = sum(1 for s, e in zip(ds, de) if s < 0 and e / s >= 1.9)
        print(f"{base:8s} {pct:4.0f}% {out:4.0%} | {len(ds)} runs | starts {min(ds):+.2f}% to {max(ds):+.2f}% "
              f"(SD {sd(ds):.2f}) | energy {min(de):+.2f}% to {max(de):+.2f}% (SD {sd(de):.2f}) | "
              f"<= -3.2% and <= -6.2%: {both} | energy/starts >= 1.9: {ratio}")


def main():
    args = sys.argv[1:]
    if len(args) < 2:
        raise SystemExit(__doc__)
    out_dir, mode = args[0], args[1]
    opts = {"kinematics": "tese", "rho": "1.0"}
    for k in range(2, len(args), 2):
        opts[args[k].lstrip("-")] = args[k + 1]
    runs = load(out_dir, opts["kinematics"])
    if mode == "baseline":
        baseline_report(runs)
    elif mode == "single":
        single_run_report(runs)
    else:
        effect_report(runs, float(opts["rho"]))


if __name__ == "__main__":
    main()
