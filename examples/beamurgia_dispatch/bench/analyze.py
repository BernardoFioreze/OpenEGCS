#!/usr/bin/env python3
"""Analysis for beamurgia_bench (Beamurgia et al. 2016): reads the snapshots it
wrote and reports what the paper reports, computed from the event timeline rather than from the collector.

    python3 -I analyze.py <outDir> [--baseline S]

Per run (one snapshot): every passenger's waiting time is the instant of their
first ENTER minus their arrivalTime - the oracle path test_metrics_consistency
uses. From those:
  AWT   the run's mean waiting time (one value of the paper's Table 6)
  SD    standard deviation over the run's passengers
  LWT   the longest waiting time
Per (scenario, variant, reversal), over its seeds:
  mean of the run AWTs and their SD across runs (Table 6's columns),
  pooled mean and SD over every passenger (Table 5's "mean/SD"),
  peak and drain AWT from the snapshot's own window split (metrics.waitingTimeByWindow),
and against the baseline variant of the same scenario and reversal, seed by
seed (common random numbers: same seed, same passengers):
  the relative change of the mean AWT and how many seeds moved down.
Finally a one-factor ANOVA over the pooled passengers of each scenario's GA
variants (section 6.4 of the paper), with the p-value from the F distribution.

Standard library only (the F tail is the regularised incomplete beta, by
Lentz's continued fraction).
"""

import glob
import json
import math
import os
import sys
from collections import defaultdict


def waiting_times(path):
    with open(path) as f:
        snap = json.load(f)
    arrival = {int(k): v["arrivalTime"] for k, v in snap["customers"].items()}
    boarded = {}
    for e in snap["events"]:
        if e["type"] == "ENTER" and e["customerId"] not in boarded:
            boarded[e["customerId"]] = e["time"]
    if set(boarded) != set(arrival):
        raise SystemExit(f"{path}: {len(arrival)} passengers, {len(boarded)} boarded")
    waits = [boarded[c] - arrival[c] for c in sorted(arrival)]
    window = snap["metrics"].get("waitingTimeByWindow")
    peak = window["peak"]["average"] if window and window["peak"]["count"] else float("nan")
    drain = window["drain"]["average"] if window and window["drain"]["count"] else float("nan")
    return waits, peak, drain, snap["customers"]


def mean(xs):
    return sum(xs) / len(xs)


def sd(xs):
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1)) if len(xs) > 1 else 0.0


def betacf(a, b, x):
    tiny, eps = 1e-300, 3e-16
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c, d = 1.0, 1.0 - qab * x / qap
    d = 1.0 / (d if abs(d) > tiny else tiny)
    h = d
    for m in range(1, 1000):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        d = 1.0 / (d if abs(d) > tiny else tiny)
        c = 1.0 + aa / c
        c = c if abs(c) > tiny else tiny
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        d = 1.0 / (d if abs(d) > tiny else tiny)
        c = 1.0 + aa / c
        c = c if abs(c) > tiny else tiny
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < eps:
            break
    return h


def betai(a, b, x):
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    lbt = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1.0 - x)
    if x < (a + 1.0) / (a + b + 2.0):
        return math.exp(lbt) * betacf(a, b, x) / a
    return 1.0 - math.exp(lbt) * betacf(b, a, 1.0 - x) / b


def anova(groups):
    """One-factor ANOVA: F and its upper-tail p-value."""
    k = len(groups)
    n = sum(len(g) for g in groups)
    grand = sum(sum(g) for g in groups) / n
    ssb = sum(len(g) * (mean(g) - grand) ** 2 for g in groups)
    ssw = sum(sum((x - mean(g)) ** 2 for x in g) for g in groups)
    df1, df2 = k - 1, n - k
    f = (ssb / df1) / (ssw / df2)
    p = betai(df2 / 2.0, df1 / 2.0, df2 / (df2 + df1 * f))
    return f, df1, df2, p


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    out = sys.argv[1]
    baseline = "S"
    if "--baseline" in sys.argv:
        baseline = sys.argv[sys.argv.index("--baseline") + 1]

    runs = defaultdict(dict)      # (scenario, variant, reversal) -> seed -> run
    arrivals = defaultdict(dict)  # (scenario, seed) -> variant -> customers (CRN check)
    for path in sorted(glob.glob(os.path.join(out, "*.json"))):
        scenario, variant, reversal, s = os.path.basename(path)[:-5].split("__")
        seed = int(s[1:])
        waits, peak, drain, customers = waiting_times(path)
        runs[(scenario, variant, reversal)][seed] = {
            "awt": mean(waits), "sd": sd(waits), "lwt": max(waits), "waits": waits,
            "peak": peak, "drain": drain}
        arrivals[(scenario, seed)][(variant, reversal)] = customers

    crn_broken = [k for k, v in arrivals.items() if any(c != next(iter(v.values())) for c in v.values())]
    print("common random numbers:", "every variant saw the same passengers per seed"
          if not crn_broken else f"BROKEN for {crn_broken}")

    for key in sorted(runs):
        scenario, variant, reversal = key
        r = runs[key]
        seeds = sorted(r)
        awts = [r[s]["awt"] for s in seeds]
        pooled = [w for s in seeds for w in r[s]["waits"]]
        line = (f"{scenario:9} {variant:13} {reversal:3}  n={len(seeds):2}  "
                f"AWT {mean(awts):7.2f} (SD runs {sd(awts):6.2f}, min {min(awts):7.2f}, max {max(awts):7.2f})  "
                f"pooled {mean(pooled):7.2f}/{sd(pooled):6.2f}  LWT max {max(r[s]['lwt'] for s in seeds):7.1f}  "
                f"peak {mean([r[s]['peak'] for s in seeds]):7.2f} drain {mean([r[s]['drain'] for s in seeds]):7.2f}")
        base = runs.get((scenario, baseline, reversal))
        if base and variant != baseline:
            common = [s for s in seeds if s in base]
            if common:
                b = mean([base[s]["awt"] for s in common])
                v = mean([r[s]["awt"] for s in common])
                down = sum(1 for s in common if r[s]["awt"] < base[s]["awt"])
                line += f"  vs {baseline}: {100.0 * (v - b) / b:+6.1f}%  lower in {down}/{len(common)} seeds"
        print(line)

    by_scenario = defaultdict(list)
    for (scenario, variant, reversal), r in runs.items():
        if variant != "greedy":
            by_scenario[(scenario, reversal)].append((variant, [w for s in sorted(r) for w in r[s]["waits"]]))
    for (scenario, reversal), groups in sorted(by_scenario.items()):
        if len(groups) < 2:
            continue
        f, df1, df2, p = anova([g for _, g in groups])
        print(f"ANOVA {scenario} {reversal}: {len(groups)} GA variants, F({df1},{df2}) = {f:.3f}, p = {p:.4g}")


if __name__ == "__main__":
    main()
