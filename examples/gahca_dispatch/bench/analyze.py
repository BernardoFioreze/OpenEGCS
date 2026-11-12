#!/usr/bin/env python3
"""Analysis for gahca_bench: reads the snapshots it wrote
and reports GAHCA against THV the way Cortes et al. (2004) do (Table 1,
Time_System), computed from the event timeline rather than from the collector.

    python3 -I analyze.py <outDir> [--baseline thv] [--target gahca]

Per run (one snapshot), per passenger, the oracle path test_metrics_consistency
uses:
  waiting  = their first ENTER minus their arrivalTime
  journey  = their last LEAVE minus their arrivalTime (the paper's Time_System)
Per controller, over its seeds: the mean of the run means and each seed's value.
Then every controller against the baseline, seed by seed over the seeds both
have: the relative change of the mean and in how many seeds it is lower. The
target-vs-baseline line is printed last, next to the paper's -23.32%.

Also says whether every controller saw the same passengers per seed (common
random numbers). `gahca` draws its GA from the engine's provider, so its
passengers diverge from THV's; `gahca-crn` does not.

Standard library only.
"""

import glob
import json
import os
import sys
from collections import defaultdict

PAPER_THV, PAPER_GAHCA = 195.60, 149.98  # Cortes 2004, Table 1, Time_System mean (s)


def run_times(path):
    with open(path) as f:
        snap = json.load(f)
    arrival = {int(k): v["arrivalTime"] for k, v in snap["customers"].items()}
    boarded, alighted = {}, {}
    for e in snap["events"]:
        if e["type"] == "ENTER" and e["customerId"] not in boarded:
            boarded[e["customerId"]] = e["time"]
        elif e["type"] == "LEAVE":
            alighted[e["customerId"]] = e["time"]
    if set(boarded) != set(arrival) or set(alighted) != set(arrival):
        raise SystemExit(f"{path}: {len(arrival)} passengers, {len(boarded)} boarded, {len(alighted)} alighted")
    waits = [boarded[c] - arrival[c] for c in sorted(arrival)]
    journeys = [alighted[c] - arrival[c] for c in sorted(arrival)]
    return mean(waits), mean(journeys), snap["customers"]


def mean(xs):
    return sum(xs) / len(xs)


def compare(runs, ctrl, base, metric):
    common = sorted(s for s in runs[ctrl] if s in runs[base])
    if not common:
        return None
    b = mean([runs[base][s][metric] for s in common])
    v = mean([runs[ctrl][s][metric] for s in common])
    down = sum(1 for s in common if runs[ctrl][s][metric] < runs[base][s][metric])
    return 100.0 * (v - b) / b, down, len(common)


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    out = sys.argv[1]
    opt = lambda name, default: sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default
    baseline, target = opt("--baseline", "thv"), opt("--target", "gahca")

    runs = defaultdict(dict)      # controller -> seed -> {"wait", "journey"}
    arrivals = defaultdict(dict)  # seed -> controller -> customers
    for path in sorted(glob.glob(os.path.join(out, "*__s*.json"))):
        controller, s = os.path.basename(path)[:-5].rsplit("__", 1)
        seed = int(s[1:])
        w, j, customers = run_times(path)
        runs[controller][seed] = {"wait": w, "journey": j}
        arrivals[seed][controller] = customers
    if not runs:
        raise SystemExit(f"no snapshots in {out}")

    if baseline in runs:
        for c in sorted(runs):
            if c == baseline:
                continue
            same = [s for s in sorted(runs[c]) if s in arrivals and baseline in arrivals[s]
                    and arrivals[s][c] == arrivals[s][baseline]]
            print(f"passengers: {c:10} saw the same passengers as {baseline} in {len(same)}/{len(runs[c])} seeds")
        print()

    for c in sorted(runs, key=lambda c: (c != baseline, c)):
        seeds = sorted(runs[c])
        for metric, label in (("journey", "journey"), ("wait", "waiting")):
            vals = [runs[c][s][metric] for s in seeds]
            per_seed = " ".join(f"s{s}={runs[c][s][metric]:.1f}" for s in seeds)
            line = f"{c:10} {label:8} mean {mean(vals):7.2f}  min {min(vals):6.1f} max {max(vals):6.1f}  [{per_seed}]"
            if c != baseline and baseline in runs:
                r = compare(runs, c, baseline, metric)
                if r:
                    line += f"  vs {baseline} {r[0]:+6.2f}%, lower in {r[1]}/{r[2]}"
            print(line)

    if target in runs and baseline in runs:
        rj, rw = compare(runs, target, baseline, "journey"), compare(runs, target, baseline, "wait")
        print()
        print(f"{target} vs {baseline}: journey {rj[0]:+.2f}% (lower in {rj[1]}/{rj[2]} seeds), "
              f"waiting {rw[0]:+.2f}% (lower in {rw[1]}/{rw[2]} seeds)")
        print(f"paper (Cortes 2004 Table 1, Time_System): GAHCA {PAPER_GAHCA} s vs THV {PAPER_THV} s = "
              f"{100.0 * (PAPER_GAHCA - PAPER_THV) / PAPER_THV:+.2f}%")


if __name__ == "__main__":
    main()
