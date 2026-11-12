#!/usr/bin/env python3
"""Analysis for dispatch_bench: reads the snapshots it wrote
and compares every controller with the greedy baseline, cell by cell.

    python3 -I analyze.py <outDir> [--baseline greedy] [--markdown]

Per run (one snapshot), computed from the event timeline as the oracle of
test_metrics_consistency does, and required to agree with the snapshot's own
metrics block, so a run the two paths disagree on stops the analysis:
  waiting  their first ENTER minus their arrivalTime
  journey  their last LEAVE minus their arrivalTime
From the snapshot's metrics block: p90 waiting (nearest rank), total energy (the
non-regenerative default drive), the peak and drain windows and the queue. The
regenerative figure is the one thing a snapshot does not hold; it comes from
runs.tsv, with the wall-clock time of the run.

Per (scenario, controller), over its seeds: the mean of the run values and their
standard deviation across seeds. Against the baseline, seed by seed (common
random numbers: same seed, same passengers, which this script checks and does
not assume): the relative change of the mean, and in how many seeds the
difference has the sign of that mean change.

Standard library only.
"""

import glob
import json
import math
import os
import sys
from collections import defaultdict

ORACLE_TOLERANCE_S = 1e-6


def mean(xs):
    return sum(xs) / len(xs)


def sd(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def read_run(path):
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

    m = snap["metrics"]
    wait = mean([boarded[c] - arrival[c] for c in arrival])
    journey = mean([alighted[c] - arrival[c] for c in arrival])
    if abs(wait - m["waitingTime"]["average"]) > ORACLE_TOLERANCE_S:
        raise SystemExit(f"{path}: waiting {wait} from the timeline, {m['waitingTime']['average']} reported")
    if abs(journey - m["journeyTime"]["average"]) > ORACLE_TOLERANCE_S:
        raise SystemExit(f"{path}: journey {journey} from the timeline, {m['journeyTime']['average']} reported")

    windows = m["waitingTimeByWindow"]
    return {
        "wait": wait, "journey": journey, "p90": m["waitingTime"]["p90"],
        "energy": m["totalEnergyConsumption"] / 1e6,
        "customers": len(arrival),
        "peak": windows["peak"]["average"] if windows["peak"]["count"] else float("nan"),
        "drain": windows["drain"]["average"] if windows["drain"]["count"] else float("nan"),
        "drainN": windows["drain"]["count"],
        "queueMax": m["queue"]["max"],
        "refused": m["passedBy"]["refusedAtDoor"]["events"],
        "end": max(e["time"] for e in snap["events"]),
    }, snap["customers"]


def read_all(out):
    runs = defaultdict(dict)      # (scenario, controller) -> seed -> run
    arrivals = defaultdict(dict)  # (scenario, seed) -> controller -> customers
    for path in sorted(glob.glob(os.path.join(out, "*__s*.json"))):
        scenario, controller, s = os.path.basename(path)[:-5].split("__")
        seed = int(s[1:])
        run, customers = read_run(path)
        runs[(scenario, controller)][seed] = run
        arrivals[(scenario, seed)][controller] = customers
    if not runs:
        raise SystemExit(f"no snapshots in {out}")

    # A re-run appends to runs.tsv, so the last line of a key is the one whose
    # snapshot is on disk.
    with open(os.path.join(out, "runs.tsv")) as f:
        for line in f:
            scenario, controller, seed, wall, regen = line.split("\t")
            run = runs.get((scenario, controller), {}).get(int(seed))
            if run is not None:
                run["wall"], run["regen"] = float(wall), float(regen) / 1e6
    return runs, arrivals


def cell_order(scenario):
    pattern, _, rest = scenario.partition("-")
    rho, _, capped = rest.partition("+")
    return (capped != "", ["up", "lunch", "uniform"].index(pattern), float(rho))


def compare(runs, key, base_key, metric):
    """Relative change of the mean and seeds that keep its sign, or None."""
    r, b = runs[key], runs.get(base_key, {})
    common = sorted(s for s in r if s in b)
    if not common:
        return None
    v, w = mean([r[s][metric] for s in common]), mean([b[s][metric] for s in common])
    sign = (v > w) - (v < w)
    same = sum(1 for s in common if ((r[s][metric] > b[s][metric]) - (r[s][metric] < b[s][metric])) == sign)
    return 100.0 * (v - w) / w, same, len(common)


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    out = sys.argv[1]
    baseline = sys.argv[sys.argv.index("--baseline") + 1] if "--baseline" in sys.argv else "greedy"
    markdown = "--markdown" in sys.argv

    runs, arrivals = read_all(out)
    broken = [k for k, v in arrivals.items() if any(c != next(iter(v.values())) for c in v.values())]
    print("common random numbers:", "every controller saw the same passengers per seed"
          if not broken else f"BROKEN for {sorted(broken)}")
    print()

    scenarios = sorted({k[0] for k in runs}, key=cell_order)
    order = {c: i for i, c in enumerate([baseline, "thv", "gahca", "beamurgia"])}
    controllers = sorted({k[1] for k in runs}, key=lambda c: (order.get(c, 99), c))
    if markdown:
        print("| cell | controller | wait mean (SD) | p90 (SD) | journey (SD) | energy MJ (SD) | kJ/pax | regen MJ |"
              " peak / drain wait | queue max | refused |")
        print("|---|---|---|---|---|---|---|---|---|---|---|")
    for scenario in scenarios:
        for controller in controllers:
            key = (scenario, controller)
            if key not in runs:
                continue
            r = runs[key]
            seeds = sorted(r)
            col = lambda m: [r[s][m] for s in seeds]
            kj = mean([r[s]["energy"] * 1000.0 / r[s]["customers"] for s in seeds])

            def stat(m, digits=1):
                base = compare(runs, key, (scenario, baseline), m) if controller != baseline else None
                text = f"{mean(col(m)):.{digits}f} ({sd(col(m)):.{digits}f})"
                if base:
                    text += f" {base[0]:+.1f}% {base[1]}/{base[2]}"
                return text

            if markdown:
                print(f"| {scenario} | {controller} | {stat('wait')} | {stat('p90')} | {stat('journey')} | "
                      f"{stat('energy', 1)} | {kj:.1f} | {stat('regen', 1)} | "
                      f"{mean(col('peak')):.1f} / {mean(col('drain')):.1f} (n={mean(col('drainN')):.0f}) | "
                      f"{mean(col('queueMax')):.0f} | {mean(col('refused')):.0f} |")
            else:
                print(f"{scenario:22} {controller:10} n={len(seeds):2}  wait {stat('wait')}  p90 {stat('p90')}  "
                      f"journey {stat('journey')}  energy {stat('energy')}  {kj:.1f} kJ/pax  regen {stat('regen')}  "
                      f"peak {mean(col('peak')):.1f} drain {mean(col('drain')):.1f} (n={mean(col('drainN')):.0f})  "
                      f"queue max {mean(col('queueMax')):.0f}  refused {mean(col('refused')):.0f}  "
                      f"end {mean(col('end')):.0f} s  wall {mean(col('wall')):.1f} s")
        if not markdown:
            print()


if __name__ == "__main__":
    main()
