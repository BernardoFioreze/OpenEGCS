#!/usr/bin/env bash
#
# Runs the project's two reference scenarios and prints the aggregates of
# each. Useful locally too, to record a baseline before changing anything:
#
#     ./.github/scripts/reference_scenarios.sh
#
# CI does *not* gate on the values. Changing event timing changes the results
# for the same seed by design, so a fixed expected value would be wrong by
# construction after the first legitimate change, and CI would claim to know
# which number is right. Determinism (same seed, same build -> same snapshot)
# is gated by the test suite instead.
#
# What *is* gated here: the binary runs, the snapshot is written, and it
# parses. This covers the command-line parsing and the wiring of floor height
# and lift dynamics into the metrics collector end to end.
set -euo pipefail

BIN="${LIFTSIM_BIN:-./build/liftsim}"
OUT="${GITHUB_STEP_SUMMARY:-/dev/stdout}"

if [ ! -x "$BIN" ]; then
	echo "ERROR: binary not found at '$BIN' (set LIFTSIM_BIN)" >&2
	exit 1
fi

emit_row() {
	local label="$1"
	shift
	local args="$*"

	rm -f snapshot.json
	"$BIN" "$@" > /dev/null

	if [ ! -s snapshot.json ]; then
		echo "ERROR: '$BIN $args' did not produce a non-empty snapshot.json" >&2
		exit 1
	fi

	LABEL="$label" ARGS="$args" python3 - <<'PY' >> "$OUT"
import json, os, sys

with open("snapshot.json") as fh:
	snap = json.load(fh)		# a failed parse fails the step, on purpose

m = snap["metrics"]
# The mean alone hides the tail, which is exactly what separates two dispatch
# algorithms: without the p90 and the worst case here, a regression that only
# worsens the tail would not show in the baseline. The dissatisfaction
# verdict is left out: it depends on a threshold these scenarios do not
# configure.
w = m["waitingTime"]
print("| {} `{}` | {} | {:.6f} | {:.6f} | {:.6f} | {:.6f} | {:.6f} |".format(
	os.environ["LABEL"], os.environ["ARGS"], snap["schemaVersion"],
	m["averageWaitingTime"], m["averageTravelTime"],
	m["totalEnergyConsumption"], w["p90"], w["max"]))
PY

	rm -f snapshot.json
}

{
	echo "## Reference scenarios"
	echo
	echo "Values are **informational**, not a gate."
	echo
	echo "| Scenario | schema | avg waiting | avg travel | total energy (J) | waiting p90 | waiting max |"
	echo "|---|---|---|---|---|---|---|"
} >> "$OUT"

emit_row small     10 3 50   0.8 1.0 true 42
emit_row saturated 20 4 3000 3.0 1.0 true 7
