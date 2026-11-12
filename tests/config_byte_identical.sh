#!/usr/bin/env bash
# A config file and the equivalent positional command must produce a
# byte-identical snapshot - the file is only a nicer front end, not a
# different run. This runs both and diffs the snapshots.
#
# The fixture equivalent_to_positional.json mirrors `10 3 50 0.8 5 true 42`:
# <Elevator speed> is 5 on both sides because metadata.speed is recorded in the
# snapshot, so an unmatched speed would make the diff fail for a reason that has
# nothing to do with the config loader.
#
# $1 = path to the liftsim binary, $2 = path to the fixture JSON.
set -euo pipefail

BIN="$1"
FIXTURE="$2"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

"$BIN" 10 3 50 0.8 5 true 42 >/dev/null 2>&1
mv snapshot.json positional.json

"$BIN" --config "$FIXTURE" >/dev/null 2>&1
mv snapshot.json config.json

diff -q positional.json config.json
