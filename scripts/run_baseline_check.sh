#!/usr/bin/env bash
# Baseline regression gate for the self-hosted SIH flight.
# Called from sim-flight-selfhosted.yml as:
#   wsl -d Ubuntu-22.04 bash run_baseline_check.sh <REPO_ROOT>
# Picks the newest trajectory CSV produced by run_sih_selfhosted.sh and compares
# it against tests/fixtures/trajectory_baseline.json via scripts/baseline_check.py.
# Exit code: 0 = pass/warn, 1 = FAIL (baseline regression), 2 = no CSV found.
set -uo pipefail

REPO="${1:-/mnt/d/AirSim/mission/px4-airsim-drone-sim}"
MISSION="/mnt/d/AirSim/mission"

latest=$(ls -t "$MISSION"/trajectory_*.csv 2>/dev/null | head -1) || true
if [ -z "$latest" ]; then
  echo "ERROR: no trajectory CSV found under $MISSION"
  exit 2
fi

echo "== Baseline regression check on: $latest =="
python3 "$REPO/scripts/baseline_check.py" --input "$latest"
