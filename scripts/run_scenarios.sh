#!/usr/bin/env bash
# Run every scenario (or the ones given) unattended and print a pass/fail table.
#
#   scripts/run_scenarios.sh                                  # all files in config/scenarios
#   scripts/run_scenarios.sh town03_route_empty town03_parked_car
#   LOCALIZATION=slam_mapping scripts/run_scenarios.sh        # same suite with real localization
#
# Needs a running CARLA server and a sourced workspace. Results: ~/av_results/*.json
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
SCEN_DIR="${HERE}/planning_bringup/config/scenarios"
LOCALIZATION="${LOCALIZATION:-ground_truth}"
RESULTS="${HOME}/av_results"
mkdir -p "${RESULTS}"

if [[ $# -gt 0 ]]; then
  NAMES=("$@")
else
  NAMES=()
  for f in "${SCEN_DIR}"/*.yaml; do NAMES+=("$(basename "${f}" .yaml)"); done
fi

START=$(date +%s)
for name in "${NAMES[@]}"; do
  echo "=== ${name} (localization: ${LOCALIZATION}) ==="
  # Hard wall-clock limit in case something hangs; the monitor normally ends the run itself.
  timeout 900 ros2 launch planning_bringup carla.launch.py \
    scenario:="${name}" localization:="${LOCALIZATION}" monitor:=true
done

echo
echo "Results since this run started:"
python3 - "$RESULTS" "$START" <<'PY'
import glob, json, os, sys
results, start = sys.argv[1], float(sys.argv[2])
rows = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(results, "*.json"))) if os.path.getmtime(f) >= start]
for r in rows:
    print(f"{r['outcome']:5}  {r['scenario']:30}  {r['reason']:35}  {r['sim_time_s']:7.1f} s  {r['distance_m']:7.1f} m")
print(f"{sum(r['outcome'] == 'PASS' for r in rows)}/{len(rows)} passed")
PY
