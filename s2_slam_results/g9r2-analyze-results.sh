#!/usr/bin/env bash

set -euo pipefail

# Activate evo environment
source ~/python_evo_slam/evo_env/bin/activate

usage() {
	cat <<EOF
Usage: $0 [--skip-existing] --run-all
	 or: $0 [--skip-existing] run_dir1 [run_dir2 ...]

Options:
	--skip-existing   Reuse existing per_run_metrics.csv for a run instead of recomputing
	--run-all         Process all run directories under the current working directory

Examples:
	$0 --skip-existing --run-all
	$0 gz_simple_palm_path1_run1_results gz_simple_palm_path1_run2_noturn_bag_results
	$0 --skip-existing gz_simple_palm_path1_run3_noturn_stop_results
EOF
}

if [[ $# -eq 0 ]]; then
	usage >&2
	exit 1
fi

RUN_ALL=false
SKIP=()
RUNS=()

for arg in "$@"; do
	case "$arg" in
		--run-all)
			RUN_ALL=true
			;;
		--skip-existing)
			SKIP+=(--skip-existing)
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			RUNS+=("$arg")
			;;
	esac
done

if [[ "$RUN_ALL" == true ]]; then
	python3 analyze_evo_results.py --save-plots "${SKIP[@]}"
else
	if [[ ${#RUNS[@]} -eq 0 ]]; then
		usage >&2
		exit 1
	fi
	python3 analyze_evo_results.py --runs "${RUNS[@]}" --save-plots "${SKIP[@]}"
fi

