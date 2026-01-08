#!/usr/bin/env bash
set -euo pipefail

# Wrapper to run automate_slam_runs.py
# Usage: ./g9r1-run-automate-slam.sh <dataset_name> [<dataset_name2> ...] [run_count]

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <dataset_name> [<dataset_name2> ...] [run_count]" >&2
  echo "Examples:" >&2
  echo "  $0 gz_simple_palm_path1_run1" >&2
  echo "  $0 gz_simple_palm_path1_run1 3" >&2
  echo "  $0 gz_simple_palm_path1_run1 gz_simple_palm_path1_run2_noturn 5" >&2
  exit 1
fi

datasets=()
runs="5"

# Check if last arg is a number (run_count)
last_arg="${@: -1}"
if [[ "$last_arg" =~ ^[0-9]+$ ]]; then
  runs="$last_arg"
  # All args except the last are datasets
  for arg in "${@:1:$#-1}"; do
    datasets+=("$arg")
  done
else
  # All args are datasets
  datasets=("$@")
fi

script_dir="$(cd "$(dirname "$0")" && pwd)"
python3 "$script_dir/automate_slam_runs.py" --runs "$runs" "${datasets[@]}"
