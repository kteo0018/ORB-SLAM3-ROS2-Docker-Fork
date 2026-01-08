#!/bin/bash

if [ -z "$1" ]; then
  echo "Usage: $0 <dataset_name>"
  echo "Example: $0 gz_simple_palm_path1_run1"
  exit 1
fi

DATASET_NAME="$1"
BAG_NAME="${DATASET_NAME}_bag"

python3 extract_trajectories_from_bag.py "$BAG_NAME"