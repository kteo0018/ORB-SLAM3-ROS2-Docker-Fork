#!/bin/bash

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <dataset_name>" >&2
    echo "Example: $0 gz_simple_palm_path1_run1" >&2
    exit 1
fi

dataset_name="$1"

cp ~/s1_gz_dataset/${dataset_name}_trajectories/ground_truth.tum ground_truth.tum
