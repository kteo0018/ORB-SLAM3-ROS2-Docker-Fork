#!/bin/bash

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <dataset_name> [run_count]" >&2
    echo "Example: $0 gz_simple_palm_path1_run1 5" >&2
    exit 1
fi

dataset_name="$1"
run_count="${2:-5}"

# Ensure run_count is a positive integer
if ! [[ "$run_count" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: run_count must be a positive integer" >&2
    exit 1
fi

# Run ORB-SLAM3 run_count times
for i in $(seq 1 "$run_count"); do
    echo "======================================"
    echo "Starting ORB-SLAM3 run $i of $run_count"
    echo "======================================"
    
    ~/../home/orb/ORB_SLAM3/Examples/RGB-D/rgbd_tum \
        ~/../home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt \
        ~/colcon_ws/src/orb_slam3_ros2_wrapper/params/c1_configs_x500_rgbd.yaml \
        ~/s1_gz_dataset/${dataset_name}_tum \
        ~/s1_gz_dataset/${dataset_name}_tum/associations.txt
    
    echo "======================================"
    echo "Completed ORB-SLAM3 run $i of $run_count"
    echo "======================================"
    echo ""
done

echo "All $run_count runs completed!"

