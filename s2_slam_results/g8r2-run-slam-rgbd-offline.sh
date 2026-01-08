#!/bin/bash

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <dataset_name>" >&2
    echo "Example: $0 gz_simple_palm_path1_run1" >&2
    exit 1
fi

dataset_name="$1"

~/../home/orb/ORB_SLAM3/Examples/RGB-D/rgbd_tum \
    ~/../home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    ~/colcon_ws/src/orb_slam3_ros2_wrapper/params/c1_configs_x500_rgbd.yaml \
    ~/s1_gz_dataset/${dataset_name}_tum \
    ~/s1_gz_dataset/${dataset_name}_tum/associations.txt
