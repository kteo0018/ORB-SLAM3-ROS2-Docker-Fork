#!/bin/bash

if [ -z "$1" ]; then
  echo "Usage: $0 <dataset_name>"
  echo "Example: $0 gz_simple_palm_path1_run1"
  exit 1
fi

DATASET_NAME="$1"
BAG_NAME="${DATASET_NAME}_bag"

python3 bag_to_tum.py "$BAG_NAME" \
  /world/z_simple_palm_plantation/model/x500_custom_0/link/rgbd_link/sensor/rgbd/image \
  /world/z_simple_palm_plantation/model/x500_custom_0/link/rgbd_link/sensor/rgbd/depth_image