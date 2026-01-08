#!/bin/bash

if [ -z "$1" ]; then
  echo "Usage: $0 <dataset_name>"
  echo "Example: $0 gz_simple_palm_path1_run1"
  exit 1
fi

DATASET_NAME="$1"
BAG_NAME="${DATASET_NAME}_bag"

ros2 bag record -o "$BAG_NAME" /world/z_simple_palm_plantation/model/x500_custom_0/link/rgbd_link/sensor/rgbd/image /world/z_simple_palm_plantation/model/x500_custom_0/link/rgbd_link/sensor/rgbd/depth_image /world/z_simple_palm_plantation/model/x500_custom_0/link/rgbd_link/sensor/rgbd/camera_info /model/x500_custom_0/odometry_with_covariance /world/z_simple_palm_plantation/model/x500_custom_0/link/gps_link/sensor/gps/navsat /world/z_simple_palm_plantation/model/x500_custom_0/link/imu_link/sensor/imu/imu