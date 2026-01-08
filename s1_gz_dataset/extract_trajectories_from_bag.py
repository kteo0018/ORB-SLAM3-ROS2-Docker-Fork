#!/usr/bin/env python3
"""
Extract Trajectories from ROS 2 Bag File

This script extracts pose trajectories (ground truth odometry and GPS) from ROS 2 bag files
and saves them in TUM format for use with evo tools (evo_ape, evo_rpe, etc.).

Usage:
    python3 extract_trajectories_from_bag.py <bag_directory> [--output-dir OUTPUT_DIR]

Example:
    python3 extract_trajectories_from_bag.py gz_simple_palm_path1_run1

The script will:
    1. Extract ground truth odometry from /model/x500_custom_0/odometry_with_covariance
    2. Extract GPS data from /world/.../gps/navsat and convert to ENU coordinates
    3. Save trajectories in TUM format compatible with evo tools
"""

import sys
import os
import argparse
import math
from pathlib import Path
from typing import Optional, Tuple

try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
except ImportError as e:
    print(f"Error: Missing required ROS 2 packages. Please install:")
    print(f"  sudo apt-get install ros-humble-rosbag2-python")
    print(f"\nImport error: {e}")
    sys.exit(1)


def timestamp_to_float(sec: int, nanosec: int) -> float:
    """Convert ROS timestamp to float seconds."""
    return float(sec) + float(nanosec) * 1e-9


def wgs84_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    """Convert WGS84 geodetic coordinates to ECEF coordinates."""
    # WGS84 ellipsoid constants
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    N = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (N + alt_m) * cos_lat * cos_lon
    y = (N + alt_m) * cos_lat * sin_lon
    z = (N * (1.0 - e2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_enu(x: float, y: float, z: float, lat0_deg: float, lon0_deg: float, alt0_m: float) -> Tuple[float, float, float]:
    """Convert ECEF coordinates to ENU coordinates relative to reference point."""
    x0, y0, z0 = wgs84_to_ecef(lat0_deg, lon0_deg, alt0_m)
    dx = x - x0
    dy = y - y0
    dz = z - z0
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u


def geodetic_to_enu(lat_deg: float, lon_deg: float, alt_m: float, lat0_deg: float, lon0_deg: float, alt0_m: float) -> Tuple[float, float, float]:
    """Convert geodetic coordinates to ENU coordinates relative to reference point."""
    x, y, z = wgs84_to_ecef(lat_deg, lon_deg, alt_m)
    return ecef_to_enu(x, y, z, lat0_deg, lon0_deg, alt0_m)


def extract_trajectories(bag_path: str, output_dir: str, odom_topic: str, gps_topic: str):
    """
    Extract trajectories from ROS 2 bag file.
    
    Args:
        bag_path: Path to bag directory
        output_dir: Output directory for trajectory files
        odom_topic: Topic name for odometry (ground truth)
        gps_topic: Topic name for GPS
    """
    print(f"Opening bag file: {bag_path}")
    
    # Auto-detect storage format
    storage_id = 'sqlite3'
    metadata_path = Path(bag_path) / "metadata.yaml"
    if metadata_path.exists():
        try:
            with open(metadata_path, 'r') as f:
                content = f.read()
                if 'storage_identifier: mcap' in content:
                    storage_id = 'mcap'
                    print(f"Detected MCAP storage format")
                else:
                    print(f"Using SQLite3 storage format (default)")
        except Exception:
            print(f"Warning: Could not read metadata.yaml, using default SQLite3 storage")
    
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    topic_type_map = {topic.name: topic.type for topic in topic_types}
    
    print(f"Found topics: {list(topic_type_map.keys())}")
    
    # Check if topics exist
    if odom_topic not in topic_type_map:
        raise ValueError(f"Odometry topic '{odom_topic}' not found in bag file!")
    if gps_topic not in topic_type_map:
        print(f"Warning: GPS topic '{gps_topic}' not found in bag file. GPS trajectory will be skipped.")
        gps_topic = None
    
    # Get message types
    odom_msg_type = get_message(topic_type_map[odom_topic])
    if gps_topic:
        gps_msg_type = get_message(topic_type_map[gps_topic])
    
    # Prepare output files
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    gt_file = open(output_path / "ground_truth.tum", 'w')
    gps_file = open(output_path / "gps_navsat.tum", 'w') if gps_topic else None
    
    # GPS reference point (will be initialized from first GPS message)
    gps_ref: Optional[Tuple[float, float, float]] = None
    
    print("Reading messages from bag file...")
    odom_count = 0
    gps_count = 0
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == odom_topic:
            msg = deserialize_message(data, odom_msg_type)
            try:
                t = timestamp_to_float(msg.header.stamp.sec, msg.header.stamp.nanosec)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                
                # Write TUM format: timestamp x y z qx qy qz qw
                gt_file.write(f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
                odom_count += 1
                if odom_count % 100 == 0:
                    print(f"  Read {odom_count} odometry messages...")
            except Exception as e:
                print(f"Warning: Failed to process odometry message: {e}")
                continue
        
        elif gps_topic and topic == gps_topic:
            msg = deserialize_message(data, gps_msg_type)
            try:
                # Initialize GPS reference from first message
                if gps_ref is None:
                    gps_ref = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
                    print(f"Initialized GPS ENU reference: lat={gps_ref[0]:.8f}, lon={gps_ref[1]:.8f}, alt={gps_ref[2]:.3f}")
                
                # Convert to ENU
                lat = float(msg.latitude)
                lon = float(msg.longitude)
                alt = float(msg.altitude)
                e, n, u = geodetic_to_enu(lat, lon, alt, gps_ref[0], gps_ref[1], gps_ref[2])
                
                t = timestamp_to_float(msg.header.stamp.sec, msg.header.stamp.nanosec)
                
                # Write TUM format: timestamp x y z qx qy qz qw
                # For GPS, we use identity quaternion (no orientation information)
                if gps_file:
                    gps_file.write(f"{t:.9f} {e:.6f} {n:.6f} {u:.6f} 0.000000 0.000000 0.000000 1.000000\n")
                    gps_count += 1
                    if gps_count % 100 == 0:
                        print(f"  Read {gps_count} GPS messages...")
            except Exception as e:
                print(f"Warning: Failed to process GPS message: {e}")
                continue
    
    gt_file.close()
    if gps_file:
        gps_file.close()
    
    print(f"\nExtraction complete!")
    print(f"  Ground truth odometry: {odom_count} poses -> {output_path / 'ground_truth.tum'}")
    if gps_topic:
        print(f"  GPS trajectory: {gps_count} poses -> {output_path / 'gps_navsat.tum'}")


def main():
    parser = argparse.ArgumentParser(
        description="Extract trajectories from ROS 2 bag file in TUM format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument(
        'bag_directory',
        type=str,
        help='Path to bag directory (containing metadata.yaml and .db3/.mcap files)'
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Output directory for trajectory files (default: <bag_directory>_trajectories)'
    )
    
    parser.add_argument(
        '--odom-topic',
        type=str,
        default='/model/x500_custom_0/odometry_with_covariance',
        help='Topic name for odometry/ground truth (default: /model/x500_custom_0/odometry_with_covariance)'
    )
    
    parser.add_argument(
        '--gps-topic',
        type=str,
        default='/world/z_simple_palm_plantation/model/x500_custom_0/link/gps_link/sensor/gps/navsat',
        help='Topic name for GPS (default: /world/.../gps/navsat)'
    )
    
    args = parser.parse_args()
    
    # Validate bag directory exists
    bag_path = Path(args.bag_directory)
    if not bag_path.exists():
        print(f"Error: Bag directory does not exist: {bag_path}", file=sys.stderr)
        return 1
    
    if not bag_path.is_dir():
        print(f"Error: Path is not a directory: {bag_path}", file=sys.stderr)
        return 1
    
    # Determine output directory
    if args.output_dir is None:
        # Strip _bag suffix if present before appending _trajectories
        base_name = bag_path.name
        if base_name.endswith('_bag'):
            base_name = base_name[:-4]
        output_dir = bag_path.parent / f"{base_name}_trajectories"
    else:
        output_dir = Path(args.output_dir)
    
    # Initialize ROS 2
    rclpy.init()
    
    try:
        extract_trajectories(
            str(bag_path.absolute()),
            str(output_dir.absolute()),
            args.odom_topic,
            args.gps_topic
        )
        
        print(f"\n✓ Trajectories saved to: {output_dir}")
        print(f"\nThese files are in TUM format and can be used with evo tools:")
        print(f"  evo_ape ground_truth.tum CameraTrajectory.txt -va --plot")
        print(f"  evo_rpe ground_truth.tum CameraTrajectory.txt -va --plot")
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        rclpy.shutdown()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

