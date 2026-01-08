#!/usr/bin/env python3
"""
ROS 2 Bag to TUM RGB-D Dataset Converter

This script converts ROS 2 bag files (.db3 or .mcap) to the TUM RGB-D dataset format
for running ORB-SLAM3 in offline "Dataset Mode" for accurate benchmarking.

Usage:
    python3 bag_to_tum.py <bag_directory> <rgb_topic> <depth_topic> [output_dir]

Example:
    python3 bag_to_tum.py gz_simple_palm_path1_run1 \\
        /world/z_simple_palm_plantation/model/x500_custom_0/link/rgbd_link/sensor/rgbd/image \\
        /world/z_simple_palm_plantation/model/x500_custom_0/link/rgbd_link/sensor/rgbd/depth_image

The script will:
    1. Extract RGB and depth images from the bag file
    2. Save RGB images as BGR (OpenCV standard) in rgb/ directory
    3. Save depth images as 16-bit PNG in depth/ directory
    4. Create associations.txt file linking RGB and depth images by closest timestamp
"""

import sys
import os
import argparse
import numpy as np
import cv2
from pathlib import Path
from collections import namedtuple

try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
    from cv_bridge import CvBridge
except ImportError as e:
    print(f"Error: Missing required ROS 2 packages. Please install:")
    print(f"  sudo apt-get install ros-humble-rosbag2-python ros-humble-cv-bridge python3-numpy python3-opencv")
    print(f"\nImport error: {e}")
    sys.exit(1)


# Image message data structure
ImageData = namedtuple('ImageData', ['timestamp', 'image', 'filename'])


def timestamp_to_float(nanoseconds):
    """Convert ROS timestamp (nanoseconds) to float seconds."""
    return nanoseconds / 1e9


def find_closest_match(target_time, candidate_list, threshold=0.02):
    """
    Find the closest timestamp match within a threshold.
    
    Args:
        target_time: Target timestamp in seconds (float)
        candidate_list: List of (timestamp, data) tuples
        threshold: Maximum time difference in seconds (default: 0.02s)
    
    Returns:
        Index of closest match or None if no match within threshold
    """
    if not candidate_list:
        return None
    
    # Find closest match using binary search
    times = [item[0] for item in candidate_list]
    idx = np.searchsorted(times, target_time)
    
    # Check both neighbors
    candidates = []
    if idx > 0:
        candidates.append((idx - 1, abs(times[idx - 1] - target_time)))
    if idx < len(times):
        candidates.append((idx, abs(times[idx] - target_time)))
    
    if not candidates:
        return None
    
    # Get the closest one
    best_idx, best_diff = min(candidates, key=lambda x: x[1])
    
    if best_diff <= threshold:
        return best_idx
    return None


def read_bag_images(bag_path, rgb_topic, depth_topic):
    """
    Read RGB and depth images from ROS 2 bag file.
    
    Args:
        bag_path: Path to bag directory (containing metadata.yaml and .db3/.mcap files)
        rgb_topic: Topic name for RGB images
        depth_topic: Topic name for depth images
    
    Returns:
        Tuple of (rgb_images, depth_images) where each is a list of ImageData
    """
    print(f"Opening bag file: {bag_path}")
    
    # Try to auto-detect storage format, default to sqlite3
    storage_id = 'sqlite3'
    metadata_path = Path(bag_path) / "metadata.yaml"
    if metadata_path.exists():
        try:
            # Simple string-based detection to avoid requiring PyYAML
            with open(metadata_path, 'r') as f:
                content = f.read()
                if 'storage_identifier: mcap' in content or 'storage_identifier: mcap\n' in content:
                    storage_id = 'mcap'
                    print(f"Detected MCAP storage format")
                else:
                    print(f"Using SQLite3 storage format (default)")
        except Exception:
            print(f"Warning: Could not read metadata.yaml, using default SQLite3 storage")
    else:
        print(f"Warning: metadata.yaml not found, using default SQLite3 storage")
    
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
    
    if rgb_topic not in topic_type_map:
        raise ValueError(f"RGB topic '{rgb_topic}' not found in bag file!")
    if depth_topic not in topic_type_map:
        raise ValueError(f"Depth topic '{depth_topic}' not found in bag file!")
    
    # Get message types
    rgb_msg_type = get_message(topic_type_map[rgb_topic])
    depth_msg_type = get_message(topic_type_map[depth_topic])
    
    bridge = CvBridge()
    rgb_images = []
    depth_images = []
    
    print("Reading messages from bag file...")
    msg_count = 0
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == rgb_topic:
            msg = deserialize_message(data, rgb_msg_type)
            try:
                # Convert ROS image message to OpenCV image (BGR format)
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if cv_image is None or cv_image.size == 0:
                    print(f"Warning: Empty RGB image at timestamp {timestamp}, skipping")
                    continue
                # Extract timestamp from message header (more accurate than bag timestamp)
                ts_float = timestamp_to_float(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
                rgb_images.append(ImageData(ts_float, cv_image, None))
                msg_count += 1
                if msg_count % 100 == 0:
                    print(f"  Read {msg_count} RGB images...")
            except Exception as e:
                print(f"Warning: Failed to convert RGB image at timestamp {timestamp}: {e}")
                continue
        
        elif topic == depth_topic:
            msg = deserialize_message(data, depth_msg_type)
            try:
                # Convert ROS depth image to OpenCV (16-bit, unchanged)
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if cv_image is None or cv_image.size == 0:
                    print(f"Warning: Empty depth image at timestamp {timestamp}, skipping")
                    continue
                # Ensure it's 16-bit
                if cv_image.dtype != np.uint16:
                    if cv_image.dtype == np.float32 or cv_image.dtype == np.float64:
                        # Convert float depth (meters) to uint16 (millimeters)
                        cv_image = (cv_image * 1000).astype(np.uint16)
                    else:
                        cv_image = cv_image.astype(np.uint16)
                
                # Extract timestamp from message header (more accurate than bag timestamp)
                ts_float = timestamp_to_float(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
                depth_images.append(ImageData(ts_float, cv_image, None))
            except Exception as e:
                print(f"Warning: Failed to convert depth image at timestamp {timestamp}: {e}")
                continue
    
    print(f"Total RGB images: {len(rgb_images)}")
    print(f"Total depth images: {len(depth_images)}")
    
    # Sort by timestamp
    rgb_images.sort(key=lambda x: x.timestamp)
    depth_images.sort(key=lambda x: x.timestamp)
    
    return rgb_images, depth_images


def save_images_and_create_associations(rgb_images, depth_images, output_dir, time_threshold=0.02):
    """
    Save images and create associations.txt file.
    
    Args:
        rgb_images: List of ImageData for RGB images
        depth_images: List of ImageData for depth images
        output_dir: Output directory path
        time_threshold: Maximum time difference for matching (seconds)
    """
    output_path = Path(output_dir)
    rgb_dir = output_path / "rgb"
    depth_dir = output_path / "depth"
    
    rgb_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\nSaving images to: {output_dir}")
    print(f"RGB directory: {rgb_dir}")
    print(f"Depth directory: {depth_dir}")
    
    # Prepare depth list for matching (timestamp, index)
    depth_list = [(img.timestamp, idx) for idx, img in enumerate(depth_images)]
    
    associations = []
    matched_depth_indices = set()
    
    # Match RGB images to closest depth images
    print("\nMatching RGB and depth images...")
    for rgb_idx, rgb_data in enumerate(rgb_images):
        # Find closest depth image
        depth_match_idx = find_closest_match(rgb_data.timestamp, depth_list, time_threshold)
        
        if depth_match_idx is None:
            print(f"Warning: No depth match found for RGB image at {rgb_data.timestamp:.6f}s (threshold: {time_threshold}s)")
            continue
        
        depth_list_idx, depth_data_idx = depth_list[depth_match_idx]
        depth_data = depth_images[depth_data_idx]
        
        # Skip if depth image already matched (prefer earlier RGB matches)
        if depth_data_idx in matched_depth_indices:
            continue
        
        matched_depth_indices.add(depth_data_idx)
        
        # Create filenames based on RGB timestamp
        timestamp_str = f"{rgb_data.timestamp:.6f}"
        rgb_filename = f"{timestamp_str}.png"
        depth_filename = f"{timestamp_str}.png"
        
        # Save RGB image (BGR format, standard OpenCV)
        rgb_filepath = rgb_dir / rgb_filename
        cv2.imwrite(str(rgb_filepath), rgb_data.image)
        
        # Save depth image (16-bit PNG)
        depth_filepath = depth_dir / depth_filename
        cv2.imwrite(str(depth_filepath), depth_data.image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        
        # Add to associations
        associations.append((
            rgb_data.timestamp,
            f"rgb/{rgb_filename}",
            depth_data.timestamp,
            f"depth/{depth_filename}"
        ))
        
        if (rgb_idx + 1) % 100 == 0:
            print(f"  Matched and saved {rgb_idx + 1} image pairs...")
    
    print(f"\nMatched {len(associations)} image pairs")
    
    # Write associations.txt file
    associations_file = output_path / "associations.txt"
    print(f"\nWriting associations file: {associations_file}")
    
    with open(associations_file, 'w') as f:
        for ts_rgb, filename_rgb, ts_depth, filename_depth in associations:
            f.write(f"{ts_rgb:.6f} {filename_rgb} {ts_depth:.6f} {filename_depth}\n")
    
    print(f"Successfully created {len(associations)} associations")
    print(f"\nOutput directory structure:")
    print(f"  {output_dir}/")
    print(f"    rgb/           ({len(associations)} images)")
    print(f"    depth/         ({len(associations)} images)")
    print(f"    associations.txt")


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS 2 bag file to TUM RGB-D dataset format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument(
        'bag_directory',
        type=str,
        help='Path to bag directory (containing metadata.yaml and .db3/.mcap files)'
    )
    
    parser.add_argument(
        'rgb_topic',
        type=str,
        help='Topic name for RGB images (e.g., /world/.../image)'
    )
    
    parser.add_argument(
        'depth_topic',
        type=str,
        help='Topic name for depth images (e.g., /world/.../depth_image)'
    )
    
    parser.add_argument(
        'output_dir',
        type=str,
        nargs='?',
        default=None,
        help='Output directory (default: <bag_directory>_tum)'
    )
    
    parser.add_argument(
        '--time-threshold',
        type=float,
        default=0.02,
        help='Maximum time difference for RGB-depth matching in seconds (default: 0.02)'
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
    
    # Initialize ROS 2
    rclpy.init()
    
    try:
        # Determine output directory
        if args.output_dir is None:
            # Strip _bag suffix if present before appending _tum
            base_name = bag_path.name
            if base_name.endswith('_bag'):
                base_name = base_name[:-4]
            output_dir = bag_path.parent / f"{base_name}_tum"
        else:
            output_dir = Path(args.output_dir)
        
        # Read images from bag
        rgb_images, depth_images = read_bag_images(
            str(bag_path.absolute()),
            args.rgb_topic,
            args.depth_topic
        )
        
        if len(rgb_images) == 0:
            print("Error: No RGB images found!")
            return 1
        
        if len(depth_images) == 0:
            print("Error: No depth images found!")
            return 1
        
        # Save images and create associations
        save_images_and_create_associations(
            rgb_images,
            depth_images,
            str(output_dir.absolute()),
            args.time_threshold
        )
        
        print(f"\n✓ Conversion complete!")
        print(f"  Dataset saved to: {output_dir}")
        print(f"\nTo run ORB-SLAM3 with this dataset:")
        print(f"  ./rgbd_tum <vocabulary_file> <settings_file> {output_dir} {output_dir}/associations.txt")
        
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

