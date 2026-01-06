#!/usr/bin/env python3
"""
Visualize trunk detection results for FOV 157 images with optimized parameters
"""

import cv2
import numpy as np
import json
import os

def load_manual_trunks(json_path):
    """Load manually marked trunks"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    if "trunks" in data:
        return data["trunks"], data.get("metadata", {})
    else:
        return data, {}

def load_config(config_path):
    """Load optimized configuration"""
    with open(config_path, 'r') as f:
        return json.load(f)

def detect_trunks(image_path, hough_params, radius_filter, intensity_filter):
    """Detect trunks with given parameters"""
    image = cv2.imread(image_path)
    if image is None:
        return []
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=hough_params["dp"],
        minDist=hough_params["minDist"],
        param1=hough_params["param1"],
        param2=hough_params["param2"],
        minRadius=hough_params["minR"],
        maxRadius=hough_params["maxR"]
    )
    
    if circles is None:
        return []
    
    circles = np.round(circles[0, :]).astype("int")
    detected = []
    
    radius_min, radius_max = radius_filter
    intensity_min, intensity_max = intensity_filter
    
    for (x, y, r) in circles:
        if r < radius_min or r > radius_max:
            continue
        if x - r < 0 or x + r >= gray.shape[1] or y - r < 0 or y + r >= gray.shape[0]:
            continue
        
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(mask, (x, y), r, 255, -1)
        intensity = cv2.mean(gray, mask)[0]
        
        if intensity_min <= intensity <= intensity_max:
            detected.append((x, y, r))
    
    return detected

def match_trunks(detected, manual, distance_threshold=15, radius_tolerance=3):
    """Match detected with manual"""
    matched_manual = set()
    matched_detected = set()
    matches = []
    
    for d_idx, (dx, dy, dr) in enumerate(detected):
        for m_idx, manual_trunk in enumerate(manual):
            if m_idx in matched_manual:
                continue
            
            mx, my = manual_trunk["center"]
            mr = manual_trunk["radius"]
            
            distance = np.sqrt((dx - mx)**2 + (dy - my)**2)
            
            if distance <= distance_threshold and abs(dr - mr) <= radius_tolerance:
                matched_manual.add(m_idx)
                matched_detected.add(d_idx)
                matches.append((d_idx, m_idx))
                break
    
    return matched_manual, matched_detected, matches

def visualize_results(image_dir, manual_trunks, config, output_dir="trunk_detection_results/fov157_visualizations"):
    """Visualize detection results"""
    os.makedirs(output_dir, exist_ok=True)
    
    hough_params = config["hough_params"]
    radius_filter = tuple(config["radius_filter"])
    intensity_filter = tuple(config["intensity_filter"])
    
    print("="*80)
    print("Visualizing FOV 157 Detection Results")
    print("="*80)
    print(f"Configuration: {config.get('dataset', 'unknown')}")
    print(f"Precision: {config['performance']['precision']:.1%}")
    print(f"Recall: {config['performance']['recall']:.1%}")
    print(f"False Positives: {config['performance']['false_positives']}")
    print()
    
    total_matched = 0
    total_manual = 0
    total_detected = 0
    total_fp = 0
    
    for img_file, trunks in manual_trunks.items():
        image_path = os.path.join(image_dir, img_file)
        image = cv2.imread(image_path)
        if image is None:
            continue
        
        detected = detect_trunks(image_path, hough_params, radius_filter, intensity_filter)
        matched_manual, matched_detected, matches = match_trunks(detected, trunks)
        
        # Draw results
        vis = image.copy()
        
        # Draw manual trunks in green
        for trunk in trunks:
            x, y = trunk["center"]
            r = trunk["radius"]
            cv2.circle(vis, (x, y), r, (0, 255, 0), 2)
            cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
        
        # Draw detected trunks
        for d_idx, (dx, dy, dr) in enumerate(detected):
            if d_idx in matched_detected:
                # Correct detection - cyan
                cv2.circle(vis, (dx, dy), dr, (255, 255, 0), 2)
                cv2.circle(vis, (dx, dy), 2, (255, 255, 0), -1)
            else:
                # False positive - red
                cv2.circle(vis, (dx, dy), dr, (0, 0, 255), 2)
                cv2.circle(vis, (dx, dy), 2, (0, 0, 255), -1)
        
        # Draw missed trunks in yellow
        for m_idx, trunk in enumerate(trunks):
            if m_idx not in matched_manual:
                x, y = trunk["center"]
                r = trunk["radius"]
                cv2.circle(vis, (x, y), r, (0, 255, 255), 2)
                cv2.circle(vis, (x, y), 2, (0, 255, 255), -1)
        
        # Add text overlay
        precision = len(matched_detected) / len(detected) if len(detected) > 0 else 0
        recall = len(matched_manual) / len(trunks) if len(trunks) > 0 else 0
        false_positives = len(detected) - len(matched_detected)
        missed = len(trunks) - len(matched_manual)
        
        text_lines = [
            f"Detected: {len(detected)}",
            f"Matched: {len(matched_manual)}/{len(trunks)}",
            f"Precision: {precision:.1%}",
            f"Recall: {recall:.1%}",
            f"FP: {false_positives}",
            f"Missed: {missed}"
        ]
        
        y_offset = 30
        for line in text_lines:
            cv2.putText(vis, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
            y_offset += 25
        
        # Legend
        legend_y = vis.shape[0] - 100
        cv2.putText(vis, "Green: Manual", (10, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis, "Cyan: Correct", (10, legend_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv2.putText(vis, "Red: False Positive", (10, legend_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(vis, "Yellow: Missed", (10, legend_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Save visualization
        output_path = os.path.join(output_dir, f"vis_{img_file}")
        cv2.imwrite(output_path, vis)
        
        total_matched += len(matched_manual)
        total_manual += len(trunks)
        total_detected += len(detected)
        total_fp += false_positives
        
        print(f"{img_file}: {len(detected)} detected, {len(matched_manual)}/{len(trunks)} matched, "
              f"FP: {false_positives}, Missed: {missed}, Precision: {precision:.1%}, Recall: {recall:.1%}")
    
    print("\n" + "="*80)
    print("Overall Results")
    print("="*80)
    print(f"Total detected: {total_detected}")
    print(f"Total matched: {total_matched}/{total_manual}")
    print(f"Total false positives: {total_fp}")
    print(f"Overall precision: {total_matched/total_detected:.1%}" if total_detected > 0 else "Overall precision: N/A")
    print(f"Overall recall: {total_matched/total_manual:.1%}")
    print(f"\nVisualizations saved to: {output_dir}")

def main():
    """Main function"""
    # Load manual trunks
    json_path = "trunk_detection_results/manually_marked_trunks_example_rgb_images_fov157_20251223_163237.json"
    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found")
        return
    
    manual_trunks, metadata = load_manual_trunks(json_path)
    image_dir = metadata.get("image_directory", "example_rgb_images_fov157")
    
    # Convert absolute path to relative if needed
    if os.path.isabs(image_dir):
        rel_dir = os.path.basename(image_dir)
        if os.path.exists(rel_dir):
            image_dir = rel_dir
    
    if not os.path.exists(image_dir):
        print(f"Error: Image directory '{image_dir}' not found")
        return
    
    # Load optimized config
    config_path = "trunk_detection_results/fov157_optimized_config.json"
    if not os.path.exists(config_path):
        print(f"Error: {config_path} not found. Run tune_fov157_fast.py first.")
        return
    
    config = load_config(config_path)
    
    # Visualize results
    visualize_results(image_dir, manual_trunks, config)

if __name__ == "__main__":
    main()

