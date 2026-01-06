#!/usr/bin/env python3
"""
Test current detection algorithm against manually marked trunks
Analyzes which trunks are detected, missed, and false positives
"""

import cv2
import numpy as np
import json
import os

# Current detection parameters
CURRENT_PARAMS = {
    "dp": 1,
    "minDist": 20,
    "param1": 50,
    "param2": 20,
    "minR": 3,
    "maxR": 30
}

RADIUS_MIN = 6
RADIUS_MAX = 10
INTENSITY_MIN = 90
INTENSITY_MAX = 140

def detect_trunks(image_path):
    """Detect trunks with current algorithm"""
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=CURRENT_PARAMS["dp"],
        minDist=CURRENT_PARAMS["minDist"],
        param1=CURRENT_PARAMS["param1"],
        param2=CURRENT_PARAMS["param2"],
        minRadius=CURRENT_PARAMS["minR"],
        maxRadius=CURRENT_PARAMS["maxR"]
    )
    
    if circles is None:
        return [], image, gray
    
    circles = np.round(circles[0, :]).astype("int")
    
    # Apply filters
    detected = []
    for (x, y, r) in circles:
        if r < RADIUS_MIN or r > RADIUS_MAX:
            continue
        if x - r < 0 or x + r >= gray.shape[1] or y - r < 0 or y + r >= gray.shape[0]:
            continue
        
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(mask, (x, y), r, 255, -1)
        intensity = cv2.mean(gray, mask)[0]
        
        if INTENSITY_MIN <= intensity <= INTENSITY_MAX:
            detected.append((x, y, r))
    
    return detected, image, gray

def match_trunks(detected, manual, distance_threshold=15):
    """Match detected trunks with manually marked trunks"""
    matched_manual = set()
    matched_detected = set()
    matches = []
    
    for d_idx, (dx, dy, dr) in enumerate(detected):
        for m_idx, manual_trunk in enumerate(manual):
            if m_idx in matched_manual:
                continue
            
            mx, my = manual_trunk["center"]
            mr = manual_trunk["radius"]
            
            # Calculate distance between centers
            distance = np.sqrt((dx - mx)**2 + (dy - my)**2)
            
            # Match if centers are close and radii are similar
            if distance <= distance_threshold and abs(dr - mr) <= 3:
                matched_manual.add(m_idx)
                matched_detected.add(d_idx)
                matches.append({
                    "manual_idx": m_idx,
                    "detected_idx": d_idx,
                    "manual": manual_trunk,
                    "detected": (dx, dy, dr),
                    "distance": distance
                })
                break
    
    return matches, matched_manual, matched_detected

def analyze_trunk_properties(trunk, image, gray):
    """Analyze properties of a trunk"""
    x, y = trunk["center"]
    r = trunk["radius"]
    
    if x - r < 0 or x + r >= gray.shape[1] or y - r < 0 or y + r >= gray.shape[0]:
        return None
    
    mask = np.zeros(gray.shape, dtype=np.uint8)
    cv2.circle(mask, (x, y), r, 255, -1)
    
    intensity = cv2.mean(gray, mask)[0]
    intensity_std = np.std(gray[mask > 0])
    
    return {
        "intensity_mean": intensity,
        "intensity_std": intensity_std,
        "radius": r
    }

def visualize_results(image, detected, manual, matches, missed_manual, false_positives, output_path):
    """Create visualization showing detection results"""
    vis = image.copy()
    
    # Draw manually marked trunks (all)
    for idx, trunk in enumerate(manual):
        x, y = trunk["center"]
        r = trunk["radius"]
        cv2.circle(vis, (x, y), r, (255, 255, 0), 2)  # Yellow for manual
        cv2.putText(vis, f"M{idx+1}", (x + r + 3, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    
    # Draw detected trunks
    for idx, (dx, dy, dr) in enumerate(detected):
        cv2.circle(vis, (dx, dy), dr, (0, 255, 255), 2)  # Cyan for detected
        cv2.putText(vis, f"D{idx+1}", (dx + dr + 3, dy + 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    
    # Draw matched trunks (green)
    for match in matches:
        mx, my = match["manual"]["center"]
        mr = match["manual"]["radius"]
        cv2.circle(vis, (mx, my), mr, (0, 255, 0), 3)  # Green for matched
        cv2.circle(vis, (mx, my), 3, (0, 255, 0), -1)
    
    # Draw missed manual trunks (red)
    for idx in missed_manual:
        trunk = manual[idx]
        x, y = trunk["center"]
        r = trunk["radius"]
        cv2.circle(vis, (x, y), r, (0, 0, 255), 3)  # Red for missed
        cv2.circle(vis, (x, y), 3, (0, 0, 255), -1)
        cv2.putText(vis, f"MISS{idx+1}", (x + r + 3, y - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Draw false positives (magenta)
    for idx in false_positives:
        dx, dy, dr = detected[idx]
        cv2.circle(vis, (dx, dy), dr, (255, 0, 255), 2)  # Magenta for false positive
        cv2.putText(vis, f"FP{idx+1}", (dx + dr + 3, dy + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
    
    # Add legend
    legend_y = 20
    cv2.putText(vis, "Green: Correctly detected", (10, legend_y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(vis, "Red: Missed trunks", (10, legend_y + 25),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(vis, "Magenta: False positives", (10, legend_y + 50),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    cv2.putText(vis, "Yellow: Manual (all)", (10, legend_y + 75),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.putText(vis, "Cyan: Detected (all)", (10, legend_y + 100),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    cv2.imwrite(output_path, vis)

def main():
    """Main testing function"""
    # Load manually marked trunks
    json_path = "trunk_detection_results/manually_marked_trunks.json"
    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found")
        print("Please run interactive_trunk_marker.py first to mark trunks")
        return
    
    with open(json_path, 'r') as f:
        manual_trunks = json.load(f)
    
    image_dir = "example_rgb_images"
    os.makedirs("trunk_detection_results", exist_ok=True)
    
    print("="*80)
    print("Testing Detection Algorithm Against Manually Marked Trunks")
    print("="*80)
    print(f"\nDetection Parameters:")
    print(f"  HoughCircles: {CURRENT_PARAMS}")
    print(f"  Post-filter: radius {RADIUS_MIN}-{RADIUS_MAX}, intensity {INTENSITY_MIN}-{INTENSITY_MAX}")
    print("\n" + "="*80)
    
    total_manual = 0
    total_detected = 0
    total_matched = 0
    total_missed = 0
    total_false_positives = 0
    
    all_results = {}
    missed_properties = []
    
    for img_file, manual_list in manual_trunks.items():
        image_path = os.path.join(image_dir, img_file)
        
        if not os.path.exists(image_path):
            print(f"Warning: {image_path} not found, skipping")
            continue
        
        print(f"\n{img_file}:")
        print("-" * 80)
        
        # Detect trunks
        detected, image, gray = detect_trunks(image_path)
        
        # Match with manual
        matches, matched_manual, matched_detected = match_trunks(detected, manual_list)
        
        # Find missed and false positives
        missed_indices = [i for i in range(len(manual_list)) if i not in matched_manual]
        false_positive_indices = [i for i in range(len(detected)) if i not in matched_detected]
        
        # Analyze missed trunks
        for idx in missed_indices:
            trunk = manual_list[idx]
            props = analyze_trunk_properties(trunk, image, gray)
            if props:
                missed_properties.append({
                    "image": img_file,
                    "trunk": trunk,
                    "properties": props
                })
        
        # Statistics
        num_manual = len(manual_list)
        num_detected = len(detected)
        num_matched = len(matches)
        num_missed = len(missed_indices)
        num_fp = len(false_positive_indices)
        
        precision = num_matched / num_detected if num_detected > 0 else 0
        recall = num_matched / num_manual if num_manual > 0 else 0
        f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
        
        print(f"  Manual trunks: {num_manual}")
        print(f"  Detected: {num_detected}")
        print(f"  Correctly matched: {num_matched}")
        print(f"  Missed: {num_missed}")
        print(f"  False positives: {num_fp}")
        print(f"  Precision: {precision:.1%}")
        print(f"  Recall: {recall:.1%}")
        print(f"  F1 Score: {f1:.3f}")
        
        if missed_indices:
            print(f"\n  Missed trunks:")
            for idx in missed_indices:
                trunk = manual_list[idx]
                props = analyze_trunk_properties(trunk, image, gray)
                if props:
                    print(f"    #{idx+1}: center=({trunk['center'][0]:3d}, {trunk['center'][1]:3d}), "
                          f"radius={trunk['radius']:2d}, intensity={props['intensity_mean']:5.1f}")
        
        # Visualize
        output_path = f"trunk_detection_results/{img_file.replace('.jpg', '_test_results.jpg')}"
        visualize_results(image, detected, manual_list, matches, missed_indices, 
                         false_positive_indices, output_path)
        print(f"  Saved visualization to: {output_path}")
        
        # Accumulate totals
        total_manual += num_manual
        total_detected += num_detected
        total_matched += num_matched
        total_missed += num_missed
        total_false_positives += num_fp
        
        all_results[img_file] = {
            "manual": num_manual,
            "detected": num_detected,
            "matched": num_matched,
            "missed": num_missed,
            "false_positives": num_fp,
            "precision": precision,
            "recall": recall,
            "f1": f1
        }
    
    # Overall statistics
    print("\n" + "="*80)
    print("Overall Statistics")
    print("="*80)
    overall_precision = total_matched / total_detected if total_detected > 0 else 0
    overall_recall = total_matched / total_manual if total_manual > 0 else 0
    overall_f1 = 2 * (overall_precision * overall_recall) / (overall_precision + overall_recall) if (overall_precision + overall_recall) > 0 else 0
    
    print(f"Total manual trunks: {total_manual}")
    print(f"Total detected: {total_detected}")
    print(f"Total correctly matched: {total_matched}")
    print(f"Total missed: {total_missed}")
    print(f"Total false positives: {total_false_positives}")
    print(f"\nOverall Precision: {overall_precision:.1%}")
    print(f"Overall Recall: {overall_recall:.1%}")
    print(f"Overall F1 Score: {overall_f1:.3f}")
    
    # Analyze missed trunk properties
    if missed_properties:
        print("\n" + "="*80)
        print("Analysis of Missed Trunks")
        print("="*80)
        
        intensities = [p["properties"]["intensity_mean"] for p in missed_properties]
        radii = [p["properties"]["radius"] for p in missed_properties]
        
        print(f"\nMissed trunks: {len(missed_properties)}")
        print(f"  Intensity range: {min(intensities):.1f} - {max(intensities):.1f} (mean: {np.mean(intensities):.1f})")
        print(f"  Radius range: {min(radii)} - {max(radii)} (mean: {np.mean(radii):.1f})")
        
        # Count how many are outside current filters
        outside_intensity = sum(1 for p in missed_properties 
                               if not (INTENSITY_MIN <= p["properties"]["intensity_mean"] <= INTENSITY_MAX))
        outside_radius = sum(1 for p in missed_properties 
                            if not (RADIUS_MIN <= p["properties"]["radius"] <= RADIUS_MAX))
        
        print(f"\n  Outside intensity filter ({INTENSITY_MIN}-{INTENSITY_MAX}): {outside_intensity}")
        print(f"  Outside radius filter ({RADIUS_MIN}-{RADIUS_MAX}): {outside_radius}")
        
        # Show specific examples
        print(f"\n  Examples of missed trunks:")
        for i, mp in enumerate(missed_properties[:5]):  # Show first 5
            props = mp["properties"]
            trunk = mp["trunk"]
            print(f"    {mp['image']}: center=({trunk['center'][0]}, {trunk['center'][1]}), "
                  f"radius={props['radius']}, intensity={props['intensity_mean']:.1f}")
    
    # Save results
    results_path = "trunk_detection_results/detection_test_results.json"
    with open(results_path, 'w') as f:
        json.dump({
            "overall": {
                "total_manual": total_manual,
                "total_detected": total_detected,
                "total_matched": total_matched,
                "total_missed": total_missed,
                "total_false_positives": total_false_positives,
                "precision": overall_precision,
                "recall": overall_recall,
                "f1": overall_f1
            },
            "per_image": all_results,
            "missed_trunks": missed_properties
        }, f, indent=2)
    
    print(f"\nDetailed results saved to: {results_path}")
    print("\nCheck the *_test_results.jpg images to see visual comparison")

if __name__ == "__main__":
    main()

