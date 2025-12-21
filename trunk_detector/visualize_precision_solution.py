#!/usr/bin/env python3
"""
Visualize the precision-optimized solution
Shows detected trunks with minimal false positives
"""

import cv2
import numpy as np
import json
import os

# Load best precision-optimized config
def load_best_config():
    """Load the best precision-optimized configuration"""
    config_path = "trunk_detection_results/precision_optimized_config.json"
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            return json.load(f)
    
    # Fallback to best found config
    return {
        "hough_params": {
            "dp": 1,
            "minDist": 15,
            "param1": 50,
            "param2": 18,
            "minR": 2,
            "maxR": 35
        },
        "radius_filter": (6, 8),
        "intensity_filter": (95, 135)
    }

# Load configuration
CONFIG = load_best_config()
FINAL_PARAMS = {
    "hough": CONFIG["hough_params"],
    "radius_filter": tuple(CONFIG["radius_filter"]),
    "intensity_filter": tuple(CONFIG["intensity_filter"]),
    "matching_distance": 15,
    "matching_radius_tolerance": 3
}

def load_manual_trunks():
    """Load manually marked trunks"""
    json_path = "trunk_detection_results/manually_marked_trunks.json"
    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found")
        return None
    with open(json_path, 'r') as f:
        return json.load(f)

def detect_trunks(image_path):
    """Detect trunks with precision-optimized parameters"""
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=FINAL_PARAMS["hough"]["dp"],
        minDist=FINAL_PARAMS["hough"]["minDist"],
        param1=FINAL_PARAMS["hough"]["param1"],
        param2=FINAL_PARAMS["hough"]["param2"],
        minRadius=FINAL_PARAMS["hough"]["minR"],
        maxRadius=FINAL_PARAMS["hough"]["maxR"]
    )
    
    if circles is None:
        return [], image, gray
    
    circles = np.round(circles[0, :]).astype("int")
    detected = []
    
    radius_min, radius_max = FINAL_PARAMS["radius_filter"]
    intensity_min, intensity_max = FINAL_PARAMS["intensity_filter"]
    
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
    
    return detected, image, gray

def match_trunks(detected, manual):
    """Match detected trunks with manual trunks"""
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
            
            if distance <= FINAL_PARAMS["matching_distance"] and abs(dr - mr) <= FINAL_PARAMS["matching_radius_tolerance"]:
                matched_manual.add(m_idx)
                matched_detected.add(d_idx)
                matches.append({
                    "manual_idx": m_idx,
                    "detected_idx": d_idx,
                    "distance": distance
                })
                break
    
    return matches, matched_manual, matched_detected

def visualize_results(image, detected, manual, matches, missed_indices, false_positive_indices, output_path):
    """Create comprehensive visualization"""
    vis = image.copy()
    h, w = vis.shape[:2]
    
    # Draw all manual trunks (yellow outline - thin)
    for idx, trunk in enumerate(manual):
        x, y = trunk["center"]
        r = trunk["radius"]
        cv2.circle(vis, (x, y), r, (0, 255, 255), 1)  # Yellow, thin
    
    # Draw all detected trunks (cyan outline - thin)
    for idx, (dx, dy, dr) in enumerate(detected):
        cv2.circle(vis, (dx, dy), dr, (255, 255, 0), 1)  # Cyan, thin
    
    # Draw matched trunks (green - thick, prominent)
    for match in matches:
        m_idx = match["manual_idx"]
        trunk = manual[m_idx]
        x, y = trunk["center"]
        r = trunk["radius"]
        cv2.circle(vis, (x, y), r, (0, 255, 0), 4)  # Green, thick
        cv2.circle(vis, (x, y), 4, (0, 255, 0), -1)
        cv2.putText(vis, f"#{m_idx+1}", (x + r + 5, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Draw missed trunks (red - medium)
    for idx in missed_indices:
        trunk = manual[idx]
        x, y = trunk["center"]
        r = trunk["radius"]
        cv2.circle(vis, (x, y), r, (0, 0, 255), 2)  # Red
        cv2.circle(vis, (x, y), 3, (0, 0, 255), -1)
        cv2.putText(vis, f"MISS#{idx+1}", (x + r + 5, y - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Draw false positives (magenta - prominent)
    for idx in false_positive_indices:
        dx, dy, dr = detected[idx]
        cv2.circle(vis, (dx, dy), dr, (255, 0, 255), 3)  # Magenta, thick
        cv2.circle(vis, (dx, dy), 3, (255, 0, 255), -1)
        cv2.putText(vis, f"FP{idx+1}", (dx + dr + 5, y + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    
    # Add legend
    legend_x = 10
    legend_y = 20
    line_height = 25
    
    # Background for legend
    cv2.rectangle(vis, (legend_x - 5, legend_y - 15), (legend_x + 280, legend_y + line_height * 5),
                 (0, 0, 0), -1)
    cv2.rectangle(vis, (legend_x - 5, legend_y - 15), (legend_x + 280, legend_y + line_height * 5),
                 (255, 255, 255), 1)
    
    cv2.putText(vis, "Legend:", (legend_x, legend_y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.circle(vis, (legend_x + 10, legend_y + line_height), 10, (0, 255, 0), -1)
    cv2.putText(vis, "Correctly detected", (legend_x + 25, legend_y + line_height + 5),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.circle(vis, (legend_x + 10, legend_y + line_height * 2), 10, (0, 0, 255), -1)
    cv2.putText(vis, "Missed trunk", (legend_x + 25, legend_y + line_height * 2 + 5),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.circle(vis, (legend_x + 10, legend_y + line_height * 3), 10, (255, 0, 255), -1)
    cv2.putText(vis, "False positive", (legend_x + 25, legend_y + line_height * 3 + 5),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.circle(vis, (legend_x + 10, legend_y + line_height * 4), 8, (0, 255, 255), 1)
    cv2.putText(vis, "Manual (all)", (legend_x + 25, legend_y + line_height * 4 + 5),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Add statistics
    stats_y = h - 80
    stats_text = f"Detected: {len(detected)} | Matched: {len(matches)}/{len(manual)} | Missed: {len(missed_indices)} | FP: {len(false_positive_indices)}"
    cv2.rectangle(vis, (10, stats_y - 5), (w - 10, stats_y + 25), (0, 0, 0), -1)
    cv2.putText(vis, stats_text, (15, stats_y + 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    cv2.imwrite(output_path, vis)

def main():
    """Main visualization function"""
    print("="*80)
    print("Precision-Optimized Solution Visualization")
    print("="*80)
    
    # Load manual trunks
    manual_trunks = load_manual_trunks()
    if manual_trunks is None:
        return
    
    image_dir = "example_rgb_images"
    os.makedirs("trunk_detection_results", exist_ok=True)
    
    print(f"\nParameters:")
    print(f"  HoughCircles: {FINAL_PARAMS['hough']}")
    print(f"  Radius filter: {FINAL_PARAMS['radius_filter']}")
    print(f"  Intensity filter: {FINAL_PARAMS['intensity_filter']}")
    print(f"  Matching distance: {FINAL_PARAMS['matching_distance']}px")
    print("\n" + "="*80)
    
    total_manual = 0
    total_detected = 0
    total_matched = 0
    total_missed = 0
    total_false_positives = 0
    
    all_results = {}
    
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
                print(f"    #{idx+1}: center=({trunk['center'][0]}, {trunk['center'][1]}), radius={trunk['radius']}")
        
        if false_positive_indices:
            print(f"\n  False positives:")
            for idx in false_positive_indices:
                dx, dy, dr = detected[idx]
                print(f"    FP#{idx+1}: center=({dx}, {dy}), radius={dr}")
        
        # Visualize
        output_path = f"trunk_detection_results/{img_file.replace('.jpg', '_precision_optimized.jpg')}"
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
    
    if overall_precision >= 0.85:
        print("\n✓ Excellent precision! Very few false positives.")
    elif overall_precision >= 0.70:
        print("\n✓ Good precision! Acceptable false positive rate.")
    else:
        print("\n⚠ Precision could be improved.")
    
    # Save results
    results_path = "trunk_detection_results/precision_optimized_visualization_results.json"
    with open(results_path, 'w') as f:
        json.dump({
            "parameters": FINAL_PARAMS,
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
            "per_image": all_results
        }, f, indent=2)
    
    print(f"\nDetailed results saved to: {results_path}")
    print("\nVisualization images saved to:")
    print("  trunk_detection_results/*_precision_optimized.jpg")
    print("\nColor coding:")
    print("  Green (thick): Correctly detected trunks")
    print("  Red: Missed trunks")
    print("  Magenta (thick): False positives")
    print("  Yellow (thin outline): All manual trunks")
    print("  Cyan (thin outline): All detected circles")

if __name__ == "__main__":
    main()

