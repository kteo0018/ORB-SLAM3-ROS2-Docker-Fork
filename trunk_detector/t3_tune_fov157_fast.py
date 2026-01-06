#!/usr/bin/env python3
"""
Fast parameter tuning for FOV 157 images
Prioritize precision (minimize false positives) while maximizing correct detections
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
                break
    
    return matched_manual, matched_detected

def analyze_trunk_properties(image_dir, manual_trunks):
    """Analyze properties of manually marked trunks"""
    intensities = []
    radii = []
    
    print("Analyzing trunk properties...")
    for img_file, trunks in list(manual_trunks.items())[:5]:  # Sample first 5 images
        image_path = os.path.join(image_dir, img_file)
        image = cv2.imread(image_path)
        if image is None:
            continue
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        for trunk in trunks:
            x, y = trunk["center"]
            r = trunk["radius"]
            
            if x - r < 0 or x + r >= gray.shape[1] or y - r < 0 or y + r >= gray.shape[0]:
                continue
            
            mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.circle(mask, (x, y), r, 255, -1)
            intensity = cv2.mean(gray, mask)[0]
            
            intensities.append(intensity)
            radii.append(r)
    
    if intensities:
        return {
            "intensity_mean": np.mean(intensities),
            "intensity_std": np.std(intensities),
            "intensity_min": np.min(intensities),
            "intensity_max": np.max(intensities),
            "radius_mean": np.mean(radii),
            "radius_std": np.std(radii),
            "radius_min": np.min(radii),
            "radius_max": np.max(radii)
        }
    return None

def test_configuration(hough_params, radius_filter, intensity_filter, image_paths, manual_trunks):
    """Test a configuration"""
    total_matched = 0
    total_manual = 0
    total_detected = 0
    
    for img_file, image_path in image_paths.items():
        detected = detect_trunks(image_path, hough_params, radius_filter, intensity_filter)
        manual = manual_trunks[img_file]
        matched_manual, matched_detected = match_trunks(detected, manual)
        
        total_matched += len(matched_manual)
        total_manual += len(manual)
        total_detected += len(detected)
    
    precision = total_matched / total_detected if total_detected > 0 else 0
    recall = total_matched / total_manual if total_manual > 0 else 0
    false_positives = total_detected - total_matched
    
    return {
        "matched": total_matched,
        "manual": total_manual,
        "detected": total_detected,
        "false_positives": false_positives,
        "precision": precision,
        "recall": recall
    }

def main():
    """Main tuning function"""
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
    
    image_paths = {f: os.path.join(image_dir, f) for f in manual_trunks.keys()}
    
    print("="*80)
    print("Fast Parameter Tuning for FOV 157 Images")
    print("="*80)
    print(f"\nDataset: {metadata.get('dataset_name', 'unknown')}")
    print(f"Total trunks: {metadata.get('total_trunks', len(manual_trunks))}")
    print(f"Total images: {len(manual_trunks)}")
    
    # Analyze trunk properties
    props = analyze_trunk_properties(image_dir, manual_trunks)
    if props:
        print(f"  Radius: {props['radius_min']:.0f}-{props['radius_max']:.0f} (mean: {props['radius_mean']:.1f} ± {props['radius_std']:.1f})")
        print(f"  Intensity: {props['intensity_min']:.1f}-{props['intensity_max']:.1f} (mean: {props['intensity_mean']:.1f} ± {props['intensity_std']:.1f})")
    else:
        # Default values if analysis fails
        props = {
            "radius_mean": 8.0,
            "radius_std": 0.5,
            "radius_min": 7,
            "radius_max": 9,
            "intensity_mean": 100.0,
            "intensity_std": 20.0,
            "intensity_min": 80,
            "intensity_max": 120
        }
    
    print("\nTesting parameter combinations...")
    
    # Focused parameter sets based on analysis
    configs = []
    
    # HoughCircles variants - fewer, more targeted
    hough_configs = [
        {"name": "base", "dp": 1, "minDist": 15, "param1": 50, "param2": 15, "minR": 2, "maxR": 35},
        {"name": "selective", "dp": 1, "minDist": 15, "param1": 50, "param2": 18, "minR": 2, "maxR": 35},
        {"name": "very_selective", "dp": 1, "minDist": 15, "param1": 50, "param2": 20, "minR": 2, "maxR": 35},
        {"name": "ultra_selective", "dp": 1, "minDist": 15, "param1": 50, "param2": 22, "minR": 2, "maxR": 35},
        {"name": "low_edge", "dp": 1, "minDist": 15, "param1": 40, "param2": 15, "minR": 2, "maxR": 35},
        {"name": "low_edge_selective", "dp": 1, "minDist": 15, "param1": 40, "param2": 18, "minR": 2, "maxR": 35},
    ]
    
    # Radius filters - based on analysis
    radius_filters = [
        (int(props['radius_mean'] - props['radius_std']), int(props['radius_mean'] + props['radius_std'])),  # Mean ± 1 std
        (props['radius_min'], props['radius_max']),  # Exact range
        (7, 9),  # Common range
        (6, 10),  # Slightly wider
        (8, 9),   # Tight range
    ]
    
    # Intensity filters - based on analysis
    intensity_filters = [
        (int(props['intensity_mean'] - props['intensity_std']), int(props['intensity_mean'] + props['intensity_std'])),  # Mean ± 1 std
        (int(props['intensity_mean'] - 0.5*props['intensity_std']), int(props['intensity_mean'] + 0.5*props['intensity_std'])),  # Tighter
        (props['intensity_min'], props['intensity_max']),  # Exact range
        (int(props['intensity_mean'] - 10), int(props['intensity_mean'] + 10)),  # ±10
        (int(props['intensity_mean'] - 15), int(props['intensity_mean'] + 15)),  # ±15
    ]
    
    total_configs = len(hough_configs) * len(radius_filters) * len(intensity_filters)
    tested = 0
    
    print(f"Testing {total_configs} configurations...\n")
    
    for hough in hough_configs:
        for radius_filter in radius_filters:
            for intensity_filter in intensity_filters:
                result = test_configuration(
                    hough,
                    radius_filter,
                    intensity_filter,
                    image_paths,
                    manual_trunks
                )
                
                config_name = f"{hough['name']}_r{radius_filter[0]}-{radius_filter[1]}_i{intensity_filter[0]}-{intensity_filter[1]}"
                result["name"] = config_name
                result["hough"] = hough
                result["radius_filter"] = radius_filter
                result["intensity_filter"] = intensity_filter
                
                # Score: prioritize precision heavily
                if result["precision"] < 0.5:
                    result["score"] = result["precision"] * 0.1 + result["recall"] * 0.9
                elif result["precision"] < 0.7:
                    result["score"] = result["precision"] * 0.5 + result["recall"] * 0.5
                else:
                    result["score"] = result["precision"] * 0.7 + result["recall"] * 0.3
                
                configs.append(result)
                tested += 1
                
                if tested % 10 == 0:
                    print(f"  Progress: {tested}/{total_configs} ({tested*100//total_configs}%)")
    
    # Sort by precision first, then score
    configs.sort(key=lambda x: (x["precision"], x["score"], x["recall"]), reverse=True)
    
    print(f"\n✓ Tested {len(configs)} configurations")
    print("\n" + "="*80)
    print("Top 20 Configurations (Prioritizing Precision)")
    print("="*80)
    print(f"{'Rank':<5} {'Name':<50} {'Precision':<12} {'Recall':<10} {'Matched':<10} {'FP':<8} {'Score':<8}")
    print("-"*100)
    
    for idx, config in enumerate(configs[:20], 1):
        print(f"{idx:<5} {config['name']:<50} {config['precision']:>10.1%} {config['recall']:>8.1%} "
              f"{config['matched']:>8}/{config['manual']:<2} {config['false_positives']:>6} {config['score']:>6.3f}")
    
    # Find best configuration - prioritize precision first
    # First try to find 100% precision configs
    perfect_precision_configs = [c for c in configs if c["precision"] == 1.0 and c["detected"] > 0]
    
    if perfect_precision_configs:
        # Among perfect precision, choose highest recall
        best = max(perfect_precision_configs, key=lambda x: (x["recall"], x["matched"]))
        print(f"\nBest perfect-precision config (100% precision, 0 false positives):")
    else:
        # Fall back to high precision configs
        high_precision_configs = [c for c in configs if c["precision"] >= 0.9]
        if high_precision_configs:
            best = max(high_precision_configs, key=lambda x: (x["recall"], x["matched"]))
            print(f"\nBest high-precision config (precision >= 90%):")
        else:
            high_precision_configs = [c for c in configs if c["precision"] >= 0.7]
            if high_precision_configs:
                best = max(high_precision_configs, key=lambda x: (x["recall"], x["matched"]))
                print(f"\nBest high-precision config (precision >= 70%):")
            else:
                best = configs[0]
                print(f"\nBest overall config:")
    
    print("="*80)
    print(f"Name: {best['name']}")
    print(f"Precision: {best['precision']:.1%} ({best['matched']}/{best['detected']} correct)")
    print(f"Recall: {best['recall']:.1%} ({best['matched']}/{best['manual']} trunks detected)")
    print(f"False Positives: {best['false_positives']}")
    print(f"Score: {best['score']:.3f}")
    
    print("\n" + "="*80)
    print("Recommended Parameters for C++ Implementation")
    print("="*80)
    print("HoughCircles:")
    hough = best["hough"]
    print(f"  dp: {hough['dp']}")
    print(f"  minDist: {hough['minDist']}")
    print(f"  param1: {hough['param1']}")
    print(f"  param2: {hough['param2']}")
    print(f"  minRadius: {hough['minR']}")
    print(f"  maxRadius: {hough['maxR']}")
    print(f"\nPost-filters:")
    print(f"  minTrunkRadius: {best['radius_filter'][0]}")
    print(f"  maxTrunkRadius: {best['radius_filter'][1]}")
    print(f"  minIntensity: {best['intensity_filter'][0]}")
    print(f"  maxIntensity: {best['intensity_filter'][1]}")
    
    # Per-image breakdown
    print("\n" + "="*80)
    print("Per-Image Results with Best Configuration")
    print("="*80)
    
    for img_file, image_path in list(image_paths.items())[:5]:  # Show first 5
        detected = detect_trunks(image_path, best["hough"], best["radius_filter"], best["intensity_filter"])
        manual = manual_trunks[img_file]
        matched_manual, matched_detected = match_trunks(detected, manual)
        
        precision = len(matched_manual) / len(detected) if len(detected) > 0 else 0
        recall = len(matched_manual) / len(manual) if len(manual) > 0 else 0
        false_positives = len(detected) - len(matched_manual)
        
        print(f"{img_file}: {len(detected)} detected, {len(matched_manual)}/{len(manual)} matched, "
              f"FP: {false_positives}, Precision: {precision:.1%}, Recall: {recall:.1%}")
    
    # Save best config
    best_config = {
        "dataset": metadata.get("dataset_name", "example_rgb_images_fov157"),
        "hough_params": {
            "dp": hough['dp'],
            "minDist": hough['minDist'],
            "param1": hough['param1'],
            "param2": hough['param2'],
            "minR": hough['minR'],
            "maxR": hough['maxR']
        },
        "radius_filter": list(best["radius_filter"]),
        "intensity_filter": list(best["intensity_filter"]),
        "performance": {
            "precision": best["precision"],
            "recall": best["recall"],
            "matched": best["matched"],
            "manual": best["manual"],
            "detected": best["detected"],
            "false_positives": best["false_positives"],
            "score": best["score"]
        }
    }
    
    results_path = "trunk_detection_results/fov157_optimized_config.json"
    with open(results_path, 'w') as f:
        json.dump(best_config, f, indent=2)
    
    print(f"\n✓ Best configuration saved to: {results_path}")

if __name__ == "__main__":
    main()

