#!/usr/bin/env python3
"""
Prepare dataset for CNN training from marked trunks
Extracts patches around trunk centers for classification training
"""

import cv2
import numpy as np
import json
import os
from pathlib import Path

def load_manual_trunks(json_path):
    """Load manually marked trunks"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    if "trunks" in data:
        return data["trunks"], data.get("metadata", {})
    else:
        return data, {}

def extract_trunk_patches(image_path, trunks, patch_size=64, num_negative=5):
    """Extract positive (trunk) and negative (non-trunk) patches"""
    image = cv2.imread(image_path)
    if image is None:
        return [], []
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    
    positive_patches = []
    negative_patches = []
    
    # Extract positive patches (trunks)
    for trunk in trunks:
        x, y = trunk["center"]
        r = trunk["radius"]
        
        # Extract patch around trunk center
        x1 = max(0, x - patch_size // 2)
        y1 = max(0, y - patch_size // 2)
        x2 = min(w, x + patch_size // 2)
        y2 = min(h, y + patch_size // 2)
        
        patch = gray[y1:y2, x1:x2]
        
        # Resize to fixed size if needed
        if patch.shape[0] != patch_size or patch.shape[1] != patch_size:
            patch = cv2.resize(patch, (patch_size, patch_size))
        
        positive_patches.append(patch)
    
    # Extract negative patches (random non-trunk regions)
    trunk_mask = np.zeros((h, w), dtype=np.uint8)
    for trunk in trunks:
        x, y = trunk["center"]
        r = trunk["radius"]
        cv2.circle(trunk_mask, (x, y), r * 2, 255, -1)  # Larger exclusion zone
    
    attempts = 0
    while len(negative_patches) < num_negative * len(trunks) and attempts < 1000:
        attempts += 1
        x = np.random.randint(patch_size // 2, w - patch_size // 2)
        y = np.random.randint(patch_size // 2, h - patch_size // 2)
        
        # Check if not in trunk region
        if trunk_mask[y, x] == 0:
            x1 = x - patch_size // 2
            y1 = y - patch_size // 2
            x2 = x + patch_size // 2
            y2 = y + patch_size // 2
            
            patch = gray[y1:y2, x1:x2]
            if patch.shape[0] == patch_size and patch.shape[1] == patch_size:
                negative_patches.append(patch)
    
    return positive_patches, negative_patches

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
    
    # Create output directories
    output_dir = "cnn_dataset"
    pos_dir = os.path.join(output_dir, "positive")
    neg_dir = os.path.join(output_dir, "negative")
    os.makedirs(pos_dir, exist_ok=True)
    os.makedirs(neg_dir, exist_ok=True)
    
    print("="*80)
    print("Preparing CNN Dataset from Marked Trunks")
    print("="*80)
    print(f"Image directory: {image_dir}")
    print(f"Output directory: {output_dir}")
    print()
    
    total_positive = 0
    total_negative = 0
    
    for img_file, trunks in manual_trunks.items():
        image_path = os.path.join(image_dir, img_file)
        if not os.path.exists(image_path):
            continue
        
        print(f"Processing {img_file}...")
        positive_patches, negative_patches = extract_trunk_patches(
            image_path, trunks, patch_size=64, num_negative=5
        )
        
        # Save positive patches
        for idx, patch in enumerate(positive_patches):
            filename = f"{os.path.splitext(img_file)[0]}_pos_{idx:03d}.png"
            cv2.imwrite(os.path.join(pos_dir, filename), patch)
            total_positive += 1
        
        # Save negative patches
        for idx, patch in enumerate(negative_patches):
            filename = f"{os.path.splitext(img_file)[0]}_neg_{idx:03d}.png"
            cv2.imwrite(os.path.join(neg_dir, filename), patch)
            total_negative += 1
        
        print(f"  Extracted {len(positive_patches)} positive, {len(negative_patches)} negative patches")
    
    print("\n" + "="*80)
    print("Dataset Summary")
    print("="*80)
    print(f"Total positive patches (trunks): {total_positive}")
    print(f"Total negative patches (non-trunks): {total_negative}")
    print(f"Positive directory: {pos_dir}")
    print(f"Negative directory: {neg_dir}")
    print()
    print("Next steps:")
    print("1. Review patches to ensure quality")
    print("2. Split into train/val/test sets (e.g., 70/15/15)")
    print("3. Train CNN classifier (see train_trunk_classifier.py)")

if __name__ == "__main__":
    main()

