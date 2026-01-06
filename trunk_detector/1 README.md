# Trunk Detection Tools

## Overview
Tools for marking and testing palm tree trunk detection in top-down (bird's eye) view images.

## Files

### `interactive_trunk_marker.py`
Interactive tool to mark trunk locations by clicking on images.

**Usage:**
```bash
# Default: use example_rgb_images_fov157
python3 interactive_trunk_marker.py

# Specify custom image directory
python3 interactive_trunk_marker.py --image-dir example_rgb_images_fov157

# Specify custom output location
python3 interactive_trunk_marker.py --image-dir example_rgb_images_fov157 \
    --output-dir trunk_detection_results \
    --output-name my_trunks.json
```

**Controls:**
- **Left Click**: Mark trunk center
- **Mouse Wheel** or **+/- keys**: Adjust radius
- **Right Click**: Remove last trunk
- **'s'**: Save and continue to next image
- **'c'**: Clear all trunks in current image
- **'i'**: Toggle info panel
- **'h'**: Toggle coordinate crosshair
- **'q'** or **ESC**: Quit

**Output:**
- JSON file saved to `trunk_detection_results/manually_marked_trunks_{dataset_name}_{timestamp}.json`
- Includes metadata: image directory, timestamps, dataset name

### `visualize_fov157_results.py`
Visualize detection results for FOV 157 images with optimized parameters.

**Usage:**
```bash
python3 visualize_fov157_results.py
```

**Output:**
- Visualization images saved to `trunk_detection_results/fov157_visualizations/`
- Shows correct detections (cyan), false positives (red), and missed trunks (yellow)

### `tune_fov157_fast.py`
Parameter tuning script for FOV 157 images. Finds optimal parameters prioritizing precision.

**Usage:**
```bash
python3 tune_fov157_fast.py
```

**Output:**
- Best configuration saved to `trunk_detection_results/fov157_optimized_config.json`
- Performance metrics: precision, recall, false positives

### `prepare_cnn_dataset.py`
Extract patches from marked trunks to prepare dataset for CNN training.

**Usage:**
```bash
python3 prepare_cnn_dataset.py
```

**Output:**
- `cnn_dataset/positive/` - Trunk patches (64x64)
- `cnn_dataset/negative/` - Non-trunk patches (64x64)

### `train_trunk_classifier.py`
Train a CNN classifier to verify trunk detections.

**Usage:**
```bash
# Requires PyTorch
pip install torch torchvision
python3 train_trunk_classifier.py
```

**Output:**
- `trunk_classifier_best.pth` - Best model weights
- `trunk_classifier.onnx` - ONNX model for C++ integration

## File Organization

```
trunk_detector/
├── example_rgb_images/              # Original test images
├── example_rgb_images_fov157/        # FOV 157 images
├── trunk_detection_results/          # All results organized here
│   ├── manually_marked_trunks_*.json    # Marked trunk data
│   ├── fov157_optimized_config.json     # Optimized parameters
│   └── fov157_visualizations/           # Visualization images
├── cnn_dataset/                      # CNN training dataset
│   ├── positive/                        # Trunk patches
│   └── negative/                        # Non-trunk patches
├── interactive_trunk_marker.py      # Marking tool
├── visualize_fov157_results.py    # Visualization tool for FOV 157
├── tune_fov157_fast.py            # Parameter tuning for FOV 157
├── prepare_cnn_dataset.py         # Prepare CNN dataset
├── train_trunk_classifier.py      # Train CNN classifier
├── visualize_precision_solution.py  # Legacy visualization (for old dataset)
├── IMPROVEMENT_STRATEGIES.md      # Strategies for improving detection
└── FOV157_OPTIMIZATION_SUMMARY.md # Current optimization summary
```

## JSON Format

The JSON files now include metadata:
```json
{
  "metadata": {
    "image_directory": "/path/to/images",
    "dataset_name": "example_rgb_images_fov157",
    "total_images": 16,
    "marked_images": 16,
    "total_trunks": 45,
    "created_at": "2024-12-21T12:00:00"
  },
  "trunks": {
    "001.png": [
      {"center": [100, 200], "radius": 6},
      ...
    ],
    ...
  }
}
```

