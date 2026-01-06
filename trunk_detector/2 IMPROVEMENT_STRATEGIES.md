# Trunk Detection Improvement Strategies

## Current Performance
- **Precision**: 100.0% (0 false positives)
- **Recall**: 48.8% (40/82 trunks detected)
- **Strategy**: Prioritize precision over recall

## Improvement Approaches

### 1. CNN-Based Approaches (Highest Potential)

#### A. Semantic Segmentation for Trunk Masking
**Concept**: Train a CNN to segment trunk regions, then apply HoughCircles only on masked areas.

**Advantages**:
- Can detect trunks that HoughCircles misses (low contrast, partial occlusion)
- Reduces false positives by focusing on trunk-like regions
- Can handle varying lighting conditions

**Implementation**:
```python
# Pseudo-code approach
1. Train U-Net or DeepLab for semantic segmentation
   - Input: RGB image
   - Output: Binary mask (trunk vs non-trunk pixels)
   
2. Apply mask to grayscale image
   masked_image = gray * trunk_mask
   
3. Run HoughCircles on masked image
   - More focused detection
   - Better recall for low-contrast trunks
   
4. Post-process with intensity/radius filters
```

**Requirements**:
- Labeled dataset (can use interactive_trunk_marker.py output)
- Training infrastructure (PyTorch/TensorFlow)
- Inference integration in C++ (ONNX Runtime, TensorRT, or LibTorch)

**Expected Improvement**:
- Recall: 48.8% → 70-85% (detect more trunks)
- Precision: 100% → 95-98% (some false positives from segmentation)

#### B. CNN Classification for Verification
**Concept**: Use CNN to verify if detected circles are actual trunks.

**Advantages**:
- Can relax HoughCircles parameters (increase recall)
- CNN filters false positives
- Maintains high precision

**Implementation**:
```python
# Two-stage approach
1. Stage 1: HoughCircles with relaxed parameters
   - Lower param2 (more detections)
   - Wider radius range
   - Result: More candidates (higher recall, lower precision)
   
2. Stage 2: CNN classifier for each candidate
   - Input: 32x32 patch around circle center
   - Output: Binary classification (trunk vs non-trunk)
   - Filter out false positives
   
3. Final result: High recall + high precision
```

**Requirements**:
- Smaller dataset (just patches, not full images)
- Faster training and inference
- Easier C++ integration

**Expected Improvement**:
- Recall: 48.8% → 65-75%
- Precision: 100% → 95-98%

#### C. Object Detection (YOLO/Faster R-CNN)
**Concept**: Direct trunk detection using object detection models.

**Advantages**:
- End-to-end detection
- Can detect multiple trunks in one pass
- State-of-the-art performance

**Disadvantages**:
- Requires more training data
- Slower inference
- Less precise localization than HoughCircles

**Expected Improvement**:
- Recall: 48.8% → 75-90%
- Precision: 100% → 90-95%

### 2. Traditional Computer Vision Improvements

#### A. Multi-Scale Detection
**Concept**: Run detection at multiple image scales to catch trunks of varying sizes.

**Implementation**:
```cpp
// In TrunkExtractor.cc
void DetectCircularTrunks(const cv::Mat& image, std::vector<cv::Vec3f>& circles)
{
    circles.clear();
    
    // Detect at multiple scales
    std::vector<float> scales = {0.8f, 1.0f, 1.2f, 1.5f};
    
    for(float scale : scales)
    {
        cv::Mat scaled;
        cv::resize(image, scaled, cv::Size(), scale, scale);
        
        // Run HoughCircles on scaled image
        std::vector<cv::Vec3f> scaledCircles;
        // ... detection code ...
        
        // Scale back to original coordinates
        for(auto& circle : scaledCircles)
        {
            circle[0] /= scale;  // x
            circle[1] /= scale;  // y
            circle[2] /= scale;  // radius
        }
        
        circles.insert(circles.end(), scaledCircles.begin(), scaledCircles.end());
    }
    
    // Remove duplicates (NMS)
    // Apply filters
}
```

**Expected Improvement**:
- Recall: +5-10% (catch trunks at different scales)

#### B. Adaptive Thresholding
**Concept**: Use adaptive intensity thresholds based on local image statistics.

**Implementation**:
```cpp
// Instead of fixed intensity range (120-129)
// Use adaptive threshold based on local mean/std

cv::Mat localMean, localStd;
cv::blur(gray, localMean, cv::Size(31, 31));
cv::Mat localVar;
cv::blur(gray.mul(gray), localVar, cv::Size(31, 31));
localVar = localVar - localMean.mul(localMean);
cv::sqrt(localVar, localStd);

// Adaptive threshold: mean - k*std to mean + k*std
float k = 1.5f;
float minIntensity = localMean.at<float>(y, x) - k * localStd.at<float>(y, x);
float maxIntensity = localMean.at<float>(y, x) + k * localStd.at<float>(y, x);
```

**Expected Improvement**:
- Recall: +3-8% (better handling of varying lighting)

#### C. Histogram Equalization / CLAHE
**Concept**: Enhance contrast to make trunks more visible.

**Implementation**:
```cpp
cv::Mat enhanced;
cv::createCLAHE(2.0, cv::Size(8, 8))->apply(gray, enhanced);
// Then run HoughCircles on enhanced image
```

**Expected Improvement**:
- Recall: +2-5%

### 3. SLAM-Specific Improvements

#### A. Temporal Consistency
**Concept**: Track trunks across frames using SLAM map.

**Advantages**:
- Trunks detected in previous frames help predict locations
- Can fill in missed detections
- Reduces false positives (trunks should persist)

**Implementation**:
```cpp
// In Frame class
std::map<int, cv::Point2f> mTrunkLocations;  // MapPoint ID -> trunk center

void Frame::MarkTrunkFeatures(const cv::Mat &im)
{
    // 1. Detect trunks in current frame
    // 2. Match with previous frame trunks (using SLAM tracking)
    // 3. Predict trunk locations from previous detections
    // 4. Boost detection near predicted locations
}
```

**Expected Improvement**:
- Recall: +5-15% (tracking helps catch missed detections)
- Precision: Maintained (temporal consistency filters false positives)

#### B. Geometric Constraints
**Concept**: Use known trunk spacing/patterns in palm plantations.

**Advantages**:
- Regular grid patterns in plantations
- Can predict trunk locations
- Validate detections against expected patterns

**Expected Improvement**:
- Recall: +10-20% (pattern-based prediction)

### 4. Hybrid Approaches (Recommended)

#### Best Combination for Maximum Improvement:
1. **CNN Segmentation Mask** → Focus detection on trunk regions
2. **Multi-scale HoughCircles** → Catch varying sizes
3. **CNN Verification** → Filter false positives
4. **Temporal Tracking** → Use SLAM map for consistency

**Expected Combined Improvement**:
- Recall: 48.8% → **80-90%**
- Precision: 100% → **95-98%**

## Implementation Priority

### Phase 1: Quick Wins (1-2 weeks)
1. Multi-scale detection
2. CLAHE contrast enhancement
3. Adaptive intensity thresholds

**Expected**: Recall 48.8% → 60-65%

### Phase 2: CNN Integration (1-2 months)
1. Train CNN classifier for verification
2. Integrate ONNX Runtime in C++
3. Two-stage detection (HoughCircles + CNN)

**Expected**: Recall 60-65% → 75-80%, Precision 100% → 95-98%

### Phase 3: Advanced (3-6 months)
1. Semantic segmentation for masking
2. Temporal tracking with SLAM map
3. Pattern-based prediction

**Expected**: Recall 75-80% → 85-90%, Precision 95-98%

## CNN Implementation Details

### Option 1: ONNX Runtime (Recommended for C++)
- Export PyTorch/TensorFlow model to ONNX
- Use ONNX Runtime C++ API
- Lightweight, fast inference
- Cross-platform

### Option 2: TensorRT (NVIDIA GPUs)
- Optimized for NVIDIA hardware
- Fastest inference
- Requires CUDA

### Option 3: LibTorch (PyTorch C++)
- Direct PyTorch C++ API
- More flexible
- Larger binary size

### Option 4: OpenCV DNN
- Built into OpenCV
- Supports ONNX, TensorFlow, Caffe
- Easy integration
- Moderate performance

## Dataset Requirements

### For Classification (Easier):
- ~500-1000 trunk patches (32x32 or 64x64)
- ~500-1000 non-trunk patches
- Can extract from existing marked trunks

### For Segmentation (More Complex):
- ~100-200 full images with pixel-level labels
- Use interactive_trunk_marker.py output as starting point
- Expand to pixel-level masks

## Next Steps

1. **Start with Phase 1** (multi-scale, CLAHE) - immediate improvement
2. **Collect/Prepare Dataset** - extract patches from marked trunks
3. **Train Simple CNN** - binary classifier (trunk vs non-trunk)
4. **Integrate ONNX Runtime** - add to ORB-SLAM3 build
5. **Two-Stage Detection** - HoughCircles + CNN verification

Would you like me to:
1. Implement Phase 1 improvements (multi-scale, CLAHE)?
2. Create a CNN training script for trunk classification?
3. Set up ONNX Runtime integration in ORB-SLAM3?

