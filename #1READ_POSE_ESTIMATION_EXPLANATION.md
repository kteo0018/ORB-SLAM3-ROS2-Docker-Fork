# ORB-SLAM3 Pose Estimation from Feature Points

This document explains how ORB-SLAM3 estimates camera pose (position and rotation) from feature points extracted from image frames.

## Overview

ORB-SLAM3 uses a **two-stage approach** for pose estimation:

1. **Initial Pose Estimation**: Uses **MLPnP (Maximum Likelihood Perspective-n-Point)** solver with RANSAC to get an initial pose estimate
2. **Pose Refinement**: Uses **Bundle Adjustment** (non-linear optimization) to refine the pose

## Key Files

- **`ORB_SLAM3/src/MLPnPsolver.cpp`**: Initial pose estimation using MLPnP algorithm
- **`ORB_SLAM3/src/Optimizer.cc`**: Pose refinement using g2o optimization framework
- **`ORB_SLAM3/src/Tracking.cc`**: Main tracking loop that calls pose estimation
- **`ORB_SLAM3/src/OptimizableTypes.cpp`**: Defines projection error edges for optimization

---

## Stage 1: Initial Pose Estimation (MLPnP)

### Location
`ORB_SLAM3/src/MLPnPsolver.cpp` - `computePose()` function (lines 356-658)

### Mathematical Formulation

The MLPnP solver estimates camera pose **T = [R | t]** from 3D-2D correspondences:
- **Input**: 3D world points **P_w** and their corresponding 2D image observations **u**
- **Output**: Camera rotation **R** (3×3) and translation **t** (3×1)

### Core Equation

The pose estimation solves for the transformation that minimizes reprojection error:

```
P_c = R * P_w + t          (Transform 3D world point to camera frame)
u_projected = π(P_c)       (Project to image plane)
error = ||u_observed - u_projected||²
```

Where:
- **P_w**: 3D point in world coordinates
- **P_c**: 3D point in camera coordinates  
- **R**: Rotation matrix (3×3)
- **t**: Translation vector (3×1)
- **π()**: Camera projection function

### Key Steps in MLPnP

1. **Bearing Vector Computation** (lines 78-80):
   ```cpp
   cv::Point3f cv_br = mpCamera->unproject(kp.pt);  // Unproject 2D to 3D ray
   cv_br /= cv_br.z;  // Normalize bearing vector
   ```

2. **Nullspace Computation** (lines 371-373):
   ```cpp
   Eigen::JacobiSVD<Eigen::MatrixXd> svd_f(f_current.transpose(), Eigen::ComputeFullV);
   nullspaces[i] = svd_f.matrixV().block(0, 1, 3, 2);  // Nullspace of bearing vector
   ```

3. **Design Matrix Construction** (lines 426-512):
   - Builds a linear system **A * x = 0** where **x** contains rotation and translation parameters
   - For non-planar case: 12 unknowns (9 rotation + 3 translation)
   - For planar case: 9 unknowns (6 rotation + 3 translation)

4. **Least Squares Solution** (lines 517-524):
   ```cpp
   AtPA = A.transpose() * A;  // Normal equations
   Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(AtPA, Eigen::ComputeFullV);
   Eigen::MatrixXd result1 = svd_A.matrixV().col(colsA - 1);  // Nullspace vector
   ```

5. **Rotation Matrix Recovery** (lines 530-637):
   - Extracts rotation matrix from the solution
   - Uses SVD to find best rotation in Frobenius sense
   - Handles planar vs non-planar cases differently

6. **Gauss-Newton Refinement** (lines 640-651):
   ```cpp
   rodrigues_t omega = rot2rodrigues(Rout);  // Convert rotation to axis-angle
   mlpnp_gn(minx, points3v, nullspaces, P, use_cov);  // Non-linear refinement
   ```

### RANSAC Integration

The MLPnP solver is wrapped in RANSAC (lines 100-223) to handle outliers:
- Randomly samples minimum set of points (6 for non-planar, 4 for planar)
- Computes pose for each sample
- Selects pose with most inliers
- Refines using all inliers

---

## Stage 2: Pose Refinement (Bundle Adjustment)

### Location
`ORB_SLAM3/src/Optimizer.cc` - `PoseOptimization()` function (lines 814-1070)

### Mathematical Formulation

Refines the pose by minimizing the sum of squared reprojection errors:

```
minimize: Σᵢ ||u_i - π(T * P_wᵢ)||²
```

Where:
- **u_i**: Observed 2D feature point
- **P_wᵢ**: 3D map point in world coordinates
- **T**: Camera pose [R | t]
- **π()**: Camera projection function

### Core Projection Equation

For monocular camera (from `G2oTypes.cc` line 170-174):

```cpp
Eigen::Vector3d Xc = Rcw[cam_idx] * Xw + tcw[cam_idx];  // Transform to camera frame
return pCamera[cam_idx]->project(Xc);  // Project to image
```

In matrix form:
```
X_c = R_cw * X_w + t_cw
u = π(X_c)
```

For pinhole camera model (`CameraModels/Pinhole.cpp`):
```
u = [fx * X_c / Z_c + cx]
    [fy * Y_c / Z_c + cy]
```

### Optimization Setup

1. **Graph Construction** (lines 829-998):
   - Creates a g2o optimizer
   - Adds pose vertex (6 DOF: 3 rotation + 3 translation)
   - Adds edges for each 3D-2D correspondence

2. **Error Computation** (`OptimizableTypes.cpp`):
   ```cpp
   // EdgeSE3ProjectXYZOnlyPose::computeError() (implicit in g2o)
   error = observed_2D - projected_2D
   ```

3. **Jacobian Computation** (lines 49-62 in `OptimizableTypes.cpp`):
   ```cpp
   // SE3 derivative matrix
   SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
              -z , 0.f, x, 0.f, 1.f, 0.f,
               y ,  -x , 0.f, 0.f, 0.f, 1.f;
   
   // Jacobian w.r.t. pose
   _jacobianOplusXi = -pCamera->projectJac(xyz_trans) * SE3deriv;
   ```

4. **Iterative Optimization** (lines 1009-1070):
   - Performs 4 optimization iterations
   - Uses Levenberg-Marquardt algorithm
   - Classifies outliers after each iteration
   - Updates pose estimate: **T_new = T_old + ΔT**

---

## Complete Pose Estimation Pipeline

### In Tracking Loop (`Tracking.cc`)

1. **Feature Matching** (line 2730):
   ```cpp
   int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);
   ```

2. **Initial Pose** (line 2739):
   ```cpp
   mCurrentFrame.SetPose(mLastFrame.GetPose());  // Use previous frame as initial guess
   ```

3. **Pose Optimization** (line 2745):
   ```cpp
   Optimizer::PoseOptimization(&mCurrentFrame);
   ```

### Alternative: MLPnP Solver (for relocalization)

When tracking is lost (`Tracking.cc` lines 3627-3745):

1. **Create MLPnP Solvers** (line 3656):
   ```cpp
   MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
   ```

2. **RANSAC Iteration** (line 3681):
   ```cpp
   MLPnPsolver* pSolver = vpMLPnPsolvers[i];
   bool bNoMore = false;
   vector<bool> vbInliers;
   int nInliers;
   Eigen::Matrix4f Tout;
   pSolver->iterate(5, bNoMore, vbInliers, nInliers, Tout);
   ```

3. **Refine with Bundle Adjustment** (line 3714):
   ```cpp
   int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
   ```

---

## Key Mathematical Equations Summary

### 1. Coordinate Transformation
```
P_camera = R_cw * P_world + t_cw
```

Where:
- **R_cw**: Rotation from world to camera frame (3×3)
- **t_cw**: Translation from world to camera frame (3×1)

### 2. Camera Projection (Pinhole Model)
```
u = fx * (X_c / Z_c) + cx
v = fy * (Y_c / Z_c) + cy
```

Where:
- **(u, v)**: Image pixel coordinates
- **(fx, fy)**: Focal lengths
- **(cx, cy)**: Principal point
- **(X_c, Y_c, Z_c)**: 3D point in camera frame

### 3. Reprojection Error
```
error = ||[u_obs, v_obs]ᵀ - π(R_cw * P_w + t_cw)||²
```

### 4. Optimization Objective
```
T* = argmin_T Σᵢ ρ(||u_i - π(T * P_wᵢ)||²)
```

Where **ρ()** is a robust kernel (Huber) to handle outliers.

### 5. Pose Update (in optimization)
```
T_new = T_old * exp(Δξ)
```

Where **Δξ** is the 6D pose increment (3 rotation + 3 translation) in Lie algebra.

---

## Important Code Locations

| Component | File | Key Function/Line |
|-----------|------|-------------------|
| MLPnP Solver | `src/MLPnPsolver.cpp` | `computePose()` (line 356) |
| Pose Optimization | `src/Optimizer.cc` | `PoseOptimization()` (line 814) |
| Projection Error | `src/OptimizableTypes.cpp` | `EdgeSE3ProjectXYZOnlyPose` |
| Camera Projection | `src/CameraModels/Pinhole.cpp` | `project()` (line 30) |
| Tracking Loop | `src/Tracking.cc` | `TrackReferenceKeyFrame()` (line 2720) |
| Feature Matching | `src/ORBmatcher.cc` | `SearchByBoW()` |

---

## Notes

1. **MLPnP** is used for initial pose estimation when tracking is lost or for relocalization
2. **Bundle Adjustment** (PoseOptimization) is used for every frame to refine the pose
3. The system uses **RANSAC** to handle outliers in feature matching
4. **Robust kernels** (Huber) are used in optimization to further reduce outlier influence
5. The pose is represented as **SE(3)** (Special Euclidean group) with 6 degrees of freedom

