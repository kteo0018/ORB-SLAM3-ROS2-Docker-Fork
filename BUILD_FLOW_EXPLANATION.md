# ORB-SLAM3 ROS2 Wrapper Build Flow Explanation

## How the ROS2 Wrapper Connects to ORB-SLAM3

### Execution Flow

```
c1_rgbd.launch.py
    ↓
    Executes: ros2 run orb_slam3_ros2_wrapper rgbd
    ↓
rgbd.cpp (main)
    ↓
    Creates: RgbdSlamNode(argv[1], argv[2], ORB_SLAM3::System::RGBD)
    ↓
rgbd-slam-node.cpp (RgbdSlamNode constructor)
    ↓
    Creates: ORBSLAM3Interface(strVocFile, strSettingsFile, sensor, ...)
    ↓
orb_slam3_interface.cpp (ORBSLAM3Interface constructor)
    ↓
    Creates: mSLAM_ = std::make_shared<ORB_SLAM3::System>(...)
    ↓
ORB_SLAM3::System (from libORB_SLAM3.so shared library)
```

### Key Connection Points

1. **ORBSLAM3Interface** (line 30 in `orb_slam3_interface.cpp`):
   ```cpp
   mSLAM_ = std::make_shared<ORB_SLAM3::System>(strVocFile_, strSettingsFile_, sensor_, bUseViewer_, loopClosing_);
   ```

2. **Tracking calls** (lines 520, 600, 663 in `orb_slam3_interface.cpp`):
   ```cpp
   Tcw = mSLAM_->TrackRGBD(cvRGB->image, cvD->image, timestamp);
   Tcw = mSLAM_->TrackMonocular(cvRGB->image, timestamp, vImuMeas);
   ```

3. **CMake Linking** (`orb_slam3_ros2_wrapper/CMakeLists.txt`):
   - Line 37: `find_package(ORB_SLAM3 REQUIRED)`
   - Line 65: `ament_target_dependencies(rgbd ... ORB_SLAM3 ...)`
   - Links against: `/home/orb/ORB_SLAM3/lib/libORB_SLAM3.so`

## Why Your Changes Don't Reflect

### The Problem

ORB-SLAM3 is built as a **shared library** (`libORB_SLAM3.so`) that gets linked into the ROS2 wrapper executable. When you modify ORB-SLAM3 source code:

1. ❌ The source code changes are **not automatically compiled**
2. ❌ The shared library (`libORB_SLAM3.so`) is **not regenerated**
3. ❌ The ROS2 wrapper executable still uses the **old library**

### Build Process

1. **ORB-SLAM3** builds to: `/home/orb/ORB_SLAM3/lib/libORB_SLAM3.so`
2. **ROS2 wrapper** links against this pre-built `.so` file
3. The wrapper executable (`rgbd`) contains a reference to the shared library

## How to Rebuild After Modifying ORB-SLAM3

### Step 1: Rebuild ORB-SLAM3 Library

```bash
cd /home/orb/ORB_SLAM3
./build.sh
```

Or manually:
```bash
cd /home/orb/ORB_SLAM3/build
make -j6
```

This regenerates `/home/orb/ORB_SLAM3/lib/libORB_SLAM3.so` with your changes.

### Step 2: Rebuild ROS2 Wrapper

```bash
cd /root/colcon_ws
source /opt/ros/humble/setup.sh
colcon build --packages-select orb_slam3_ros2_wrapper
```

Or rebuild everything:
```bash
colcon build
```

### Step 3: Source and Run

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch orb_slam3_ros2_wrapper c1_rgbd.launch.py
```

## Important Notes

### Library Location

The wrapper finds ORB-SLAM3 via `FindORB_SLAM3.cmake`:
- **Hardcoded path**: `/home/orb/ORB_SLAM3` (line 8 in `CMakeModules/FindORB_SLAM3.cmake`)
- **Library**: `/home/orb/ORB_SLAM3/lib/libORB_SLAM3.so`
- **Headers**: `/home/orb/ORB_SLAM3/include/`

### Shared Library vs Static Library

- ORB-SLAM3 builds as a **shared library** (`.so` file)
- This means the library is loaded at runtime
- If you modify ORB-SLAM3, you **must rebuild the library** before rebuilding the wrapper
- The wrapper executable will automatically use the new library if it's rebuilt

### Quick Rebuild Script

Create a script to rebuild both:

```bash
#!/bin/bash
# rebuild_orb_slam.sh

echo "Rebuilding ORB-SLAM3..."
cd /home/orb/ORB_SLAM3
./build.sh

echo "Rebuilding ROS2 wrapper..."
cd /root/colcon_ws
source /opt/ros/humble/setup.sh
colcon build --packages-select orb_slam3_ros2_wrapper

echo "Done! Source the workspace and run your launch file."
```

## Verification

To verify your changes are included:

1. **Check library timestamp**:
   ```bash
   ls -lh /home/orb/ORB_SLAM3/lib/libORB_SLAM3.so
   ```

2. **Check wrapper executable timestamp**:
   ```bash
   ls -lh /root/colcon_ws/install/orb_slam3_ros2_wrapper/lib/orb_slam3_ros2_wrapper/rgbd
   ```

3. **Add debug prints** in ORB-SLAM3 code to verify changes are active

## Common Issues

### Issue: Changes still not reflecting

**Solution**: 
- Make sure you rebuilt **both** ORB-SLAM3 and the wrapper
- Check that you're running the newly built executable (not an old one)
- Verify library paths are correct

### Issue: Library not found

**Solution**:
- Check `/home/orb/ORB_SLAM3/lib/libORB_SLAM3.so` exists
- Verify `FindORB_SLAM3.cmake` has correct path
- Ensure ORB-SLAM3 build completed successfully

### Issue: Header file changes not reflected

**Solution**:
- Header changes require rebuilding ORB-SLAM3 (to update any inline functions)
- Then rebuild the wrapper (to pick up new header definitions)

