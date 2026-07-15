# KernelMind Apex

ROS 2 Humble robot software stack for NVIDIA Jetson (ARM64).

## Requirements

- NVIDIA Jetson running Ubuntu 22.04
- ROS 2 Humble (`ros-humble-ros-base`)
- `colcon`, `dpkg-deb` (for building)

### RealSense Cameras

This repository does not track `external/realsense_runtime/`; it is a local
runtime copy used only when present. On a new machine, install the RealSense
runtime from system packages:

```bash
./scripts/setup_realsense.sh
```

Then verify the cameras:

```bash
rs-enumerate-devices
```

---

## Running the Robot

### Standard bringup (M6)

```bash
./bringup_RM.sh
```

Launches `node_manager/bringup_all_dm_m6.launch.py` with RViz enabled.

### Glove integration bringup (M6)

```bash
./bringup_RM_glove.sh
```

Launches `node_manager/bringup_all_dm_m6_glove.launch.py` with RViz enabled.

### Override launch target

Both scripts respect `LAUNCH_PKG` and `LAUNCH_FILE` environment variables:

```bash
LAUNCH_PKG=node_manager LAUNCH_FILE=bringup_all_dm_m3.launch.py ./bringup_RM.sh
```

Extra arguments are forwarded to `ros2 launch`:

```bash
./bringup_RM.sh use_rviz:=false
```

---

## Building the .deb Package

### Automated (CI)

Pushing a version tag triggers the GitHub Actions workflow, which builds the workspace on the self-hosted runner and uploads `kernelmind-apex_<version>_arm64.deb` to the GitHub release.

```bash
git tag v1.2.3
git push origin v1.2.3
```

### Manual

**1. Build the ROS 2 workspace**

```bash
source /opt/ros/humble/setup.bash

# Create workspace layout
WS=~/ws_apex
mkdir -p "${WS}/src"
find /path/to/this/repo -name "package.xml" -exec dirname {} \; | \
    xargs -I{} ln -sfn {} "${WS}/src/"
cd "${WS}"

# Build message/description packages first (other packages depend on them)
colcon build --packages-select marvin_msgs marvin_description
source install/setup.bash

# Build remaining packages
colcon build --packages-select \
    marvin_ros_control marvin_teleop \
    gmsl_quadcam dm_gripper_py \
    bag_playback_nodes_py bag_recorder_nodes_py \
    node_manager robot_setting
```

**2. Tag the commit** (the script reads the version from `git describe`)

```bash
git tag v1.2.3
```

**3. Run the packaging script** from the workspace root (one level above the repo)

```bash
cd "${WS}"
bash /path/to/this/repo/packaging/build_deb.sh
```

Output: `${WS}/kernelmind-apex_1.2.3_arm64.deb`

---

## Installing the .deb

```bash
sudo apt install ./kernelmind-apex_1.2.3_arm64.deb
```

After installation, re-login or source the environment manually:

```bash
source /etc/profile.d/kernelmind_apex.sh
```

The installed bringup scripts are at `/opt/kernelmind/apex/`:

```bash
/opt/kernelmind/apex/bringup_RM.sh
/opt/kernelmind/apex/bringup_RM_glove.sh
```

### Uninstalling

```bash
sudo apt remove kernelmind-apex
```

---

## Package Contents

| ROS Package | Description |
|---|---|
| `marvin_msgs` | Custom message definitions |
| `marvin_description` | Robot URDF and meshes |
| `marvin_ros2_control` | Robot arm control node (C++) |
| `marvin_teleop` | Teleoperation node |
| `gmsl_quadcam` | GMSL quad-camera integration (Jetson) |
| `dm_gripper_py` | Gripper control (Python) |
| `bag_recorder_nodes_py` | ROS bag recording (MCAP) |
| `bag_playback_nodes_py` | ROS bag playback |
| `node_manager` | Top-level launch orchestration |
| `robot_setting` | Robot configuration web service (FastAPI) |
