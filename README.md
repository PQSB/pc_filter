# pc_filter

A ROS 2 package for filtering LiDAR point clouds and exporting the results as ROS 2 bags or filtered point cloud files.

The package provides several filtering strategies that can be applied independently or combined:

- Camera field-of-view (FOV) filtering
- Semantic segmentation filtering
- 3D object detection filtering

The generated ROS 2 bag can contain synchronized point clouds and camera images, making it suitable for visualization in RViz 2.

---

# Features

The package currently supports:

- Reading KITTI-style binary point clouds (`.bin`)
- Camera FOV filtering using calibration parameters
- Semantic filtering using SemanticKITTI labels
- Point cloud filtering using 3D object detections
- Exporting the original point cloud
- Exporting filtered point clouds
- Exporting synchronized RGB images
- Direct export of filtered point clouds to disk
- Generation of ROS 2 bags

---

# Processing Pipeline

The filtering pipeline supports:

- **3D object detections**
  - Removes points inside detected 3D bounding boxes.
  - Score threshold.
  - Maximum detection distance.
  - Class filtering.

- **Semantic segmentation**
  - Removes points belonging to selected semantic classes.
  - Maximum filtering distance.
  - Compatible with SemanticKITTI labels.

- **Camera FOV filtering**
  - Keeps only LiDAR points projected inside the camera image.

- **Dataset export**
  - ROS 2 bag generation.
  - Direct export of filtered point clouds.

The filters are applied sequentially:

```
Input Point Cloud
        │
        ▼
Camera FOV filter (optional)
        │
        ▼
Semantic Segmentation filter (optional)
        │
        ▼
3D Detection filter (optional)
        │
        ▼
ROS 2 Bag / Filtered Point Cloud
```

Each filtering stage can be independently enabled or disabled.

---

# Requirements

## Operating System

- Tested on Ubuntu 24.04

## ROS

- ROS 2 Jazzy

## Compiler

- C++17 or later

<!-- ## Required ROS packages

```bash
sudo apt install \
    ros-humble-rosbag2* \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge
```

## Required system libraries

```bash
sudo apt install \
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev
```
-->
---

# Building

Clone the repository inside your ROS 2 workspace.

```bash
cd ~/ros2_ws/src

git clone <repository_url>
```

Build the package.

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --packages-select pc_filter
```

Source the workspace.

```bash
source install/setup.bash
```

---

# Dataset Structure

The expected directory structure is:

```
├── pointclouds/
│   ├── 000000.bin
│   ├── 000001.bin
│   └── ...

├── images/
│   ├── 000000.png
│   ├── 000001.png
│   └── ...

├── detections/
│   ├── 000000.txt
│   ├── 000001.txt
│   └── ...

├── semantic_labels/
│   ├── 000000.label
│   ├── 000001.label
│   └── ...

├── timestamps.txt

└── calibration.txt
```

Only the following files are mandatory:

- Point cloud directory
- Timestamp file

The remaining files are optional depending on the enabled filters.

---

# Input File Formats

## Point Clouds

Each point cloud must be stored as a KITTI binary file.

Each point consists of:

```
float x
float y
float z
float intensity
```

---

## Semantic Labels

Semantic labels must follow the SemanticKITTI format.

Each label is stored as:

```
uint32 label
```

Only the lower 16 bits (semantic class) are used during filtering.

---

## Detection Files

Each line represents one detected object:

```
class x y z width length height yaw score
```

Example:

```
Car 10.42 1.53 -1.20 1.90 4.45 1.68 0.52 0.97
```

Objects below the configured score threshold are ignored.

---

## Timestamp File

The timestamp file contains one timestamp per line expressed in seconds.

Example:

```
0.000000
0.100000
0.200000
...
```
---

## Camera Calibration

The calibration file must contain:

1. Image width and height
2. Projection matrix **P2**
3. LiDAR-to-camera transformation matrix **Tr_velo_to_cam**

Example:

```
1242 375

P2

Tr_velo_to_cam

```

---

# Command Line Usage

```
ros2 run pc_filter offline_pc_filter [OPTIONS]
```

---

# Main Options

| Option | Description |
|----------|-------------|
| `--pc_dir` | Input point cloud directory |
| `--ts_file` | Timestamp file |
| `--pc_topic` | Point cloud topic |
| `--fov_filter` | Calibration file |
| `--img_dir` | Image directory |
| `--img_topic` | Image topic |
| `--det_dir` | Detection directory |
| `--det_topic` | Detection point cloud topic |
| `--det_score` | Detection score threshold |
| `--det_max_dist` | Maximum detection distance |
| `--seg_dir` | Semantic label directory |
| `--seg_topic` | Semantic filtered point cloud topic |
| `--seg_max_dist` | Maximum semantic filtering distance |
| `--clouds_out_dir` | Directory for filtered point clouds |
| `--rosbag_out` | Output ROS 2 bag |

---
# Output

Depending on the selected configuration, the package may generate:

- ROS 2 bag
- Original point cloud topic
- Semantic filtered point cloud topic
- Detection filtered point cloud topic
- RGB image topic
- Filtered point cloud files (`.bin`)
---

# Usage Examples

## Generate a ROS 2 bag with all supported data

```bash
ros2 run pc_filter offline_pc_filter --pc_dir /pointclouds \
    --pc_topic /original_clouds \
    --fov_filter fov_calib_file.txt \
    --seg_dir /labels --seg_classes 10,30,31,31 \
    --seg_max_dist 20 --seg_topic /seg_filtered_cloud \
    --det_dir /detections --det_classes "Car,Person,Cyclist" \
    --det_score 0.4 --det_max_dist 20 --det_topic /det_filtered_cloud \
    --img_dir /images --img_topic /img \
    --ts_file times.txt \
    --rosbag_out out_rosbag
```

## Export filtered point clouds to disk

```bash
ros2 run pc_filter offline_pc_filter --pc_dir /pointclouds
    --fov_filter fov_calib_file.txt \
    --seg_dir /labels --seg_classes 10,30,31,31 \
    --seg_max_dist 20 \
    --det_dir /detections --det_classes "Car,Person,Cyclist" \
    --det_score 0.4 --det_max_dist 20
    --clouds_out_dir out_point_clouds
```
---

# Notes

- Point cloud, image, detection and semantic files must share the same filename (without extension).

Example:

```
000154.bin
000154.png
000154.txt
000154.label
```

- The timestamp file must contain exactly one timestamp per point cloud.

- Images, detections and semantic labels must contain the same number of frames as the point cloud sequence.
---
