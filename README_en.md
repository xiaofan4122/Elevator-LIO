# Elevator-LIO

[Chinese](README.md) | [English](README_en.md)

Elevator-LIO is a LiDAR-inertial odometry system for multi-floor navigation under elevator-induced non-inertial motion. The main test platform is Livox MID-360, and the code also supports LiDARs such as Ouster, Velodyne, and XT32, but LiDARs other than MID-360 have not yet been tested at large scale.

[![Project Page](https://img.shields.io/badge/Project-Page-blue)](https://xiaofan4122.github.io/Elevator_LIO_Page/)
[![arXiv](https://img.shields.io/badge/arXiv-2605.24495-b31b1b)](https://arxiv.org/abs/2605.24495)
[![Dataset](https://img.shields.io/badge/HuggingFace-Dataset-ffcc00)](https://huggingface.co/datasets/xiaofan0100/Elevator-LIO-Dataset)

Dataset download and usage notes are available in [DATASET.md](DATASET.md).

When elevator mode is disabled in YAML, Elevator-LIO can be used as a regular LIO system while retaining trigger-based updates and adaptive downsampling.

<p align="center">
  <img src="docs/images/background.jpg" alt="Elevator-LIO overview" width="100%">
</p>

> [!WARNING]
> Elevator-LIO allows the robot to move freely inside an elevator, but we recommend using a wide-FOV LiDAR such as Livox MID-360 with a tilted mounting angle to obtain geometric constraints from as many directions as possible. If your LiDAR is mounted horizontally and the robot barely moves vertically inside the elevator, set `elevator.strong_prior_enable` under `yaml/runtime` to `true`. This option strongly interprets vertical IMU acceleration as elevator acceleration, allowing the system to work under this mounting condition.

## Timeline

- **2026-05-23**: [arXiv preprint](https://arxiv.org/abs/2605.24495) released.
- **2026-05-26**: Public announcement on [Xiaohongshu / RedNote](http://xhslink.com/o/9MTzcbzjGaQ).
- **2026-06-20**: [Elevator-LIO dataset](https://huggingface.co/datasets/xiaofan0100/Elevator-LIO-Dataset) released, including 20 real-world sequences and 79 elevator rides.
- **2026-06-26**: ROS 1 source code released.
- **Planned**: ROS 2 source code release.
- **Planned**: More dataset releases, including additional complete sequences with images.
- **Planned**: Source code and documentation for the handheld data-collection platform.

## Scenarios

![Elevator-LIO multi-floor and elevator scenarios](docs/images/Scenarios.png)

## Method Overview

Conventional LIO assumes an inertial navigation frame. Inside a moving elevator cabin, however, the IMU senses the elevator motion while the LiDAR mainly observes relative cabin geometry, which breaks the assumptions of most existing LIO systems. Elevator-LIO decouples robot motion relative to the elevator from elevator motion itself, and uses a mode-aware iterated error-state Kalman filter for continuous multi-floor localization. Standard LIO propagation is used in normal indoor environments, while non-inertial state propagation and constraints are enabled after entering an elevator.

![Elevator-LIO system overview](docs/images/system_overview.png)

The system processes IMU and LiDAR data in timestamp order, including static IMU initialization, adaptive downsampling, mode-dependent propagation, IESKF LiDAR update, optional elevator-exit update, and incremental ikd-tree mapping.

### Elevator Mode Management

Entry detection uses LiDAR range statistics to determine whether the robot has moved from an open area into a closed elevator cabin. Exit detection uses the estimated vertical elevator motion state and its covariance to determine whether the elevator has stopped. Both events can also be triggered manually through ROS topics.

<p align="center">
  <img src="docs/images/elevator_mode_manager.png" alt="Elevator mode entry and exit detection" width="70%">
</p>

### Elevator Exit Update

After the elevator stops, the system applies zero-velocity and zero-acceleration constraints, anchors the estimated vertical elevator displacement back into the robot state, and resets elevator-related states. This suppresses height drift accumulated during elevator motion.

<p align="center">
  <img src="docs/images/exit_update.png" alt="Event-triggered update when exiting the elevator" width="70%">
</p>

### Adaptive Downsampling

The system adjusts voxel size online so that the number of downsampled effective points remains near the target. This preserves enough geometric information inside the elevator cabin while controlling computation in open scenes.

<p align="center">
  <img src="docs/images/adaptive_downsampling.png" alt="Adaptive voxel downsampling" width="70%">
</p>

## Data Collection Platform

<p align="center">
  <img src="docs/images/handheld_platform.png" alt="Elevator-LIO handheld data collection platform" width="85%">
</p>

The dataset was collected with a portable handheld device integrating a Livox MID-360, an industrial camera, and a Jetson Orin Nano.

> [!NOTE]
> - [ ] Hardware and software design documents for the handheld platform will be released later.
>

## 🔥 Design Notes

Elevator-LIO is designed to work out of the box, with practical features, detailed comments, and a logging and debugging system.

Useful features include:

- **No Livox ROS driver dependency**: custom messages are built inside this package and can be compiled directly in a ROS workspace.
- **Initial gravity alignment**: regardless of LiDAR mounting direction, the world frame is initialized horizontally.
- **Spherical or box blind-zone filtering**: near-field points can be removed using either a spherical blind zone or a rectangular box.
- **Additional body-frame output**: configure `lidar_R_vehicle` and `lidar_t_vehicle` under `yaml/sensors`.
- **Simple relocation mode**: load a PCD map, start near the map origin, and run localization against it.
- **Covariance visualization**: can be enabled under `yaml/logging` for debugging.
- **High-frequency odometry output**: can be enabled under `yaml/runtime` to publish IMU-preintegrated poses for downstream applications.

Unlike mainstream LIO systems, Elevator-LIO does not rely on an explicit measurement package abstraction. It follows an "update when data arrives" design, which is more convenient for multi-sensor fusion. The tradeoff is that the state machine must poll at a high rate to detect incoming data.

## 🛠️ Installation and Usage

### Requirements

The current code is mainly developed and tested on:

- Ubuntu 20.04
- ROS Noetic

OpenCV is currently only used for debugging windows such as covariance matrix and elevator-state curve visualization. These windows are disabled in the default configuration, but the source code and CMake still include and link OpenCV. You can remove these dependencies if needed.

### Build

Place the package under a catkin workspace `src` directory and build:

```bash
catkin_make
```

### Run

Launch directly with:

```bash
roslaunch lio start.launch
```

You can specify a YAML root configuration. The default is `root_config.yaml`:

```bash
roslaunch lio start.launch config_path:=path_to_yaml
```

### Elevator Mode

Elevator behavior is controlled by one global switch:

```yaml
elevator:
  enable: true
```

Automatic elevator entry is controlled by the door-closing detector:

```yaml
elevator:
  door_detector:
    enable: true
```

This detector is based on point-cloud range percentiles and may be triggered incorrectly in narrow scenes such as corridors. You can disable automatic detection and use manual topic triggers instead.

Enter elevator mode:

```bash
rostopic pub /LIO/set_elevator_flag std_msgs/Bool "data: true" -1
```

Exit elevator mode:

```bash
rostopic pub /LIO/set_elevator_flag std_msgs/Bool "data: false" -1
```

Automatic exit is controlled by:

```yaml
elevator:
  self_exit_detector: true # Exit automatically according to estimated elevator velocity and covariance.
```

The internal elevator state is published at LiDAR rate. `/LIO/in_elevator` keeps the boolean mode flag, and `/LIO/elevator_state` publishes the elevator mode, relative displacement, velocity, and acceleration for the RViz elevator status panel.

### Simulation Node

The early development version includes a `sim_node` for constructing elevator scenarios and generating LiDAR and IMU messages. The code and configuration are under `src/sim`.

### Configuration

Select one sensor, runtime, and logging configuration in `root_config.yaml`:

```yaml
sensor_config: "sensors/livox.yaml"
runtime_config: "runtime/mapping.yaml"
logging_config: "logging/default.yaml"
```

Detailed configuration notes are in [yaml/README.md](yaml/README.md).

Key configuration examples:

- **Downsampling**. On embedded devices or slow platforms, increase `point_filter_num`, reduce `adaptive.target_points`, or increase `adaptive.min_voxel`:

```yaml
downsample:
  point_filter_num: 2
  blind: 0.8
  use_box_blind: false
  box_corner:
    box_corner_x: [-0.7, 0.1]
    box_corner_y: [-0.3, 0.3]
    box_corner_z: [-0.4, 0.4]
  filter_size: 0.1
  adaptive:
    enable: true
    target_points: 20000
    alpha: 1.2
    min_voxel: 0.05
    max_voxel: 0.8
```

- **Elevator configuration**. When `elevator.enable` is disabled, the system falls back to regular LIO. When enabled, LiDAR door-closing detection can automatically enter elevator mode, and IMU motion-state detection can automatically exit:

```yaml
elevator:
  enable: true
  self_exit_detector: true
  door_detector:
    enable: true
    dist_threshold: 3.0
    time_threshold: 2.0
    filter_percent: 0.06
    cooldown_time: 5.0
  zupt:
    enable: true
    waiting:
      enable: true
      period_s: 3.0
      vz_abs_thresh: 0.1
      az_abs_thresh: 0.1
  exit_icp_z:
    enable: false
```

Automatic entry detection may be triggered in narrow corridors. Disable `door_detector.enable` and use `/LIO/set_elevator_flag` for manual triggering when needed. `exit_icp_z.enable` is enabled by default in relocation mode and disabled by default in mapping mode. See [yaml/README.md](yaml/README.md) for all parameters.

- **Relocation**. When enabling relocation, update the reference map file:

```yaml
relocation:
  relocation_enable: false
  pcd_load_name: "scans.pcd"
```

## Directory Layout

After mapping, maps are saved under the `PCD` directory.

Log files are generated under a newly created `Temp` directory.

Core code lives under `src` and `include`.

The `temp` directory stores key data records from the current run.

```text
├── CMakeLists.txt
├── PCD                         # Mapping results, relocation maps, and temporary point clouds
│   └── Temp                    # Temporary point clouds generated at runtime
├── docs                        # Images and documentation assets used by README
│   └── images
├── include                     # C++ headers
│   ├── support                 # Common types, config loading, buffers, and node interfaces
│   ├── elevator                # Elevator detection, state machine, and ZUPT interfaces
│   ├── estimator               # ESEKF and IMU preintegration/propagation interfaces
│   ├── AdaptiveFilter          # Adaptive voxel downsampling
│   ├── ikd_tree                # Incremental ikd-tree map and nearest-neighbor query
│   ├── node                    # Internal LiDAR pipeline interfaces
│   └── rviz                    # Debug visualization interfaces
├── launch                      # ROS launch files
├── msg                         # Custom ROS messages
├── package.xml
├── README.md
├── README_en.md
├── rviz                        # RViz display configuration
├── src                         # C++ implementation
│   ├── main.cpp
│   ├── elevator
│   ├── estimator
│   ├── node
│   ├── rviz
│   ├── support
│   └── sim
└── yaml                        # Layered runtime configuration
    ├── root_config.yaml
    ├── sensors                 # LiDAR type, extrinsics, and subscribed topics
    ├── runtime                 # Mapping, relocation, estimator, and elevator parameters
    └── logging                 # Console logging, file logging, and debug visualization
```

## Citation

If you use this software or dataset, please cite the Elevator-LIO paper:

```bibtex
@article{zhang2026elevatorlio,
  title={Elevator-LIO: Robust LiDAR-Inertial Odometry for Multi-Floor Navigation under Elevator-Induced Non-Inertial Motion},
  author={Zhang, Yifan and Huang, Yudong and Zhang, Yuchong and Li, Changze and Liu, Haoran and Yang, Ming and Qin, Tong},
  journal={arXiv preprint arXiv:2605.24495},
  year={2026}
}
```

## License

This project is released under the [GNU General Public License v2.0 or later](LICENSE). Independent MIT components retain their file-level MIT SPDX identifiers. Third-party code and local modification notes are documented in [THIRD_PARTY.md](include/ikd_tree/THIRD_PARTY.md).
