# Elevator-LIO 数据集

## 数据集概述

Elevator-LIO 数据集包含 20 条自采真实场景序列，共覆盖 79 次电梯乘坐，数据总时长约 70 分钟、总大小约 16.36 GiB。场景包括校园、宿舍楼、商场和办公楼，涵盖跨楼层建图、长距离垂直运动、电梯轿厢内手持晃动、动态行人和镜面内饰等情况。

采集平台包括：

- Livox MID-360 激光雷达及内置 IMU
- Jetson Orin Nano
- 同步工业相机

由于不同环境下固定曝光容易出现过曝或欠曝，大部分序列仅包含 LiDAR 和 IMU 数据。目前只有 `Mall2.bag` 和 `Office3.bag` 包含相机图像。相机数据通过 [hik_camera_ros_driver](https://github.com/xiaofan4122/hik_camera_ros_driver) 采集。

> [!NOTE]
> 除主数据集外，Hugging Face 仓库还提供 `community_contributions/` 作为社区贡献数据区，用于收录外部用户自愿提供的电梯相关 rosbag。社区贡献数据作为可选补充材料发布，默认不会随下载脚本一起下载。我们欢迎更多用户贡献自己的电梯相关数据，可联系 `xiaofan@sjtu.edu.cn`。

- [ ] 待完成：更多带图像完整序列补充，预计 7 月中下旬

## 序列列表

| 序列 | 场景 | 时长 | 文件大小 | 图像 |
|---|---|---:|---:|---|
| `Campus1.bag` | 校园 | 4:41 | 1.02 GiB | 无 |
| `Campus2.bag` | 校园 | 11:46 | 2.55 GiB | 无 |
| `Campus3.bag` | 校园 | 2:18 | 511 MiB | 无 |
| `Campus4.bag` | 校园 | 5:34 | 1.21 GiB | 无 |
| `Dormitory1.bag` | 宿舍楼 | 1:39 | 366 MiB | 无 |
| `Dormitory2.bag` | 宿舍楼 | 2:49 | 627 MiB | 无 |
| `Dormitory3.bag` | 宿舍楼 | 2:47 | 620 MiB | 无 |
| `Dormitory4.bag` | 宿舍楼 | 5:44 | 1.24 GiB | 无 |
| `Mall1.bag` | 商场 | 3:27 | 764 MiB | 无 |
| `Mall2.bag` | 商场 | 2:26 | 960 MiB | 4016 帧 |
| `Office1.bag` | 办公楼 | 1:34 | 348 MiB | 无 |
| `Office2.bag` | 办公楼 | 1:49 | 405 MiB | 无 |
| `Office3.bag` | 办公楼 | 3:49 | 1.66 GiB | 5516 帧 |
| `Office4.bag` | 办公楼 | 1:36 | 356 MiB | 无 |
| `Office5.bag` | 办公楼 | 1:36 | 353 MiB | 无 |
| `Office6.bag` | 办公楼 | 4:35 | 1015 MiB | 无 |
| `Office7.bag` | 办公楼 | 2:19 | 515 MiB | 无 |
| `Office8.bag` | 办公楼 | 1:48 | 399 MiB | 无 |
| `Office9.bag` | 办公楼 | 4:58 | 1.08 GiB | 无 |
| `Office10.bag` | 办公楼 | 2:30 | 556 MiB | 无 |

### 社区贡献数据列表

社区贡献数据位于 Hugging Face 数据集仓库的 `community_contributions/` 目录中，默认不会随下载脚本一起下载。

| 序列 | 贡献者 | 描述 | 文件大小 | 是否回到起始楼层 | 包含传感器 |
|---|---|---|---:|---|---|
| `@编程猫小渐_2floors.bag` | @编程猫小渐 / 小红书 9556244270 | 两层电梯序列 | 452 MB | 是 | Livox MID-360 LiDAR/IMU |
| `@编程猫小渐_3floors.bag` | @编程猫小渐 / 小红书 9556244270 | 三层电梯序列 | 807 MB | 是 | Livox MID-360 LiDAR/IMU |

## ROS 话题与消息类型

所有序列均包含以下 LiDAR 和 IMU 消息：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox MID-360 原始点云 |
| `/livox/imu` | `sensor_msgs/Imu` | MID-360 内置 IMU |

`livox_ros_driver2/CustomMsg` 来自 [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)。本代码仓库的 `msg/` 目录也提供了运行 Elevator-LIO 所需的兼容消息定义。

`Mall2.bag` 和 `Office3.bag` 还包含以下相机消息：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | 压缩相机图像 |
| `/camera/exposure_time` | `std_msgs/Float32` | 图像曝光时间 |
| `/camera/gain` | `std_msgs/Float32` | 相机增益 |

相机驱动来自 [xiaofan4122/hik_camera_ros_driver](https://github.com/xiaofan4122/hik_camera_ros_driver)。

部分序列还记录了手动触发进入电梯模式标志：

| 序列 | 话题 | 消息类型 |
|---|---|---|
| `Campus1.bag`、`Campus3.bag`、`Campus4.bag`、`Mall1.bag` | `/LIO/set_elevator_flag` | `std_msgs/Bool` |

这是因为这些序列的电梯内存在明显反射和多径效应，配套算法 Elevator-LIO 的距离阈值检测会失效。虽然调整阈值可以成功检测，但为了让测评更反映真实效果，并保留后续采用更合适电梯检测方法的空间，我们没有对此类序列做针对性调参。

## 下载

数据集与代码仓库分开分发：

- [交大云盘](https://pan.sjtu.edu.cn/web/share/3792f963d0e41237c86303c1586a4ba1)：公开分享链接，无需登录 SJTU 账号，国内用户推荐使用。
- [Hugging Face](https://huggingface.co/datasets/xiaofan0100/Elevator-LIO-Dataset)：支持通过 `huggingface_hub` 断点续传。

### 使用下载脚本（Hugging Face 源）

```bash
./scripts/download_dataset.sh
```

默认只下载主数据集，不下载 `community_contributions/` 社区贡献数据。

默认下载到代码包的同级目录：

```text
../Elevator-LIO-Dataset
```

指定输出目录：

```bash
./scripts/download_dataset.sh --output-dir /data/Elevator-LIO-Dataset
```

如需同时下载社区贡献数据：

```bash
./scripts/download_dataset.sh --include-community
```

显示交大云盘地址：

```bash
./scripts/download_dataset.sh --sjtu
```

脚本仅使用 `huggingface_hub`。未安装时会询问并执行：

```bash
python3 -m pip install --user --upgrade huggingface_hub
```

脚本不会自动设置代理，而是继承当前终端的代理环境变量。下载中断后，重新执行相同命令即可继续。

## 数据完整性检查

数据集目录提供 `check_bag_files.py`，下载完成后可执行：

```bash
cd ../Elevator-LIO-Dataset
python3 check_bag_files.py .
```

## 数据集许可证

Elevator-LIO 数据集采用 [知识共享署名 4.0 国际许可协议](https://creativecommons.org/licenses/by/4.0/)（CC BY 4.0）。

你可以自由分享和改编本数据集，包括用于商业目的，前提是注明出处、附上许可证链接并说明修改内容。数据集许可证与本仓库的 GPL-2.0-or-later 软件许可证相互独立。

## 引用

使用本数据集时，请引用 Elevator-LIO 论文：

```bibtex
@article{zhang2026elevator,
  title={Elevator-LIO: Robust LiDAR-Inertial Odometry for Multi-Floor Navigation under Elevator-Induced Non-Inertial Motion},
  author={Zhang, Yifan and Huang, Yudong and Zhang, Yuchong and Li, Changze and Liu, Haoran and Yang, Ming and Qin, Tong},
  journal={arXiv preprint arXiv:2605.24495},
  year={2026}
}
```

另见仓库根目录的 [CITATION.cff](CITATION.cff)。
