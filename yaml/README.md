# YAML 配置说明

本目录保存 `lio` 软件包的全部运行配置。配置被拆分为三类：

```text
yaml/
├── root_config.yaml          # 根配置，只负责选择下面三类配置文件
├── sensors/                  # 雷达类型、外参和输入话题
│   ├── livox.yaml
│   ├── ouster.yaml
│   ├── velodyne.yaml
│   └── xt32.yaml
├── runtime/                  # 降采样、地图、电梯、估计器和输出配置
│   ├── mapping.yaml
│   └── relocation.yaml
└── logging/                  # 控制台日志、文件日志和调试可视化
    ├── default.yaml
    └── visualize.yaml
```

正常使用时只需要在 `root_config.yaml` 中分别选择一个传感器配置、一个运行配置和一个日志配置，不需要把所有参数复制到同一个文件中。

## 1. 配置加载方式

### 1.1 启动时指定根配置

默认启动命令为：

```bash
roslaunch lio start.launch
```

此时读取：

```text
yaml/root_config.yaml
```

也可以通过 `config_path` 指定 `yaml/` 目录下的其他根配置：

```bash
roslaunch lio start.launch config_path:=root_config.yaml
```

`config_path` 是相对于当前软件包 `yaml/` 目录的路径，不是相对于终端工作目录的路径。

### 1.2 `root_config.yaml`

根配置必须同时包含以下三个字段：

```yaml
sensor_config: "sensors/livox.yaml"
runtime_config: "runtime/mapping.yaml"
logging_config: "logging/default.yaml"
```

| 字段 | 作用 |
| --- | --- |
| `sensor_config` | 选择雷达解析器、传感器外参和订阅话题 |
| `runtime_config` | 选择建图或重定位模式，并配置算法运行参数 |
| `logging_config` | 选择日志记录方式和调试可视化方式 |

路径均相对于 `yaml/` 目录。例如 `sensors/livox.yaml` 的完整位置是 `lio/yaml/sensors/livox.yaml`。

### 1.3 配置合并规则

程序会将选中的三个配置文件合并后读取，但合并规则不是“后面的文件覆盖前面的文件”，而是：

- 三个文件只能包含互不冲突的配置项。
- 任意两个文件出现相同的叶子字段会直接报错。
- 改变三个文件的加载顺序不会改变字段值。
- 根节点必须是 YAML 映射，不能是列表或单个标量。
- 文件不存在、YAML 语法错误、字段类型错误都会在启动终端中报告。

例如，不要在传感器配置和运行配置中同时定义 `downsample.filter_size`。公共运行参数应只写在 `runtime/` 中。

建议每次修改配置后检查程序启动时打印的 `LIO Configuration`，确认实际加载的文件和值与预期一致。

## 2. 传感器配置 `sensors/*.yaml`

传感器配置包含三部分：雷达解析器类型、传感器外参和输入话题。

### 2.1 `lidar_type`

```yaml
lidar_type: 1
```

| 数值 | 配置文件 | 输入消息 | 说明 |
| --- | --- | --- | --- |
| `1` | `livox.yaml` | `lio/CustomMsg` | 使用 Livox 自定义点云解析器 |
| `2` | `ouster.yaml` | `sensor_msgs/PointCloud2` | 使用 Ouster 字段和时间戳解析方式 |
| `3` | `velodyne.yaml` | `sensor_msgs/PointCloud2` | 使用 Velodyne 字段和时间戳解析方式 |
| `4` | `xt32.yaml` | `sensor_msgs/PointCloud2` | 使用 XT32 字段和时间戳解析方式 |

Ouster 和 Velodyne 的点时间解析已按 Fast-LIO2 的约定对齐：点云 `curvature` 字段保存相对帧起点的毫秒级时间偏移。XT32 使用独立解析模板，尚未按 Fast-LIO2 进行系统性对齐验证。

### 2.2 IMU 与 LiDAR 外参

```yaml
offset:
  imu_t_lidar: [-0.011, -0.02329, 0.04412]
  imu_R_lidar:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
```

程序使用如下变换关系：

```text
p_imu = imu_R_lidar × p_lidar + imu_t_lidar
```

因此：

- `imu_t_lidar`：LiDAR 原点在 IMU 坐标系中的位置，单位为米。
- `imu_R_lidar`：将 LiDAR 坐标系中的向量旋转到 IMU 坐标系的 `3×3` 旋转矩阵。

这里需要填写 LiDAR 到 IMU 的变换，不要误填为 IMU 到 LiDAR 的逆变换。

### 2.3 LiDAR 与车体外参

```yaml
offset:
  lidar_t_body: [-0.2, 0.0, -0.15]
  lidar_R_body:
    - [0.0, -1.0, 0.0]
    - [1.0,  0.0, 0.0]
    - [0.0,  0.0, 1.0]
```

这组外参用于计算车体中心的里程计：

```text
world_T_body = world_T_imu × imu_T_lidar × lidar_T_body
```

因此：

- `lidar_t_body`：车体坐标系原点在 LiDAR 坐标系中的位置，单位为米。
- `lidar_R_body`：将车体坐标系中的向量旋转到 LiDAR 坐标系的旋转矩阵。
- 这组参数主要影响 `topic_pub.odom_vehicle_topic_name` 对应的车体位姿和 `world -> body` TF。

如果您使用LIO输出的body系tf树，则此项无需关注。

### 2.4 输入话题 `topic_sub`

```yaml
topic_sub:
  lidar_topic_name: "/livox/lidar"
  imu_topic_name: "/livox/imu"
  wheel_topic_name: "/wheel_info"
  elevator_flag_topic_name: "/LIO/set_elevator_flag"
```

| 字段 | 说明 |
| --- | --- |
| `lidar_topic_name` | 原始 LiDAR 点云话题，消息内容必须与 `lidar_type` 对应的解析器匹配 |
| `imu_topic_name` | IMU 数据话题，消息类型为 `sensor_msgs/Imu` |
| `wheel_topic_name` | 轮速计话题，当前消息类型为 `lio/wheel_info`；仅在 `wheel.wheel_enable: true` 时参与状态更新 |
| `elevator_flag_topic_name` | 手动控制电梯状态的话题，消息类型为 `std_msgs/Bool`；`true` 表示触发，`false` 表示取消 |

轮速计回调使用的是本项目保留的特定数据格式和语义。接入其他底盘时，不能只修改话题名，还需要检查并适配 `receive_wheel` 的消息类型、单位、符号和车体坐标方向。

当 `elevator.enable: false` 时，手动电梯触发消息也会被忽略。

## 3. 运行配置 `runtime/*.yaml`

当前提供两套运行配置：

| 文件 | 用途 |
| --- | --- |
| `runtime/mapping.yaml` | 正常 LIO 建图，持续向 ikd-tree 添加新点 |
| `runtime/relocation.yaml` | 预加载已有 PCD 地图，在已有地图中定位，不继续积累地图 |

两份文件几乎一致，主要差距在`relocation`字段。

## 4. 原始点筛选和体素降采样 `downsample`

### 4.1 原始点抽取 `point_filter_num`

```yaml
downsample:
  point_filter_num: 2
```

订阅点云后按点的顺序进行第一次抽取。值为 `2` 表示大致每两个点保留一个，值越大，进入后续处理的点越少。

- 必须为正整数。
- 增大该值不一定会让 LIO 的实际处理点数按比例减少，因为后续还有体素滤波。
- 它与后面的体素降采样是两级不同的降采样，不会互相替代。

### 4.2 近距离球形盲区 `blind`

```yaml
downsample:
  blind: 0.8
  use_box_blind: false
```

当 `use_box_blind: false` 时，程序删除 LiDAR 原点附近、距离小于 `blind` 的点：

```text
x² + y² + z² < blind * blind
```

`blind` 的单位是米。它通常用于过滤传感器近场无效点。

### 4.3 车体长方体盲区 `use_box_blind`

```yaml
downsample:
  use_box_blind: true
  box_corner:
    box_corner_x: [-0.7, 0.1]
    box_corner_y: [-0.3, 0.3]
    box_corner_z: [-0.4, 0.4]
```

当 `use_box_blind: true` 时：

- 球形 `blind` 不再参与近场过滤。
- 程序删除 LiDAR 坐标系中位于指定轴对齐长方体内部或边界上的点。
- 三个范围的单位均为米。
- 范围表示 `[min, max]`。

该功能适合过滤车顶、车身、保护罩、安装支架或手持设备外壳的固定回波。

长方体必须在 LiDAR 坐标系下填写，不是 IMU、车体或世界坐标系。

### 4.4 固定体素大小 `filter_size`

```yaml
downsample:
  filter_size: 0.1
```

- 当 `adaptive.enable: false` 时，它是固定体素降采样的体素边长，单位为米。
- 当 `adaptive.enable: true` 时，它是自适应控制器的初始体素边长。
- 值越大，输出点越少、运行速度越快，但地图分辨率和小物体细节越低。

### 4.5 自适应体素降采样 `adaptive`

```yaml
downsample:
  adaptive:
    enable: true
    target_points: 20000
    alpha: 1.2
    min_voxel: 0.05
    max_voxel: 0.8
```

| 字段 | 说明 |
| --- | --- |
| `enable` | 是否动态调整体素边长 |
| `target_points` | 目标处理点率，单位为点/秒，不是点/帧 |
| `alpha` | 幂律控制指数，必须大于零 |
| `min_voxel` | 允许使用的最小体素边长，单位为米 |
| `max_voxel` | 允许使用的最大体素边长，单位为米 |

程序会根据平滑后的 LiDAR 帧间隔，把 `target_points` 换算为当前帧目标点数。控制器按下式调整下一帧体素：

```text
v_next = v_current × (N_current / N_target)^(1 / alpha)
```

含义如下：

- 当前输出点数高于目标时，体素增大，从而减少后续点数。
- 当前输出点数低于目标时，体素减小，从而保留更多点。
- 最终体素会限制在 `[min_voxel, max_voxel]` 范围内。
- `alpha` 越小，调整越激进；越大，变化越平缓。

`target_points` 是负载目标，不保证每一帧都精确得到相同点数。

自适应控制器会对明显无效的配置进行保护：

- `alpha <= 0` 时按 `1.0` 使用。
- `target_points <= 0` 时按 `1.0` 使用。
- `min_voxel <= 0` 时按 `0.001 m` 使用。
- `max_voxel < min_voxel` 时，会把 `max_voxel` 修正为 `min_voxel`。
- 当前体素始终被限制在最终有效的 `[min_voxel, max_voxel]` 范围内。

ikd-tree 初始化时使用的最小地图分辨率为：

```text
adaptive.enable = true  : min(filter_size, min_voxel)
adaptive.enable = false : filter_size
```

因此，希望降低地图点数和内存占用时，应同时检查 `filter_size` 与 `adaptive.min_voxel`。只增大其中一个，而另一个仍然很小，可能无法提高 ikd-tree 的实际最小分辨率。

嵌入式设备不能实时运行时，建议依次尝试：

1. 关闭全局地图、ikd-tree 发布和 OpenCV 可视化
2. 关闭 RViz
3. 降低 `adaptive.target_points`
4. 同时增大 `filter_size` 和 `adaptive.min_voxel`
5. 增大 `point_filter_num`
6. 增大 `clouds_lidar_pub_every_n`

## 5. 局部地图 `map`

```yaml
map:
  local_map_cube_len: 1500.0
```

`local_map_cube_len` 是局部 ikd-tree 地图立方体的边长，单位为米。设备接近局部地图边缘时，程序会移动局部地图区域并清理远离当前位置的旧地图点。

- 值较大：保留范围更广，但内存占用和树维护成本更高。
- 值较小：内存压力更低，但长距离回环到旧区域时可能已经没有历史点。
- `mapping.yaml` 当前为 `1500 m`。
- `relocation.yaml` 当前为 `3000 m`，用于尽量保留更大的预加载地图范围。

该参数不是体素分辨率，也不是允许移动的最大距离。

## 6. 建图与重定位 `relocation`

```yaml
relocation:
  relocation_enable: false
  pcd_load_name: "20251229-203113_scans.pcd"
```

### 6.1 `relocation_enable`

- `false`：正常建图。每帧点云会转换到世界坐标系并加入 ikd-tree。
- `true`：重定位。启动时加载已有 PCD 到 ikd-tree，并停止向地图持续添加当前点云。

重定位模式当前依赖较好的初始位姿，只适合在原地图原点附近启动；它不是带全局搜索能力的任意位置重定位模块。

### 6.2 `pcd_load_name`

PCD 文件固定从以下目录读取：

```text
lio/PCD/<pcd_load_name>
```

例如：

```yaml
pcd_load_name: "building_map.pcd"
```

对应：

```text
lio/PCD/building_map.pcd
```

加载后程序会先以 `0.1 m` 体素进行一次降采样，再加入 ikd-tree。启用重定位前应确认：

- 文件存在且可以被 PCL 正常读取。
- PCD 坐标系与算法的 `world` 坐标系定义一致。
- 地图原点、初始朝向和实际启动位置足够接近。
- 地图单位为米。

## 7. PCD 保存 `pcd_save`

```yaml
pcd_save:
  global_map_save_enable: true
  every_n_frame_save: -1
  timestamp_prefix: true
  pcd_save_name: "scans.pcd"
```

| 字段 | 说明 |
| --- | --- |
| `global_map_save_enable` | 退出程序时是否保存累计的全局点云 |
| `every_n_frame_save` | 是否按固定帧间隔单独保存世界坐标系点云；`-1` 表示关闭 |
| `timestamp_prefix` | 是否在输出文件名前添加时间戳，避免覆盖旧文件 |
| `pcd_save_name` | 输出 PCD 文件名 |

输出目录固定为：

```text
lio/PCD/
```

注意：

- 全局地图保存只在非重定位模式下有效。
- `every_n_frame_save` 设置为较小正数会产生大量文件并增加磁盘写入负载。
- `timestamp_prefix: false` 时，同名文件可能被后续运行覆盖。
- 异常退出时不一定能执行最终地图合并保存，但过程文件会存在`/PCD/Temp`文件夹中。

## 8. ROS 发布负载 `pub`

```yaml
pub:
  global_map_pub_enable: false
  ikdtree_pub_enable: false
  clouds_lidar_pub_every_n: 1
  high_frequency_odom: false
  lidar_frequency: 10.0
```

### 8.1 `global_map_pub_enable`

发布累计全局点云。它只在以下条件同时满足时有效：

- 非重定位模式；
- `pcd_save.global_map_save_enable: true`；
- `global_map_pub_enable: true`。

全局地图会不断增大，序列化和发布耗时明显，通常只在调试或生成展示结果时临时开启。

### 8.2 `ikdtree_pub_enable`

发布 ikd-tree 内当前保存的地图点。导出整棵树同样耗时，实时运行时建议关闭。

### 8.3 `clouds_lidar_pub_every_n`

控制当前帧主点云的发布频率：

- `1`：每帧发布。
- `5`：每五帧发布一次。
- 小于 `1` 的值会被程序修正为 `1`。

该参数只降低 ROS 点云发布和 RViz 显示负载，不降低状态估计本身处理点云的频率。高频雷达或低性能设备上，适当增大该值可避免 RViz 和 ROS 通信占用过多 CPU。

### 8.4 高频里程计 `high_frequency_odom`

`high_frequency_odom: false` 时，`odom_imu_topic_name` 和 `odom_vehicle_topic_name` 只在每帧 LiDAR 更新完成后发布。

`high_frequency_odom: true` 时，程序会在下一帧 LiDAR 到来前，把 IMU 预积分得到的先验状态也发布到两个里程计话题，使里程计输出频率接近 IMU 频率。该先验状态还没有经过下一帧 LiDAR 点面匹配修正，因此短时间内更平滑、频率更高，但漂移约束仍来自后续 LiDAR 更新。

启用该功能时必须把 `lidar_frequency` 设置为实际 LiDAR 帧率，单位为 Hz。程序用它估计下一帧 LiDAR 到达时间，只在 `lidar_end_time + 1 / lidar_frequency` 之前发布 IMU 先验里程计；频率设置错误会导致高频里程计发布窗口过短或越过下一帧 LiDAR 时间。

旧版单文件 YAML 中写在顶层的 `high_frequency_odom` 和 `lidar_frequency` 仍可读取；新配置建议统一写在 `pub` 下。

## 9. 输出话题 `topic_pub`

```yaml
topic_pub:
  clouds_lidar_topic_name: "/LIO/clouds_lidar"
  clouds_lidar_effect_topic_name: "/LIO/clouds_lidar/effect"
  clouds_lidar_reject_topic_name: "/LIO/clouds_lidar/reject"
  global_map_topic_name: "/LIO/global_map"
  ikdtree_topic_name: "/LIO/ikdtree"
  odom_imu_topic_name: "/LIO/odom_imu"
  odom_vehicle_topic_name: "/LIO/odom_vehicle"
```

| 字段 | 数据内容与坐标系 |
| --- | --- |
| `clouds_lidar_topic_name` | 当前帧去畸变点云，已转换到 `world` 坐标系；受 `clouds_lidar_pub_every_n` 控制 |
| `clouds_lidar_effect_topic_name` | 当前帧参与点面匹配的有效点，消息 `frame_id` 为 `lidar` |
| `clouds_lidar_reject_topic_name` | 当前帧未被接受为有效点面匹配的调试点，消息 `frame_id` 为 `lidar` |
| `global_map_topic_name` | 累计全局地图，`world` 坐标系；受全局地图发布开关控制 |
| `ikdtree_topic_name` | ikd-tree 地图点，`world` 坐标系；受 ikd-tree 发布开关控制 |
| `odom_imu_topic_name` | IMU 中心的里程计，父坐标系为 `world` |
| `odom_vehicle_topic_name` | 根据 LiDAR–车体外参换算出的车体中心里程计，父坐标系为 `world` |

一个容易混淆的地方是：字段名仍为 `clouds_lidar`，但主点云已经转换到 `world`；只有 `effect` 和 `reject` 调试点云仍使用 `lidar` 坐标系。RViz 中显示异常时，应首先检查 Fixed Frame 和消息 `frame_id`，不要仅根据话题名称判断坐标系。

如果没有配置 `clouds_lidar_effect_topic_name` 或 `clouds_lidar_reject_topic_name`，程序会分别使用主点云话题名加 `/effect` 和 `/reject` 作为默认值。

## 10. 电梯功能 `elevator`

### 10.1 总开关与自动退出

```yaml
elevator:
  enable: true
  self_exit_detector: true
```

- `enable`：Elevator-LIO 电梯处理总开关。关闭后自动检测和手动触发都会被忽略，可将系统作为普通 LIO 使用。
- `self_exit_detector`：根据 IMU 观测到的电梯运动阶段和最终停稳状态自动退出电梯模式。

关闭 `self_exit_detector` 后，需要由外部逻辑正确控制电梯状态，否则系统可能一直停留在电梯处理阶段。

### 10.2 电梯门关闭检测 `door_detector`

```yaml
elevator:
  door_detector:
    enable: true
    dist_threshold: 3.0
    time_threshold: 2.0
    filter_percent: 0.06
    cooldown_time: 5.0
```

| 字段 | 说明 |
| --- | --- |
| `enable` | 是否根据 LiDAR 周围空间自动判断电梯门关闭 |
| `dist_threshold` | 过滤异常远点后，最远点仍小于该距离时认为空间封闭，单位为米 |
| `time_threshold` | 封闭状态需要连续保持的时间，单位为秒 |
| `filter_percent` | 计算最远距离前忽略最远端点的比例 |
| `cooldown_time` | 两次自动触发之间的最短间隔，单位为秒 |

`filter_percent` 用于抑制少量穿过门缝、玻璃反射或离群远点。例如 `0.06` 表示计算封闭程度时忽略最远端约 `6%` 的点。程序会把该值限制在 `[0, 0.5]`。

调参规律：

- `dist_threshold` 过小：较大的轿厢可能无法触发。
- `dist_threshold` 过大：窄走廊、房间或靠墙场景可能误触发。
- `time_threshold` 过短：瞬时遮挡容易误触发。
- `filter_percent` 过大：真实的开门方向远点也可能被忽略。
- `cooldown_time` 过短：同一次过程可能重复触发。

`mapping.yaml` 当前使用 `filter_percent: 0.06`，`relocation.yaml` 当前使用 `0.03`。

### 10.3 竖直加速度强先验

```yaml
elevator:
  strong_prior_enable: false
  strong_prior_acc_var: 0.001
```

该 Beta 功能假设设备相对电梯轿厢没有竖直方向运动，从而在电梯模式中使用 IMU 竖直加速度作为电梯加速度的强约束。它主要用于 LiDAR 水平安装、环境又缺少水平面，导致 z 方向几何约束不足的情况。

- `strong_prior_enable`：是否启用该先验。
- `strong_prior_acc_var`：量测方差，单位为 `(m/s²)²`；数值越小，滤波器越信任该先验。

`strong_prior_acc_var <= 0` 时程序会回退到 `0.001`。

如果设备在电梯内被拿起、上下晃动，或机器人相对轿厢存在明显竖直运动，该假设不成立，强先验可能引入错误高度，因此默认关闭。

### 10.4 电梯 ZUPT

```yaml
elevator:
  zupt:
    enable: true
    waiting:
      enable: true
      period_s: 3.0
      vz_abs_thresh: 0.1
      az_abs_thresh: 0.1
```

ZUPT 是零速更新，用于在设备确定静止时约束速度漂移。

- `zupt.enable`：电梯相关 ZUPT 总开关。
- `waiting.enable`：进入电梯模式后、但电梯尚未开始运动时，是否周期性检查并执行等待阶段 ZUPT。
- `waiting.period_s`：等待阶段检查周期，单位为秒。
- `waiting.vz_abs_thresh`：只有竖直速度绝对值小于该阈值时才允许等待 ZUPT。
- `waiting.az_abs_thresh`：只有竖直加速度绝对值小于该阈值时才允许等待 ZUPT。

`period_s`、`vz_abs_thresh` 或 `az_abs_thresh` 为负数时会被修正为 `0`。

等待阶段 ZUPT 在检测到电梯开始运动后停止，避免把真实电梯速度错误压到零。

当前预设中：

- `mapping.yaml`：`zupt.enable: true`。
- `relocation.yaml`：`zupt.enable: false`。

如果使用 Bag Runner 分析电梯总高度 `Total Z`，还应开启日志配置中的 `log.file.elevator_zupt`。算法 ZUPT 开关和 ZUPT 文件日志开关作用不同：前者控制状态更新，后者控制是否把事件写入分析日志。

### 10.5 退出电梯时的单轴 z-ICP

```yaml
elevator:
  exit_icp_z:
    enable: false
    max_correction_m: 0.3
    max_iterations: 5
    min_effective_points: 20
    min_information_ratio: 0.1
    max_neighbor_distance_m: 0.5
    min_abs_normal_z: 0.7
    max_residual_m: 0.3
    converge_m: 1.0e-3
```

该 Beta 功能在到达新楼层并完成退出 ZUPT 后，将当前帧与已有 ikd-tree 地图匹配，但只求解 z 方向平移，不修改 x、y 和姿态。

| 字段 | 说明 |
| --- | --- |
| `enable` | 是否启用退出阶段 z-ICP |
| `max_correction_m` | 最终累计 z 修正量的绝对值必须严格小于该值才接受，单位为米 |
| `max_iterations` | 最大迭代次数，至少为 `1` |
| `min_effective_points` | 有效点面对应少于该数量时拒绝修正 |
| `min_information_ratio` | `mean(n_z²)` 的最小值，用于判断 z 方向几何信息是否充足 |
| `max_neighbor_distance_m` | 当前点和地图近邻点允许的最大距离，单位为米 |
| `min_abs_normal_z` | 平面法向量 z 分量绝对值的最小值，只使用接近水平的平面 |
| `max_residual_m` | 点到平面的最大残差，单位为米 |
| `converge_m` | 单次 z 增量小于该值时认为收敛，单位为米 |

参数关系：

- `min_abs_normal_z` 越大，越偏向地面和天花板等水平面。
- `min_information_ratio` 过高会使修正经常被拒绝，过低则可能接受退化结果。
- `max_neighbor_distance_m` 和 `max_residual_m` 过大可能产生跨楼层、跨轿厢结构的错误对应。
- `max_correction_m` 是最终安全门限，不是每次迭代的门限。

程序会把 `min_information_ratio` 和 `min_abs_normal_z` 限制在 `[0, 1]`；迭代次数和最少有效点数至少为 `1`。无效的非正距离或收敛阈值会回退到代码中的安全默认值，但实际使用时仍应在 YAML 中填写合理参数。

该功能依赖已有地图。`relocation.yaml` 默认开启，`mapping.yaml` 默认关闭。

### 10.6 退出后重建地图

```yaml
elevator:
  rebuild_map_on_exit:
    enable: false
    frames: 5
    min_points: 2000
```

- `enable`：退出电梯后是否清空旧地图，并使用后续若干帧重新建立 ikd-tree。
- `frames`：参与重建的 LiDAR 帧数。
- `min_points`：累计点数达到该值后才执行重建。

该功能只在非重定位模式下生效。开启后会丢弃先前地图，不适合需要保留跨楼层完整地图的常规建图任务。它主要用于明确希望在新楼层重新建立局部参考地图的特殊场景。

## 11. 轮速计 `wheel`

```yaml
wheel:
  wheel_enable: false
```

`wheel_enable` 控制是否使用轮速计量测更新状态。默认关闭。

启用前必须确认：

- 订阅消息类型与当前回调实现一致。
- 速度单位为程序期望的单位。
- 正方向与车体坐标定义一致。
- 时间戳可靠。
- `lidar_t_body` 和 `lidar_R_body` 与实际车体坐标系一致。

仅把该值改为 `true`，但没有适配底盘数据，可能比不使用轮速计产生更差的结果。

## 12. 估计器 `estimator`

### 12.1 LiDAR 更新维度

```yaml
estimator:
  lidar_update_mode: "pose6"
```

支持：

- `pose6` 或 `pose_6`：只构造与旋转、位置直接相关的 6 维 LiDAR 更新，推荐使用。
- `full21` 或 `full_21`：按完整 21 维误差状态构造更新。

LiDAR 点面残差只对位姿具有直接雅可比，因此 `pose6` 可以减少矩阵计算量，设计上与完整状态更新保持一致结果。无法识别的字符串会打印警告并回退到 `pose6`。

### 12.2 在线重力估计

```yaml
estimator:
  online_gravity_estimation_enable: false
```

- `false`：初始化完成后固定使用初始化得到的重力，通常更稳定。
- `true`：允许后续状态更新继续修正重力。

只有在数据和运动能够充分观测重力方向时才建议开启在线估计。约束不足时，重力与加速度偏置、姿态误差可能耦合。

### 12.3 LiDAR IESKF 迭代

```yaml
estimator:
  lidar_ieskf:
    max_iterations: 4
    converge_rot_rad: 1.0e-3
    converge_pos_m: 1.0e-3
    rematch_rot_rad: 2.0e-3
    rematch_pos_m: 2.0e-3
```

| 字段 | 说明 |
| --- | --- |
| `max_iterations` | 每帧 LiDAR 更新的最大迭代次数 |
| `converge_rot_rad` | 旋转增量小于该值时满足旋转收敛条件，单位为弧度 |
| `converge_pos_m` | 位置增量小于该值时满足位置收敛条件，单位为米 |
| `rematch_rot_rad` | 位姿旋转改变量超过该值时，重新查询地图近邻 |
| `rematch_pos_m` | 位姿平移改变量超过该值时，重新查询地图近邻 |

增大 `max_iterations` 可能提高大初值误差下的收敛能力，但会增加每帧耗时。收敛阈值过小会产生不必要的迭代；过大可能提前停止。

重新匹配阈值用于判断当前线性化点变化是否已经足以使旧的点面对应失效。阈值过小会频繁 KNN 查询，阈值过大则可能继续使用不合适的旧对应。

### 12.4 ikd-tree 点面匹配

```yaml
estimator:
  ikdtree_match:
    knn: 5
    plane_fit_rmse_threshold: 0.05
    neighbor_max_dist2: 5.0
    gate_mode: "linear_range"
    score_min: 0.95
    match_thresh_base: 0.03
    match_thresh_scale: 0.01
```

| 字段 | 说明 |
| --- | --- |
| `knn` | 为每个当前点查询的地图近邻数量 |
| `plane_fit_rmse_threshold` | 近邻点拟合平面的最大 RMSE，单位为米 |
| `neighbor_max_dist2` | 最远近邻允许的最大平方距离，单位为 `m²`，注意不是米 |
| `gate_mode` | 点面残差筛选方式，可选 `linear_range` 或 `fastlio_score` |
| `score_min` | `fastlio_score` 模式的最低接受分数 |
| `match_thresh_base` | `linear_range` 模式的基础残差门限，单位为米 |
| `match_thresh_scale` | `linear_range` 模式随点距离增加的门限系数 |

匹配首先要求：

1. 找到足够数量的近邻。
2. 第 `knn` 个近邻的平方距离不超过 `neighbor_max_dist2`。
3. 近邻能够拟合出满足 RMSE 条件的平面。
4. 点面残差通过所选的 gate。

`linear_range` 模式的接受条件为：

```text
|point_to_plane_residual|
    < match_thresh_base + match_thresh_scale × point_range
```

其中 `point_range` 是该点在 LiDAR 坐标系中的距离。

`fastlio_score` 模式使用：

```text
score = 1 - 0.9 × |residual| / sqrt(max(point_range, 1e-6))
score > score_min
```

只有当前 gate 对应的参数会生效：

- `linear_range` 使用 `match_thresh_base` 和 `match_thresh_scale`。
- `fastlio_score` 使用 `score_min`。

`plane_fit_rmse_threshold`、`neighbor_max_dist2` 和 `knn` 在两种模式下都会生效。

调参时不要同时大幅放宽所有门限。过严会导致有效点不足，过松会把错误对应加入滤波器。

## 13. 日志配置 `logging/*.yaml`

当前提供：

- `logging/default.yaml`：正常运行配置，关闭 OpenCV 调试窗口，减少控制台输出和原始数据记录。
- `logging/visualize.yaml`：调试配置，开启性能输出、原始 IMU 记录和两个 OpenCV 调试窗口，资源占用明显更高。

### 13.1 日志总开关

```yaml
log:
  enabled: true
  session_dir: "temp"
  flush_interval_ms: 1000
```

| 字段 | 说明 |
| --- | --- |
| `enabled` | 日志总开关；关闭后分类控制台日志和所有文件日志均不记录 |
| `session_dir` | 日志根目录，相对于 `lio` 软件包目录 |
| `flush_interval_ms` | 文件缓冲区刷新周期，单位为毫秒 |

默认日志输出到：

```text
lio/temp/<启动时间>/
```

刷新周期越短，异常退出时丢失的缓冲日志越少，但磁盘写入更频繁。该设置不等于每条日志的采样周期。

### 13.2 控制台日志 `log.console`

```yaml
log:
  console:
    enabled: true
    info: true
    warn: true
    error: true
    debug: false
    fsm: false
    elevator: false
    perf: false
```

控制台输出同时受总开关、控制台开关、等级开关和分类开关影响。

等级开关：

| 字段 | 说明 |
| --- | --- |
| `enabled` | 控制台日志总开关 |
| `info` | 是否显示 INFO |
| `warn` | 是否显示 WARN |
| `error` | 是否显示 ERROR |
| `debug` | 是否显示 DEBUG；可能包含传感器频率等高频信息 |

分类开关：

| 字段 | 说明 |
| --- | --- |
| `fsm` | 电梯有限状态机切换信息 |
| `elevator` | 门检测、ZUPT、退出 z-ICP 等详细信息 |
| `perf` | LiDAR 更新耗时、迭代次数和匹配质量统计 |

分类开关不是等级开关的替代。例如一条电梯 INFO 日志通常需要 `info: true` 和 `elevator: true` 同时允许。

### 13.3 文件日志总开关

```yaml
log:
  file:
    enabled: true
```

文件日志需要满足三层条件：

```text
log.enabled = true
log.file.enabled = true
对应通道.enabled = true
```

每个通道的 `file` 或 `file_prefix` 都相对于本次运行的时间戳日志目录。

### 13.4 后验轨迹 `trajectory`

```yaml
trajectory:
  enabled: true
  every_n: 1
  file: "state/post_state.csv"
```

记录 LiDAR 更新后的后验状态轨迹，用于轨迹评估和 Bag Runner 分析。

- `every_n: 1` 表示每次都记录。
- 增大 `every_n` 会降低日志量，但轨迹时间分辨率也会下降。
- 如果需要完整轨迹评估，建议保持开启。

### 13.5 IMU 传播先验轨迹 `trajectory_prior`

```yaml
trajectory_prior:
  enabled: false
  every_n: 10
  file: "state/pre_state.csv"
```

记录 LiDAR 更新前、仅经过 IMU 传播的先验状态。主要用于比较 IMU 传播和 LiDAR 校正之间的差异，正常运行不必开启。

### 13.6 原始 IMU `raw_imu`

```yaml
raw_imu:
  enabled: false
  every_n: 20
  file: "sensor/raw_imu.csv"
```

保存接收到的原始 IMU 数据：

- IMU 频率通常很高，完整记录会产生较大文件。
- `every_n` 表示每 N 条 IMU 消息记录一次。
- `default.yaml` 默认关闭、采样间隔为 20。
- `visualize.yaml` 默认开启、采样间隔为 5。

排查时间戳、噪声、饱和或振动问题时可临时开启。

### 13.7 自适应降采样 `adaptive_downsample`

```yaml
adaptive_downsample:
  enabled: true
  every_n: 1
  file: "map/adaptive_downsample.csv"
```

记录自适应体素大小、输入点数、输出点数和目标点数。用于判断设备无法实时运行是否由点数过多造成，也可检查体素是否长期被限制在 `min_voxel` 或 `max_voxel`。

### 13.8 轮速计日志

```yaml
wheel_data:
  enabled: false
  every_n: 10
  file: "sensor/wheel_data.csv"

wheel_residual:
  enabled: false
  every_n: 10
  file: "debug/wheel_residual.csv"
```

- `wheel_data`：记录接收到的轮速原始数据。
- `wheel_residual`：记录轮速量测与当前状态之间的速度残差。

只有启用并实际接入轮速计时，这两类日志才有分析意义。

### 13.9 电梯状态调试 `elevator_debug`

```yaml
elevator_debug:
  enabled: true
  every_n: 5
  file: "debug/elevator_debug.csv"
```

记录电梯状态机输入、门检测量、运动阶段等信息。`every_n` 按 IMU 处理周期采样，不是按 LiDAR 帧采样。

出现不进入电梯、误进入电梯、提前退出或长时间不退出时，应优先检查该文件。

### 13.10 电梯 ZUPT 与高度提交 `elevator_zupt`

```yaml
elevator_zupt:
  enabled: true
  file: "debug/elevator_zupt.txt"
```

记录：

- ZUPT 前后状态；
- 电梯退出事件；
- 楼层高度提交；
- 退出 z-ICP 的执行和接受情况。

该通道没有配置 `every_n` 时，每次事件都会记录。

Bag Runner 的 `Total Z` 分析依赖该日志中的高度提交事件。若此通道关闭，即使算法本身仍在运行电梯逻辑，分析结果也可能缺少 `Total Z`。

### 13.11 发布耗时 `publish_time`

```yaml
publish_time:
  enabled: false
  every_n: 20
  file: "perf/publish_time.csv"
```

记录点云、里程计等 ROS 消息发布耗时。用于区分“算法计算慢”和“点云序列化、ROS 通信或 RViz 显示慢”。

### 13.12 ikd-tree 快照 `ikdtree_snapshot`

```yaml
ikdtree_snapshot:
  enabled: false
  file_prefix: "debug/ikdtree_cloud"
```

在特定异常或退出保存路径中导出 ikd-tree 点云快照。程序会在前缀后添加时间戳和 `.pcd`。

导出大地图会阻塞并占用大量磁盘，只应在定位地图匹配问题时临时开启。

## 14. OpenCV 调试可视化 `visualize`

```yaml
visualize:
  CovMatrix_vis: false
  EleState_vis: false
```

| 字段 | 说明 |
| --- | --- |
| `CovMatrix_vis` | 使用 OpenCV 窗口实时显示 `21×21` 状态协方差矩阵 |
| `EleState_vis` | 使用 OpenCV 窗口显示电梯状态历史曲线 |

这两个选项：

- 都会额外占用 CPU/GPU 和图形界面资源。
- 在无显示器、无 X11 转发或容器环境中可能无法创建窗口。
- 不影响 RViz 点云发布。
- 正常运行和嵌入式部署建议关闭。

`logging/visualize.yaml` 默认同时开启二者，主要用于算法调试，不建议作为长期运行配置。

## 15. 两套运行配置的关键差异

| 配置项 | `mapping.yaml` | `relocation.yaml` |
| --- | ---: | ---: |
| `map.local_map_cube_len` | `1500.0` | `3000.0` |
| `relocation.relocation_enable` | `false` | `true` |
| 是否持续添加新地图点 | 是 | 否 |
| `pub.high_frequency_odom` | `false` | `false` |
| `pub.lidar_frequency` | `10.0` | `10.0` |
| `elevator.door_detector.filter_percent` | `0.06` | `0.03` |
| `elevator.zupt.enable` | `true` | `false` |
| `elevator.exit_icp_z.enable` | `false` | `true` |

其余参数虽然当前多数相同，但两份 YAML 是独立文件，后续修改不会自动同步。

## 16. 两套日志配置的关键差异

| 配置项 | `default.yaml` | `visualize.yaml` |
| --- | ---: | ---: |
| `log.console.perf` | `false` | `true` |
| `log.file.raw_imu.enabled` | `false` | `true` |
| `log.file.raw_imu.every_n` | `20` | `5` |
| `visualize.CovMatrix_vis` | `false` | `true` |
| `visualize.EleState_vis` | `false` | `true` |

两者当前都开启后验轨迹、自适应降采样、电梯调试和电梯 ZUPT 文件日志。

## 17. 常用配置组合

### 17.1 Livox 正常建图

```yaml
sensor_config: "sensors/livox.yaml"
runtime_config: "runtime/mapping.yaml"
logging_config: "logging/default.yaml"
```

### 17.2 在已有地图中重定位

```yaml
sensor_config: "sensors/livox.yaml"
runtime_config: "runtime/relocation.yaml"
logging_config: "logging/default.yaml"
```

同时把目标 PCD 放入 `PCD/`，并修改 `relocation.yaml` 中的 `pcd_load_name`。

### 17.3 调试电梯状态

选择 `logging/visualize.yaml` 前，应确认运行环境支持 OpenCV 图形窗口。若只需要详细文本日志而不需要窗口，更推荐复制 `default.yaml`，只开启：

```yaml
log:
  console:
    elevator: true
    fsm: true
    perf: true
```

并保持：

```yaml
visualize:
  CovMatrix_vis: false
  EleState_vis: false
```

### 17.4 作为普通 LIO 使用

在所选运行配置中设置：

```yaml
elevator:
  enable: false
```

此时门检测、自动退出和手动电梯触发均不参与运行。

## 18. 常见问题检查

### 18.1 程序提示找不到配置

检查：

- `config_path` 是否相对于 `yaml/`。
- `root_config.yaml` 中三个子配置路径是否相对于 `yaml/`。
- 文件名大小写是否一致。
- 三个根字段是否都存在且非空。

### 18.2 提示 `Duplicate YAML key is not allowed`

所选的多个子配置文件定义了相同叶子字段。删除重复定义，把字段保留在所属的唯一配置文件中。调整文件顺序不能解决该问题。

### 18.3 RViz 不显示点云

依次检查：

1. 对应点云话题是否实际发布。
2. RViz Fixed Frame 是否能与消息 `frame_id` 建立 TF。
3. `/LIO/clouds_lidar` 使用 `world`，而 `/effect`、`/reject` 使用 `lidar`。
4. 是否错误开启了重定位但 PCD 没有成功加载。
5. `clouds_lidar_pub_every_n` 是否设置得过大。
6. 点云时间戳与 TF 时间是否一致。
7. 雷达消息字段是否与所选 `lidar_type` 解析器匹配。

### 18.4 点云重影或运动时撕裂

优先检查：

- `imu_R_lidar` 和 `imu_t_lidar` 是否方向正确。
- LiDAR 点内时间字段是否被所选解析器正确读取。
- LiDAR 与 IMU 时间戳是否同步。
- IMU 单位和轴方向是否正确。

### 18.5 运行不能实时

优先降低实际参与计算和发布的点数，而不是盲目降低迭代次数：

- 增大 `point_filter_num`。
- 降低 `adaptive.target_points`。
- 同时增大 `filter_size` 与 `adaptive.min_voxel`。
- 增大 `clouds_lidar_pub_every_n`。
- 关闭 `global_map_pub_enable`、`ikdtree_pub_enable`。
- 使用 `logging/default.yaml` 并关闭高频日志。
- 关闭两个 OpenCV 可视化选项。

### 18.6 Bag Runner 没有 `Total Z`

确认所选日志配置中：

```yaml
log:
  enabled: true
  file:
    enabled: true
    elevator_zupt:
      enabled: true
```

同时确认运行过程中实际完成了电梯退出和楼层高度提交。仅开启日志不能制造未发生的高度提交事件。

## 19. 修改配置后的建议验证流程

1. 启动节点，确认终端列出的三个配置文件正确。
2. 检查启动打印中的雷达类型、外参、话题、降采样和电梯开关。
3. 使用 `rostopic info` 确认输入消息类型与解析器一致。
4. 使用短 bag 运行，检查点云、里程计和 TF。
5. 检查本次运行日志目录是否生成预期文件。
6. 修改电梯或估计器门限后，用固定数据集进行前后对比，不要只根据单次 RViz 外观判断效果。
