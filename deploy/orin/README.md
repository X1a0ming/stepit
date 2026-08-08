# V12B / V12C 在 Jetson Orin 上部署

本目录提供三种互相隔离的运行模式：

| 模式 | 模型 | 输出 | 是否直接接管电机 |
| --- | --- | --- | --- |
| `go2_motor` | V12B 或 V12C | Go2 的 12 维关节位置指令 | 是 |
| `b2w_motor` | V12B 或 V12C | B2W 的 12 维关节位置和 4 维轮速指令 | 是 |
| `cmd_vel` | V12B 或 V12C | ROS 2 `geometry_msgs/msg/Twist` | 否 |

三个模式都使用 458 维 `joint_graph_v5` 观测和 50 Hz 推理。路径输入默认要求以
10 Hz 更新；若超过 0.25 s 未更新，控制链会进入超时保护。V12B 会维护 128 维循环
状态，V12C 是前馈模型。

> [!CAUTION]
> `go2_motor` 和 `b2w_motor` 会向 Unitree `rt/lowcmd` 发送低层命令。第一次上机必须
> 悬空机器人、保持急停可用，并按本文的分阶段流程检查。`cmd_vel` 模式不加载
> `RobotApi`，不会发布 `rt/lowcmd`，适合把速度交给另一个已经验证的低层控制器。

## 1. Orin 环境

推荐使用 JetPack 对应的 Ubuntu、ROS 2 和 aarch64 系统。以下命令假设 ROS 2 已安装；
先把变量设为设备上的发行版（JetPack 6 / Ubuntu 22.04 通常使用 `humble`）：

```bash
export ROS_DISTRO=humble
source "/opt/ros/${ROS_DISTRO}/setup.bash"
sudo apt update
sudo apt install -y git build-essential cmake python3-yaml python3-colcon-common-extensions \
  libboost-dev libboost-filesystem-dev libboost-program-options-dev \
  libeigen3-dev libfmt-dev libyaml-cpp-dev \
  ros-${ROS_DISTRO}-ament-cmake ros-${ROS_DISTRO}-diagnostic-msgs \
  ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-tf2-ros
```

本部署不依赖 grid-map；若还要编译 Stepit 的高度图订阅模块，再安装
`ros-${ROS_DISTRO}-grid-map-msgs` 和 `ros-${ROS_DISTRO}-grid-map-ros`。缺少它们时构建系统只会
禁用该模块，不影响本文的三个运行模式。

本分支默认使用 ONNX Runtime，首次构建会下载 aarch64 运行库。先用它完成正确性和
安全验证；只有在目标 Orin 上完成等价性测试后，再考虑换 TensorRT。

## 2. 获取 `dev` 分支并构建

```bash
mkdir -p ~/stepit_ws/src
cd ~/stepit_ws
git clone --branch dev https://github.com/X1a0ming/stepit.git src/stepit
git -C src/stepit submodule update --init extern/llu extern/robot_sdk/unitree_sdk2

ln -sfn "$PWD/src/stepit/scripts" "$PWD/scripts"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
./scripts/build.sh ROS2 --release -j 6 --workspace "$PWD" \
  --cmake-arg -DPython3_EXECUTABLE=/usr/bin/python3
```

若 Orin 内存较小，将 `-j 6` 调低到 `-j 2`。构建后安装和校验四个电机策略包：

```bash
cd ~/stepit_ws
./src/stepit/scripts/install_orin_joint_graph.sh --workspace "$PWD"
python3 ./src/stepit/scripts/verify_orin_joint_graph.py \
  --repo ./src/stepit --workspace "$PWD"
```

校验器会从各自的 manifest 动态核对模型 SHA-256、ONNX 输入输出、V12B 循环状态、
机器人 DoF、动作切片、拓扑和路径消息布局；无需手工复制校验值。

## 3. 输入接口

### 路径点

三个模式都订阅 `/stepit/path_points_body`：

- 类型：`std_msgs/msg/Float32MultiArray`
- 数据：20 个机器人机体坐标系下的 `(x, y)` 点，共 40 个 `float32`
- 布局：`point: size=20, stride=40`；`xy: size=2, stride=2`；`data_offset=0`
- QoS：Sensor Data / best effort / depth 1
- 推荐频率：10 Hz；最低要求是每 0.25 s 内至少到达一次有效消息

仓库提供了格式正确的测试发布器。零路径用于站立/零速接口检查；`--spacing`
仅用于低速、受控的前向测试：

```bash
source ~/stepit_ws/install/setup.bash
python3 ~/stepit_ws/src/stepit/scripts/publish_joint_graph_path.py --rate 10
```

### `cmd_vel` 模式额外输入/输出

- 输入 `/stepit/imu`：`sensor_msgs/msg/Imu`，必须提供有限的角速度和有效姿态四元数，
  推荐至少 50 Hz；超过 0.10 s 未更新会输出零速度并清空模型状态。
- 输出 `/stepit/cmd_vel`：`geometry_msgs/msg/Twist`，使用 `linear.x`、`linear.y`、
  `angular.z`，发布频率 50 Hz，默认逐轴限幅到 `[-2, 2]`。

话题名、超时和限幅可在 `deploy/orin/cmd_vel/params.yml` 中调整。

## 4. 直接输出 `cmd_vel`

该模式不需要 Unitree 网卡，也不会访问电机 DDS：

```bash
cd ~/stepit_ws
source "/opt/ros/${ROS_DISTRO}/setup.bash"
./src/stepit/scripts/run_orin_joint_graph.sh v12b cmd_vel --workspace "$PWD"
# 或使用 V12C
./src/stepit/scripts/run_orin_joint_graph.sh v12c cmd_vel --workspace "$PWD"
```

另一个终端启动 IMU 和路径发布方，然后确认输出：

```bash
source ~/stepit_ws/install/setup.bash
ros2 topic hz /stepit/cmd_vel
ros2 topic echo /stepit/cmd_vel
```

先保持零路径。停止 IMU 或路径发布后，必须观察到 `/stepit/cmd_vel` 在对应 watchdog
时间内变为零，再将它连接到真实低层控制器。

## 5. Go2 / B2W 电机模式

将 Orin 的有线网卡设为机器人网段，并确认只有计划使用的网卡接入机器人：

```bash
ip -brief address
ping -c 3 192.168.123.161
```

运行脚本要求显式给出非回环网卡，并以 `Resting` 状态启动：

```bash
cd ~/stepit_ws
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Go2 + V12B；把 enP8p1s0 换成实际机器人网卡
./src/stepit/scripts/run_orin_joint_graph.sh v12b go2_motor \
  --workspace "$PWD" --netif enP8p1s0

# B2W + V12C
./src/stepit/scripts/run_orin_joint_graph.sh v12c b2w_motor \
  --workspace "$PWD" --netif enP8p1s0
```

可选参数 `--unitree-domain` 设置 Unitree DDS domain，`--ros-domain` 设置 ROS 2 domain。
启动并不等于可以立即走动：控制台中必须人工执行状态切换。

### 首次上机门禁

1. 断开运动空间、悬空机器人，操作员持有硬件急停；不要先发布非零路径。
2. 保持 `Resting`，核对收到的关节位置、IMU 方向、机器人型号和 DoF。
3. Go2 核对 12 个关节顺序为每条腿 `hip, thigh, calf`，腿序为
   `FR, FL, RR, RL`。
4. B2W 必须逐轴验证 DDS 索引和正方向：前 12 个为上述腿关节，后 4 个预期为
   `FR, FL, RR, RL` 轮。当前配置源于训练/仿真契约，尚未在本 Orin 对应的真实 B2W
   硬件版本上完成闭环确认；任何一轴不符都应停止并修改 `B2WSpec`，不得继续接管。
5. 执行 `Agent/StandUp`，只检查站姿与小误差保持；异常时立即 `Agent/Resting` 或急停。
6. 保持零路径，再执行 `Agent/PolicyOn`，确认轮速/关节指令有限且机器人稳定。
7. 最后以短距离、低速路径测试，逐步扩大范围。丢失路径超过 0.25 s 时必须退出策略
   或回到安全状态。

B2W 的负功率保护当前处于禁用状态，因为仓库内没有该硬件版本的权威阈值。取得电池、
驱动和制动回灌限制后，应先补齐 `config/robot/b2w.yml` 的保护参数再进行自由运动测试。

## 6. 选择 V12B 还是 V12C

- V12B 使用循环状态，适合需要历史信息的跟踪，但每次输入超时都会清空状态。
- V12C 是单阶段前馈模型，运行和故障恢复更直接。
- 两者共享完全相同的路径协议、50 Hz 控制周期和机器人 profile，可用相同流程切换；
  上机选择应依据目标机器人上的轨迹误差、抖动和计算时延实测，而不是完成率。

切换前先回到 `Resting`，终止旧进程，再启动另一模型；不要让两个进程同时拥有
`rt/lowcmd` 或 `/stepit/cmd_vel`。

## 7. 故障处理

- `path_points_body` 被拒绝：使用仓库发布器检查 40 个数据和二维 layout 是否完全一致。
- `IMU/path stream became stale`：检查频率、QoS、ROS domain 和时间戳链；这是安全停机，
  不应通过放大超时掩盖。
- 找不到 `actor.onnx` 或校验失败：重新运行安装脚本；已有包需要替换时使用 `--force`，
  脚本会先保留带时间戳的备份。
- 找不到 Unitree 动态库：确认已初始化 `unitree_sdk2` 子模块，并通过本目录运行脚本；
  脚本会把对应 aarch64 库目录置于 `LD_LIBRARY_PATH` 前端。
- ROS 构建调用了 Conda 的 Python：退出 Conda，或保留构建命令中的
  `-DPython3_EXECUTABLE=/usr/bin/python3`；ROS 消息生成应使用系统 Python。
- 机器人运动方向错误：立即停止。不要靠重排路径或反转速度掩盖，应修正 RobotApi 的
  电机索引/方向表并重新完成悬空逐轴检查。
