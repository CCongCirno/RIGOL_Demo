# RIGOL_Demo

## 项目简介

`RIGOL_Demo` 是一个基于 ROS 2 Humble 的机械臂轨迹绘制与控制演示项目。主要功能包括：

- 通过相机与屏幕分割检测获取屏幕区域
- 在画布上绘制轨迹，并将轨迹重投影到相机图像
- 将绘制路径转换为机械臂目标位姿
- 使用 MoveIt 进行笛卡尔直线运动与 OMPL 路径规划
- 支持调参窗口输出和轨迹绘制调试

## 目录结构

- `src/rigol_demo/launch/`：ROS 2 启动文件
- `src/rigol_demo/configs/`：轨迹绘制节点配置文件
- `src/rigol_demo/src/module/`：主要节点实现代码
- `requirements.txt`：Python 依赖

## 环境依赖

1. ROS 2 Humble
2. Python 3.10+
3. `mujoco-py==2.1.2.14`（如果需要运行 Python 代码或依赖项）
4. OpenCV、`yaml-cpp`、MoveIt、`cv_bridge` 等 ROS 依赖

## 安装与构建

```bash
cd RIGOL_Demo
source /opt/ros/humble/setup.bash

# 安装 ROS 依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 安装 Python 依赖
python3 -m pip install -r requirements.txt
python3 -m pip install python-can
python3 -m pip install scipy
python3 -m pip install piper_sdk

# 安装 ROS2 控制相关包
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager

# 安装 CAN 工具
sudo apt update && sudo apt install can-utils ethtool

# 编译项目
colcon build --packages-select rigol_demo --symlink-install

# 编译成功后加载环境
source install/setup.bash
```

> 如果你希望编译整个工作区，可以去掉 `--packages-select rigol_demo`。

> 注：`python-can` 版本应高于 `4.3.1`。

## 安装 CAN 工具

本项目使用 `can-utils` 和 `ethtool` 配置 CAN 模块。

```bash
bash find_all_can_port.sh
```

如果可以检测到 CAN 模块，输出类似：

```text
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

直接执行激活 CAN 模块：

```bash
bash can_activate.sh can0 1000000
```

## 轨迹绘制节点运行

轨迹绘制节点使用 `trajectory_draw.launch.py` 启动，默认会加载 `src/rigol_demo/configs/trajectory_draw.yaml`。

```bash
source install/setup.bash
ros2 launch rigol_demo trajectory_draw.launch.py
```

### 启用调参窗口

```bash
ros2 launch rigol_demo trajectory_draw.launch.py \
  enable_segmentation_debug:=true \
  show_reproject_windows:=true
```

### 常用参数说明

- `config_file`：配置文件路径，默认指向 `src/rigol_demo/configs/trajectory_draw.yaml`
- `enable_segmentation_debug`：是否显示颜色梯度分割调试窗口
- `show_reproject_windows`：是否显示重投影窗口

## Piper 控制与 MoveIt 运行

使用 `piper_control.launch.py` 启动机械臂控制节点、Piper 硬件接口以及 MoveIt 演示。

```bash
source install/setup.bash
ros2 launch rigol_demo piper_control.launch.py
```

### 可选启动参数

- `can_port`：CAN 端口，默认 `can0`
- `auto_enable`：是否自动使能机械臂，默认 `true`
- `gripper_exist`：是否存在夹爪，默认 `false`
- `log_level`：日志级别，默认 `warn`
- `batch_mode`：是否批量模式运行，默认 `true`
- `cartesian_speed`：笛卡尔运动速度，默认 `0.1`
- `planning_time`：MoveIt 规划时间，默认 `5.0`

示例：

```bash
ros2 launch rigol_demo piper_control.launch.py \
  can_port:=can0 \
  auto_enable:=true \
  gripper_exist:=false \
  batch_mode:=true \
  cartesian_speed:=0.08
```

## 仅启动 MoveIt 目标位姿节点

如果你仅需要启动 MoveIt 和 `/target_pose` 接收节点：

```bash
source install/setup.bash
ros2 launch rigol_demo moveit_pose_goal.launch.py
```

## 调试与配置

`src/rigol_demo/configs/trajectory_draw.yaml` 是轨迹绘制节点的主配置文件。常用项包括：

- `rgb_topic`, `depth_topic`, `camera_info_topic`：相机话题名称
- `canvas_width`, `canvas_height`：绘制画布尺寸
- `sample_step_px`：轨迹离散化采样间距
- `z_draw_height`：绘制高度偏移
- `tool_offset`：工具中心点偏移
- `use_screen_plane_pose`：是否使用屏幕拟合姿态
- `use_home_orientation`：是否使用固定 home 姿态
- `enable_segmentation_debug`：是否开启分割调参
- `show_reproject_windows`：是否显示重投影窗口

## 运行顺序建议

1. 启动 ROS 2 环境：`source /opt/ros/humble/setup.bash`
2. 编译并加载工作区：`source install/setup.bash`
3. 启动 `trajectory_draw.launch.py` 进行轨迹绘制
4. 在新的终端启动 `piper_control.launch.py` 执行轨迹控制

## 备注

- 若使用真实硬件，请确保 `can_port` 与物理 CAN 端口匹配。
- 若使用仿真或仅测试轨迹绘制，可先单独运行 `trajectory_draw.launch.py`。
- 如果出现依赖项缺失，请先检查 `rosdep install` 是否成功。

---

感谢使用 `RIGOL_Demo`，如需更多调试帮助，请查看 `src/rigol_demo/configs/trajectory_draw.yaml` 中的参数说明。