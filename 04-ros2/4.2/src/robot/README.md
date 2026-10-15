# 四足机器人 ROS 2 仿真与控制

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install mujoco
cd <workspace>
colcon build --packages-select quadruped_simulation
source install/setup.bash
ros2 launch quadruped_simulation simulation.launch.py
```

启动文件会一并启动 `joy_node` 和手柄遥控节点。连接 Xbox 兼容手柄后，默认按 A 键站起、
按 B 键趴下；默认使用 Linux joystick 设备 0。若只运行仿真和控制器，不启动手柄相关节点：

```bash
ros2 launch quadruped_simulation simulation.launch.py use_gamepad:=false
```

若手柄设备编号不同，可运行：

```bash
ros2 launch quadruped_simulation simulation.launch.py device_id:=1
```

仿真需要支持 OpenGL 的图形桌面环境。MuJoCo 模型和 `meshes/` 网格会随 ROS 包安装，
保持模型引用的相对路径。也可以在不同终端分别启动以下节点：

```bash
ros2 run quadruped_simulation simulation_node
ros2 run quadruped_simulation controller_node
ros2 run joy joy_node
ros2 run quadruped_simulation gamepad_node
```

若从源码直接运行 `colcon`，请先将本项目目录放入 ROS 2 工作区的 `src/` 下。

## 节点与话题

- `mujoco_simulation`（Python）：运行 MuJoCo 与查看器，发布 `/joint_states`、
  `/motor_state` 和 `/imu/data`，订阅 `/mit_command`；查看器中按 S 站起、按 X 趴下。
- `gamepad_remote`（Python）：订阅 `/joy` 和控制器的 `/control_status`，发布
  `/control_mode` 请求；Xbox A 键请求站起，B 键请求趴下。
- `quadruped_controller`（C++）：订阅 `/motor_state`、`/imu/data` 和 `/control_mode`，
  发布 `/mit_command` 与当前 `/control_status`。手柄节点收到控制器发布的状态作为回报。
- `/joint_states` 使用 `sensor_msgs/msg/JointState` 供可视化；电机状态和指令分别使用
  `my_robot_interfaces/msg/MotorState` 与 `my_robot_interfaces/msg/MitCommand`；
  `/control_mode` 和 `/control_status` 使用 `std_msgs/msg/String`，模式包括 `standing`、
  `crouching` 与 `damping`。

## 控制

- 启动时机器人处于低姿态；躯干使用自由根关节，在重力和地面接触作用下运动。
- 手柄 A 键站起、B 键趴下；也可在 MuJoCo 查看器中按 S 站起、按 X 趴下。12 个关节从
  切换模式时的实际角度出发，在 2 秒内平滑插值至对应目标。起立时读取 IMU 姿态，并对
  左右髋关节和前后大腿关节施加限幅补偿。
- 控制器启动时处于 `damping` 模式。该模式下控制器持续将关节当前位置作为位置伺服目标。
  仿真节点当前只使用 `/mit_command` 中的期望位置 `q`，不应用 `kp`、`kd`、`w`、`tau` 字段；执行器
  的位置和速度增益由 MuJoCo 模型配置。

模型包含碰撞地面和方向光源。仿真按 MuJoCo 模型步长推进，查看器以约 60 Hz 同步渲染。
