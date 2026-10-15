#!/usr/bin/env python3
"""MuJoCo simulation ROS 2 node for the quadruped."""

import math
import time

import mujoco
import mujoco.viewer
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from my_robot_interfaces.msg import MitCommand, MotorState
from robot_msgs.msg import RobotState, RobotCommand, MotorState as RL_MotorState

JOINT_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
)
DEFAULT_ROBOT_NAME = "GO_2026"
STARTUP_JOINT_POSITIONS_BY_ROBOT = {
    "GO_2026": (
        0.0, 0.8014, -1.527,
        0.0, -0.8014, 1.527,
        0.0, 0.8014, -1.527,
        0.0, -0.8014, 1.527,
    ),
}
STARTUP_ROOT_HEIGHT = -0.1462
GLFW_KEY_F8 = 297
GLFW_KEY_F9 = 298
GLFW_KEY_F10 = 299
STATE_PUBLISH_DIVIDER = 4
RENDER_FPS = 15


class MuJoCoSimulationNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_simulation")
        self.declare_parameter("robot_name", DEFAULT_ROBOT_NAME)
        self.declare_parameter("model_path", f"{get_package_share_directory('simulation_description')}/black_description.xml")
        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.robot_name = robot_name

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = 0.005
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        self._state_publish_counter = 0
        self._joint_qpos_addresses: list[int] = []
        self._joint_dof_addresses: list[int] = []
        self._actuator_ids: list[int] = []
        self._imu_sensor_address = self._validate_model()
        self._initialize_state()

        # ---------- 发布者 ----------
        # 保留给你自己调试用
        self._motor_state_publisher = self.create_publisher(MotorState, "/motor_state", 10)
        self._joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._imu_publisher = self.create_publisher(Imu, "/imu", 10)      # rl_sim 期望的话题名
        self._mode_publisher = self.create_publisher(String, "/control_mode", 10)

        # 给 rl_sim 读的状态
        self._rl_state_publisher = self.create_publisher(
            RobotState, "/robot_joint_controller/state", 10
        )

        # ---------- 订阅者 ----------
        # 保留：自己手动调试用
        self._command_subscription = self.create_subscription(
            MitCommand, "/mit_command", self._command_callback, 10
        )

        # rl_sim 的命令
        self._rl_cmd_sub = self.create_subscription(
            RobotCommand, "/robot_joint_controller/command", self._rl_cmd_cb, 10
        )

        self._timer = self.create_timer(self.model.opt.timestep, self._simulation_tick)

    def _validate_model(self) -> int:
        root_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        imu_sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_orientation")
        if (root_joint_id < 0 or self.model.jnt_type[root_joint_id] != mujoco.mjtJoint.mjJNT_FREE or
                imu_sensor_id < 0 or self.model.sensor_dim[imu_sensor_id] != 4):
            raise ValueError("Model must contain a free root joint and a 4D IMU orientation sensor")

        for name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_servo")
            if joint_id < 0 or actuator_id < 0:
                raise ValueError(f"Model is missing joint or actuator for {name}")
            if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"{name} must be a hinge joint")
            if self.model.actuator_trnid[actuator_id, 0] != joint_id:
                raise ValueError(f"Actuator for {name} is attached to the wrong joint")

            self._joint_qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
            self._joint_dof_addresses.append(int(self.model.jnt_dofadr[joint_id]))
            self._actuator_ids.append(actuator_id)

        self._root_dof_address = int(self.model.jnt_dofadr[root_joint_id])
        return int(self.model.sensor_adr[imu_sensor_id])

    def _initialize_state(self) -> None:
        root_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        root_qpos_address = int(self.model.jnt_qposadr[root_joint_id])
        start_joint_positions = STARTUP_JOINT_POSITIONS_BY_ROBOT.get(self.robot_name, STARTUP_JOINT_POSITIONS_BY_ROBOT[DEFAULT_ROBOT_NAME])
        self.data.qpos[root_qpos_address : root_qpos_address + 3] = (0.0, 0.0, STARTUP_ROOT_HEIGHT)
        for address, actuator_id, position in zip(
            self._joint_qpos_addresses, self._actuator_ids, start_joint_positions, strict=True,
        ):
            self.data.qpos[address] = position
            self.data.ctrl[actuator_id] = 0.0     # motor 执行器，ctrl 是力矩，初值 0
        mujoco.mj_forward(self.model, self.data)

    def set_viewer(self, viewer: mujoco.viewer.Handle) -> None:
        self.viewer = viewer

    # ---------- 保留：你自己的 MIT 手动调试通道 ----------
    def _command_callback(self, message: MitCommand) -> None:
        if len(message.q) != 12 or len(message.kp) != 12 or len(message.kd) != 12:
            self.get_logger().warning("收到的指令长度不是12，忽略")
            return
        if self.viewer is None:
            return
        with self.viewer.lock():
            for i, actuator_id in enumerate(self._actuator_ids):
                q_des = message.q[i]
                kp = message.kp[i]
                kd = message.kd[i]
                if not (math.isfinite(q_des) and math.isfinite(kp) and math.isfinite(kd)):
                    continue
                q = self.data.qpos[self._joint_qpos_addresses[i]]
                dq = self.data.qvel[self._joint_dof_addresses[i]]
                tau = kp * (q_des - q) - kd * dq
                tau = max(min(tau, 33.5), -33.5)
                self.data.ctrl[actuator_id] = tau

    # ---------- rl_sim 的命令 ----------
    def _rl_cmd_cb(self, message: RobotCommand) -> None:
        if self.viewer is None:
            return
        if len(message.motor_command) != 12:
            return
        with self.viewer.lock():
            for i, mc in enumerate(message.motor_command):
                actuator_id = self._actuator_ids[i]
                q = self.data.qpos[self._joint_qpos_addresses[i]]
                dq = self.data.qvel[self._joint_dof_addresses[i]]
                # 标准 PD + 前馈力矩
                tau = mc.kp * (mc.q - q) + mc.kd * (mc.dq - dq) + mc.tau
                tau = max(min(tau, 33.5), -33.5)
                self.data.ctrl[actuator_id] = tau

    def _simulation_tick(self) -> None:
        if self.viewer is None:
            return
        with self.viewer.lock():
            mujoco.mj_step(self.model, self.data)
            self._state_publish_counter += 1
            if self._state_publish_counter < STATE_PUBLISH_DIVIDER:
                return
            self._state_publish_counter = 0
            stamp = self.get_clock().now().to_msg()

            # 1. JointState（RViz 用）
            joint_state = JointState()
            joint_state.header.stamp = stamp
            joint_state.name = list(JOINT_NAMES)
            joint_state.position = [float(self.data.qpos[addr]) for addr in self._joint_qpos_addresses]
            joint_state.velocity = [float(self.data.qvel[addr]) for addr in self._joint_dof_addresses]
            self._joint_publisher.publish(joint_state)

            # 2. 你自己的 MotorState（保留调试通道）
            motor_state = MotorState()
            for i in range(12):
                motor_state.q[i] = float(self.data.qpos[self._joint_qpos_addresses[i]])
                motor_state.dq[i] = float(self.data.qvel[self._joint_dof_addresses[i]])
                motor_state.ddq[i] = float(self.data.qacc[self._joint_dof_addresses[i]])
                motor_state.tau[i] = float(self.data.actuator_force[self._actuator_ids[i]])
                motor_state.cur[i] = motor_state.tau[i] * 0.1
            self._motor_state_publisher.publish(motor_state)

            # 3. rl_sim 期望的 RobotState
            rl_state = RobotState()
            w, x, y, z = self.data.sensordata[self._imu_sensor_address : self._imu_sensor_address + 4]
            rl_state.imu.quaternion = [float(w), float(x), float(y), float(z)]
            rl_state.imu.gyroscope = [
                float(self.data.qvel[self._root_dof_address + 3]),
                float(self.data.qvel[self._root_dof_address + 4]),
                float(self.data.qvel[self._root_dof_address + 5]),
            ]
            rl_state.imu.accelerometer = [0.0, 0.0, 0.0]
            rl_state.motor_state = []                  # 变长列表，逐个 append
            for i in range(12):
                ms = RL_MotorState()
                ms.q       = float(self.data.qpos[self._joint_qpos_addresses[i]])
                ms.dq      = float(self.data.qvel[self._joint_dof_addresses[i]])
                ms.ddq     = float(self.data.qacc[self._joint_dof_addresses[i]])   # 有了
                ms.tau_est = float(self.data.actuator_force[self._actuator_ids[i]])
                ms.cur     = ms.tau_est * 0.1                                       # 估算，没传感器
                rl_state.motor_state.append(ms)
            self._rl_state_publisher.publish(rl_state)

            # 4. IMU
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "imu_link"
            imu.orientation.w = float(w)
            imu.orientation.x = float(x)
            imu.orientation.y = float(y)
            imu.orientation.z = float(z)
            imu.angular_velocity.x = float(self.data.qvel[self._root_dof_address + 3])
            imu.angular_velocity.y = float(self.data.qvel[self._root_dof_address + 4])
            imu.angular_velocity.z = float(self.data.qvel[self._root_dof_address + 5])
            self._imu_publisher.publish(imu)

    def _key_callback(self, key: int) -> None:
        mode_by_key = {
            GLFW_KEY_F8: "standing",
            GLFW_KEY_F9: "crouching",
            GLFW_KEY_F10: "damping",
        }
        mode = mode_by_key.get(key)
        if mode is None:
            return
        message = String()
        message.data = mode
        self._mode_publisher.publish(message)
        self.get_logger().info(f"Keyboard requested {mode} mode")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MuJoCoSimulationNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        with mujoco.viewer.launch_passive(
            node.model, node.data, key_callback=node._key_callback
        ) as viewer:
            node.set_viewer(viewer)
            node.get_logger().info("Simulation running.")
            next_render = time.perf_counter()
            while rclpy.ok() and viewer.is_running():
                executor.spin_once(timeout_sec=0.001)
                now = time.perf_counter()
                if now >= next_render:
                    viewer.sync()
                    next_render = now + 1.0 / RENDER_FPS
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
