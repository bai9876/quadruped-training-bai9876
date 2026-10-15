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

JOINT_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)
STARTUP_JOINT_POSITIONS = (
    0.0, 1.5, -2.5,
    0.0, -1.5, 2.5,
    0.0, -1.5, 2.5,
    0.0, 1.5, -2.5,
)
STARTUP_ROOT_HEIGHT = -0.42

class MuJoCoSimulationNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_simulation")
        default_model_path = f"{get_package_share_directory('quadruped_simulation')}/black_description.xml"
        self.declare_parameter("model_path", default_model_path)
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        self._joint_qpos_addresses: list[int] = []
        self._joint_dof_addresses: list[int] = []
        self._actuator_ids: list[int] = []
        self._imu_sensor_address = self._validate_model()
        self._initialize_state()

        # 状态反馈发布者：向控制器发送电机状态 (自定义 MotorState)
        self._motor_state_publisher = self.create_publisher(MotorState, "/motor_state", 10)
        # 保留标准的 JointState 发布者，方便 RViz 可视化
        self._joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        
        self._imu_publisher = self.create_publisher(Imu, "/imu/data", 10)
        self._mode_publisher = self.create_publisher(String, "/control_mode", 10)
        
        # 指令订阅者：接收控制器的 MIT 指令 (自定义 MitCommand)
        self._command_subscription = self.create_subscription(
            MitCommand, "/mit_command", self._command_callback, 10
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
        return int(self.model.sensor_adr[imu_sensor_id])

    def _initialize_state(self) -> None:
        root_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        root_qpos_address = int(self.model.jnt_qposadr[root_joint_id])
        self.data.qpos[root_qpos_address : root_qpos_address + 3] = (0.0, 0.0, STARTUP_ROOT_HEIGHT)
        for address, actuator_id, position in zip(
            self._joint_qpos_addresses, self._actuator_ids, STARTUP_JOINT_POSITIONS, strict=True,
        ):
            self.data.qpos[address] = position
            self.data.ctrl[actuator_id] = position
        mujoco.mj_forward(self.model, self.data)

    def set_viewer(self, viewer: mujoco.viewer.Handle) -> None:
        self.viewer = viewer

    def _command_callback(self, message: MitCommand) -> None:
        """接收自定义 MIT 控制指令"""
        if len(message.q) != 12:
            self.get_logger().warning("收到的指令长度不是12，忽略")
            return

        if self.viewer is None:
            return

        
        with self.viewer.lock():
            for i, actuator_id in enumerate(self._actuator_ids):
                q_des = message.q[i]
                kp = message.kp[i]
               
                if not (math.isfinite(q_des) and math.isfinite(kp)):
                    continue
                
                if kp<0.001:
                    self.data.ctrl[actuator_id] = self.data.qpos[self._joint_qpos_addresses[i]]
                else:
                    self.data.ctrl[actuator_id] = q_des    
             
    def _simulation_tick(self) -> None:
        if self.viewer is None:
            return
        with self.viewer.lock():
            mujoco.mj_step(self.model, self.data)
            stamp = self.get_clock().now().to_msg()

            # 1. 发布标准的 JointState (用于 RViz)
            joint_state = JointState()
            joint_state.header.stamp = stamp
            joint_state.name = list(JOINT_NAMES)
            joint_state.position = [float(self.data.qpos[addr]) for addr in self._joint_qpos_addresses]
            joint_state.velocity = [float(self.data.qvel[addr]) for addr in self._joint_dof_addresses]
            self._joint_publisher.publish(joint_state)

            # 2. 发布自定义的 MotorState (反馈给控制器)
            motor_state = MotorState()
            for i in range(12):
                motor_state.q[i] = float(self.data.qpos[self._joint_qpos_addresses[i]])
                motor_state.dq[i] = float(self.data.qvel[self._joint_dof_addresses[i]])
                # MuJoCo 加速度在 qacc 中
                motor_state.ddq[i] = float(self.data.qacc[self._joint_dof_addresses[i]])
                # 执行器施加的实际力矩 (actuator_force)
                motor_state.tau[i] = float(self.data.actuator_force[self._actuator_ids[i]])
                # 电流估算 (如果没有电流传感器，此处仅为演示估算)
                motor_state.cur[i] = motor_state.tau[i] * 0.1 
            self._motor_state_publisher.publish(motor_state)

            # 3. 发布 IMU 数据
            w, x, y, z = self.data.sensordata[self._imu_sensor_address : self._imu_sensor_address + 4]
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "imu_link"
            imu.orientation.w = float(w)
            imu.orientation.x = float(x)
            imu.orientation.y = float(y)
            imu.orientation.z = float(z)
            self._imu_publisher.publish(imu)

    def _key_callback(self, key: int) -> None:
        mode = "standing" if key == ord("S") else "damping" if key == ord("X") else None
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
            node.get_logger().info(
                "Simulation running; press S to stand and X to crouch, "
                "or use the Xbox gamepad"
            )
            next_render = time.perf_counter()
            while rclpy.ok() and viewer.is_running():
                executor.spin_once(timeout_sec=0.001)
                now = time.perf_counter()
                if now >= next_render:
                    viewer.sync()
                    next_render = now + 1.0 / 60.0
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
