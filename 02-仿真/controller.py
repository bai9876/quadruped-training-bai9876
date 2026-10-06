"""MuJoCo state-machine controller for the quadruped model."""

from __future__ import annotations

import enum
import math
import queue
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer


MODEL_PATH = Path(__file__).with_name("black_description.xml")
TRANSITION_SECONDS = 2.0
RENDER_HZ = 60.0
STARTUP_ROOT_HEIGHT = -0.42
POSTURE_DEADZONE = 0.02
MAX_POSTURE_CORRECTION = 0.25
ROLL_COMPENSATION_GAIN = -0.5
PITCH_COMPENSATION_GAIN = -0.5

JOINT_NAMES = (
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
)

STANDING_TARGETS = {
    "FL_hip_joint": 0.0,
    "FL_thigh_joint": 0.45,
    "FL_calf_joint": -0.85,
    "FR_hip_joint": 0.0,
    "FR_thigh_joint": -0.45,
    "FR_calf_joint": 0.85,
    "RR_hip_joint": 0.0,
    "RR_thigh_joint": -0.45,
    "RR_calf_joint": 0.85,
    "RL_hip_joint": 0.0,
    "RL_thigh_joint": 0.45,
    "RL_calf_joint": -0.85,
}

STARTUP_JOINT_POSITIONS = {
    "FL_hip_joint": 0.0,
    "FL_thigh_joint": 1.5,
    "FL_calf_joint": -2.5,
    "FR_hip_joint": 0.0,
    "FR_thigh_joint": -1.5,
    "FR_calf_joint": 2.5,
    "RR_hip_joint": 0.0,
    "RR_thigh_joint": -1.5,
    "RR_calf_joint": 2.5,
    "RL_hip_joint": 0.0,
    "RL_thigh_joint": 1.5,
    "RL_calf_joint": -2.5,
}


class Mode(enum.Enum):
    DAMPING = "damping"
    STANDING = "standing"


class QuadrupedController:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self._request_lock = threading.Lock()
        self._requested_mode = Mode.DAMPING
        self._mode = Mode.DAMPING
        self._transition_elapsed = TRANSITION_SECONDS
        self._transition_start: list[float] = []

        root_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        imu_sensor_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_orientation"
        )
        if (
            root_joint_id < 0
            or model.jnt_type[root_joint_id] != mujoco.mjtJoint.mjJNT_FREE
            or imu_sensor_id < 0
            or model.sensor_dim[imu_sensor_id] != 4
        ):
            raise ValueError("Model must have a free root joint and IMU orientation sensor")
        root_qpos_address = int(model.jnt_qposadr[root_joint_id])
        self.data.qpos[root_qpos_address : root_qpos_address + 3] = (
            0.0,
            0.0,
            STARTUP_ROOT_HEIGHT,
        )
        self._imu_sensor_address = int(model.sensor_adr[imu_sensor_id])

        self._qpos_addresses: list[int] = []
        self._joint_ids: list[int] = []
        self._actuator_ids: list[int] = []
        self._standing_targets: list[float] = []
        for joint_name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint_name}_servo"
            )
            if joint_id < 0 or actuator_id < 0:
                raise ValueError(f"Model is missing joint or actuator for {joint_name}")
            if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"{joint_name} must be a hinge joint")
            if model.actuator_trnid[actuator_id, 0] != joint_id:
                raise ValueError(f"Actuator for {joint_name} is attached to the wrong joint")

            self._qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
            self._joint_ids.append(joint_id)
            self._actuator_ids.append(actuator_id)
            target = STANDING_TARGETS[joint_name]
            if model.jnt_limited[joint_id] and not (
                model.jnt_range[joint_id, 0]
                <= target
                <= model.jnt_range[joint_id, 1]
            ):
                raise ValueError(
                    f"Standing target for {joint_name} is outside its limits"
                )
            self._standing_targets.append(target)

        for address, actuator_id, target in zip(
            self._qpos_addresses,
            self._actuator_ids,
            (STARTUP_JOINT_POSITIONS[name] for name in JOINT_NAMES),
        ):
            self.data.qpos[address] = target
            self.data.ctrl[actuator_id] = target
        self._transition_start = [
            float(self.data.qpos[address]) for address in self._qpos_addresses
        ]
        mujoco.mj_forward(model, data)

    def _posture_corrections(self) -> tuple[float, float]:
        address = self._imu_sensor_address
        w, x, y, z = self.data.sensordata[address : address + 4]
        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )
        sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sin_pitch)

        def compensate(angle: float, gain: float) -> float:
            if abs(angle) <= POSTURE_DEADZONE:
                return 0.0
            correction = -gain * angle
            return max(
                -MAX_POSTURE_CORRECTION,
                min(MAX_POSTURE_CORRECTION, correction),
            )

        return (
            compensate(roll, ROLL_COMPENSATION_GAIN),
            compensate(pitch, PITCH_COMPENSATION_GAIN),
        )

    def handle_key(self, key: int) -> None:
        if key == ord("S"):
            requested_mode = Mode.STANDING
        elif key == ord("X"):
            requested_mode = Mode.DAMPING
        else:
            return

        with self._request_lock:
            self._requested_mode = requested_mode
        print(f"Requested mode: {requested_mode.value}")

    def step(self, timestep: float) -> None:
        with self._request_lock:
            requested_mode = self._requested_mode

        if requested_mode is not self._mode:
            self._mode = requested_mode
            if self._mode is not Mode.DAMPING:
                self._transition_start = [
                    float(self.data.qpos[address]) for address in self._qpos_addresses
                ]
                self._transition_elapsed = 0.0

        if self._mode is Mode.DAMPING:
            for address, actuator_id in zip(self._qpos_addresses, self._actuator_ids):
                self.data.ctrl[actuator_id] = self.data.qpos[address]
            return

        self._transition_elapsed = min(
            self._transition_elapsed + timestep, TRANSITION_SECONDS
        )
        progress = self._transition_elapsed / TRANSITION_SECONDS
        smooth_progress = progress * progress * (3.0 - 2.0 * progress)
        roll_correction, pitch_correction = (
            self._posture_corrections()
            if self._mode is Mode.STANDING
            else (0.0, 0.0)
        )
        for joint_name, joint_id, start, target, actuator_id in zip(
            JOINT_NAMES,
            self._joint_ids,
            self._transition_start,
            self._standing_targets,
            self._actuator_ids,
            strict=True,
        ):
            desired = start + (target - start) * smooth_progress
            if joint_name.endswith("_hip_joint"):
                side = 1.0 if joint_name.startswith(("FL", "RL")) else -1.0
                desired += side * roll_correction
            elif joint_name.endswith("_thigh_joint"):
                pitch_sign = 1.0 if joint_name.startswith(("FL", "RL")) else -1.0
                desired += pitch_sign * pitch_correction
            if self.model.jnt_limited[joint_id]:
                lower, upper = self.model.jnt_range[joint_id]
                desired = max(float(lower), min(float(upper), desired))
            self.data.ctrl[actuator_id] = desired


def simulate(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: QuadrupedController,
    viewer: mujoco.viewer.Handle,
    stop_event: threading.Event,
    errors: queue.SimpleQueue[Exception],
) -> None:
    timestep = model.opt.timestep
    next_step = time.perf_counter()
    try:
        while viewer.is_running() and not stop_event.is_set():
            next_step += timestep
            with viewer.lock():
                controller.step(timestep)
                mujoco.mj_step(model, data)

            delay = next_step - time.perf_counter()
            if delay > 0:
                stop_event.wait(delay)
            else:
                next_step = time.perf_counter()
    except Exception as error:
        errors.put(error)
        stop_event.set()


def main() -> None:
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(f"MuJoCo model not found: {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    controller = QuadrupedController(model, data)
    stop_event = threading.Event()
    errors: queue.SimpleQueue[Exception] = queue.SimpleQueue()

    print(
        "Startup damping mode active. Press S to stand up with IMU posture "
        "compensation; X returns to damping mode."
    )
    with mujoco.viewer.launch_passive(
        model, data, key_callback=controller.handle_key
    ) as viewer:
        simulation_thread = threading.Thread(
            target=simulate,
            args=(model, data, controller, viewer, stop_event, errors),
            name="mujoco-simulation",
            daemon=True,
        )
        simulation_thread.start()
        try:
            while viewer.is_running():
                try:
                    error = errors.get_nowait()
                except queue.Empty:
                    pass
                else:
                    raise RuntimeError("MuJoCo simulation thread failed") from error

                viewer.sync()
                time.sleep(1.0 / RENDER_HZ)
        finally:
            stop_event.set()
            simulation_thread.join()

        try:
            error = errors.get_nowait()
        except queue.Empty:
            pass
        else:
            raise RuntimeError("MuJoCo simulation thread failed") from error


if __name__ == "__main__":
    main()
