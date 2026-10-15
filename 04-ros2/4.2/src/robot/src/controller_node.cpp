#include <cstddef>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // 保留，供 RViz 观察
#include "std_msgs/msg/string.hpp"
#include "my_robot_interfaces/msg/mit_command.hpp"
#include "my_robot_interfaces/msg/motor_state.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t kJointCount = 12;
constexpr double kTransitionSeconds = 4.0;
constexpr double kPostureDeadzone = 0.02;
constexpr double kMaxPostureCorrection = 0.25;
constexpr double kRollCompensationGain = -0.5;
constexpr double kPitchCompensationGain = -0.5;

const std::array<std::string, kJointCount> kJointNames = {
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

const std::array<double, kJointCount> kStandingTargets = {
  0.0, 0.45, -0.85,
  0.0, -0.45, 0.85,
  0.0, -0.45, 0.85,
  0.0, 0.45, -0.85};

const std::array<double, kJointCount> kCrouchingTargets = {
  0.0, 1.5, -2.5,
  0.0, -1.5, 2.5,
  0.0, -1.5, 2.5,
  0.0, 1.5, -2.5};

double clamp_value(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

bool ends_with(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

class QuadrupedControllerNode : public rclcpp::Node
{
public:
  QuadrupedControllerNode()
  : Node("quadruped_controller"),
    transition_elapsed_(kTransitionSeconds),
    mode_(Mode::kDamping),
    last_update_(std::chrono::steady_clock::now())
  {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      joint_indices_.emplace(kJointNames[i], i);
    }

    // 发布者：自定义 MitCommand
    command_publisher_ = create_publisher<my_robot_interfaces::msg::MitCommand>("/mit_command", 10);

    // 订阅者：自定义 MotorState
    joint_subscription_ = create_subscription<my_robot_interfaces::msg::MotorState>(
      "/motor_state", 10,
      std::bind(&QuadrupedControllerNode::joint_state_callback, this, std::placeholders::_1));

    // 订阅 IMU 和控制模式（保持不变）
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&QuadrupedControllerNode::imu_callback, this, std::placeholders::_1));
    mode_subscription_ = create_subscription<std_msgs::msg::String>(
      "/control_mode", 10,
      std::bind(&QuadrupedControllerNode::mode_callback, this, std::placeholders::_1));
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/control_status", rclcpp::QoS(1).transient_local());
    publish_mode_status();
      
    timer_ = create_wall_timer(10ms, std::bind(&QuadrupedControllerNode::control_tick, this));
  }

private:
  enum class Mode
  {
    kDamping,
    kStanding,
    kCrouching
  };

  // 回调函数：接收 MotorState
  void joint_state_callback(const my_robot_interfaces::msg::MotorState::SharedPtr message)
  {
    // 直接通过索引对应，无需再通过名字查找，提高效率
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      const double position = message->q[i];
      if (!std::isfinite(position)) {
        continue;
      }
      joint_positions_[i] = position;
    }

    has_joint_state_ = true;
    
    if (has_joint_state_ && transition_pending_)
    {
      transition_start_ = joint_positions_;
      transition_elapsed_ = 0.0;
      transition_pending_ = false;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const auto & q = message->orientation;
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!std::isfinite(norm) || norm < 1.0e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring invalid IMU orientation quaternion");
      return;
    }

    const double w = q.w / norm;
    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    roll_ = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double sin_pitch = clamp_value(2.0 * (w * y - z * x), -1.0, 1.0);
    pitch_ = std::asin(sin_pitch);
    has_imu_ = true;
  }

  void mode_callback(const std_msgs::msg::String::SharedPtr message)
  {
    Mode requested_mode;
    if (message->data == "standing") {
      requested_mode = Mode::kStanding;
    } else if (message->data == "crouching") {
      requested_mode = Mode::kCrouching;
    } else if (message->data == "damping") {
      requested_mode = Mode::kDamping;
    } else {
      RCLCPP_WARN(get_logger(), "Ignoring unknown control mode '%s'", message->data.c_str());
      return;
    }

    if (requested_mode == mode_) {
      return;
    }
    mode_ = requested_mode;
    if (mode_ == Mode::kStanding || mode_ == Mode::kCrouching) {
      if (has_joint_state_) {
        transition_start_ = joint_positions_;
        transition_elapsed_ = 0.0;
      } else {
        transition_pending_ = true;
      }
    } else {
      transition_pending_ = false;
    }
    last_update_ = std::chrono::steady_clock::now();
    publish_mode_status();
    RCLCPP_INFO(
      get_logger(), "Control mode: %s",
      mode_ == Mode::kStanding ? "standing" :
      mode_ == Mode::kCrouching ? "crouching" : "damping");
  }

  void publish_mode_status()
  {
    auto status = std_msgs::msg::String();
    status.data =
      mode_ == Mode::kStanding ? "standing" :
      mode_ == Mode::kCrouching ? "crouching" : "damping";
    status_publisher_->publish(status);
  }

  static double compensate(const double angle, const double gain)
  {
    if (std::abs(angle) <= kPostureDeadzone) {
      return 0.0;
    }
    return clamp_value(-gain * angle, -kMaxPostureCorrection, kMaxPostureCorrection);
  }

  void control_tick()
  {
    if (!has_joint_state_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double timestep = std::chrono::duration<double>(now - last_update_).count();
    last_update_ = now;

    double smooth_progress = 0.0;
    double roll_correction = 0.0;
    double pitch_correction = 0.0;
    if (mode_ == Mode::kStanding || mode_ == Mode::kCrouching) {
      transition_elapsed_ = std::min(transition_elapsed_ + timestep, kTransitionSeconds);
      const double progress = transition_elapsed_ / kTransitionSeconds;
      smooth_progress = progress * progress * (3.0 - 2.0 * progress);
      if (mode_ == Mode::kStanding && has_imu_) {
        roll_correction = compensate(roll_, kRollCompensationGain);
        pitch_correction = compensate(pitch_, kPitchCompensationGain);
      }
    }

    // 构造自定义 MitCommand
    auto command = my_robot_interfaces::msg::MitCommand();

    for (std::size_t i = 0; i < kJointCount; ++i) {
      double desired_q = joint_positions_[i];
      
      if (mode_ == Mode::kStanding || mode_ == Mode::kCrouching) {
        const auto & targets =
          mode_ == Mode::kStanding ? kStandingTargets : kCrouchingTargets;
        desired_q = transition_start_[i] +
          (targets[i] - transition_start_[i]) * smooth_progress;
        if (mode_ == Mode::kStanding) {
          const std::string & joint_name = kJointNames[i];
          if (ends_with(joint_name, "_hip_joint")) {
            const double side =
              starts_with(joint_name, "FL") || starts_with(joint_name, "RL") ? 1.0 : -1.0;
            desired_q += side * roll_correction;
          } else if (ends_with(joint_name, "_thigh_joint")) {
            const double pitch_sign =
              starts_with(joint_name, "FL") || starts_with(joint_name, "RL") ? 1.0 : -1.0;
            desired_q += pitch_sign * pitch_correction;
          }
        }
      }

      // 填充 MIT 参数 (12个电机的指令)
      if (mode_ == Mode::kDamping) {
        command.q[i] = 0.0;   // 期望位置
        command.kp[i] = 0.0;       // 刚度 (设定一个基准值，MuJoCo 位置伺服会以此为参考
        command.kd[i] = 2.0;        // 阻尼
        command.w[i] = 0.0;         // 期望速度
        command.tau[i] = 0.0;       // 前馈力矩
      } else {
        command.q[i] = desired_q;   // 期望位置
        command.kp[i] = 80.0;       // 刚度 (设定一个基准值，MuJoCo 位置伺服会以此为参考
        command.kd[i] = 1.0;        // 阻尼
        command.w[i] = 0.0;         // 期望速度
        command.tau[i] = 0.0;       // 前馈力矩
      }
    }

    command_publisher_->publish(command);
  }

  std::unordered_map<std::string, std::size_t> joint_indices_;
  std::array<double, kJointCount> joint_positions_{};
  std::array<double, kJointCount> transition_start_{};
  double transition_elapsed_;
  double roll_{0.0};
  double pitch_{0.0};
  bool has_joint_state_{false};
  bool has_imu_{false};
  bool transition_pending_{false};
  Mode mode_;
  std::chrono::steady_clock::time_point last_update_;

  rclcpp::Publisher<my_robot_interfaces::msg::MitCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<my_robot_interfaces::msg::MotorState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadrupedControllerNode>());
  rclcpp::shutdown();
  return 0;
}
