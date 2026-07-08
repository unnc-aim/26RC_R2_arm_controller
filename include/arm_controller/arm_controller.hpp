#ifndef ARM_CONTROLLER__ARM_CONTROLLER_HPP_
#define ARM_CONTROLLER__ARM_CONTROLLER_HPP_

#include <chrono>
#include <string>

#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"
#include "custom_msgs/msg/write_dm_motor_position_control_with_speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_controller/motor_controller.hpp"
#include "arm_controller/sbus_parser.hpp"
#include "arm_controller/state_machine.hpp"

namespace arm_controller
{

struct JointTopics
{
  std::string read_topic;
  std::string write_topic;
};

class ArmControllerNode : public rclcpp::Node
{
public:
  explicit ArmControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void load_parameters();
  void setup_ros_interfaces();

  void sbus_callback(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg);
  void joint1_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
  void joint2_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
  void joint3_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
  void joint4_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
  void control_timer_callback();

  void apply_transition(const StateTransition & transition);
  void send_pose_target(const PoseTarget & target, const std::string & pose_name);
  void publish_target_command(const PoseTarget & target, float speed);
  [[nodiscard]] PoseTarget build_command_target(const PoseTarget & target, bool force_joint1_positive_direction) const;
  void send_disable_command_once();
  void log_status_warnings();

  static const char * state_to_string(ArmState state);

  SbusConfig sbus_config_{};
  JointTopics joint1_topics_{};
  JointTopics joint2_topics_{};
  JointTopics joint3_topics_{};
  JointTopics joint4_topics_{};

  double loop_hz_{1000.0};
  float motion_speed_{5.0F};
  float position_tolerance_{5.0F};
  bool prefer_joint1_positive_direction_to_show_high_{true};
  int warn_throttle_ms_{1000};

  PoseTarget manual_pose_{};
  PoseTarget show_low_pose_{};
  PoseTarget show_high_pose_{};

  custom_msgs::msg::ReadSBUSRC latest_sbus_{};
  bool has_sbus_message_{false};

  SbusParser sbus_parser_;
  StateMachine state_machine_;
  MotorController motor_controller_;

  rclcpp::Subscription<custom_msgs::msg::ReadSBUSRC>::SharedPtr sbus_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr joint1_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr joint2_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr joint3_sub_;
  rclcpp::Subscription<custom_msgs::msg::ReadDmMotor>::SharedPtr joint4_sub_;

  rclcpp::Publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr joint1_pub_;
  rclcpp::Publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr joint2_pub_;
  rclcpp::Publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr joint3_pub_;
  rclcpp::Publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr joint4_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  bool disable_command_sent_{false};
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__ARM_CONTROLLER_HPP_
