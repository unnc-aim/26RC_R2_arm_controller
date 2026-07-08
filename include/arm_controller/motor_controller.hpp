#ifndef ARM_CONTROLLER__MOTOR_CONTROLLER_HPP_
#define ARM_CONTROLLER__MOTOR_CONTROLLER_HPP_

#include <optional>

#include "custom_msgs/msg/read_dm_motor.hpp"

namespace arm_controller
{

struct PoseTarget
{
  float joint1{0.0F};
  float joint2{0.0F};
  float joint3{0.0F};
  float joint4{0.0F};
};

class MotorController
{
public:
  void update_joint1_feedback(const custom_msgs::msg::ReadDmMotor & msg);
  void update_joint2_feedback(const custom_msgs::msg::ReadDmMotor & msg);
  void update_joint3_feedback(const custom_msgs::msg::ReadDmMotor & msg);
  void update_joint4_feedback(const custom_msgs::msg::ReadDmMotor & msg);

  [[nodiscard]] bool feedback_ready() const;
  [[nodiscard]] bool target_reached(float tolerance) const;

  void set_active_target(const PoseTarget & target);
  void clear_active_target();

  void mark_motion_completed();
  [[nodiscard]] bool motion_completed() const;

  [[nodiscard]] bool has_active_target() const;
  [[nodiscard]] const std::optional<PoseTarget> & active_target() const;

  [[nodiscard]] const std::optional<custom_msgs::msg::ReadDmMotor> & joint1_feedback() const;
  [[nodiscard]] const std::optional<custom_msgs::msg::ReadDmMotor> & joint2_feedback() const;
  [[nodiscard]] const std::optional<custom_msgs::msg::ReadDmMotor> & joint3_feedback() const;
  [[nodiscard]] const std::optional<custom_msgs::msg::ReadDmMotor> & joint4_feedback() const;

  [[nodiscard]] bool joint1_has_fault() const;
  [[nodiscard]] bool joint2_has_fault() const;
  [[nodiscard]] bool joint3_has_fault() const;
  [[nodiscard]] bool joint4_has_fault() const;

private:
  [[nodiscard]] static bool has_fault(const custom_msgs::msg::ReadDmMotor & msg);

  std::optional<custom_msgs::msg::ReadDmMotor> joint1_feedback_;
  std::optional<custom_msgs::msg::ReadDmMotor> joint2_feedback_;
  std::optional<custom_msgs::msg::ReadDmMotor> joint3_feedback_;
  std::optional<custom_msgs::msg::ReadDmMotor> joint4_feedback_;
  std::optional<PoseTarget> active_target_;
  bool motion_completed_{false};
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__MOTOR_CONTROLLER_HPP_
