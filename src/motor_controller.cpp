#include "arm_controller/motor_controller.hpp"

#include <cmath>

namespace arm_controller
{

void MotorController::update_joint1_feedback(const custom_msgs::msg::ReadDmMotor & msg)
{
  joint1_feedback_ = msg;
}

void MotorController::update_joint2_feedback(const custom_msgs::msg::ReadDmMotor & msg)
{
  joint2_feedback_ = msg;
}

void MotorController::update_joint3_feedback(const custom_msgs::msg::ReadDmMotor & msg)
{
  joint3_feedback_ = msg;
}

void MotorController::update_joint4_feedback(const custom_msgs::msg::ReadDmMotor & msg)
{
  joint4_feedback_ = msg;
}

bool MotorController::feedback_ready() const
{
  return joint1_feedback_.has_value() &&
         joint2_feedback_.has_value() &&
         joint3_feedback_.has_value() &&
         joint4_feedback_.has_value();
}

bool MotorController::target_reached(float tolerance) const
{
  if (!active_target_.has_value() || !feedback_ready()) {
    return false;
  }

  const float joint1_error = std::fabs(active_target_->joint1 - joint1_feedback_->position);
  const float joint2_error = std::fabs(active_target_->joint2 - joint2_feedback_->position);
  const float joint3_error = std::fabs(active_target_->joint3 - joint3_feedback_->position);
  const float joint4_error = std::fabs(active_target_->joint4 - joint4_feedback_->position);

  return joint1_error < tolerance &&
         joint2_error < tolerance &&
         joint3_error < tolerance &&
         joint4_error < tolerance;
}

void MotorController::set_active_target(const PoseTarget & target)
{
  active_target_ = target;
  motion_completed_ = false;
}

void MotorController::clear_active_target()
{
  active_target_.reset();
  motion_completed_ = false;
}

void MotorController::mark_motion_completed()
{
  motion_completed_ = true;
}

bool MotorController::motion_completed() const
{
  return motion_completed_;
}

bool MotorController::has_active_target() const
{
  return active_target_.has_value();
}

const std::optional<PoseTarget> & MotorController::active_target() const
{
  return active_target_;
}

const std::optional<custom_msgs::msg::ReadDmMotor> & MotorController::joint1_feedback() const
{
  return joint1_feedback_;
}

const std::optional<custom_msgs::msg::ReadDmMotor> & MotorController::joint2_feedback() const
{
  return joint2_feedback_;
}

const std::optional<custom_msgs::msg::ReadDmMotor> & MotorController::joint3_feedback() const
{
  return joint3_feedback_;
}

const std::optional<custom_msgs::msg::ReadDmMotor> & MotorController::joint4_feedback() const
{
  return joint4_feedback_;
}

bool MotorController::joint1_has_fault() const
{
  return joint1_feedback_.has_value() && has_fault(joint1_feedback_.value());
}

bool MotorController::joint2_has_fault() const
{
  return joint2_feedback_.has_value() && has_fault(joint2_feedback_.value());
}

bool MotorController::joint3_has_fault() const
{
  return joint3_feedback_.has_value() && has_fault(joint3_feedback_.value());
}

bool MotorController::joint4_has_fault() const
{
  return joint4_feedback_.has_value() && has_fault(joint4_feedback_.value());
}

bool MotorController::has_fault(const custom_msgs::msg::ReadDmMotor & msg)
{
  return (msg.online == 0U) ||
         (msg.communication_lost != 0U) ||
         (msg.overvoltage != 0U) ||
         (msg.undervoltage != 0U) ||
         (msg.overcurrent != 0U) ||
         (msg.mos_overtemperature != 0U) ||
         (msg.rotor_overtemperature != 0U) ||
         (msg.overload != 0U);
}

}  // namespace arm_controller
