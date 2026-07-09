#ifndef ARM_CONTROLLER__ARM_CONTROLLER_HPP_
#define ARM_CONTROLLER__ARM_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

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

struct JointLimit
{
  float min_position{0.0F};
  float max_position{0.0F};
};

enum class PoseActionId : uint8_t
{
  POSE_INIT = 0,
  POSE_READ_KFS_BELOW = 1,
  POSE_READ_KFS_ABOVE = 2,
  POSE_READ_ARUCO_FORWARD = 3,
  POSE_CLIMB_R1 = 4,
  POSE_WEIGHT_FORWARD = 5,
  POSE_LEVEL2 = 6,
  POSE_LEVEL3 = 7,
};

struct PoseExecutionRequest
{
  uint8_t pose_id{0U};
};

struct HandleKfsExecutionRequest
{
  uint8_t mode{0U};
  uint8_t target_stair_level{0U};
};

struct PoseSequenceStep
{
  PoseTarget target{};
  double dwell_sec{0.0};
};

enum class ExecutionKind : uint8_t
{
  NONE = 0,
  POSE = 1,
  SEQUENCE = 2,
};

struct PoseExecutionFeedback
{
  uint8_t stage{0U};
  float progress{0.0F};
};

struct PoseExecutionResult
{
  bool success{false};
  std::string message;
};

struct PoseExecutionState
{
  bool active{false};
  ExecutionKind kind{ExecutionKind::NONE};
  std::optional<PoseActionId> active_pose{};
  std::optional<std::string> active_sequence_name{};
  size_t active_step_index{0U};
  std::optional<rclcpp::Time> started_at{};
  std::optional<rclcpp::Time> step_started_at{};
  std::optional<rclcpp::Time> dwell_started_at{};
  PoseExecutionFeedback feedback{};
};

class ArmControllerNode : public rclcpp::Node
{
public:
  explicit ArmControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // PoseArm.action adapter can call this method directly (non-blocking).
  [[nodiscard]] PoseExecutionResult execute_pose_request(const PoseExecutionRequest & request);
  // HandleForestKFS.action adapter can call this method directly (non-blocking).
  [[nodiscard]] PoseExecutionResult execute_handle_kfs_request(const HandleKfsExecutionRequest & request);
  [[nodiscard]] PoseExecutionFeedback current_pose_feedback() const;

private:
  static constexpr size_t kPoseCount = 8U;

  void load_parameters();
  void setup_ros_interfaces();

  void sbus_callback(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg);
  void joint1_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
  void joint2_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
  void joint3_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
  void joint4_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg);
  void control_timer_callback();

  void initialize_pose_targets();
  void initialize_handle_kfs_sequences();
  void update_pose_execution_status();
  void update_single_pose_execution_status();
  void update_sequence_execution_status();
  void mark_pose_execution_failed(const std::string & message);
  [[nodiscard]] bool is_pose_execution_timeout() const;
  [[nodiscard]] bool is_sequence_step_timeout() const;
  [[nodiscard]] bool is_sequence_total_timeout() const;
  [[nodiscard]] bool resolve_pose_target_safe(
    const PoseTarget & requested_target,
    PoseTarget * resolved_target,
    std::string * error_message) const;
  [[nodiscard]] bool load_sequence_steps_from_parameters(
    const std::string & base_param,
    const std::vector<PoseActionId> & fallback_pose_ids,
    std::vector<PoseSequenceStep> * sequence_steps);
  [[nodiscard]] static std::optional<float> find_safe_target_position(
    float current_position,
    float requested_target,
    const JointLimit & limit);

  void apply_transition(const StateTransition & transition);
  void send_pose_target(const PoseTarget & target, const std::string & pose_name);
  [[nodiscard]] PoseExecutionResult execute_pose_by_id(uint8_t pose_id, const std::string & trigger_source);
  [[nodiscard]] PoseExecutionResult execute_sequence(
    const std::vector<PoseSequenceStep> & sequence_steps,
    const std::string & sequence_name,
    const std::string & trigger_source);
  [[nodiscard]] static std::optional<std::string> resolve_handle_kfs_sequence_key(
    uint8_t mode,
    uint8_t target_stair_level);
  [[nodiscard]] static std::optional<PoseActionId> to_pose_action_id(uint8_t pose_id);
  [[nodiscard]] const PoseTarget * pose_target_of(PoseActionId pose_id) const;
  [[nodiscard]] static const char * pose_to_string(PoseActionId pose_id);

  void publish_target_command(const PoseTarget & target, float speed);
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
  double pose_timeout_sec_{8.0};
  double sequence_step_timeout_sec_{8.0};
  double sequence_total_timeout_sec_{60.0};
  int warn_throttle_ms_{1000};

  PoseTarget manual_pose_{};
  PoseTarget show_low_pose_{};
  PoseTarget show_high_pose_{};

  PoseTarget pose_init_{};
  PoseTarget pose_read_kfs_below_{};
  PoseTarget pose_read_kfs_above_{};
  PoseTarget pose_read_aruco_forward_{};
  PoseTarget pose_climb_r1_{};
  PoseTarget pose_weight_forward_{};
  PoseTarget pose_level2_{};
  PoseTarget pose_level3_{};
  std::array<PoseTarget, kPoseCount> pose_targets_{};
  std::unordered_map<std::string, std::vector<PoseSequenceStep>> handle_kfs_sequences_{};
  std::array<JointLimit, 4> joint_limits_{};

  PoseExecutionState pose_execution_state_{};
  PoseExecutionResult last_pose_result_{};

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
