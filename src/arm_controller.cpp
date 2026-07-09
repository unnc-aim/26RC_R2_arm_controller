#include "arm_controller/arm_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;

namespace arm_controller
{

namespace
{

constexpr uint8_t kStageInit = 0U;
constexpr uint8_t kStageReaching = 2U;
constexpr uint8_t kStageReached = 4U;
constexpr float kTwoPi = 6.28318530717958647692F;

}  // namespace

ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
: Node("arm_controller", options),
	sbus_parser_(sbus_config_)
{
	load_parameters();
	sbus_parser_ = SbusParser(sbus_config_);
	setup_ros_interfaces();

	const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::duration<double>(1.0 / loop_hz_));
	control_timer_ = this->create_wall_timer(period, std::bind(&ArmControllerNode::control_timer_callback, this));

	RCLCPP_INFO(this->get_logger(), "arm_controller started. control_loop_hz=%.2f", loop_hz_);
}

void ArmControllerNode::load_parameters()
{
	sbus_config_.topic = this->declare_parameter<std::string>("sbus.topic", "/sbus/read");
	sbus_config_.mode_channel = this->declare_parameter<int>("sbus.mode_channel", 4);
	sbus_config_.estop_channel = this->declare_parameter<int>("sbus.estop_channel", 6);
	sbus_config_.estop_high = this->declare_parameter<int>("sbus.estop_high", 1600);
	sbus_config_.estop_low = this->declare_parameter<int>("sbus.estop_low", 360);
	sbus_config_.mode_high = this->declare_parameter<int>("sbus.mode_high", 1600);
	sbus_config_.mode_mid = this->declare_parameter<int>("sbus.mode_mid", 1024);
	sbus_config_.mode_low = this->declare_parameter<int>("sbus.mode_low", 360);
	sbus_config_.debounce_ms = this->declare_parameter<int>("sbus.debounce_ms", 100);

	joint1_topics_.read_topic = this->declare_parameter<std::string>("joint1.read_topic", "/joint1/read");
	joint1_topics_.write_topic = this->declare_parameter<std::string>("joint1.write_topic", "/joint1/write");
	joint2_topics_.read_topic = this->declare_parameter<std::string>("joint2.read_topic", "/joint2/read");
	joint2_topics_.write_topic = this->declare_parameter<std::string>("joint2.write_topic", "/joint2/write");
	joint3_topics_.read_topic = this->declare_parameter<std::string>("joint3.read_topic", "/joint3/read");
	joint3_topics_.write_topic = this->declare_parameter<std::string>("joint3.write_topic", "/joint3/write");
	joint4_topics_.read_topic = this->declare_parameter<std::string>("joint4.read_topic", "/joint4/read");
	joint4_topics_.write_topic = this->declare_parameter<std::string>("joint4.write_topic", "/joint4/write");

	loop_hz_ = this->declare_parameter<double>("motion.control_loop_hz", 1000.0);
	motion_speed_ = this->declare_parameter<double>("motion.speed", 5.0);
	position_tolerance_ = this->declare_parameter<double>("motion.position_tolerance", 5.0);
	pose_timeout_sec_ = this->declare_parameter<double>("motion.pose_timeout_sec", 8.0);
	warn_throttle_ms_ = this->declare_parameter<int>("diagnostics.warn_throttle_ms", 1000);

	joint_limits_[0].min_position = this->declare_parameter<double>("joint_limits.joint1.min", -3.0);
	joint_limits_[0].max_position = this->declare_parameter<double>("joint_limits.joint1.max", 3.0);
	joint_limits_[1].min_position = this->declare_parameter<double>("joint_limits.joint2.min", -3.0);
	joint_limits_[1].max_position = this->declare_parameter<double>("joint_limits.joint2.max", 3.0);
	joint_limits_[2].min_position = this->declare_parameter<double>("joint_limits.joint3.min", -3.0);
	joint_limits_[2].max_position = this->declare_parameter<double>("joint_limits.joint3.max", 3.0);
	joint_limits_[3].min_position = this->declare_parameter<double>("joint_limits.joint4.min", -3.0);
	joint_limits_[3].max_position = this->declare_parameter<double>("joint_limits.joint4.max", 3.0);

	manual_pose_.joint1 = this->declare_parameter<double>("poses.manual.joint1", 1000.0);
	manual_pose_.joint2 = this->declare_parameter<double>("poses.manual.joint2", 2000.0);
	manual_pose_.joint3 = this->declare_parameter<double>("poses.manual.joint3", 3000.0);
	manual_pose_.joint4 = this->declare_parameter<double>("poses.manual.joint4", 4000.0);
	show_low_pose_.joint1 = this->declare_parameter<double>("poses.show_low.joint1", 3000.0);
	show_low_pose_.joint2 = this->declare_parameter<double>("poses.show_low.joint2", 4000.0);
	show_low_pose_.joint3 = this->declare_parameter<double>("poses.show_low.joint3", 5000.0);
	show_low_pose_.joint4 = this->declare_parameter<double>("poses.show_low.joint4", 6000.0);
	show_high_pose_.joint1 = this->declare_parameter<double>("poses.show_high.joint1", 5000.0);
	show_high_pose_.joint2 = this->declare_parameter<double>("poses.show_high.joint2", 6000.0);
	show_high_pose_.joint3 = this->declare_parameter<double>("poses.show_high.joint3", 7000.0);
	show_high_pose_.joint4 = this->declare_parameter<double>("poses.show_high.joint4", 8000.0);

	pose_init_.joint1 = this->declare_parameter<double>("poses.pose_init.joint1", manual_pose_.joint1);
	pose_init_.joint2 = this->declare_parameter<double>("poses.pose_init.joint2", manual_pose_.joint2);
	pose_init_.joint3 = this->declare_parameter<double>("poses.pose_init.joint3", manual_pose_.joint3);
	pose_init_.joint4 = this->declare_parameter<double>("poses.pose_init.joint4", manual_pose_.joint4);

	pose_read_kfs_below_.joint1 = this->declare_parameter<double>("poses.pose_read_kfs_below.joint1", show_low_pose_.joint1);
	pose_read_kfs_below_.joint2 = this->declare_parameter<double>("poses.pose_read_kfs_below.joint2", show_low_pose_.joint2);
	pose_read_kfs_below_.joint3 = this->declare_parameter<double>("poses.pose_read_kfs_below.joint3", show_low_pose_.joint3);
	pose_read_kfs_below_.joint4 = this->declare_parameter<double>("poses.pose_read_kfs_below.joint4", show_low_pose_.joint4);

	pose_read_kfs_above_.joint1 = this->declare_parameter<double>("poses.pose_read_kfs_above.joint1", show_high_pose_.joint1);
	pose_read_kfs_above_.joint2 = this->declare_parameter<double>("poses.pose_read_kfs_above.joint2", show_high_pose_.joint2);
	pose_read_kfs_above_.joint3 = this->declare_parameter<double>("poses.pose_read_kfs_above.joint3", show_high_pose_.joint3);
	pose_read_kfs_above_.joint4 = this->declare_parameter<double>("poses.pose_read_kfs_above.joint4", show_high_pose_.joint4);

	pose_read_aruco_forward_.joint1 = this->declare_parameter<double>("poses.pose_read_aruco_forward.joint1", pose_read_kfs_below_.joint1);
	pose_read_aruco_forward_.joint2 = this->declare_parameter<double>("poses.pose_read_aruco_forward.joint2", pose_read_kfs_below_.joint2);
	pose_read_aruco_forward_.joint3 = this->declare_parameter<double>("poses.pose_read_aruco_forward.joint3", pose_read_kfs_below_.joint3);
	pose_read_aruco_forward_.joint4 = this->declare_parameter<double>("poses.pose_read_aruco_forward.joint4", pose_read_kfs_below_.joint4);

	pose_climb_r1_.joint1 = this->declare_parameter<double>("poses.pose_climb_r1.joint1", pose_read_kfs_above_.joint1);
	pose_climb_r1_.joint2 = this->declare_parameter<double>("poses.pose_climb_r1.joint2", pose_read_kfs_above_.joint2);
	pose_climb_r1_.joint3 = this->declare_parameter<double>("poses.pose_climb_r1.joint3", pose_read_kfs_above_.joint3);
	pose_climb_r1_.joint4 = this->declare_parameter<double>("poses.pose_climb_r1.joint4", pose_read_kfs_above_.joint4);

	pose_weight_forward_.joint1 = this->declare_parameter<double>("poses.pose_weight_forward.joint1", pose_init_.joint1);
	pose_weight_forward_.joint2 = this->declare_parameter<double>("poses.pose_weight_forward.joint2", pose_init_.joint2);
	pose_weight_forward_.joint3 = this->declare_parameter<double>("poses.pose_weight_forward.joint3", pose_init_.joint3);
	pose_weight_forward_.joint4 = this->declare_parameter<double>("poses.pose_weight_forward.joint4", pose_init_.joint4);

	pose_level2_.joint1 = this->declare_parameter<double>("poses.pose_level2.joint1", pose_read_kfs_above_.joint1);
	pose_level2_.joint2 = this->declare_parameter<double>("poses.pose_level2.joint2", pose_read_kfs_above_.joint2);
	pose_level2_.joint3 = this->declare_parameter<double>("poses.pose_level2.joint3", pose_read_kfs_above_.joint3);
	pose_level2_.joint4 = this->declare_parameter<double>("poses.pose_level2.joint4", pose_read_kfs_above_.joint4);

	pose_level3_.joint1 = this->declare_parameter<double>("poses.pose_level3.joint1", pose_read_kfs_above_.joint1);
	pose_level3_.joint2 = this->declare_parameter<double>("poses.pose_level3.joint2", pose_read_kfs_above_.joint2);
	pose_level3_.joint3 = this->declare_parameter<double>("poses.pose_level3.joint3", pose_read_kfs_above_.joint3);
	pose_level3_.joint4 = this->declare_parameter<double>("poses.pose_level3.joint4", pose_read_kfs_above_.joint4);

	initialize_pose_targets();

	if (loop_hz_ <= 0.0) {
		RCLCPP_ERROR(
			this->get_logger(),
			"Invalid parameter motion.control_loop_hz=%.3f, must be > 0.",
			loop_hz_);
		throw std::invalid_argument("motion.control_loop_hz must be > 0");
	}

	if (pose_timeout_sec_ <= 0.0) {
		RCLCPP_ERROR(
			this->get_logger(),
			"Invalid parameter motion.pose_timeout_sec=%.3f, must be > 0.",
			pose_timeout_sec_);
		throw std::invalid_argument("motion.pose_timeout_sec must be > 0");
	}

	for (size_t i = 0; i < joint_limits_.size(); ++i) {
		if (joint_limits_[i].min_position >= joint_limits_[i].max_position) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Invalid joint limit for joint%zu: min=%.3f max=%.3f, expected min < max.",
				i + 1,
				joint_limits_[i].min_position,
				joint_limits_[i].max_position);
			throw std::invalid_argument("joint_limits min must be < max");
		}
	}
}

void ArmControllerNode::setup_ros_interfaces()
{
	sbus_sub_ = this->create_subscription<custom_msgs::msg::ReadSBUSRC>(
		sbus_config_.topic,
		rclcpp::SensorDataQoS(),
		std::bind(&ArmControllerNode::sbus_callback, this, std::placeholders::_1));

	joint1_sub_ = this->create_subscription<custom_msgs::msg::ReadDmMotor>(
		joint1_topics_.read_topic,
		rclcpp::SensorDataQoS(),
		std::bind(&ArmControllerNode::joint1_callback, this, std::placeholders::_1));

	joint2_sub_ = this->create_subscription<custom_msgs::msg::ReadDmMotor>(
		joint2_topics_.read_topic,
		rclcpp::SensorDataQoS(),
		std::bind(&ArmControllerNode::joint2_callback, this, std::placeholders::_1));

	joint3_sub_ = this->create_subscription<custom_msgs::msg::ReadDmMotor>(
		joint3_topics_.read_topic,
		rclcpp::SensorDataQoS(),
		std::bind(&ArmControllerNode::joint3_callback, this, std::placeholders::_1));

	joint4_sub_ = this->create_subscription<custom_msgs::msg::ReadDmMotor>(
		joint4_topics_.read_topic,
		rclcpp::SensorDataQoS(),
		std::bind(&ArmControllerNode::joint4_callback, this, std::placeholders::_1));

	joint1_pub_ = this->create_publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
		joint1_topics_.write_topic,
		10);
	joint2_pub_ = this->create_publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
		joint2_topics_.write_topic,
		10);
	joint3_pub_ = this->create_publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
		joint3_topics_.write_topic,
		10);
	joint4_pub_ = this->create_publisher<custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
		joint4_topics_.write_topic,
		10);
}

void ArmControllerNode::sbus_callback(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg)
{
	latest_sbus_ = *msg;
	has_sbus_message_ = true;
}

void ArmControllerNode::joint1_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
	motor_controller_.update_joint1_feedback(*msg);
}

void ArmControllerNode::joint2_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
	motor_controller_.update_joint2_feedback(*msg);
}

void ArmControllerNode::joint3_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
	motor_controller_.update_joint3_feedback(*msg);
}

void ArmControllerNode::joint4_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
	motor_controller_.update_joint4_feedback(*msg);
}

void ArmControllerNode::control_timer_callback()
{
	// 统一在定时器执行状态机，回调仅更新最新数据。
	log_status_warnings();

	bool estop_asserted = false;
	bool estop_released = false;
	ModeEvent mode_event = ModeEvent::NONE;

	if (has_sbus_message_) {
		const auto parse_output = sbus_parser_.parse(latest_sbus_, this->now());
		if (!parse_output.message_valid) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				warn_throttle_ms_,
				"SBUS message ignored: channel index invalid. mode_channel=%d, estop_channel=%d, channels_size=%zu",
				sbus_config_.mode_channel,
				sbus_config_.estop_channel,
				latest_sbus_.channels.size());
		}

		estop_asserted = parse_output.estop_asserted;
		estop_released = parse_output.estop_released;
		mode_event = parse_output.mode_event;
	}

	const auto transition = state_machine_.update(estop_asserted, estop_released, mode_event);
	apply_transition(transition);

	if (state_machine_.current_state() == ArmState::ESTOP) {
		// 急停态不发送运动指令，仅发送一次 disable。
		if (pose_execution_state_.active) {
			mark_pose_execution_failed("Pose execution interrupted by ESTOP.");
		}
		send_disable_command_once();
		return;
	}

	disable_command_sent_ = false;

	if (motor_controller_.has_active_target()) {
		publish_target_command(motor_controller_.active_target().value(), motion_speed_);
	}

	if (motor_controller_.has_active_target() && motor_controller_.target_reached(position_tolerance_)) {
		if (!motor_controller_.motion_completed()) {
			RCLCPP_INFO(this->get_logger(), "Motion finished: all joints reached target within tolerance.");
			motor_controller_.mark_motion_completed();
		}
	}

	update_pose_execution_status();
}

void ArmControllerNode::apply_transition(const StateTransition & transition)
{
	if (transition.changed) {
		RCLCPP_INFO(
			this->get_logger(),
			"State changed: %s -> %s",
			state_to_string(transition.from),
			state_to_string(transition.to));
	}

	if (transition.entered_estop) {
		RCLCPP_WARN(this->get_logger(), "ESTOP asserted. All motor action states cleared.");
	}

	if (transition.exited_estop) {
		RCLCPP_INFO(this->get_logger(), "ESTOP released. Back to IDLE.");
	}

	if (transition.clear_action_state) {
		motor_controller_.clear_active_target();
		pose_execution_state_ = PoseExecutionState{};
	}

	if (transition.trigger_manual_pose) {
		(void)execute_pose_by_id(static_cast<uint8_t>(PoseActionId::POSE_INIT), "SBUS_MANUAL");
	}

	if (transition.trigger_show_low_pose) {
		(void)execute_pose_by_id(static_cast<uint8_t>(PoseActionId::POSE_READ_KFS_BELOW), "SBUS_SHOW_LOW");
	}

	if (transition.trigger_show_high_pose) {
		(void)execute_pose_by_id(static_cast<uint8_t>(PoseActionId::POSE_READ_KFS_ABOVE), "SBUS_SHOW_HIGH");
	}
}

PoseExecutionResult ArmControllerNode::execute_pose_request(const PoseExecutionRequest & request)
{
	return execute_pose_by_id(request.pose_id, "EXTERNAL_REQUEST");
}

PoseExecutionFeedback ArmControllerNode::current_pose_feedback() const
{
	return pose_execution_state_.feedback;
}

void ArmControllerNode::initialize_pose_targets()
{
	pose_targets_[static_cast<size_t>(PoseActionId::POSE_INIT)] = pose_init_;
	pose_targets_[static_cast<size_t>(PoseActionId::POSE_READ_KFS_BELOW)] = pose_read_kfs_below_;
	pose_targets_[static_cast<size_t>(PoseActionId::POSE_READ_KFS_ABOVE)] = pose_read_kfs_above_;
	pose_targets_[static_cast<size_t>(PoseActionId::POSE_READ_ARUCO_FORWARD)] = pose_read_aruco_forward_;
	pose_targets_[static_cast<size_t>(PoseActionId::POSE_CLIMB_R1)] = pose_climb_r1_;
	pose_targets_[static_cast<size_t>(PoseActionId::POSE_WEIGHT_FORWARD)] = pose_weight_forward_;
	pose_targets_[static_cast<size_t>(PoseActionId::POSE_LEVEL2)] = pose_level2_;
	pose_targets_[static_cast<size_t>(PoseActionId::POSE_LEVEL3)] = pose_level3_;
}

void ArmControllerNode::update_pose_execution_status()
{
	if (!pose_execution_state_.active) {
		return;
	}

	if (is_pose_execution_timeout()) {
		mark_pose_execution_failed("Pose execution timeout.");
		return;
	}

	if (motor_controller_.motion_completed()) {
		pose_execution_state_.active = false;
		pose_execution_state_.feedback.stage = kStageReached;
		pose_execution_state_.feedback.progress = 1.0F;
		last_pose_result_.success = true;
		last_pose_result_.message = "Pose execution completed.";
		RCLCPP_INFO(this->get_logger(), "%s", last_pose_result_.message.c_str());
		return;
	}

	pose_execution_state_.feedback.stage = kStageReaching;
	pose_execution_state_.feedback.progress = 0.2F;

	if (!motor_controller_.feedback_ready() || !motor_controller_.has_active_target()) {
		return;
	}

	const auto & target = motor_controller_.active_target().value();
	const float joint1_error = std::fabs(target.joint1 - motor_controller_.joint1_feedback().value().position);
	const float joint2_error = std::fabs(target.joint2 - motor_controller_.joint2_feedback().value().position);
	const float joint3_error = std::fabs(target.joint3 - motor_controller_.joint3_feedback().value().position);
	const float joint4_error = std::fabs(target.joint4 - motor_controller_.joint4_feedback().value().position);
	const float max_error = std::max(std::max(joint1_error, joint2_error), std::max(joint3_error, joint4_error));

	const float safe_scale = std::max(position_tolerance_ * 10.0F, 0.001F);
	const float normalized_remaining = std::clamp(max_error / safe_scale, 0.0F, 1.0F);
	pose_execution_state_.feedback.progress = std::max(0.2F, 1.0F - normalized_remaining);
}

void ArmControllerNode::mark_pose_execution_failed(const std::string & message)
{
	pose_execution_state_.active = false;
	pose_execution_state_.feedback.stage = kStageReached;
	pose_execution_state_.feedback.progress = 1.0F;
	pose_execution_state_.active_pose.reset();
	pose_execution_state_.started_at.reset();
	motor_controller_.clear_active_target();
	last_pose_result_.success = false;
	last_pose_result_.message = message;
	RCLCPP_WARN(this->get_logger(), "%s", last_pose_result_.message.c_str());
}

bool ArmControllerNode::is_pose_execution_timeout() const
{
	if (!pose_execution_state_.started_at.has_value()) {
		return false;
	}

	return (this->now() - pose_execution_state_.started_at.value()).seconds() > pose_timeout_sec_;
}

PoseExecutionResult ArmControllerNode::execute_pose_by_id(uint8_t pose_id, const std::string & trigger_source)
{
	PoseExecutionResult result;

	const auto action_id = to_pose_action_id(pose_id);
	if (!action_id.has_value()) {
		result.success = false;
		result.message = "Invalid pose id: " + std::to_string(pose_id);
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	if (state_machine_.current_state() == ArmState::ESTOP) {
		result.success = false;
		result.message = "Cannot execute pose while ESTOP is active.";
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	if (pose_execution_state_.active) {
		result.success = false;
		result.message = "Another pose execution is in progress.";
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	const PoseTarget * pose_target = pose_target_of(action_id.value());
	if (pose_target == nullptr) {
		result.success = false;
		result.message = "Pose target not configured.";
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	PoseTarget command_target;
	std::string resolve_error;
	if (!resolve_pose_target_safe(*pose_target, &command_target, &resolve_error)) {
		result.success = false;
		result.message = resolve_error;
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	send_pose_target(command_target, pose_to_string(action_id.value()));

	pose_execution_state_.active = true;
	pose_execution_state_.active_pose = action_id.value();
	pose_execution_state_.started_at = this->now();
	pose_execution_state_.feedback.stage = kStageInit;
	pose_execution_state_.feedback.progress = 0.0F;

	last_pose_result_.success = false;
	last_pose_result_.message = "Pose execution started.";

	result.success = true;
	result.message = "Accepted pose " + std::string(pose_to_string(action_id.value())) +
		" from " + trigger_source + ".";
	RCLCPP_INFO(this->get_logger(), "%s", result.message.c_str());
	return result;
}

bool ArmControllerNode::resolve_pose_target_safe(
	const PoseTarget & requested_target,
	PoseTarget * resolved_target,
	std::string * error_message) const
{
	if ((resolved_target == nullptr) || (error_message == nullptr)) {
		return false;
	}

	if (!motor_controller_.feedback_ready()) {
		*error_message = "Cannot execute pose before all joint feedback is ready.";
		return false;
	}

	const std::array<float, 4> current_positions{
		motor_controller_.joint1_feedback().value().position,
		motor_controller_.joint2_feedback().value().position,
		motor_controller_.joint3_feedback().value().position,
		motor_controller_.joint4_feedback().value().position};

	const std::array<float, 4> requested_positions{
		requested_target.joint1,
		requested_target.joint2,
		requested_target.joint3,
		requested_target.joint4};

	std::array<float, 4> resolved_positions{};
	for (size_t i = 0; i < 4; ++i) {
		const auto resolved = find_safe_target_position(current_positions[i], requested_positions[i], joint_limits_[i]);
		if (!resolved.has_value()) {
			*error_message =
				"No safe target for joint" + std::to_string(i + 1) +
				" within limits [" + std::to_string(joint_limits_[i].min_position) +
				", " + std::to_string(joint_limits_[i].max_position) + "] from current=" +
				std::to_string(current_positions[i]) + " requested=" + std::to_string(requested_positions[i]);
			return false;
		}

		resolved_positions[i] = resolved.value();
	}

	resolved_target->joint1 = resolved_positions[0];
	resolved_target->joint2 = resolved_positions[1];
	resolved_target->joint3 = resolved_positions[2];
	resolved_target->joint4 = resolved_positions[3];
	return true;
}

std::optional<float> ArmControllerNode::find_safe_target_position(
	float current_position,
	float requested_target,
	const JointLimit & limit)
{
	if ((current_position < limit.min_position) || (current_position > limit.max_position)) {
		return std::nullopt;
	}

	const float kEpsilon = 1e-5F;
	const int kMinTurns = static_cast<int>(std::ceil((limit.min_position - requested_target) / kTwoPi));
	const int kMaxTurns = static_cast<int>(std::floor((limit.max_position - requested_target) / kTwoPi));

	float best_target = 0.0F;
	float best_cost = std::numeric_limits<float>::max();
	bool found = false;

	for (int turns = kMinTurns; turns <= kMaxTurns; ++turns) {
		const float candidate = requested_target + static_cast<float>(turns) * kTwoPi;
		if ((candidate < limit.min_position - kEpsilon) || (candidate > limit.max_position + kEpsilon)) {
			continue;
		}

		const float movement_cost = std::fabs(candidate - current_position);
		if (!found || (movement_cost < best_cost)) {
			best_cost = movement_cost;
			best_target = candidate;
			found = true;
		}
	}

	if (!found) {
		return std::nullopt;
	}

	return best_target;
}

std::optional<PoseActionId> ArmControllerNode::to_pose_action_id(uint8_t pose_id)
{
	switch (pose_id) {
		case static_cast<uint8_t>(PoseActionId::POSE_INIT):
			return PoseActionId::POSE_INIT;
		case static_cast<uint8_t>(PoseActionId::POSE_READ_KFS_BELOW):
			return PoseActionId::POSE_READ_KFS_BELOW;
		case static_cast<uint8_t>(PoseActionId::POSE_READ_KFS_ABOVE):
			return PoseActionId::POSE_READ_KFS_ABOVE;
		case static_cast<uint8_t>(PoseActionId::POSE_READ_ARUCO_FORWARD):
			return PoseActionId::POSE_READ_ARUCO_FORWARD;
		case static_cast<uint8_t>(PoseActionId::POSE_CLIMB_R1):
			return PoseActionId::POSE_CLIMB_R1;
		case static_cast<uint8_t>(PoseActionId::POSE_WEIGHT_FORWARD):
			return PoseActionId::POSE_WEIGHT_FORWARD;
		case static_cast<uint8_t>(PoseActionId::POSE_LEVEL2):
			return PoseActionId::POSE_LEVEL2;
		case static_cast<uint8_t>(PoseActionId::POSE_LEVEL3):
			return PoseActionId::POSE_LEVEL3;
		default:
			return std::nullopt;
	}
}

const PoseTarget * ArmControllerNode::pose_target_of(PoseActionId pose_id) const
{
	const size_t index = static_cast<size_t>(pose_id);
	if (index >= pose_targets_.size()) {
		return nullptr;
	}

	return &pose_targets_[index];
}

const char * ArmControllerNode::pose_to_string(PoseActionId pose_id)
{
	switch (pose_id) {
		case PoseActionId::POSE_INIT:
			return "POSE_INIT";
		case PoseActionId::POSE_READ_KFS_BELOW:
			return "POSE_READ_KFS_BELOW";
		case PoseActionId::POSE_READ_KFS_ABOVE:
			return "POSE_READ_KFS_ABOVE";
		case PoseActionId::POSE_READ_ARUCO_FORWARD:
			return "POSE_READ_ARUCO_FORWARD";
		case PoseActionId::POSE_CLIMB_R1:
			return "POSE_CLIMB_R1";
		case PoseActionId::POSE_WEIGHT_FORWARD:
			return "POSE_WEIGHT_FORWARD";
		case PoseActionId::POSE_LEVEL2:
			return "POSE_LEVEL2";
		case PoseActionId::POSE_LEVEL3:
			return "POSE_LEVEL3";
		default:
			return "POSE_UNKNOWN";
	}
}

void ArmControllerNode::send_pose_target(const PoseTarget & target, const std::string & pose_name)
{
	publish_target_command(target, motion_speed_);

	motor_controller_.set_active_target(target);

	RCLCPP_INFO(
		this->get_logger(),
		"%s target sent once: joint1=%.3f, joint2=%.3f, joint3=%.3f, joint4=%.3f, speed=%.3f",
		pose_name.c_str(),
		target.joint1,
		target.joint2,
		target.joint3,
		target.joint4,
		motion_speed_);
}

void ArmControllerNode::send_disable_command_once()
{
	if (disable_command_sent_) {
		return;
	}

	custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit msg;
	msg.enable = 0U;
	msg.position = 0.0F;
	msg.speed = 0.0F;

	joint1_pub_->publish(msg);
	joint2_pub_->publish(msg);
	joint3_pub_->publish(msg);
	joint4_pub_->publish(msg);
	disable_command_sent_ = true;
}

void ArmControllerNode::publish_target_command(const PoseTarget & target, float speed)
{
	custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit joint1_msg;
	custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit joint2_msg;
	custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit joint3_msg;
	custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit joint4_msg;

	joint1_msg.enable = 1U;
	joint1_msg.position = target.joint1;
	joint1_msg.speed = speed;

	joint2_msg.enable = 1U;
	joint2_msg.position = target.joint2;
	joint2_msg.speed = speed;

	joint3_msg.enable = 1U;
	joint3_msg.position = target.joint3;
	joint3_msg.speed = speed;

	joint4_msg.enable = 1U;
	joint4_msg.position = target.joint4;
	joint4_msg.speed = speed;

	joint1_pub_->publish(joint1_msg);
	joint2_pub_->publish(joint2_msg);
	joint3_pub_->publish(joint3_msg);
	joint4_pub_->publish(joint4_msg);
}
void ArmControllerNode::log_status_warnings()
{
	// 当前仅做告警日志，后续可在此扩展更复杂的容错策略。
	if (!has_sbus_message_) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"No SBUS message received yet.");
	} else if ((latest_sbus_.online == 0U) || (latest_sbus_.fail_safe != 0U) || (latest_sbus_.frame_lost != 0U)) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"SBUS abnormal: online=%u fail_safe=%u frame_lost=%u",
			latest_sbus_.online,
			latest_sbus_.fail_safe,
			latest_sbus_.frame_lost);
	}

	if (motor_controller_.joint1_has_fault()) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"Joint1 feedback indicates communication/power/thermal fault.");
	}
	if (motor_controller_.joint2_has_fault()) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"Joint2 feedback indicates communication/power/thermal fault.");
	}
	if (motor_controller_.joint3_has_fault()) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"Joint3 feedback indicates communication/power/thermal fault.");
	}
	if (motor_controller_.joint4_has_fault()) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"Joint4 feedback indicates communication/power/thermal fault.");
	}
}

const char * ArmControllerNode::state_to_string(ArmState state)
{
	switch (state) {
		case ArmState::ESTOP:
			return "ESTOP";
		case ArmState::IDLE:
			return "IDLE";
		case ArmState::MANUAL:
			return "MANUAL";
		case ArmState::SHOW_LOW:
			return "SHOW_LOW";
		case ArmState::SHOW_HIGH:
			return "SHOW_HIGH";
		default:
			return "UNKNOWN";
	}
}

}  // namespace arm_controller

