#include "arm_controller/arm_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>

using namespace std::chrono_literals;

namespace arm_controller
{

namespace
{

constexpr uint8_t kStageInit = 0U;
constexpr uint8_t kStageTurningOnSuction = 1U;
constexpr uint8_t kStageReachingOut = 2U;
constexpr uint8_t kStageTurningOffSuction = 3U;
constexpr uint8_t kStageReachingBack = 4U;
constexpr uint8_t kHandleKfsModePick = 0U;
constexpr uint8_t kHandleKfsModePlace = 1U;
constexpr uint8_t kHandleKfsLevelDown = 0U;
constexpr uint8_t kHandleKfsLevelUp = 1U;
constexpr uint8_t kHandleKfsLevel40Up = 2U;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 6.28318530717958647692F;

float normalize_wrapped_angle(float angle)
{
	while (angle > kPi) {
		angle -= kTwoPi;
	}
	while (angle < -kPi) {
		angle += kTwoPi;
	}
	return angle;
}

bool is_wrapped_joint_limit(const JointLimit & limit)
{
	return limit.min_position > limit.max_position;
}

bool is_position_within_joint_limit(float position, const JointLimit & limit, float epsilon)
{
	if (!is_wrapped_joint_limit(limit)) {
		return (position >= limit.min_position - epsilon) && (position <= limit.max_position + epsilon);
	}

	const float normalized_position = normalize_wrapped_angle(position);
	return (normalized_position >= limit.min_position - epsilon) ||
		(normalized_position <= limit.max_position + epsilon);
}

float unwrap_angle_near_reference(float raw_position, float reference_position)
{
	const float normalized_position = normalize_wrapped_angle(raw_position);
	const int nearest_turn = static_cast<int>(std::lround((reference_position - normalized_position) / kTwoPi));

	float best_position = normalized_position;
	float best_error = std::numeric_limits<float>::max();
	for (int turn_offset = -1; turn_offset <= 1; ++turn_offset) {
		const float candidate = normalized_position + static_cast<float>(nearest_turn + turn_offset) * kTwoPi;
		const float candidate_error = std::fabs(candidate - reference_position);
		if (candidate_error < best_error) {
			best_position = candidate;
			best_error = candidate_error;
		}
	}

	return best_position;
}

std::string describe_joint_limit(const JointLimit & limit)
{
	if (!is_wrapped_joint_limit(limit)) {
		return "[" + std::to_string(limit.min_position) + ", " + std::to_string(limit.max_position) + "]";
	}

	return "[" + std::to_string(limit.min_position) + ", pi] U [-pi, " +
		std::to_string(limit.max_position) + "]";
}

uint8_t stage_for_sequence_dwell(const std::string & sequence_name)
{
	if (sequence_name.rfind("pick_", 0U) == 0U) {
		return kStageTurningOnSuction;
	}

	if (sequence_name.rfind("place_", 0U) == 0U) {
		return kStageTurningOffSuction;
	}

	return kStageInit;
}

std::string to_lower_ascii_copy(const std::string & input)
{
	std::string output = input;
	std::transform(
		output.begin(),
		output.end(),
		output.begin(),
		[](unsigned char c) {return static_cast<char>(std::tolower(c));});
	return output;
}

std::optional<SequenceSuctionAction> parse_sequence_suction_action(int raw)
{
	switch (raw) {
		case 0:
		return SequenceSuctionAction::NONE;
		case 1:
		return SequenceSuctionAction::ON;
		case 2:
		return SequenceSuctionAction::OFF;
		default:
			return std::nullopt;
	}
}

std::optional<SequenceSuctionWhen> parse_sequence_suction_when(const std::string & raw)
{
	const std::string value = to_lower_ascii_copy(raw);
	if ((value == "") || (value == "after_reach")) {
		return SequenceSuctionWhen::AFTER_REACH;
	}

	if (value == "after_dwell") {
		return SequenceSuctionWhen::AFTER_DWELL;
	}

	return std::nullopt;
}

const char * suction_action_to_string(SequenceSuctionAction action)
{
	switch (action) {
		case SequenceSuctionAction::NONE:
			return "none";
		case SequenceSuctionAction::ON:
			return "on";
		case SequenceSuctionAction::OFF:
			return "off";
		default:
			return "unknown";
	}
}

const char * suction_when_to_string(SequenceSuctionWhen when)
{
	switch (when) {
		case SequenceSuctionWhen::AFTER_REACH:
			return "after_reach";
		case SequenceSuctionWhen::AFTER_DWELL:
			return "after_dwell";
		default:
			return "unknown";
	}
}

}  // namespace

ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
: Node("arm_controller", options),
	sbus_parser_(sbus_config_)
{
	load_parameters();
	sbus_parser_ = SbusParser(sbus_config_);
	setup_ros_interfaces();
	setup_action_servers();

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
	joint_enabled_[0] = this->declare_parameter<bool>("joint1.enabled", true);
	joint2_topics_.read_topic = this->declare_parameter<std::string>("joint2.read_topic", "/joint2/read");
	joint2_topics_.write_topic = this->declare_parameter<std::string>("joint2.write_topic", "/joint2/write");
	joint_enabled_[1] = this->declare_parameter<bool>("joint2.enabled", true);
	joint3_topics_.read_topic = this->declare_parameter<std::string>("joint3.read_topic", "/joint3/read");
	joint3_topics_.write_topic = this->declare_parameter<std::string>("joint3.write_topic", "/joint3/write");
	joint_enabled_[2] = this->declare_parameter<bool>("joint3.enabled", true);
	joint4_topics_.read_topic = this->declare_parameter<std::string>("joint4.read_topic", "/joint4/read");
	joint4_topics_.write_topic = this->declare_parameter<std::string>("joint4.write_topic", "/joint4/write");
	joint_enabled_[3] = this->declare_parameter<bool>("joint4.enabled", true);

	loop_hz_ = this->declare_parameter<double>("motion.control_loop_hz", 1000.0);
	sbus_offline_estop_delay_sec_ = this->declare_parameter<double>("sbus.offline_estop_delay_sec", 0.5);
	motion_speed_ = this->declare_parameter<double>("motion.speed", 5.0);
	position_tolerance_ = this->declare_parameter<double>("motion.position_tolerance", 5.0);
	pose_timeout_sec_ = this->declare_parameter<double>("motion.pose_timeout_sec", 8.0);
	sequence_step_timeout_sec_ = this->declare_parameter<double>("motion.sequence_step_timeout_sec", pose_timeout_sec_);
	sequence_total_timeout_sec_ = this->declare_parameter<double>("motion.sequence_total_timeout_sec", 60.0);
	warn_throttle_ms_ = this->declare_parameter<int>("diagnostics.warn_throttle_ms", 1000);
	pose_arm_action_name_ = this->declare_parameter<std::string>("actions.pose_arm_name", "r2/motion/pose_arm");
	handle_kfs_action_name_ = this->declare_parameter<std::string>("actions.handle_kfs_name", "r2/motion/place_kfs");
	suction_enabled_ = this->declare_parameter<bool>("suction.enabled", true);
	suction_service_name_ = this->declare_parameter<std::string>("suction.service_name", "/set_suction");
	suction_call_timeout_sec_ = this->declare_parameter<double>("suction.call_timeout_sec", 1.0);
	suction_fail_on_error_ = this->declare_parameter<bool>("suction.fail_on_error", true);

	joint_limits_[0].min_position = this->declare_parameter<double>("joint_limits.joint1.min", -3.0);
	joint_limits_[0].max_position = this->declare_parameter<double>("joint_limits.joint1.max", 3.0);
	joint_limits_[1].min_position = this->declare_parameter<double>("joint_limits.joint2.min", -3.0);
	joint_limits_[1].max_position = this->declare_parameter<double>("joint_limits.joint2.max", 3.0);
	joint_limits_[2].min_position = this->declare_parameter<double>("joint_limits.joint3.min", -3.0);
	joint_limits_[2].max_position = this->declare_parameter<double>("joint_limits.joint3.max", 3.0);
	joint_limits_[3].min_position = this->declare_parameter<double>("joint_limits.joint4.min", -3.0);
	joint_limits_[3].max_position = this->declare_parameter<double>("joint_limits.joint4.max", 3.0);

	named_poses_.clear();
	manual_pose_.joint1 = this->declare_parameter<double>("poses.manual.joint1", 1000.0);
	manual_pose_.joint2 = this->declare_parameter<double>("poses.manual.joint2", 2000.0);
	manual_pose_.joint3 = this->declare_parameter<double>("poses.manual.joint3", 3000.0);
	manual_pose_.joint4 = this->declare_parameter<double>("poses.manual.joint4", 4000.0);
	named_poses_["manual"] = manual_pose_;

	show_low_pose_.joint1 = this->declare_parameter<double>("poses.show_low.joint1", 3000.0);
	show_low_pose_.joint2 = this->declare_parameter<double>("poses.show_low.joint2", 4000.0);
	show_low_pose_.joint3 = this->declare_parameter<double>("poses.show_low.joint3", 5000.0);
	show_low_pose_.joint4 = this->declare_parameter<double>("poses.show_low.joint4", 6000.0);
	named_poses_["show_low"] = show_low_pose_;

	show_high_pose_.joint1 = this->declare_parameter<double>("poses.show_high.joint1", 5000.0);
	show_high_pose_.joint2 = this->declare_parameter<double>("poses.show_high.joint2", 6000.0);
	show_high_pose_.joint3 = this->declare_parameter<double>("poses.show_high.joint3", 7000.0);
	show_high_pose_.joint4 = this->declare_parameter<double>("poses.show_high.joint4", 8000.0);
	named_poses_["show_high"] = show_high_pose_;

	pose_init_.joint1 = this->declare_parameter<double>("poses.pose_init.joint1", manual_pose_.joint1);
	pose_init_.joint2 = this->declare_parameter<double>("poses.pose_init.joint2", manual_pose_.joint2);
	pose_init_.joint3 = this->declare_parameter<double>("poses.pose_init.joint3", manual_pose_.joint3);
	pose_init_.joint4 = this->declare_parameter<double>("poses.pose_init.joint4", manual_pose_.joint4);
	named_poses_["pose_init"] = pose_init_;

	pose_read_kfs_below_.joint1 = this->declare_parameter<double>("poses.pose_read_kfs_below.joint1", show_low_pose_.joint1);
	pose_read_kfs_below_.joint2 = this->declare_parameter<double>("poses.pose_read_kfs_below.joint2", show_low_pose_.joint2);
	pose_read_kfs_below_.joint3 = this->declare_parameter<double>("poses.pose_read_kfs_below.joint3", show_low_pose_.joint3);
	pose_read_kfs_below_.joint4 = this->declare_parameter<double>("poses.pose_read_kfs_below.joint4", show_low_pose_.joint4);
	named_poses_["pose_read_kfs_below"] = pose_read_kfs_below_;

	pose_read_kfs_above_.joint1 = this->declare_parameter<double>("poses.pose_read_kfs_above.joint1", show_high_pose_.joint1);
	pose_read_kfs_above_.joint2 = this->declare_parameter<double>("poses.pose_read_kfs_above.joint2", show_high_pose_.joint2);
	pose_read_kfs_above_.joint3 = this->declare_parameter<double>("poses.pose_read_kfs_above.joint3", show_high_pose_.joint3);
	pose_read_kfs_above_.joint4 = this->declare_parameter<double>("poses.pose_read_kfs_above.joint4", show_high_pose_.joint4);
	named_poses_["pose_read_kfs_above"] = pose_read_kfs_above_;

	pose_read_aruco_forward_.joint1 = this->declare_parameter<double>("poses.pose_read_aruco_forward.joint1", pose_read_kfs_below_.joint1);
	pose_read_aruco_forward_.joint2 = this->declare_parameter<double>("poses.pose_read_aruco_forward.joint2", pose_read_kfs_below_.joint2);
	pose_read_aruco_forward_.joint3 = this->declare_parameter<double>("poses.pose_read_aruco_forward.joint3", pose_read_kfs_below_.joint3);
	pose_read_aruco_forward_.joint4 = this->declare_parameter<double>("poses.pose_read_aruco_forward.joint4", pose_read_kfs_below_.joint4);
	named_poses_["pose_read_aruco_forward"] = pose_read_aruco_forward_;

	pose_climb_r1_.joint1 = this->declare_parameter<double>("poses.pose_climb_r1.joint1", pose_read_kfs_above_.joint1);
	pose_climb_r1_.joint2 = this->declare_parameter<double>("poses.pose_climb_r1.joint2", pose_read_kfs_above_.joint2);
	pose_climb_r1_.joint3 = this->declare_parameter<double>("poses.pose_climb_r1.joint3", pose_read_kfs_above_.joint3);
	pose_climb_r1_.joint4 = this->declare_parameter<double>("poses.pose_climb_r1.joint4", pose_read_kfs_above_.joint4);
	named_poses_["pose_climb_r1"] = pose_climb_r1_;

	pose_weight_forward_.joint1 = this->declare_parameter<double>("poses.pose_weight_forward.joint1", pose_init_.joint1);
	pose_weight_forward_.joint2 = this->declare_parameter<double>("poses.pose_weight_forward.joint2", pose_init_.joint2);
	pose_weight_forward_.joint3 = this->declare_parameter<double>("poses.pose_weight_forward.joint3", pose_init_.joint3);
	pose_weight_forward_.joint4 = this->declare_parameter<double>("poses.pose_weight_forward.joint4", pose_init_.joint4);
	named_poses_["pose_weight_forward"] = pose_weight_forward_;

	pose_level2_.joint1 = this->declare_parameter<double>("poses.pose_level2.joint1", pose_read_kfs_above_.joint1);
	pose_level2_.joint2 = this->declare_parameter<double>("poses.pose_level2.joint2", pose_read_kfs_above_.joint2);
	pose_level2_.joint3 = this->declare_parameter<double>("poses.pose_level2.joint3", pose_read_kfs_above_.joint3);
	pose_level2_.joint4 = this->declare_parameter<double>("poses.pose_level2.joint4", pose_read_kfs_above_.joint4);
	named_poses_["pose_level2"] = pose_level2_;

	pose_level3_.joint1 = this->declare_parameter<double>("poses.pose_level3.joint1", pose_read_kfs_above_.joint1);
	pose_level3_.joint2 = this->declare_parameter<double>("poses.pose_level3.joint2", pose_read_kfs_above_.joint2);
	pose_level3_.joint3 = this->declare_parameter<double>("poses.pose_level3.joint3", pose_read_kfs_above_.joint3);
	pose_level3_.joint4 = this->declare_parameter<double>("poses.pose_level3.joint4", pose_read_kfs_above_.joint4);
	named_poses_["pose_level3"] = pose_level3_;

	const std::vector<std::string> required_pose_names{
		"manual",
		"show_low",
		"show_high",
		"pose_init",
		"pose_read_kfs_below",
		"pose_read_kfs_above",
		"pose_read_aruco_forward",
		"pose_climb_r1",
		"pose_weight_forward",
		"pose_level2",
		"pose_level3"};

	auto pose_names = this->declare_parameter<std::vector<std::string>>("poses.names", required_pose_names);
	std::unordered_set<std::string> pose_name_set(pose_names.begin(), pose_names.end());
	for (const auto & required_pose_name : required_pose_names) {
		if (pose_name_set.find(required_pose_name) != pose_name_set.end()) {
			continue;
		}

		pose_names.push_back(required_pose_name);
		pose_name_set.insert(required_pose_name);
	}

	for (const auto & pose_name : pose_names) {
		if (named_poses_.find(pose_name) != named_poses_.end()) {
			continue;
		}

		PoseTarget default_pose = pose_init_;
		PoseTarget loaded_pose;
		loaded_pose.joint1 = this->declare_parameter<double>("poses." + pose_name + ".joint1", default_pose.joint1);
		loaded_pose.joint2 = this->declare_parameter<double>("poses." + pose_name + ".joint2", default_pose.joint2);
		loaded_pose.joint3 = this->declare_parameter<double>("poses." + pose_name + ".joint3", default_pose.joint3);
		loaded_pose.joint4 = this->declare_parameter<double>("poses." + pose_name + ".joint4", default_pose.joint4);
		named_poses_[pose_name] = loaded_pose;
	}

	initialize_pose_targets();
	initialize_handle_kfs_sequences();

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

	if (sequence_step_timeout_sec_ <= 0.0) {
		RCLCPP_ERROR(
			this->get_logger(),
			"Invalid parameter motion.sequence_step_timeout_sec=%.3f, must be > 0.",
			sequence_step_timeout_sec_);
		throw std::invalid_argument("motion.sequence_step_timeout_sec must be > 0");
	}

	if (sequence_total_timeout_sec_ <= 0.0) {
		RCLCPP_ERROR(
			this->get_logger(),
			"Invalid parameter motion.sequence_total_timeout_sec=%.3f, must be > 0.",
			sequence_total_timeout_sec_);
		throw std::invalid_argument("motion.sequence_total_timeout_sec must be > 0");
	}

	if (suction_call_timeout_sec_ <= 0.0) {
		RCLCPP_ERROR(
			this->get_logger(),
			"Invalid parameter suction.call_timeout_sec=%.3f, must be > 0.",
			suction_call_timeout_sec_);
		throw std::invalid_argument("suction.call_timeout_sec must be > 0");
	}

	for (size_t i = 0; i < joint_limits_.size(); ++i) {
		if (joint_limits_[i].min_position == joint_limits_[i].max_position) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Invalid joint limit for joint%zu: min=%.3f max=%.3f, expected min != max.",
				i + 1,
				joint_limits_[i].min_position,
				joint_limits_[i].max_position);
			throw std::invalid_argument("joint_limits min must not equal max");
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

	if (suction_enabled_) {
		suction_client_ = this->create_client<suction_control::srv::SetSuction>(suction_service_name_);
	}
}

void ArmControllerNode::setup_action_servers()
{
	pose_arm_action_server_ = rclcpp_action::create_server<PoseArmAction>(
		this,
		pose_arm_action_name_,
		std::bind(&ArmControllerNode::handle_pose_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&ArmControllerNode::handle_pose_arm_cancel, this, std::placeholders::_1),
		std::bind(&ArmControllerNode::handle_pose_arm_accepted, this, std::placeholders::_1));

	handle_kfs_action_server_ = rclcpp_action::create_server<HandleForestKfsAction>(
		this,
		handle_kfs_action_name_,
		std::bind(&ArmControllerNode::handle_handle_kfs_goal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&ArmControllerNode::handle_handle_kfs_cancel, this, std::placeholders::_1),
		std::bind(&ArmControllerNode::handle_handle_kfs_accepted, this, std::placeholders::_1));

	RCLCPP_INFO(
		this->get_logger(),
		"Action servers ready: pose='%s', handle_kfs='%s'",
		pose_arm_action_name_.c_str(),
		handle_kfs_action_name_.c_str());
}

bool ArmControllerNode::has_active_action_goal() const
{
	return (active_pose_arm_goal_ != nullptr) || (active_handle_kfs_goal_ != nullptr);
}

void ArmControllerNode::cancel_current_execution(const std::string & reason)
{
	if (!pose_execution_state_.active) {
		return;
	}
	mark_pose_execution_failed(reason);
}

rclcpp_action::GoalResponse ArmControllerNode::handle_pose_arm_goal(
	const rclcpp_action::GoalUUID & uuid,
	std::shared_ptr<const PoseArmAction::Goal> goal)
{
	(void)uuid;
	if (goal == nullptr) {
		return rclcpp_action::GoalResponse::REJECT;
	}

	if (has_active_action_goal() || pose_execution_state_.active) {
		RCLCPP_WARN(this->get_logger(), "Reject PoseArm goal: another action is running.");
		return rclcpp_action::GoalResponse::REJECT;
	}

	if (goal->pose > PoseArmAction::Goal::POSE_LEVEL3) {
		RCLCPP_WARN(this->get_logger(), "Reject PoseArm goal: invalid pose=%u.", goal->pose);
		return rclcpp_action::GoalResponse::REJECT;
	}

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmControllerNode::handle_pose_arm_cancel(
	const std::shared_ptr<PoseArmGoalHandle> goal_handle)
{
	if ((goal_handle != nullptr) && (active_pose_arm_goal_ == goal_handle)) {
		cancel_current_execution("Pose action canceled by client.");
	}
	return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmControllerNode::handle_pose_arm_accepted(const std::shared_ptr<PoseArmGoalHandle> goal_handle)
{
	if (goal_handle == nullptr) {
		return;
	}

	PoseExecutionRequest request;
	request.pose_id = goal_handle->get_goal()->pose;
	const PoseExecutionResult accepted = execute_pose_request(request);

	if (!accepted.success) {
		auto result = std::make_shared<PoseArmAction::Result>();
		result->success = false;
		result->message = accepted.message;
		goal_handle->abort(result);
		return;
	}

	active_pose_arm_goal_ = goal_handle;
}

rclcpp_action::GoalResponse ArmControllerNode::handle_handle_kfs_goal(
	const rclcpp_action::GoalUUID & uuid,
	std::shared_ptr<const HandleForestKfsAction::Goal> goal)
{
	(void)uuid;
	if (goal == nullptr) {
		return rclcpp_action::GoalResponse::REJECT;
	}

	if (has_active_action_goal() || pose_execution_state_.active) {
		RCLCPP_WARN(this->get_logger(), "Reject HandleForestKFS goal: another action is running.");
		return rclcpp_action::GoalResponse::REJECT;
	}

	if ((goal->mode != HandleForestKfsAction::Goal::MODE_PICK) &&
		(goal->mode != HandleForestKfsAction::Goal::MODE_PLACE)) {
		RCLCPP_WARN(this->get_logger(), "Reject HandleForestKFS goal: invalid mode=%u.", goal->mode);
		return rclcpp_action::GoalResponse::REJECT;
	}

	if ((goal->target_stair_level != HandleForestKfsAction::Goal::TARGET_STAIR_LEVEL_DOWN) &&
		(goal->target_stair_level != HandleForestKfsAction::Goal::TARGET_STAIR_LEVEL_UP) &&
		(goal->target_stair_level != HandleForestKfsAction::Goal::TARGET_STAIR_LEVEL_40_UP)) {
		RCLCPP_WARN(
			this->get_logger(),
			"Reject HandleForestKFS goal: invalid target_stair_level=%u.",
			goal->target_stair_level);
		return rclcpp_action::GoalResponse::REJECT;
	}

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmControllerNode::handle_handle_kfs_cancel(
	const std::shared_ptr<HandleForestKfsGoalHandle> goal_handle)
{
	if ((goal_handle != nullptr) && (active_handle_kfs_goal_ == goal_handle)) {
		cancel_current_execution("HandleForestKFS action canceled by client.");
	}
	return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmControllerNode::handle_handle_kfs_accepted(const std::shared_ptr<HandleForestKfsGoalHandle> goal_handle)
{
	if (goal_handle == nullptr) {
		return;
	}

	HandleKfsExecutionRequest request;
	request.mode = goal_handle->get_goal()->mode;
	request.target_stair_level = goal_handle->get_goal()->target_stair_level;
	const PoseExecutionResult accepted = execute_handle_kfs_request(request);

	if (!accepted.success) {
		auto result = std::make_shared<HandleForestKfsAction::Result>();
		result->success = false;
		result->message = accepted.message;
		goal_handle->abort(result);
		return;
	}

	active_handle_kfs_goal_ = goal_handle;
}

void ArmControllerNode::update_action_servers()
{
	if (active_pose_arm_goal_ != nullptr) {
		auto feedback = std::make_shared<PoseArmAction::Feedback>();
		const PoseExecutionFeedback pose_feedback = current_pose_feedback();
		feedback->stage = pose_feedback.stage;
		feedback->progress = pose_feedback.progress;
		active_pose_arm_goal_->publish_feedback(feedback);

		if (!pose_execution_state_.active) {
			auto result = std::make_shared<PoseArmAction::Result>();
			result->success = last_pose_result_.success;
			result->message = last_pose_result_.message;

			if (active_pose_arm_goal_->is_canceling()) {
				active_pose_arm_goal_->canceled(result);
			} else if (last_pose_result_.success) {
				active_pose_arm_goal_->succeed(result);
			} else {
				active_pose_arm_goal_->abort(result);
			}

			active_pose_arm_goal_.reset();
		}
	}

	if (active_handle_kfs_goal_ != nullptr) {
		auto feedback = std::make_shared<HandleForestKfsAction::Feedback>();
		const PoseExecutionFeedback pose_feedback = current_pose_feedback();
		feedback->stage = pose_feedback.stage;
		feedback->progress = pose_feedback.progress;
		active_handle_kfs_goal_->publish_feedback(feedback);

		if (!pose_execution_state_.active) {
			auto result = std::make_shared<HandleForestKfsAction::Result>();
			result->success = last_pose_result_.success;
			result->message = last_pose_result_.message;

			if (active_handle_kfs_goal_->is_canceling()) {
				active_handle_kfs_goal_->canceled(result);
			} else if (last_pose_result_.success) {
				active_handle_kfs_goal_->succeed(result);
			} else {
				active_handle_kfs_goal_->abort(result);
			}

			active_handle_kfs_goal_.reset();
		}
	}
}

void ArmControllerNode::sbus_callback(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg)
{
	latest_sbus_ = *msg;
	has_sbus_message_ = true;
}

void ArmControllerNode::joint1_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
	update_continuous_joint_position(0U, msg->position);
	motor_controller_.update_joint1_feedback(*msg);
}

void ArmControllerNode::joint2_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
	update_continuous_joint_position(1U, msg->position);
	motor_controller_.update_joint2_feedback(*msg);
}

void ArmControllerNode::joint3_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
	update_continuous_joint_position(2U, msg->position);
	motor_controller_.update_joint3_feedback(*msg);
}

void ArmControllerNode::joint4_callback(const custom_msgs::msg::ReadDmMotor::SharedPtr msg)
{
	update_continuous_joint_position(3U, msg->position);
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

		if (latest_sbus_.online == 0U) {
			const rclcpp::Time now = this->now();
			if (!sbus_offline_since_.has_value()) {
				sbus_offline_since_ = now;
			}

			const auto offline_duration = now - sbus_offline_since_.value();
			const auto offline_delay = rclcpp::Duration::from_nanoseconds(
				static_cast<int64_t>(sbus_offline_estop_delay_sec_ * 1000.0 * 1000.0 * 1000.0));

			if (offline_duration < offline_delay) {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(),
					*this->get_clock(),
					warn_throttle_ms_,
					"SBUS offline detected, waiting %.3f s before force disable.",
					sbus_offline_estop_delay_sec_);
				return;
			}

			RCLCPP_WARN_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				warn_throttle_ms_,
				"SBUS offline persisted for %.3f s, force disable outputs.",
				sbus_offline_estop_delay_sec_);
			if (pose_execution_state_.active) {
				mark_pose_execution_failed("Pose execution interrupted by SBUS offline.");
			}
			update_action_servers();
			send_disable_command_once();
			return;
		}

		sbus_offline_since_.reset();

		if (!parse_output.sbus_ok) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				warn_throttle_ms_,
				"SBUS is not healthy. online=%u frame_lost=%u",
				latest_sbus_.online,
				latest_sbus_.frame_lost);
		}

		estop_asserted = parse_output.estop_asserted;
		estop_released = parse_output.estop_released;
		// 临时策略：屏蔽 SBUS 模式切换，只保留急停链路。
		mode_event = ModeEvent::NONE;
	}

	const auto transition = state_machine_.update(estop_asserted, estop_released, mode_event);
	apply_transition(transition);

	if (state_machine_.current_state() == ArmState::ESTOP) {
		// 急停态不发送运动指令，仅发送一次 disable。
		if (pose_execution_state_.active) {
			mark_pose_execution_failed("Pose execution interrupted by ESTOP.");
		}
		update_action_servers();
		send_disable_command_once();
		return;
	}

	disable_command_sent_ = false;

	if (motor_controller_.has_active_target()) {
		publish_target_command(motor_controller_.active_target().value(), motion_speed_);
	}

	if (motor_controller_.has_active_target() && motion_target_reached()) {
		if (!motor_controller_.motion_completed()) {
			RCLCPP_INFO(this->get_logger(), "Motion finished: all joints reached target within tolerance.");
			motor_controller_.mark_motion_completed();
		}
	}

	update_pose_execution_status();
	update_action_servers();
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

PoseExecutionResult ArmControllerNode::execute_handle_kfs_request(const HandleKfsExecutionRequest & request)
{
	const auto sequence_key = resolve_handle_kfs_sequence_key(request.mode, request.target_stair_level);
	if (!sequence_key.has_value()) {
		PoseExecutionResult result;
		result.success = false;
		result.message =
			"Invalid HandleForestKFS mode/target_stair_level: mode=" + std::to_string(request.mode) +
			" target_stair_level=" + std::to_string(request.target_stair_level);
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	const auto sequence_it = handle_kfs_sequences_.find(sequence_key.value());
	if (sequence_it == handle_kfs_sequences_.end()) {
		PoseExecutionResult result;
		result.success = false;
		result.message = "HandleForestKFS sequence not configured: " + sequence_key.value();
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	return execute_sequence(sequence_it->second, sequence_key.value(), "HANDLE_FOREST_KFS");
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

void ArmControllerNode::initialize_handle_kfs_sequences()
{
	handle_kfs_sequences_.clear();

	std::vector<PoseSequenceStep> pick_down_steps;
	if (load_sequence_steps_from_parameters(
			"handle_kfs.pick_down",
			{PoseActionId::POSE_INIT, PoseActionId::POSE_READ_KFS_BELOW, PoseActionId::POSE_INIT},
			&pick_down_steps)) {
		handle_kfs_sequences_["pick_down"] = pick_down_steps;
	}

	std::vector<PoseSequenceStep> pick_up_steps;
	if (load_sequence_steps_from_parameters(
			"handle_kfs.pick_up",
			{PoseActionId::POSE_INIT, PoseActionId::POSE_READ_KFS_ABOVE, PoseActionId::POSE_INIT},
			&pick_up_steps)) {
		handle_kfs_sequences_["pick_up"] = pick_up_steps;
	}

	std::vector<PoseSequenceStep> place_down_steps;
	if (load_sequence_steps_from_parameters(
			"handle_kfs.place_down",
			{PoseActionId::POSE_INIT, PoseActionId::POSE_READ_KFS_BELOW, PoseActionId::POSE_INIT},
			&place_down_steps)) {
		handle_kfs_sequences_["place_down"] = place_down_steps;
	}

	std::vector<PoseSequenceStep> place_up_steps;
	if (load_sequence_steps_from_parameters(
			"handle_kfs.place_up",
			{PoseActionId::POSE_INIT, PoseActionId::POSE_READ_KFS_ABOVE, PoseActionId::POSE_INIT},
			&place_up_steps)) {
		handle_kfs_sequences_["place_up"] = place_up_steps;
	}

	std::vector<PoseSequenceStep> pick_40_up_steps;
	if (load_sequence_steps_from_parameters(
			"handle_kfs.pick_40_up",
			{PoseActionId::POSE_INIT, PoseActionId::POSE_READ_KFS_ABOVE, PoseActionId::POSE_INIT},
			&pick_40_up_steps)) {
		handle_kfs_sequences_["pick_40_up"] = pick_40_up_steps;
	}
}

void ArmControllerNode::update_pose_execution_status()
{
	if (pose_execution_state_.kind == ExecutionKind::SEQUENCE) {
		update_sequence_execution_status();
		return;
	}

	update_single_pose_execution_status();
}

void ArmControllerNode::update_single_pose_execution_status()
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
		pose_execution_state_.feedback.stage = kStageReachingBack;
		pose_execution_state_.feedback.progress = 1.0F;
		last_pose_result_.success = true;
		last_pose_result_.message = "Pose execution completed.";
		RCLCPP_INFO(this->get_logger(), "%s", last_pose_result_.message.c_str());
		return;
	}

	pose_execution_state_.feedback.stage = kStageReachingOut;
	pose_execution_state_.feedback.progress = 0.2F;

	if (!motion_feedback_ready() || !motor_controller_.has_active_target()) {
		return;
	}

	const auto & target = motor_controller_.active_target().value();
	const float max_error = max_active_joint_error(target);

	const float safe_scale = std::max(position_tolerance_ * 10.0F, 0.001F);
	const float normalized_remaining = std::clamp(max_error / safe_scale, 0.0F, 1.0F);
	pose_execution_state_.feedback.progress = std::max(0.2F, 1.0F - normalized_remaining);
}

void ArmControllerNode::update_sequence_execution_status()
{
	if (!pose_execution_state_.active) {
		return;
	}

	if (is_sequence_total_timeout()) {
		mark_pose_execution_failed("Sequence execution timeout.");
		return;
	}

	if (is_sequence_step_timeout()) {
		mark_pose_execution_failed(
			"Sequence step timeout at step index " + std::to_string(pose_execution_state_.active_step_index) + ".");
		return;
	}

	if (!pose_execution_state_.active_sequence_name.has_value()) {
		mark_pose_execution_failed("Internal error: active sequence name is missing.");
		return;
	}

	const auto sequence_it = handle_kfs_sequences_.find(pose_execution_state_.active_sequence_name.value());
	if (sequence_it == handle_kfs_sequences_.end()) {
		mark_pose_execution_failed(
			"Sequence removed during execution: " + pose_execution_state_.active_sequence_name.value());
		return;
	}

	const auto & sequence_steps = sequence_it->second;
	if (pose_execution_state_.active_step_index >= sequence_steps.size()) {
		pose_execution_state_.active = false;
		pose_execution_state_.kind = ExecutionKind::NONE;
		pose_execution_state_.active_pose.reset();
		pose_execution_state_.active_sequence_name.reset();
		pose_execution_state_.started_at.reset();
		pose_execution_state_.step_started_at.reset();
		pose_execution_state_.dwell_started_at.reset();
		pose_execution_state_.step_suction_done = false;
		clear_pending_suction_call();
		pose_execution_state_.feedback.stage = kStageReachingBack;
		pose_execution_state_.feedback.progress = 1.0F;
		motor_controller_.clear_active_target();
		last_pose_result_.success = true;
		last_pose_result_.message = "Sequence execution completed.";
		RCLCPP_INFO(this->get_logger(), "%s", last_pose_result_.message.c_str());
		return;
	}

	if (!motor_controller_.motion_completed()) {
		if (pose_execution_state_.active_step_index + 1U < sequence_steps.size()) {
			pose_execution_state_.feedback.stage = kStageReachingOut;
		} else {
			pose_execution_state_.feedback.stage = kStageReachingBack;
		}
		pose_execution_state_.feedback.progress = 0.2F;

		if (!motion_feedback_ready() || !motor_controller_.has_active_target()) {
			return;
		}

		const auto & target = motor_controller_.active_target().value();
		const float max_error = max_active_joint_error(target);

		const float safe_scale = std::max(position_tolerance_ * 10.0F, 0.001F);
		const float normalized_remaining = std::clamp(max_error / safe_scale, 0.0F, 1.0F);
		pose_execution_state_.feedback.progress = std::max(0.2F, 1.0F - normalized_remaining);
		return;
	}

	const auto & current_step = sequence_steps[pose_execution_state_.active_step_index];
	std::string suction_error;
	if (!process_step_suction_action(current_step, SequenceSuctionWhen::AFTER_REACH, &suction_error)) {
		if (!suction_error.empty()) {
			mark_pose_execution_failed(suction_error);
		}
		return;
	}

	if (current_step.dwell_sec > 0.0) {
		if (!pose_execution_state_.dwell_started_at.has_value()) {
			pose_execution_state_.dwell_started_at = this->now();
		}

		const double dwell_elapsed =
			(this->now() - pose_execution_state_.dwell_started_at.value()).seconds();
		if (dwell_elapsed < current_step.dwell_sec) {
			pose_execution_state_.feedback.stage =
				stage_for_sequence_dwell(pose_execution_state_.active_sequence_name.value());
			pose_execution_state_.feedback.progress = 1.0F;
			return;
		}
	}

	if (!process_step_suction_action(current_step, SequenceSuctionWhen::AFTER_DWELL, &suction_error)) {
		if (!suction_error.empty()) {
			mark_pose_execution_failed(suction_error);
		}
		return;
	}

	const size_t next_step_index = pose_execution_state_.active_step_index + 1U;
	if (next_step_index >= sequence_steps.size()) {
		pose_execution_state_.active_step_index = next_step_index;
		return;
	}

	PoseTarget command_target;
	std::string resolve_error;
	if (!resolve_pose_target_safe(sequence_steps[next_step_index].target, &command_target, &resolve_error)) {
		mark_pose_execution_failed(resolve_error);
		return;
	}

	send_pose_target(
		command_target,
		pose_execution_state_.active_sequence_name.value() + "_step_" + std::to_string(next_step_index));

	pose_execution_state_.active_step_index = next_step_index;
	pose_execution_state_.step_started_at = this->now();
	pose_execution_state_.dwell_started_at.reset();
	pose_execution_state_.step_suction_done = false;
	pose_execution_state_.feedback.stage = kStageInit;
	pose_execution_state_.feedback.progress = 0.0F;
}

void ArmControllerNode::mark_pose_execution_failed(const std::string & message)
{
	pose_execution_state_.active = false;
	pose_execution_state_.kind = ExecutionKind::NONE;
	pose_execution_state_.feedback.stage = kStageReachingBack;
	pose_execution_state_.feedback.progress = 1.0F;
	pose_execution_state_.active_pose.reset();
	pose_execution_state_.active_sequence_name.reset();
	pose_execution_state_.active_step_index = 0U;
	pose_execution_state_.started_at.reset();
	pose_execution_state_.step_started_at.reset();
	pose_execution_state_.dwell_started_at.reset();
	pose_execution_state_.step_suction_done = false;
	clear_pending_suction_call();
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

bool ArmControllerNode::is_sequence_step_timeout() const
{
	if (!pose_execution_state_.active || (pose_execution_state_.kind != ExecutionKind::SEQUENCE)) {
		return false;
	}

	if (motor_controller_.motion_completed()) {
		return false;
	}

	if (!pose_execution_state_.step_started_at.has_value()) {
		return false;
	}

	return (this->now() - pose_execution_state_.step_started_at.value()).seconds() > sequence_step_timeout_sec_;
}

bool ArmControllerNode::is_sequence_total_timeout() const
{
	if (!pose_execution_state_.active || (pose_execution_state_.kind != ExecutionKind::SEQUENCE)) {
		return false;
	}

	if (!pose_execution_state_.started_at.has_value()) {
		return false;
	}

	return (this->now() - pose_execution_state_.started_at.value()).seconds() > sequence_total_timeout_sec_;
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
	pose_execution_state_.kind = ExecutionKind::POSE;
	pose_execution_state_.active_pose = action_id.value();
	pose_execution_state_.active_sequence_name.reset();
	pose_execution_state_.active_step_index = 0U;
	pose_execution_state_.started_at = this->now();
	pose_execution_state_.step_started_at = this->now();
	pose_execution_state_.dwell_started_at.reset();
	pose_execution_state_.step_suction_done = false;
	clear_pending_suction_call();
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

PoseExecutionResult ArmControllerNode::execute_sequence(
	const std::vector<PoseSequenceStep> & sequence_steps,
	const std::string & sequence_name,
	const std::string & trigger_source)
{
	PoseExecutionResult result;

	if (state_machine_.current_state() == ArmState::ESTOP) {
		result.success = false;
		result.message = "Cannot execute sequence while ESTOP is active.";
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	if (pose_execution_state_.active) {
		result.success = false;
		result.message = "Another execution is in progress.";
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	if (sequence_steps.empty()) {
		result.success = false;
		result.message = "Sequence is empty: " + sequence_name;
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	for (size_t i = 0; i < sequence_steps.size(); ++i) {
		if (sequence_steps[i].dwell_sec < 0.0) {
			result.success = false;
			result.message =
				"Sequence has negative dwell_sec at step " + std::to_string(i) + " for " + sequence_name;
			RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
			return result;
		}
	}

	if (suction_enabled_ && (suction_client_ != nullptr) && !suction_client_->service_is_ready()) {
		if (!suction_client_->wait_for_service(200ms)) {
			RCLCPP_WARN(
				this->get_logger(),
				"Suction service '%s' is not ready before starting sequence '%s'.",
				suction_service_name_.c_str(),
				sequence_name.c_str());
		}
	}

	PoseTarget first_target;
	std::string resolve_error;
	if (!resolve_pose_target_safe(sequence_steps.front().target, &first_target, &resolve_error)) {
		result.success = false;
		result.message = resolve_error;
		RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
		return result;
	}

	send_pose_target(first_target, sequence_name + "_step_0");

	pose_execution_state_.active = true;
	pose_execution_state_.kind = ExecutionKind::SEQUENCE;
	pose_execution_state_.active_pose.reset();
	pose_execution_state_.active_sequence_name = sequence_name;
	pose_execution_state_.active_step_index = 0U;
	pose_execution_state_.started_at = this->now();
	pose_execution_state_.step_started_at = this->now();
	pose_execution_state_.dwell_started_at.reset();
	pose_execution_state_.step_suction_done = false;
	clear_pending_suction_call();
	pose_execution_state_.feedback.stage = kStageInit;
	pose_execution_state_.feedback.progress = 0.0F;

	last_pose_result_.success = false;
	last_pose_result_.message = "Sequence execution started.";

	result.success = true;
	result.message =
		"Accepted sequence " + sequence_name + " from " + trigger_source + ". steps=" +
		std::to_string(sequence_steps.size());
	RCLCPP_INFO(this->get_logger(), "%s", result.message.c_str());
	return result;
}

std::optional<std::string> ArmControllerNode::resolve_handle_kfs_sequence_key(
	uint8_t mode,
	uint8_t target_stair_level)
{
	if ((mode == kHandleKfsModePick) && (target_stair_level == kHandleKfsLevelDown)) {
		return std::string("pick_down");
	}

	if ((mode == kHandleKfsModePick) && (target_stair_level == kHandleKfsLevelUp)) {
		return std::string("pick_up");
	}

	if ((mode == kHandleKfsModePlace) && (target_stair_level == kHandleKfsLevelDown)) {
		return std::string("place_down");
	}

	if ((mode == kHandleKfsModePlace) && (target_stair_level == kHandleKfsLevelUp)) {
		return std::string("place_up");
	}

	if ((mode == kHandleKfsModePick) && (target_stair_level == kHandleKfsLevel40Up)) {
		return std::string("pick_40_up");
	}

	if ((mode == kHandleKfsModePlace) && (target_stair_level == kHandleKfsLevel40Up)) {
		return std::string("place_up");
	}

	return std::nullopt;
}

bool ArmControllerNode::load_sequence_steps_from_parameters(
	const std::string & base_param,
	const std::vector<PoseActionId> & fallback_pose_ids,
	std::vector<PoseSequenceStep> * sequence_steps)
{
	if (sequence_steps == nullptr) {
		return false;
	}

	const int default_step_count = static_cast<int>(fallback_pose_ids.size());
	const int configured_step_count = this->declare_parameter<int>(base_param + ".step_count", default_step_count);
	if (configured_step_count <= 0) {
		RCLCPP_ERROR(
			this->get_logger(),
			"Invalid %s.step_count=%d, expected > 0.",
			base_param.c_str(),
			configured_step_count);
		return false;
	}

	sequence_steps->clear();
	sequence_steps->reserve(static_cast<size_t>(configured_step_count));
	for (int i = 0; i < configured_step_count; ++i) {
		PoseTarget fallback_target = pose_init_;
		if (static_cast<size_t>(i) < fallback_pose_ids.size()) {
			const auto pose_target = pose_target_of(fallback_pose_ids[static_cast<size_t>(i)]);
			if (pose_target == nullptr) {
				RCLCPP_ERROR(
					this->get_logger(),
					"Fallback pose is not configured for %s at step %d.",
					base_param.c_str(),
					i);
				return false;
			}
			fallback_target = *pose_target;
		} else if (!sequence_steps->empty()) {
			fallback_target = sequence_steps->back().target;
		}

		PoseSequenceStep step;
		step.target = fallback_target;
		step.dwell_sec = 0.0;

		const std::string step_prefix = base_param + ".steps." + std::to_string(i);
		const std::string pose_name = this->declare_parameter<std::string>(step_prefix + ".pose", "");
		if (!pose_name.empty()) {
			PoseTarget referenced_pose;
			if (!lookup_named_pose(pose_name, &referenced_pose)) {
				RCLCPP_ERROR(
					this->get_logger(),
					"Unknown pose key '%s' for %s at step %d. Add it under poses.<name> and list it in poses.names.",
					pose_name.c_str(),
					base_param.c_str(),
					i);
				return false;
			}

			step.target = referenced_pose;
		}

		step.target.joint1 = this->declare_parameter<double>(step_prefix + ".joint1", step.target.joint1);
		step.target.joint2 = this->declare_parameter<double>(step_prefix + ".joint2", step.target.joint2);
		step.target.joint3 = this->declare_parameter<double>(step_prefix + ".joint3", step.target.joint3);
		step.target.joint4 = this->declare_parameter<double>(step_prefix + ".joint4", step.target.joint4);
		step.dwell_sec = this->declare_parameter<double>(step_prefix + ".dwell_sec", 0.0);

		const int suction_action_raw =
			this->declare_parameter<int>(step_prefix + ".suction_action", 0);
		const auto suction_action = parse_sequence_suction_action(suction_action_raw);
		if (!suction_action.has_value()) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Invalid suction_action for %s at step %d: %d. Expected one of [0,1,2].",
				base_param.c_str(),
				i,
				suction_action_raw);
			return false;
		}
		step.suction_action = suction_action.value();

		const std::string suction_when_raw =
			this->declare_parameter<std::string>(step_prefix + ".suction_when", "after_reach");
		const auto suction_when = parse_sequence_suction_when(suction_when_raw);
		if (!suction_when.has_value()) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Invalid suction_when for %s at step %d: '%s'. Expected one of [after_reach,after_dwell].",
				base_param.c_str(),
				i,
				suction_when_raw.c_str());
			return false;
		}
		step.suction_when = suction_when.value();

		if (step.dwell_sec < 0.0) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Invalid dwell_sec for %s at step %d: %.3f",
				base_param.c_str(),
				i,
				step.dwell_sec);
			return false;
		}

		sequence_steps->push_back(step);
	}

	RCLCPP_INFO(
		this->get_logger(),
		"Loaded sequence %s with %zu steps.",
		base_param.c_str(),
		sequence_steps->size());
	return true;
}

void ArmControllerNode::clear_pending_suction_call()
{
	has_pending_suction_future_ = false;
	pending_suction_started_at_.reset();
	pending_suction_step_index_ = 0U;
	pending_suction_target_ = false;
}

bool ArmControllerNode::process_step_suction_action(
	const PoseSequenceStep & current_step,
	SequenceSuctionWhen trigger_when,
	std::string * error_message)
{
	if (error_message == nullptr) {
		return false;
	}

	error_message->clear();

	if (!suction_enabled_ || (current_step.suction_action == SequenceSuctionAction::NONE)) {
		pose_execution_state_.step_suction_done = true;
		clear_pending_suction_call();
		return true;
	}

	if (current_step.suction_when != trigger_when) {
		return true;
	}

	if (pose_execution_state_.step_suction_done) {
		return true;
	}

	if (suction_client_ == nullptr) {
		*error_message = "Suction service client is not initialized.";
		return false;
	}

	if (!has_pending_suction_future_) {
		if (!suction_client_->service_is_ready()) {
			*error_message = "Suction service '" + suction_service_name_ + "' is not ready.";
			if (suction_fail_on_error_) {
				return false;
			}

			RCLCPP_WARN(this->get_logger(), "%s", error_message->c_str());
			error_message->clear();
			pose_execution_state_.step_suction_done = true;
			return true;
		}

		auto request = std::make_shared<suction_control::srv::SetSuction::Request>();
		request->suck = (current_step.suction_action == SequenceSuctionAction::ON);
		pending_suction_future_ = suction_client_->async_send_request(request).future.share();
		has_pending_suction_future_ = true;
		pending_suction_started_at_ = this->now();
		pending_suction_step_index_ = pose_execution_state_.active_step_index;
		pending_suction_target_ = request->suck;
		pose_execution_state_.feedback.stage = request->suck ? kStageTurningOnSuction : kStageTurningOffSuction;
		pose_execution_state_.feedback.progress = 1.0F;
		RCLCPP_INFO(
			this->get_logger(),
			"Requested suction action at sequence step %zu: action=%s, trigger=%s",
			pose_execution_state_.active_step_index,
			suction_action_to_string(current_step.suction_action),
			suction_when_to_string(current_step.suction_when));
		return false;
	}

	if (pending_suction_step_index_ != pose_execution_state_.active_step_index) {
		*error_message = "Internal error: pending suction response step index mismatch.";
		clear_pending_suction_call();
		return false;
	}

	if (pending_suction_future_.wait_for(0ms) != std::future_status::ready) {
		if (pending_suction_started_at_.has_value() &&
			((this->now() - pending_suction_started_at_.value()).seconds() > suction_call_timeout_sec_)) {
			*error_message =
				"Suction service call timeout at step " + std::to_string(pose_execution_state_.active_step_index) + ".";
			clear_pending_suction_call();
			if (!suction_fail_on_error_) {
				RCLCPP_WARN(this->get_logger(), "%s", error_message->c_str());
				error_message->clear();
				pose_execution_state_.step_suction_done = true;
				return true;
			}
		}
		return false;
	}

	suction_control::srv::SetSuction::Response::SharedPtr response;
	try {
		response = pending_suction_future_.get();
	} catch (const std::exception & ex) {
		*error_message =
			"Suction service call threw exception at step " +
			std::to_string(pose_execution_state_.active_step_index) + ": " + ex.what();
		clear_pending_suction_call();
		if (!suction_fail_on_error_) {
			RCLCPP_WARN(this->get_logger(), "%s", error_message->c_str());
			error_message->clear();
			pose_execution_state_.step_suction_done = true;
			return true;
		}
		return false;
	} catch (...) {
		*error_message =
			"Suction service call threw unknown exception at step " +
			std::to_string(pose_execution_state_.active_step_index) + ".";
		clear_pending_suction_call();
		if (!suction_fail_on_error_) {
			RCLCPP_WARN(this->get_logger(), "%s", error_message->c_str());
			error_message->clear();
			pose_execution_state_.step_suction_done = true;
			return true;
		}
		return false;
	}
	const bool requested_suck = pending_suction_target_;
	clear_pending_suction_call();
	if (response == nullptr) {
		*error_message =
			"Suction service returned null response at step " + std::to_string(pose_execution_state_.active_step_index) + ".";
		if (!suction_fail_on_error_) {
			RCLCPP_WARN(this->get_logger(), "%s", error_message->c_str());
			error_message->clear();
			pose_execution_state_.step_suction_done = true;
			return true;
		}
		return false;
	}

	if (!response->success) {
		*error_message =
			"Suction service failed at step " + std::to_string(pose_execution_state_.active_step_index) +
			": " + response->message;
		if (!suction_fail_on_error_) {
			RCLCPP_WARN(this->get_logger(), "%s", error_message->c_str());
			error_message->clear();
			pose_execution_state_.step_suction_done = true;
			return true;
		}
		return false;
	}

	pose_execution_state_.step_suction_done = true;
	RCLCPP_INFO(
		this->get_logger(),
		"Suction action completed at step %zu: target=%s, current=%s",
		pose_execution_state_.active_step_index,
		requested_suck ? "on" : "off",
		response->current_suck ? "on" : "off");
	return true;
}

bool ArmControllerNode::lookup_named_pose(const std::string & pose_name, PoseTarget * pose_target) const
{
	if (pose_target == nullptr) {
		return false;
	}

	const auto it = named_poses_.find(pose_name);
	if (it == named_poses_.end()) {
		return false;
	}

	*pose_target = it->second;
	return true;
}

bool ArmControllerNode::resolve_pose_target_safe(
	const PoseTarget & requested_target,
	PoseTarget * resolved_target,
	std::string * error_message) const
{
	if ((resolved_target == nullptr) || (error_message == nullptr)) {
		return false;
	}

	if (!motion_feedback_ready()) {
		*error_message = "Cannot execute pose before all joint feedback is ready.";
		return false;
	}

	const std::array<const std::optional<custom_msgs::msg::ReadDmMotor> *, 4> feedbacks{
		&motor_controller_.joint1_feedback(),
		&motor_controller_.joint2_feedback(),
		&motor_controller_.joint3_feedback(),
		&motor_controller_.joint4_feedback()};

	const std::array<float, 4> requested_positions{
		requested_target.joint1,
		requested_target.joint2,
		requested_target.joint3,
		requested_target.joint4};

	std::array<float, 4> resolved_positions{};
	for (size_t i = 0; i < 4; ++i) {
		if (!joint_enabled_[i]) {
			resolved_positions[i] = requested_positions[i];
			continue;
		}

		if (!feedbacks[i]->has_value()) {
			*error_message = "Cannot execute pose before all enabled joint feedback is ready.";
			return false;
		}

		const auto current_position = current_joint_position_for_control(i);
		if (!current_position.has_value()) {
			*error_message = "Cannot execute pose before all enabled joint feedback is ready.";
			return false;
		}

		const auto resolved = find_safe_target_position(
			current_position.value(),
			requested_positions[i],
			joint_limits_[i]);
		if (!resolved.has_value()) {
			*error_message =
				"No safe target for joint" + std::to_string(i + 1) +
				" within limits " + describe_joint_limit(joint_limits_[i]) + " from current=" +
				std::to_string(current_position.value()) + " requested=" + std::to_string(requested_positions[i]);
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

std::optional<float> ArmControllerNode::current_joint_position_for_control(size_t joint_index) const
{
	if (joint_index >= continuous_joint_positions_.size()) {
		return std::nullopt;
	}

	return continuous_joint_positions_[joint_index];
}

void ArmControllerNode::update_continuous_joint_position(size_t joint_index, float raw_position)
{
	if (joint_index >= continuous_joint_positions_.size()) {
		return;
	}

	if (!is_wrapped_joint_limit(joint_limits_[joint_index])) {
		continuous_joint_positions_[joint_index] = raw_position;
		return;
	}

	if (!continuous_joint_positions_[joint_index].has_value()) {
		continuous_joint_positions_[joint_index] = normalize_wrapped_angle(raw_position);
		return;
	}

	continuous_joint_positions_[joint_index] = unwrap_angle_near_reference(
		raw_position,
		continuous_joint_positions_[joint_index].value());
}

bool ArmControllerNode::motion_feedback_ready() const
{
	for (size_t i = 0; i < continuous_joint_positions_.size(); ++i) {
		if (!joint_enabled_[i]) {
			continue;
		}
		if (!continuous_joint_positions_[i].has_value()) {
			return false;
		}
	}

	return true;
}

bool ArmControllerNode::motion_target_reached() const
{
	if (!motor_controller_.has_active_target() || !motion_feedback_ready()) {
		return false;
	}

	return max_active_joint_error(motor_controller_.active_target().value()) < position_tolerance_;
}

float ArmControllerNode::max_active_joint_error(const PoseTarget & target) const
{
	float max_error = 0.0F;

	if (joint_enabled_[0]) {
		const float joint1_error = std::fabs(target.joint1 - current_joint_position_for_control(0U).value());
		max_error = std::max(max_error, joint1_error);
	}
	if (joint_enabled_[1]) {
		const float joint2_error = std::fabs(target.joint2 - current_joint_position_for_control(1U).value());
		max_error = std::max(max_error, joint2_error);
	}
	if (joint_enabled_[2]) {
		const float joint3_error = std::fabs(target.joint3 - current_joint_position_for_control(2U).value());
		max_error = std::max(max_error, joint3_error);
	}
	if (joint_enabled_[3]) {
		const float joint4_error = std::fabs(target.joint4 - current_joint_position_for_control(3U).value());
		max_error = std::max(max_error, joint4_error);
	}

	return max_error;
}

std::optional<float> ArmControllerNode::find_safe_target_position(
	float current_position,
	float requested_target,
	const JointLimit & limit)
{
	const float kEpsilon = 1e-5F;
	if (!is_position_within_joint_limit(current_position, limit, kEpsilon)) {
		return std::nullopt;
	}

	if (is_wrapped_joint_limit(limit)) {
		const int nearest_turn = static_cast<int>(std::lround((current_position - requested_target) / kTwoPi));

		float best_target = 0.0F;
		float best_cost = std::numeric_limits<float>::max();
		bool found = false;

		for (int turn_offset = -1; turn_offset <= 1; ++turn_offset) {
			const float candidate = requested_target + static_cast<float>(nearest_turn + turn_offset) * kTwoPi;
			if (!is_position_within_joint_limit(candidate, limit, kEpsilon)) {
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

	joint1_msg.enable = joint_enabled_[0] ? 1U : 0U;
	joint1_msg.position = joint_enabled_[0] ? target.joint1 : 0.0F;
	joint1_msg.speed = joint_enabled_[0] ? speed : 0.0F;

	joint2_msg.enable = joint_enabled_[1] ? 1U : 0U;
	joint2_msg.position = joint_enabled_[1] ? target.joint2 : 0.0F;
	joint2_msg.speed = joint_enabled_[1] ? speed : 0.0F;

	joint3_msg.enable = joint_enabled_[2] ? 1U : 0U;
	joint3_msg.position = joint_enabled_[2] ? target.joint3 : 0.0F;
	joint3_msg.speed = joint_enabled_[2] ? speed : 0.0F;

	joint4_msg.enable = joint_enabled_[3] ? 1U : 0U;
	joint4_msg.position = joint_enabled_[3] ? target.joint4 : 0.0F;
	joint4_msg.speed = joint_enabled_[3] ? speed : 0.0F;

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
	} else if ((latest_sbus_.online == 0U) || (latest_sbus_.frame_lost != 0U)) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"SBUS abnormal: online=%u frame_lost=%u",
			latest_sbus_.online,
			latest_sbus_.frame_lost);
	}

	if (joint_enabled_[0] && motor_controller_.joint1_has_fault()) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"Joint1 feedback indicates communication/power/thermal fault.");
	}
	if (joint_enabled_[1] && motor_controller_.joint2_has_fault()) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"Joint2 feedback indicates communication/power/thermal fault.");
	}
	if (joint_enabled_[2] && motor_controller_.joint3_has_fault()) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			warn_throttle_ms_,
			"Joint3 feedback indicates communication/power/thermal fault.");
	}
	if (joint_enabled_[3] && motor_controller_.joint4_has_fault()) {
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

