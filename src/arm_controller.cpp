#include "arm_controller/arm_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

using namespace std::chrono_literals;

namespace arm_controller
{

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
	prefer_joint1_positive_direction_to_show_high_ =
		this->declare_parameter<bool>("motion.prefer_joint1_positive_direction_to_show_high", true);
	warn_throttle_ms_ = this->declare_parameter<int>("diagnostics.warn_throttle_ms", 1000);

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

	if (loop_hz_ <= 0.0) {
		RCLCPP_ERROR(
			this->get_logger(),
			"Invalid parameter motion.control_loop_hz=%.3f, must be > 0.",
			loop_hz_);
		throw std::invalid_argument("motion.control_loop_hz must be > 0");
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
	}

	if (transition.trigger_manual_pose) {
		send_pose_target(manual_pose_, "MANUAL");
	}

	if (transition.trigger_show_low_pose) {
		send_pose_target(show_low_pose_, "SHOW_LOW");
	}

	if (transition.trigger_show_high_pose) {
		send_pose_target(show_high_pose_, "SHOW_HIGH");
	}
}

void ArmControllerNode::send_pose_target(const PoseTarget & target, const std::string & pose_name)
{
	const bool force_joint1_positive_direction =
		prefer_joint1_positive_direction_to_show_high_ && (pose_name == "SHOW_HIGH");
	const PoseTarget command_target = build_command_target(target, force_joint1_positive_direction);
	publish_target_command(command_target, motion_speed_);

	motor_controller_.set_active_target(command_target);

	RCLCPP_INFO(
		this->get_logger(),
		"%s target sent once: joint1=%.3f, joint2=%.3f, joint3=%.3f, joint4=%.3f, speed=%.3f",
		pose_name.c_str(),
		command_target.joint1,
		command_target.joint2,
		command_target.joint3,
		command_target.joint4,
		motion_speed_);
}

PoseTarget ArmControllerNode::build_command_target(
	const PoseTarget & target,
	bool force_joint1_positive_direction) const
{
	PoseTarget command_target = target;
	if (!force_joint1_positive_direction) {
		return command_target;
	}

	const auto & joint1_feedback = motor_controller_.joint1_feedback();
	if (!joint1_feedback.has_value()) {
		return command_target;
	}

	constexpr float kTwoPi = 6.28318530717958647692F;
	const float current_joint1 = joint1_feedback->position;
	if ((current_joint1 > 0.0F) && (command_target.joint1 < 0.0F)) {
		while (command_target.joint1 <= current_joint1) {
			command_target.joint1 += kTwoPi;
		}
	}

	return command_target;
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

