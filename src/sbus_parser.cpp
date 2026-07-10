#include "arm_controller/sbus_parser.hpp"

#include <algorithm>
#include <cstddef>

namespace arm_controller
{

SbusParser::SbusParser(const SbusConfig & config)
: config_(config)
{
}

SbusParseOutput SbusParser::parse(
  const custom_msgs::msg::ReadSBUSRC & msg,
  const rclcpp::Time & now)
{
  SbusParseOutput output;
  output.message_valid = true;

  // 先做通道索引安全检查，避免越界访问。
  if (!channel_index_valid(msg.channels.size(), config_.mode_channel) ||
    !channel_index_valid(msg.channels.size(), config_.estop_channel))
  {
    output.message_valid = false;
    return output;
  }

  // 临时策略：fail_safe 不参与 arm_controller 的健康判定。
  output.sbus_ok = (msg.online != 0U) && (msg.frame_lost == 0U);

  const uint16_t estop_value = msg.channels[static_cast<std::size_t>(config_.estop_channel)];
  // 急停采用滞回：高阈值触发，低阈值解除，避免拨杆抖动反复进出。
  if (!estop_latched_ && static_cast<int>(estop_value) >= config_.estop_high) {
    estop_latched_ = true;
    output.estop_asserted = true;
  }
  if (estop_latched_ && static_cast<int>(estop_value) <= config_.estop_low) {
    estop_latched_ = false;
    output.estop_released = true;
  }
  output.estop_active = estop_latched_;

  const uint16_t mode_value = msg.channels[static_cast<std::size_t>(config_.mode_channel)];
  const ModeZone current_zone = classify_mode_zone(mode_value);

  if (!zone_initialized_) {
    // 初始化时仅建立稳定区间，不触发动作事件。
    zone_initialized_ = true;
    stable_zone_ = current_zone;
    candidate_zone_ = current_zone;
    candidate_since_ = now;
    return output;
  }

  if (current_zone != candidate_zone_) {
    candidate_zone_ = current_zone;
    candidate_since_ = now;
    return output;
  }

  if (!candidate_since_.has_value()) {
    candidate_since_ = now;
    return output;
  }

  // 只有候选区间持续超过去抖时间才承认状态变化。
  const auto debounce_duration = rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(config_.debounce_ms) * 1000LL * 1000LL);

  if (now - candidate_since_.value() < debounce_duration) {
    return output;
  }

  if (candidate_zone_ == stable_zone_) {
    return output;
  }

  stable_zone_ = candidate_zone_;

  // 三段拨杆稳定后直接映射到姿态：高=MANUAL，中=SHOW_LOW，低=SHOW_HIGH。
  if (stable_zone_ == ModeZone::HIGH) {
    output.mode_event = ModeEvent::ENTER_MANUAL;
  } else if (stable_zone_ == ModeZone::MID) {
    output.mode_event = ModeEvent::ENTER_SHOW_LOW;
  } else if (stable_zone_ == ModeZone::LOW) {
    output.mode_event = ModeEvent::ENTER_SHOW_HIGH;
  }

  return output;
}

bool SbusParser::channel_index_valid(std::size_t channels_size, int index) const
{
  return index >= 0 && static_cast<std::size_t>(index) < channels_size;
}

ModeZone SbusParser::classify_mode_zone(uint16_t value) const
{
  const int high_mid_boundary = (config_.mode_high + config_.mode_mid) / 2;
  const int mid_low_boundary = (config_.mode_mid + config_.mode_low) / 2;
  const int raw_value = static_cast<int>(value);

  if (raw_value >= high_mid_boundary) {
    return ModeZone::HIGH;
  }
  if (raw_value <= mid_low_boundary) {
    return ModeZone::LOW;
  }
  return ModeZone::MID;
}

}  // namespace arm_controller
