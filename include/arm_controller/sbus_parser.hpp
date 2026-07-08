#ifndef ARM_CONTROLLER__SBUS_PARSER_HPP_
#define ARM_CONTROLLER__SBUS_PARSER_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "custom_msgs/msg/read_sbusrc.hpp"
#include "rclcpp/time.hpp"

#include "arm_controller/state_machine.hpp"

namespace arm_controller
{

enum class ModeZone
{
  HIGH,
  MID,
  LOW
};

struct SbusConfig
{
  std::string topic{"/sbus/read"};
  int mode_channel{4};
  int estop_channel{6};

  int estop_high{1600};
  int estop_low{360};

  int mode_high{1600};
  int mode_mid{1024};
  int mode_low{360};
  int debounce_ms{100};
};

struct SbusParseOutput
{
  bool message_valid{false};
  bool sbus_ok{false};

  bool estop_asserted{false};
  bool estop_released{false};
  bool estop_active{false};

  ModeEvent mode_event{ModeEvent::NONE};
};

class SbusParser
{
public:
  explicit SbusParser(const SbusConfig & config);

  [[nodiscard]] SbusParseOutput parse(
    const custom_msgs::msg::ReadSBUSRC & msg,
    const rclcpp::Time & now);

private:
  [[nodiscard]] bool channel_index_valid(std::size_t channels_size, int index) const;
  [[nodiscard]] ModeZone classify_mode_zone(uint16_t value) const;

  SbusConfig config_;

  bool estop_latched_{false};

  bool zone_initialized_{false};
  ModeZone stable_zone_{ModeZone::HIGH};
  ModeZone candidate_zone_{ModeZone::HIGH};
  std::optional<rclcpp::Time> candidate_since_;
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__SBUS_PARSER_HPP_
