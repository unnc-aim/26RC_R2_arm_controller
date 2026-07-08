#ifndef ARM_CONTROLLER__STATE_MACHINE_HPP_
#define ARM_CONTROLLER__STATE_MACHINE_HPP_

namespace arm_controller
{

enum class ArmState
{
  ESTOP,
  IDLE,
  MANUAL,
  SHOW_LOW,
  SHOW_HIGH
};

enum class ModeEvent
{
  NONE,
  ENTER_MANUAL,
  ENTER_SHOW_LOW,
  ENTER_SHOW_HIGH
};

struct StateTransition
{
  ArmState from{ArmState::IDLE};
  ArmState to{ArmState::IDLE};
  bool changed{false};
  bool entered_estop{false};
  bool exited_estop{false};
  bool clear_action_state{false};
  bool trigger_manual_pose{false};
  bool trigger_show_low_pose{false};
  bool trigger_show_high_pose{false};
};

class StateMachine
{
public:
  StateMachine() = default;

  [[nodiscard]] StateTransition update(
    bool estop_asserted,
    bool estop_released,
    ModeEvent mode_event);

  [[nodiscard]] ArmState current_state() const;

private:
  ArmState state_{ArmState::IDLE};
  bool estop_latched_{false};
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__STATE_MACHINE_HPP_
