#include "arm_controller/state_machine.hpp"

namespace arm_controller
{

StateTransition StateMachine::update(
  bool estop_asserted,
  bool estop_released,
  ModeEvent mode_event)
{
  StateTransition transition;
  transition.from = state_;
  transition.to = state_;

  // 急停具有最高优先级，可从任意状态抢占进入。
  if (!estop_latched_ && estop_asserted) {
    estop_latched_ = true;
    state_ = ArmState::ESTOP;
    transition.to = state_;
    transition.changed = (transition.from != transition.to);
    transition.entered_estop = true;
    transition.clear_action_state = true;
    return transition;
  }

  if (estop_latched_) {
    // 急停锁存后仅允许通过释放条件退出到 IDLE。
    if (estop_released) {
      estop_latched_ = false;
      state_ = ArmState::IDLE;
      transition.to = state_;
      transition.changed = (transition.from != transition.to);
      transition.exited_estop = true;
      transition.clear_action_state = true;
      return transition;
    }
    state_ = ArmState::ESTOP;
    transition.to = state_;
    transition.changed = (transition.from != transition.to);
    return transition;
  }

  // 非急停时，按边沿事件切换工作模式。
  if (mode_event == ModeEvent::ENTER_MANUAL && state_ != ArmState::MANUAL) {
    state_ = ArmState::MANUAL;
    transition.to = state_;
    transition.changed = true;
    transition.trigger_manual_pose = true;
    return transition;
  }

  if (mode_event == ModeEvent::ENTER_SHOW_LOW && state_ != ArmState::SHOW_LOW) {
    state_ = ArmState::SHOW_LOW;
    transition.to = state_;
    transition.changed = true;
    transition.trigger_show_low_pose = true;
    return transition;
  }

  if (mode_event == ModeEvent::ENTER_SHOW_HIGH && state_ != ArmState::SHOW_HIGH) {
    state_ = ArmState::SHOW_HIGH;
    transition.to = state_;
    transition.changed = true;
    transition.trigger_show_high_pose = true;
    return transition;
  }

  return transition;
}

ArmState StateMachine::current_state() const
{
  return state_;
}

}  // namespace arm_controller
