#include "robotos_core/safety_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace robotos_core {
namespace {
constexpr double kJointLimit = 1.57;
constexpr double kSensorTimeout = 1.0;
constexpr double kHeartbeatTimeout = 3.0;
constexpr double kMotionTimeout = 10.0;
constexpr double kTolerance = 0.02;
double total_error(const std::array<double, 2>& a, const std::array<double, 2>& b) {
  return std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]);
}
}  // namespace

SafetyStateMachine::SafetyStateMachine(double started_at)
    : started_at_(started_at), last_sensor_at_(started_at), last_heartbeat_at_(started_at) {}

bool SafetyStateMachine::valid_goal(const std::array<double, 2>& target, double max_velocity) {
  return std::isfinite(target[0]) && std::isfinite(target[1]) &&
         std::abs(target[0]) <= kJointLimit && std::abs(target[1]) <= kJointLimit &&
         std::isfinite(max_velocity) && max_velocity > 0.0 && max_velocity <= 0.5;
}

bool SafetyStateMachine::healthy(double now) const {
  return good_samples_ >= 2 && have_heartbeat_ &&
         now - last_sensor_at_ <= kSensorTimeout &&
         now - last_heartbeat_at_ <= kHeartbeatTimeout;
}

void SafetyStateMachine::on_sensor(const std::array<double, 2>& position, double now) {
  if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
      std::abs(position[0]) > kJointLimit + 0.01 ||
      std::abs(position[1]) > kJointLimit + 0.01) return;
  position_ = position;
  last_sensor_at_ = now;
  good_samples_ = std::min(good_samples_ + 1, 2);
  if (mode_ == Mode::STARTING && healthy(now)) mode_ = Mode::IDLE;
}

void SafetyStateMachine::on_heartbeat(double now) {
  have_heartbeat_ = true;
  last_heartbeat_at_ = now;
  if (mode_ == Mode::STARTING && healthy(now)) mode_ = Mode::IDLE;
}

bool SafetyStateMachine::start(const std::array<double, 2>& target, double max_velocity,
                               const std::string& task_id, double now) {
  if (mode_ != Mode::IDLE || !healthy(now) || !valid_goal(target, max_velocity) ||
      task_id.empty()) return false;
  target_ = target;
  max_velocity_ = max_velocity;
  task_id_ = task_id;
  motion_started_at_ = now;
  initial_error_ = total_error(position_, target_);
  completed_progress_ = 0.0f;
  settled_ticks_ = 0;
  mode_ = Mode::RUNNING;
  return true;
}

void SafetyStateMachine::finish(Terminal terminal, const std::string& code) {
  if (task_id_.empty()) return;
  completed_progress_ = terminal == Terminal::SUCCEEDED ? 1.0f :
      static_cast<float>(std::clamp(1.0 - total_error(position_, target_) /
          std::max(initial_error_, 1e-9), 0.0, 1.0));
  completion_ = Completion{terminal, code, task_id_, position_};
  task_id_.clear();
  max_velocity_ = 0.0;
}

bool SafetyStateMachine::cancel(double now) {
  if (mode_ != Mode::RUNNING) return false;
  finish(Terminal::CANCELED, "CANCELED");
  mode_ = healthy(now) ? Mode::IDLE : Mode::FAULT;
  if (mode_ == Mode::FAULT) {
    fault_code_ = "SENSOR_TIMEOUT";
    good_samples_ = 0;
  }
  return true;
}

void SafetyStateMachine::fault(const std::string& code) {
  if (mode_ == Mode::ESTOP) {
    if (fault_code_ == "NONE") {
      fault_code_ = code;
      good_samples_ = 0;
    }
    return;
  }
  if (mode_ == Mode::FAULT) return;
  finish(Terminal::ABORTED, code);
  fault_code_ = code;
  good_samples_ = 0;
  mode_ = Mode::FAULT;
}

void SafetyStateMachine::set_estop(bool engage, double now) {
  if (engage) {
    if (mode_ != Mode::ESTOP) finish(Terminal::ABORTED, "ESTOP");
    mode_ = Mode::ESTOP;
    return;
  }
  if (mode_ != Mode::ESTOP || !healthy(now)) return;
  mode_ = fault_code_ == "NONE" ? Mode::IDLE : Mode::FAULT;
}

bool SafetyStateMachine::reset_fault(double now) {
  if (mode_ != Mode::FAULT || !healthy(now)) return false;
  fault_code_ = "NONE";
  mode_ = Mode::IDLE;
  return true;
}

std::array<double, 2> SafetyStateMachine::tick(double now) {
  if (mode_ == Mode::STARTING && now - started_at_ > kSensorTimeout && good_samples_ == 0)
    fault("SENSOR_TIMEOUT");
  if (mode_ != Mode::FAULT && now - last_sensor_at_ > kSensorTimeout)
    fault("SENSOR_TIMEOUT");
  if (mode_ != Mode::FAULT && now - last_heartbeat_at_ > kHeartbeatTimeout)
    fault("SIM_HEARTBEAT_TIMEOUT");
  if (mode_ == Mode::RUNNING && now - motion_started_at_ > kMotionTimeout)
    fault("MOTION_TIMEOUT");
  if (mode_ != Mode::RUNNING) return {0.0, 0.0};
  // Stop on a short sensor gap, then latch a fault if it persists for 1 s.
  if (now - last_sensor_at_ > 0.2) return {0.0, 0.0};
  if (std::abs(target_[0] - position_[0]) <= kTolerance &&
      std::abs(target_[1] - position_[1]) <= kTolerance) {
    if (++settled_ticks_ >= 3) {
      finish(Terminal::SUCCEEDED, "OK");
      mode_ = Mode::IDLE;
      return {0.0, 0.0};
    }
  } else {
    settled_ticks_ = 0;
  }
  std::array<double, 2> command{};
  for (int i = 0; i < 2; ++i) {
    command[i] = std::clamp(2.0 * (target_[i] - position_[i]), -max_velocity_, max_velocity_);
    if ((position_[i] >= kJointLimit && command[i] > 0.0) ||
        (position_[i] <= -kJointLimit && command[i] < 0.0)) command[i] = 0.0;
  }
  return command;
}

Snapshot SafetyStateMachine::snapshot(double now) const {
  const double remaining = total_error(position_, target_);
  const float progress = mode_ == Mode::RUNNING
      ? static_cast<float>(std::clamp(1.0 - remaining / std::max(initial_error_, 1e-9), 0.0, 1.0))
      : completed_progress_;
  return {mode_, fault_code_, now - last_sensor_at_ <= kSensorTimeout && good_samples_ >= 2,
          position_, target_, task_id_, progress};
}

std::optional<Completion> SafetyStateMachine::take_completion() {
  auto result = completion_;
  completion_.reset();
  return result;
}

const char* SafetyStateMachine::mode_name(Mode mode) {
  switch (mode) {
    case Mode::STARTING: return "STARTING";
    case Mode::IDLE: return "IDLE";
    case Mode::RUNNING: return "RUNNING";
    case Mode::FAULT: return "FAULT";
    case Mode::ESTOP: return "ESTOP";
  }
  return "UNKNOWN";
}

}  // namespace robotos_core
