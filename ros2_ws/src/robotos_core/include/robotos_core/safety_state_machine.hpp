#pragma once

#include <array>
#include <optional>
#include <string>

namespace robotos_core {

enum class Mode { STARTING, IDLE, RUNNING, FAULT, ESTOP };
enum class Terminal { SUCCEEDED, CANCELED, ABORTED };

struct Completion {
  Terminal terminal;
  std::string code;
  std::string task_id;
  std::array<double, 2> final_position;
};

struct Snapshot {
  Mode mode;
  std::string fault_code;
  bool sensor_fresh;
  std::array<double, 2> position;
  std::array<double, 2> target;
  std::string task_id;
  float progress;
};

class SafetyStateMachine {
 public:
  explicit SafetyStateMachine(double started_at);
  static bool valid_goal(const std::array<double, 2>& target, double max_velocity);
  void on_sensor(const std::array<double, 2>& position, double now);
  void on_heartbeat(double now);
  bool start(const std::array<double, 2>& target, double max_velocity,
             const std::string& task_id, double now);
  bool cancel(double now);
  void set_estop(bool engage, double now);
  bool reset_fault(double now);
  std::array<double, 2> tick(double now);
  Snapshot snapshot(double now) const;
  std::optional<Completion> take_completion();
  static const char* mode_name(Mode mode);

 private:
  bool healthy(double now) const;
  void fault(const std::string& code);
  void finish(Terminal terminal, const std::string& code);

  Mode mode_{Mode::STARTING};
  std::string fault_code_{"NONE"};
  std::array<double, 2> position_{0.0, 0.0};
  std::array<double, 2> target_{0.0, 0.0};
  std::string task_id_;
  double started_at_;
  double last_sensor_at_;
  double last_heartbeat_at_;
  double motion_started_at_{0.0};
  double max_velocity_{0.0};
  double initial_error_{0.0};
  float completed_progress_{0.0f};
  int good_samples_{0};
  int settled_ticks_{0};
  bool have_heartbeat_{false};
  std::optional<Completion> completion_;
};

}  // namespace robotos_core
