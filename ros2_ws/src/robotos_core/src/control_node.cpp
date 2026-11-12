#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robotos_interfaces/action/move_joints.hpp"
#include "robotos_interfaces/msg/joint_velocity_command.hpp"
#include "robotos_interfaces/msg/robot_status.hpp"
#include "robotos_interfaces/msg/sim_heartbeat.hpp"
#include "robotos_interfaces/srv/reset_fault.hpp"
#include "robotos_interfaces/srv/set_e_stop.hpp"
#include "robotos_core/safety_state_machine.hpp"

using namespace std::chrono_literals;
using Move = robotos_interfaces::action::MoveJoints;
using GoalHandle = rclcpp_action::ServerGoalHandle<Move>;

namespace {
double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string uuid_string(const rclcpp_action::GoalUUID& uuid) {
  std::ostringstream stream;
  for (const auto byte : uuid) stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  return stream.str();
}
}  // namespace

class ControlNode : public rclcpp::Node {
 public:
  ControlNode() : Node("control_node"), machine_(steady_seconds()) {
    command_ = create_publisher<robotos_interfaces::msg::JointVelocityCommand>("/sim/cmd_velocity", rclcpp::QoS(1).reliable());
    status_ = create_publisher<robotos_interfaces::msg::RobotStatus>("/robot/status", rclcpp::QoS(10).reliable());
    joints_ = create_subscription<sensor_msgs::msg::JointState>(
        "/robot/joint_states", rclcpp::QoS(1).reliable(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (msg->name.size() != 2 || msg->position.size() != 2 ||
              msg->name[0] != "joint_1" || msg->name[1] != "joint_2") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid joint state shape");
            return;
          }
          machine_.on_sensor({msg->position[0], msg->position[1]}, steady_seconds());
        });
    heartbeat_ = create_subscription<robotos_interfaces::msg::SimHeartbeat>(
        "/sim/heartbeat", rclcpp::QoS(10).reliable(),
        [this](robotos_interfaces::msg::SimHeartbeat::ConstSharedPtr) {
          machine_.on_heartbeat(steady_seconds());
        });
    estop_ = create_service<robotos_interfaces::srv::SetEStop>(
        "/robot/set_estop",
        [this](const std::shared_ptr<robotos_interfaces::srv::SetEStop::Request> req,
               std::shared_ptr<robotos_interfaces::srv::SetEStop::Response> res) {
          machine_.set_estop(req->engage, steady_seconds());
          complete_action();
          res->accepted = req->engage || machine_.snapshot(steady_seconds()).mode != robotos_core::Mode::ESTOP;
          res->code = res->accepted ? "OK" : "SENSOR_UNHEALTHY";
          publish_status();
        });
    reset_ = create_service<robotos_interfaces::srv::ResetFault>(
        "/robot/reset_fault",
        [this](const std::shared_ptr<robotos_interfaces::srv::ResetFault::Request>,
               std::shared_ptr<robotos_interfaces::srv::ResetFault::Response> res) {
          const auto before = machine_.snapshot(steady_seconds());
          res->accepted = machine_.reset_fault(steady_seconds());
          res->code = res->accepted ? "OK" :
              (before.mode == robotos_core::Mode::ESTOP ? "ESTOP_ACTIVE" :
              (before.mode != robotos_core::Mode::FAULT ? "NOT_IN_FAULT" : "SENSOR_UNHEALTHY"));
          publish_status();
        });
    action_ = rclcpp_action::create_server<Move>(
        get_node_base_interface(), get_node_clock_interface(),
        get_node_logging_interface(), get_node_waitables_interface(),
        "/robot/move_joints",
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Move::Goal> goal) {
          (void)uuid;
          const auto state = machine_.snapshot(steady_seconds());
          if (state.mode != robotos_core::Mode::IDLE || !state.sensor_fresh || active_goal_ ||
              !robotos_core::SafetyStateMachine::valid_goal(
                  {goal->target[0], goal->target[1]}, goal->max_velocity))
            return rclcpp_action::GoalResponse::REJECT;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle> handle) {
          if (!active_goal_ || handle != active_goal_ ||
              machine_.snapshot(steady_seconds()).mode != robotos_core::Mode::RUNNING)
            return rclcpp_action::CancelResponse::REJECT;
          cancel_requested_ = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> handle) {
          const auto goal = handle->get_goal();
          if (!machine_.start({goal->target[0], goal->target[1]}, goal->max_velocity,
                              uuid_string(handle->get_goal_id()), steady_seconds())) {
            auto result = std::make_shared<Move::Result>();
            result->success = false;
            result->code = "GOAL_REJECTED";
            result->final_position = machine_.snapshot(steady_seconds()).position;
            handle->abort(result);
            return;
          }
          active_goal_ = handle;
          publish_status();
        });
    control_timer_ = create_wall_timer(50ms, [this]() {
      if (cancel_requested_ && active_goal_ && active_goal_->is_canceling()) {
        machine_.cancel(steady_seconds());
        cancel_requested_ = false;
      }
      const auto velocity = machine_.tick(steady_seconds());
      publish_command(velocity);
      if (active_goal_ && machine_.snapshot(steady_seconds()).mode == robotos_core::Mode::RUNNING) {
        const auto state = machine_.snapshot(steady_seconds());
        auto feedback = std::make_shared<Move::Feedback>();
        feedback->position = state.position;
        feedback->progress = state.progress;
        active_goal_->publish_feedback(feedback);
      }
      complete_action();
    });
    status_timer_ = create_wall_timer(200ms, [this]() { publish_status(); });
  }

 private:
  void publish_command(const std::array<double, 2>& velocity) {
    robotos_interfaces::msg::JointVelocityCommand msg;
    msg.header.stamp = now();
    msg.velocity = velocity;
    command_->publish(msg);
  }
  void complete_action() {
    if (!active_goal_) return;
    auto completion = machine_.take_completion();
    if (!completion) return;
    auto result = std::make_shared<Move::Result>();
    result->success = completion->terminal == robotos_core::Terminal::SUCCEEDED;
    result->code = completion->code;
    result->final_position = completion->final_position;
    if (completion->terminal == robotos_core::Terminal::SUCCEEDED) active_goal_->succeed(result);
    else if (completion->terminal == robotos_core::Terminal::CANCELED) active_goal_->canceled(result);
    else active_goal_->abort(result);
    active_goal_.reset();
    cancel_requested_ = false;
    publish_status();
  }
  void publish_status() {
    const auto state = machine_.snapshot(steady_seconds());
    robotos_interfaces::msg::RobotStatus msg;
    msg.header.stamp = now();
    msg.mode = robotos_core::SafetyStateMachine::mode_name(state.mode);
    msg.fault_code = state.fault_code;
    msg.sensor_fresh = state.sensor_fresh;
    msg.position = state.position;
    msg.target = state.target;
    msg.active_task_id = state.task_id;
    msg.progress = state.progress;
    status_->publish(msg);
  }
  robotos_core::SafetyStateMachine machine_;
  std::shared_ptr<GoalHandle> active_goal_;
  bool cancel_requested_{false};
  rclcpp::Publisher<robotos_interfaces::msg::JointVelocityCommand>::SharedPtr command_;
  rclcpp::Publisher<robotos_interfaces::msg::RobotStatus>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<robotos_interfaces::msg::SimHeartbeat>::SharedPtr heartbeat_;
  rclcpp::Service<robotos_interfaces::srv::SetEStop>::SharedPtr estop_;
  rclcpp::Service<robotos_interfaces::srv::ResetFault>::SharedPtr reset_;
  rclcpp_action::Server<Move>::SharedPtr action_;
  rclcpp::TimerBase::SharedPtr control_timer_, status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
