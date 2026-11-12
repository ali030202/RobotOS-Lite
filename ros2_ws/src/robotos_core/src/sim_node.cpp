#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robotos_interfaces/msg/joint_velocity_command.hpp"
#include "robotos_interfaces/msg/sim_heartbeat.hpp"
#include "robotos_interfaces/srv/set_sensor_dropout.hpp"

using namespace std::chrono_literals;

class SimNode : public rclcpp::Node {
 public:
  SimNode() : Node("sim_node") {
    injection_enabled_ = declare_parameter<bool>("enable_fault_injection", false);
    joints_ = create_publisher<sensor_msgs::msg::JointState>("/robot/joint_states", rclcpp::QoS(1).reliable());
    heartbeat_ = create_publisher<robotos_interfaces::msg::SimHeartbeat>("/sim/heartbeat", rclcpp::QoS(10).reliable());
    command_ = create_subscription<robotos_interfaces::msg::JointVelocityCommand>(
        "/sim/cmd_velocity", rclcpp::QoS(1).reliable(),
        [this](robotos_interfaces::msg::JointVelocityCommand::ConstSharedPtr msg) {
          if (!std::isfinite(msg->velocity[0]) || !std::isfinite(msg->velocity[1])) return;
          velocity_[0] = std::clamp(msg->velocity[0], -0.5, 0.5);
          velocity_[1] = std::clamp(msg->velocity[1], -0.5, 0.5);
          last_command_ = std::chrono::steady_clock::now();
        });
    if (injection_enabled_) {
      dropout_service_ = create_service<robotos_interfaces::srv::SetSensorDropout>(
          "/sim/set_sensor_dropout",
          [this](const std::shared_ptr<robotos_interfaces::srv::SetSensorDropout::Request> req,
                 std::shared_ptr<robotos_interfaces::srv::SetSensorDropout::Response> res) {
            dropout_ = req->enabled;
            res->accepted = true;
            res->code = "OK";
            RCLCPP_WARN(get_logger(), "sensor dropout %s", dropout_ ? "on" : "off");
          });
    }
    integrate_timer_ = create_wall_timer(20ms, [this]() { integrate(); });
    sensor_timer_ = create_wall_timer(100ms, [this]() { publish_sensor(); });
    heartbeat_timer_ = create_wall_timer(1s, [this]() { publish_heartbeat(); });
  }

 private:
  void integrate() {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::clamp(std::chrono::duration<double>(now - last_integration_).count(), 0.0, 0.05);
    last_integration_ = now;
    if (now - last_command_ > 250ms) velocity_ = {0.0, 0.0};
    for (int i = 0; i < 2; ++i) position_[i] = std::clamp(position_[i] + velocity_[i] * dt, -1.57, 1.57);
  }
  void publish_sensor() {
    if (dropout_) return;
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {"joint_1", "joint_2"};
    msg.position = {position_[0], position_[1]};
    msg.velocity = {velocity_[0], velocity_[1]};
    joints_->publish(msg);
  }
  void publish_heartbeat() {
    robotos_interfaces::msg::SimHeartbeat msg;
    msg.header.stamp = now();
    msg.sequence = ++sequence_;
    heartbeat_->publish(msg);
  }
  bool injection_enabled_{false};
  bool dropout_{false};
  uint32_t sequence_{0};
  std::array<double, 2> position_{0.0, 0.0};
  std::array<double, 2> velocity_{0.0, 0.0};
  std::chrono::steady_clock::time_point last_command_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_integration_{std::chrono::steady_clock::now()};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<robotos_interfaces::msg::SimHeartbeat>::SharedPtr heartbeat_;
  rclcpp::Subscription<robotos_interfaces::msg::JointVelocityCommand>::SharedPtr command_;
  rclcpp::Service<robotos_interfaces::srv::SetSensorDropout>::SharedPtr dropout_service_;
  rclcpp::TimerBase::SharedPtr integrate_timer_, sensor_timer_, heartbeat_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
