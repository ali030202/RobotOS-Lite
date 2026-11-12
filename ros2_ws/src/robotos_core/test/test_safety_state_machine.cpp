#include <cmath>
#include <limits>
#include "gtest/gtest.h"
#include "robotos_core/safety_state_machine.hpp"

using robotos_core::Mode;
using robotos_core::SafetyStateMachine;
using robotos_core::Terminal;

static void ready(SafetyStateMachine& machine) {
  machine.on_sensor({0.0, 0.0}, 0.1);
  machine.on_sensor({0.0, 0.0}, 0.2);
  machine.on_heartbeat(0.3);
  machine.on_sensor({0.0, 0.0}, 0.3);
  ASSERT_EQ(machine.snapshot(0.3).mode, Mode::IDLE);
}

TEST(Safety, RejectsInvalidAndConcurrentGoals) {
  SafetyStateMachine machine(0.0);
  ready(machine);
  EXPECT_FALSE(SafetyStateMachine::valid_goal({2.0, 0.0}, 0.2));
  EXPECT_FALSE(SafetyStateMachine::valid_goal({NAN, 0.0}, 0.2));
  EXPECT_FALSE(SafetyStateMachine::valid_goal({0.0, 0.0}, 0.0));
  EXPECT_TRUE(machine.start({0.5, -0.5}, 0.3, "a", 0.4));
  EXPECT_FALSE(machine.start({0.2, 0.2}, 0.3, "b", 0.4));
  const auto command = machine.tick(0.45);
  EXPECT_GT(command[0], 0.0);
  EXPECT_LT(command[1], 0.0);
  EXPECT_LE(std::abs(command[0]), 0.3);
}

TEST(Safety, CompletesOnlyAfterThreeSettledTicks) {
  SafetyStateMachine machine(0.0);
  ready(machine);
  ASSERT_TRUE(machine.start({0.5, -0.5}, 0.3, "a", 0.4));
  machine.on_sensor({0.5, -0.5}, 0.5);
  machine.tick(0.51);
  machine.tick(0.56);
  EXPECT_EQ(machine.snapshot(0.56).mode, Mode::RUNNING);
  EXPECT_FALSE(machine.take_completion().has_value());
  EXPECT_EQ(machine.tick(0.61)[0], 0.0);
  auto result = machine.take_completion();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->terminal, Terminal::SUCCEEDED);
  EXPECT_EQ(machine.snapshot(0.61).mode, Mode::IDLE);
  EXPECT_FALSE(machine.take_completion().has_value());
}

TEST(Safety, SensorLossLatchesFaultUntilManualReset) {
  SafetyStateMachine machine(0.0);
  ready(machine);
  ASSERT_TRUE(machine.start({1.0, 0.0}, 0.3, "a", 0.4));
  EXPECT_EQ(machine.tick(0.45)[0], 0.3);
  EXPECT_EQ(machine.tick(0.8)[0], 0.0);
  EXPECT_EQ(machine.snapshot(0.8).mode, Mode::RUNNING);
  EXPECT_EQ(machine.tick(1.31)[0], 0.0);
  EXPECT_EQ(machine.snapshot(1.31).mode, Mode::FAULT);
  EXPECT_EQ(machine.take_completion()->code, "SENSOR_TIMEOUT");
  EXPECT_FALSE(machine.start({0.0, 0.0}, 0.2, "b", 1.31));
  machine.on_sensor({0.0, 0.0}, 1.4);
  EXPECT_FALSE(machine.reset_fault(1.4));
  machine.on_sensor({0.0, 0.0}, 1.5);
  EXPECT_TRUE(machine.reset_fault(1.5));
  EXPECT_EQ(machine.snapshot(1.5).mode, Mode::IDLE);
}

TEST(Safety, EStopAndCancelStopMotion) {
  SafetyStateMachine machine(0.0);
  ready(machine);
  ASSERT_TRUE(machine.start({1.0, 0.0}, 0.3, "a", 0.4));
  machine.on_sensor({0.25, 0.0}, 0.45);
  EXPECT_TRUE(machine.cancel(0.5));
  EXPECT_NEAR(machine.snapshot(0.5).progress, 0.25f, 0.001f);
  EXPECT_EQ(machine.take_completion()->terminal, Terminal::CANCELED);
  EXPECT_EQ(machine.tick(0.5)[0], 0.0);
  ASSERT_TRUE(machine.start({1.0, 0.0}, 0.3, "b", 0.6));
  machine.set_estop(true, 0.7);
  EXPECT_EQ(machine.tick(0.7)[0], 0.0);
  EXPECT_EQ(machine.take_completion()->code, "ESTOP");
  EXPECT_FALSE(machine.start({0.0, 0.0}, 0.2, "c", 0.8));
  machine.set_estop(false, 0.8);
  EXPECT_EQ(machine.snapshot(0.8).mode, Mode::IDLE);
}

TEST(Safety, MotionTimeoutAborts) {
  SafetyStateMachine machine(0.0);
  ready(machine);
  ASSERT_TRUE(machine.start({1.0, 0.0}, 0.3, "a", 0.4));
  machine.on_sensor({0.0, 0.0}, 10.5);
  machine.on_heartbeat(10.5);
  EXPECT_EQ(machine.tick(10.5)[0], 0.0);
  EXPECT_EQ(machine.take_completion()->code, "MOTION_TIMEOUT");
}
