#include "state.hpp"
#include <gtest/gtest.h>

// RobotStateTest

TEST(RobotStateTest, NewRobotIsIdle) {
  RobotState robot;
  EXPECT_EQ(robot.status, RobotStatus::Idle); // excpects lhs = rhs
}

TEST(RobotStateTest, NewRobotHasZeroPose) {
  RobotState robot;
  EXPECT_DOUBLE_EQ(robot.pose.x, 0.0);
  EXPECT_DOUBLE_EQ(robot.pose.y, 0.0);
  EXPECT_DOUBLE_EQ(robot.pose.yaw, 0.0);
}

TEST(RobotStateTest, NewRobotHasN0Goal) {
  RobotState robot;
  EXPECT_TRUE(robot.current_goal_id.empty());
}

TEST(GoalStateTest, NewGoalIsPending) {
  GoalState goal;
  EXPECT_EQ(goal.status, GoalStatus::Pending);
}
