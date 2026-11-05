#include "state.hpp"
#include "run_comparator.hpp"
#include <gtest/gtest.h>

// RobotStateTest

TEST(RobotStateTest, NewRobotIsIdle) {
  RobotState robot;
  EXPECT_EQ(robot.status, RobotStatus::Idle);
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

TEST(RunComparatorTest, LoadAndCompareBagsJson) {
  fleet_commander::RunSession session_a;
  fleet_commander::RunSession session_b;

  bool ok_a = fleet_commander::RunComparator::load_run_from_json(
      "/home/pritam/fleet_commander/bags/run_astar.json", session_a);
  bool ok_b = fleet_commander::RunComparator::load_run_from_json(
      "/home/pritam/fleet_commander/bags/run_hybrid_astar.json", session_b);

  EXPECT_TRUE(ok_a);
  EXPECT_TRUE(ok_b);

  EXPECT_GT(session_a.trajectory.size(), 0UL);
  EXPECT_GT(session_b.trajectory.size(), 0UL);

  auto report = fleet_commander::RunComparator::compare(session_a, session_b);

  EXPECT_EQ(report.winner_name, session_b.run_name);
  EXPECT_GT(report.time_savings_percent, 50.0);
  EXPECT_GT(report.success_rate_delta_percent, 10.0);
  EXPECT_GT(report.replan_reduction_count, 100);
}

TEST(RunComparatorTest, LoadDirectRosbagFolder) {
  fleet_commander::RunSession session_a;
  fleet_commander::RunSession session_b;

  bool ok_a = fleet_commander::RunComparator::load_run(
      "/home/pritam/fleet_commander/bags/run_astar", session_a);
  bool ok_b = fleet_commander::RunComparator::load_run(
      "/home/pritam/fleet_commander/bags/run_hybrid_astar", session_b);

  EXPECT_TRUE(ok_a);
  EXPECT_TRUE(ok_b);

  EXPECT_GT(session_a.trajectory.size(), 0UL);
  EXPECT_GT(session_b.trajectory.size(), 0UL);

  auto discovered = fleet_commander::RunComparator::discover_runs(
      "/home/pritam/fleet_commander/bags");
  EXPECT_GE(discovered.size(), 2UL);
}
