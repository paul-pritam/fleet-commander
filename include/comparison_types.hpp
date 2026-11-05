#pragma once

#include <string>
#include <vector>
#include <optional>

namespace fleet_commander
{

struct Point2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double time{0.0};
};

struct MissionSummary {
  std::string name;
  std::vector<double> goal_pose; // [x, y, yaw]
  std::string status;
  double efficiency_percent{0.0};
  double duration_sec{0.0};
};

struct RunSession {
  std::string run_name;
  std::string planner_type;
  std::string file_path;
  double total_duration_sec{0.0};
  double total_distance_m{0.0};
  double optimal_distance_m{0.0};
  double path_efficiency_percent{0.0};
  double success_rate_percent{0.0};
  double max_curvature{0.0};
  double mean_curvature{0.0};
  int num_replans{0};
  double avg_velocity_mps{0.0};
  std::vector<Point2D> trajectory;
  std::vector<Point2D> planned_path;
  std::vector<MissionSummary> missions;
};

struct ComparisonReport {
  RunSession run_a;
  RunSession run_b;
  std::string winner_name;
  double efficiency_delta_percent{0.0};
  double time_savings_percent{0.0};
  double success_rate_delta_percent{0.0};
  double curvature_reduction_percent{0.0};
  int replan_reduction_count{0};
  std::string summary_verdict;
};

} // namespace fleet_commander
