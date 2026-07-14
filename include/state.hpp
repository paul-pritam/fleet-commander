#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <uuid/uuid.h>

struct MapData {
  std::vector<uint8_t> pixels;
  uint32_t width = 0;
  uint32_t height = 0;
  double resolution = 0.0;
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();

  Eigen::Vector2i world_to_pixel(double wx, double wy) const {
    if (resolution <= 0.0 || width == 0 || height == 0)
      return {-1, -1};
    int px = static_cast<int>((wx - origin.x()) / resolution);
    int py = static_cast<int>((wy - origin.y()) / resolution);
    return {px, py};
  }

  Eigen::Vector2d pixel_to_world(int px, int py) const {
    double wx = px * resolution + origin.x();
    double wy = py * resolution + origin.y();
    return {wx, wy};
  }

  bool empty() const { return pixels.empty(); }
};

struct RobotPose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

enum class RobotStatus { Idle, Navigating, Offline };

struct RobotState {
  std::string id;
  RobotPose pose;
  RobotStatus status = RobotStatus::Idle;
  std::string current_goal_id;
  std::chrono::steady_clock::time_point last_tf_update;

  float distance_to(const Eigen::Vector2d &point) const {
    return static_cast<float>((Eigen::Vector2d(pose.x, pose.y) - point).norm());
  }

  bool is_reachable() const {
    return std::chrono::steady_clock::now() - last_tf_update <
           std::chrono::seconds(5);
  }
};

enum class GoalStatus { Pending, Active, Succeeded, Failed, Cancelled };

inline std::string generate_goal_id() {
  uuid_t uuid;
  uuid_generate(uuid);
  char buf[37];
  uuid_unparse_lower(uuid, buf);
  return std::string(buf, 8);
}

struct GoalState {
  std::string id;
  Eigen::Vector2d target = Eigen::Vector2d::Zero();
  GoalStatus status = GoalStatus::Pending;
  std::string assigned_robot;
  std::chrono::steady_clock::time_point created_at;
  std::chrono::steady_clock::time_point completed_at;

  bool is_terminal() const {
    return status == GoalStatus::Succeeded || status == GoalStatus::Failed ||
           status == GoalStatus::Cancelled;
  }

  double age_secs() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         created_at)
        .count();
  }

  static const char *status_icon(GoalStatus s) {
    switch (s) {
    case GoalStatus::Active:
      return "[>]";
    case GoalStatus::Pending:
      return "[ ]";
    case GoalStatus::Succeeded:
      return "[OK]";
    case GoalStatus::Failed:
      return "[!!]";
    case GoalStatus::Cancelled:
      return "[--]";
    }
    return "?";
  }

  static const char *status_name(GoalStatus s) {
    switch (s) {
    case GoalStatus::Active:
      return "Active";
    case GoalStatus::Pending:
      return "Pending";
    case GoalStatus::Succeeded:
      return "Succeeded";
    case GoalStatus::Failed:
      return "Failed";
    case GoalStatus::Cancelled:
      return "Cancelled";
    }
    return "?";
  }
};

struct FleetState {
  std::map<std::string, RobotState> robots;
  std::vector<GoalState> goals;
  MapData map;
  void add_goal(GoalState goal) {
    goals.push_back(
        std::move(goal)); // goal is moved instead of copy , saves some memory
  }

  std::vector<GoalState *> pending_goals() {
    std::vector<GoalState *> result;
    for (auto &g : goals) {
      if (g.status == GoalStatus::Pending) {
        result.push_back(&g);
      }
    }
    return result;
  }

  std::vector<RobotState *> idle_robots() {
    std::vector<RobotState *> result;
    for (auto &[_, r] : robots) {
      if (r.status == RobotStatus::Idle && r.is_reachable()) {
        result.push_back(&r);
      }
    }
    return result;
  }

  void cleanup_old_goals(int max_age_secs = 300) {
    goals.erase(std::remove_if(goals.begin(), goals.end(),
                               [max_age_secs](const GoalState &g) {
                                 return g.is_terminal() &&
                                        (g.age_secs() > max_age_secs);
                               }),
                goals.end());
  }
};
