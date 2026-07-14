#include "scheduler.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <set>
#include <string>
#include <vector>

double EuclideanCost::compute_cost(double rx, double ry, double ryaw, double gx,
                                   double gy) const {
  return (Eigen::Vector2d(rx, ry) - Eigen::Vector2d(gx, gy)).norm();
}

const char *EuclideanCost::name() const { return "EuclideanCost"; }

double HeadingAwareCost::compute_cost(double rx, double ry, double ryaw,
                                      double gx, double gy) const {
  double dist = (Eigen::Vector2d(rx, ry) - Eigen::Vector2d(gx, gy)).norm();

  double angle_to_goal = std::atan2(gy - ry, gx - rx);

  double yaw_diff = std::abs(ryaw - angle_to_goal);

  if (yaw_diff > angle_to_goal) {
    yaw_diff = 2.0 * M_PI - yaw_diff;
  }

  return dist + yaw_diff * 1.0;
}

const char *HeadingAwareCost::name() const { return "HeadingAwareCost"; }

std::vector<Assignment>
Scheduler::assign_goals(const std::vector<AssignmentInput> &robots,
                        const std::vector<GoalInput> &goals) const {
  std::vector<Assignment> assignments;
  std::set<std::string> used_robots, used_goals;

  struct Entry {
    double cost;
    size_t ri, gi;
  };
  std::vector<Entry> entries;

  for (size_t gi = 0; gi < goals.size(); ++gi) {
    for (size_t ri = 0; ri < robots.size(); ++ri) {
      entries.push_back(
          {cost_fn_->compute_cost(robots[ri].x, robots[ri].y, robots[ri].yaw,
                                  goals[gi].x, goals[gi].y),
           ri, gi});
    }
  }
  std::sort(entries.begin(), entries.end(),
            [](const Entry &a, const Entry &b) { return a.cost < b.cost; });

  for (const auto &e : entries) {
    const auto &r = robots[e.ri].name;
    const auto &g = goals[e.gi].id;

    if (used_robots.count(r) || used_goals.count(g))
      continue;

    // skip if robot has max_goals_
    size_t count = 0;
    for (const auto &a : assignments)
      if (a.robot == r)
        count++;
    if (count >= max_goals_)
      continue;

    // make assignments
    assignments.push_back({r, g});
    used_robots.insert(r);
    used_goals.insert(g);
  }

  return assignments;
}
