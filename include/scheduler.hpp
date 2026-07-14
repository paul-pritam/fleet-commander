#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

struct AssignmentInput {
  std::string name;
  double x, y, yaw;
};

struct GoalInput {
  std::string id;
  double x, y;
};

struct Assignment {
  std::string robot;
  std::string goal;
};

class CostFunction {
public:
  virtual ~CostFunction() = default;
  virtual double compute_cost(double rx, double ry, double ryaw, double gx,
                              double gy) const = 0;
  virtual const char *name() const = 0; // returns the name of the cost function
};

class EuclideanCost : public CostFunction {
public:
  double compute_cost(double rx, double ry, double ryaw, double gx,
                      double gy) const override;

  const char *name() const override;
};

class HeadingAwareCost : public CostFunction {
public:
  double compute_cost(double rx, double ry, double ryaw, double gx,
                      double gy) const override;
  const char *name() const override;
};
class Scheduler {
public:
  explicit Scheduler(std::unique_ptr<CostFunction> cost_fn,
                     size_t max_goals = 1)
      : cost_fn_(std::move(cost_fn)), max_goals_(max_goals) {}

  std::vector<Assignment>
  assign_goals(const std::vector<AssignmentInput> &robots,
               const std::vector<GoalInput> &goals) const;

private:
  std::unique_ptr<CostFunction> cost_fn_;
  size_t max_goals_;
};
