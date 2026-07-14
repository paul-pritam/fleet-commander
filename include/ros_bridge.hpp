#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "state.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// stores map->odom for TF
struct OdomOffset {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool valid = false;
};

class RosBridge : public rclcpp::Node {
public:
  RosBridge();
  std::mutex state_mutex;
  FleetState state;
  bool map_updated = false;

  std::function<void(const std::string &goal_id, const std::string &robot_id,
                     bool success)>
      on_goal_result;

  void send_goal(const std::string &robot_id, const std::string &goal_id,
                 double x, double y);

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  void tf_callback_for_robot(const std::string &robot_ns,
                             const tf2_msgs::msg::TFMessage::SharedPtr msg);

  void discover_robots();
  void subscribe_robot_tf(const std::string &robot_ns);
  void update_robot_pose(const std::string &ns);

  void try_subscribe_map();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::TimerBase::SharedPtr map_retry_timer_;

  std::map<std::string, rclcpp_action::Client<NavigateToPose>::SharedPtr>
      action_clients_;
  std::map<std::string,
           rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr>
      robot_tf_subs_;
  std::map<std::string, OdomOffset> odom_offsets_;
  std::map<std::string, OdomOffset> odom_base_transforms_;

  rclcpp::TimerBase::SharedPtr discovery_timer_;
};
