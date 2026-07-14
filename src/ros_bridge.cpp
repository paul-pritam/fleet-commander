#include "ros_bridge.hpp"
#include <cmath>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/client.hpp>

RosBridge::RosBridge() : Node("fleet_commander") {
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      std::bind(&RosBridge::map_callback, this, std::placeholders::_1));
  tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100),
      std::bind(&RosBridge::tf_callback, this, std::placeholders::_1));

  map_retry_timer_ = create_wall_timer(
      std::chrono::seconds(2), std::bind(&RosBridge::try_subscribe_map, this));

  discovery_timer_ = create_wall_timer(
      std::chrono::seconds(2), std::bind(&RosBridge::discover_robots, this));

  RCLCPP_INFO(get_logger(), "Fleet commander RosBridge init");
}

void RosBridge::try_subscribe_map() {
  std::lock_guard<std::mutex> lock(state_mutex);
  if (!state.map.empty()) {
    map_retry_timer_->cancel();
    return;
  }

  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      std::bind(&RosBridge::map_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Retrying /map subscription...");
}
void RosBridge::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

  MapData m;

  m.height = msg->info.height;
  m.width = msg->info.width;
  m.resolution = msg->info.resolution;
  m.origin =
      Eigen::Vector2d(msg->info.origin.position.x, msg->info.origin.position.y);
  m.pixels.resize(msg->data.size());

  for (size_t i = 0; i < msg->data.size(); ++i) {
    int8_t cell = msg->data[i];
    m.pixels[i] =
        (cell == -1) ? 128 : static_cast<uint8_t>(255.0 * (1 - cell / 100.0));
  }
  std::lock_guard<std::mutex> lock(state_mutex);

  state.map = std::move(m);
  map_updated = true;
  RCLCPP_INFO(get_logger(), "Map received: %dx%d", state.map.width,
              state.map.height);
}

void RosBridge::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {

  std::lock_guard<std::mutex> lock(state_mutex);

  for (const auto &t : msg->transforms) {
    const auto &parent = t.header.frame_id;
    const auto &child = t.child_frame_id;

    double x = t.transform.translation.x;
    double y = t.transform.translation.y;

    Eigen::Quaterniond q(t.transform.rotation.w, t.transform.rotation.x,
                         t.transform.rotation.y, t.transform.rotation.z);

    double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    if (parent == "map" && child.find("/odom") != std::string::npos) {
      std::string ns = child.substr(0, child.find("/odom"));
      if (ns.empty())
        continue;
      odom_offsets_[ns] = {x, y, yaw, true};
      update_robot_pose(ns);
    } else if (child.find("/base_footprint") != std::string::npos &&
               parent.find("/odom") != std::string::npos) {
      std::string ns = child.substr(0, child.find("/base_footprint"));
      if (ns.empty())
        continue;
      odom_base_transforms_[ns] = {x, y, yaw, true};
      update_robot_pose(ns);
    }
  }
}

void RosBridge::tf_callback_for_robot(
    const std::string &robot_ns,
    const tf2_msgs::msg::TFMessage::SharedPtr msg) {

  std::lock_guard<std::mutex> lock(state_mutex);

  for (const auto &t : msg->transforms) {

    const auto &parent = t.header.frame_id;
    const auto &child = t.child_frame_id;

    if ((child == "base_footprint" || child == "base_link") &&
        parent == "odom") {
      double x = t.transform.translation.x;
      double y = t.transform.translation.y;

      Eigen::Quaterniond q(t.transform.rotation.w, t.transform.rotation.x,
                           t.transform.rotation.y, t.transform.rotation.z);

      double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

      odom_base_transforms_[robot_ns] = {x, y, yaw, true};
      update_robot_pose(robot_ns);
    }
  }
}

void RosBridge::update_robot_pose(const std::string &ns) {

  auto &robot = state.robots[ns];
  robot.id = ns;

  bool has_map_odom = odom_offsets_.count(ns) && odom_offsets_[ns].valid;
  bool has_odom_base =
      odom_base_transforms_.count(ns) && odom_base_transforms_[ns].valid;

  if (has_map_odom && has_odom_base) {

    const auto &mo = odom_offsets_[ns];
    const auto &ob = odom_base_transforms_[ns];

    double cos_yaw = std::cos(mo.yaw);
    double sin_yaw = std::sin(mo.yaw);

    double map_x = mo.x + ob.x * cos_yaw - ob.y * sin_yaw;
    double map_y = mo.y + ob.x * sin_yaw + ob.y * cos_yaw;
    double map_yaw = mo.yaw + ob.yaw;

    robot.pose = RobotPose{map_x, map_y, map_yaw};
  } else if (has_odom_base) {
    const auto &ob = odom_base_transforms_[ns];
    robot.pose = RobotPose{ob.x, ob.y, ob.yaw};
  }

  robot.last_tf_update = std::chrono::steady_clock::now();

  if (robot.status == RobotStatus::Offline) {
    robot.status = RobotStatus::Idle;
  }
}

void RosBridge::discover_robots() {

  std::lock_guard<std::mutex> lock(state_mutex);

  for (const auto &[ns, robot] : state.robots) {

    if (action_clients_.count(ns))
      continue;

    std::string action_name = "/" + ns + "/navigate_to_pose";

    auto client =
        rclcpp_action::create_client<NavigateToPose>(this, action_name);
    action_clients_[ns] = client;

    RCLCPP_INFO(get_logger(),
                "Created aan action client for %s (action: %s, ready: %s)",
                ns.c_str(), action_name.c_str(),
                client->action_server_is_ready() ? "yes" : "no");

    subscribe_robot_tf(ns);
  }

  auto topics = get_topic_names_and_types();

  for (const auto &[topic_name, types] : topics) {
    if (!topic_name.ends_with("/navigate_to_pose/_action/send_goal"))
      continue;

    std::string ns;
    std::string action_topic =
        topic_name.substr(0, topic_name.size() - strlen("/_action/send_goal"));
    size_t nav_pos = action_topic.rfind("/navigate_to_pose");

    if (nav_pos != std::string::npos && nav_pos > 0) {
      ns = action_topic.substr(1, nav_pos - 1);
    } else {
      continue;
    }

    if (ns.empty() || action_clients_.count(ns))
      continue;

    auto client =
        rclcpp_action::create_client<NavigateToPose>(this, action_topic);
    action_clients_[ns] = client;
    RCLCPP_INFO(get_logger(),
                "Discovered robot via topic: %s (action: %s, ready: %s)",
                ns.c_str(), action_topic.c_str(),
                client->action_server_is_ready() ? "yes" : "no");
    subscribe_robot_tf(ns);
  }
}

void RosBridge::subscribe_robot_tf(const std::string &robot_ns) {
  if (robot_tf_subs_.count(robot_ns))
    return;

  std::string tf_topic = "/" + robot_ns + "/tf";
  auto sub = create_subscription<tf2_msgs::msg::TFMessage>(
      tf_topic, rclcpp::QoS(300),
      [this, robot_ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        tf_callback_for_robot(robot_ns, msg);
      });
  robot_tf_subs_[robot_ns] = sub;
  RCLCPP_INFO(get_logger(), "Subscribed to Tf for robot: %s", robot_ns.c_str());
}

void RosBridge::send_goal(const std::string &robot_id,
                          const std::string &goal_id, double x, double y) {

  auto it = action_clients_.find(robot_id);

  if (it == action_clients_.end()) {
    RCLCPP_WARN(get_logger(), "No action client for robot %s",
                robot_id.c_str());
    return;
  }

  auto client = it->second;

  if (!client->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(),
                "Action server for %s is not ready , retrying.........",
                robot_id.c_str());
    return;
  }

  auto goal = NavigateToPose::Goal();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now(); // rclcpp::Node::now()
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.w = 1.0;

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  opts.result_callback = [this, robot_id,
                          goal_id](const GoalHandleNav::WrappedResult &result) {
    bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    if (on_goal_result)
      on_goal_result(goal_id, robot_id, success);
  };
  client->async_send_goal(goal, opts);
}
