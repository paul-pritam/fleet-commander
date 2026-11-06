#include "ros_bridge.hpp"
#include "state.hpp"
#include <cmath>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/client.hpp>

RosBridge::RosBridge()
    : Node("fleet_commander",
           rclcpp::NodeOptions().parameter_overrides(
               {rclcpp::Parameter("use_sim_time", true)})) {
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

  vla_prompt_pub_ = create_publisher<std_msgs::msg::String>(
      "/fleet_commander/natural_prompt", 10);
  vla_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/bcr_bot_1/vla_target_pose", 10,
      std::bind(&RosBridge::vla_pose_callback, this, std::placeholders::_1));
  vla_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/bcr_bot_1/vla_status", 10,
      std::bind(&RosBridge::vla_status_callback, this, std::placeholders::_1));

  if (!this->has_parameter("use_sim_time")) {
    this->declare_parameter("use_sim_time", true);
  } else {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  tf_poll_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std::lock_guard<std::recursive_mutex> lock(state_mutex);
        for (auto &[ns, robot] : state.robots) {
          try {
            auto t = tf_buffer_->lookupTransform("map", ns + "/base_footprint",
                                                 tf2::TimePointZero);

            double raw_x = t.transform.translation.x;
            double raw_y = t.transform.translation.y;

            tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y,
                              t.transform.rotation.z, t.transform.rotation.w);
            double roll, pitch, raw_yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, raw_yaw);

            if (!robot.pose_initialized) {
              robot.pose.x = raw_x;
              robot.pose.y = raw_y;
              robot.pose.yaw = raw_yaw;
              robot.pose_initialized = true;
            } else {
              const double alpha = 0.35;
              robot.pose.x += alpha * (raw_x - robot.pose.x);
              robot.pose.y += alpha * (raw_y - robot.pose.y);
              double dyaw = std::atan2(std::sin(raw_yaw - robot.pose.yaw),
                                       std::cos(raw_yaw - robot.pose.yaw));
              robot.pose.yaw += alpha * dyaw;
            }

            robot.last_tf_update = std::chrono::steady_clock::now();
            robot.last_tf_update_sec = this->now().seconds();
            if (robot.status == RobotStatus::Offline) {
              robot.status = RobotStatus::Idle;
            }
          } catch (const tf2::TransformException &ex) {
            static std::map<std::string, std::string> last_err;
            if (last_err[ns] != ex.what()) {
              last_err[ns] = ex.what();
              RCLCPP_WARN(get_logger(), "TF lookup for %s failed: %s", ns.c_str(), ex.what());
            }
            continue;
          }
        }

        static auto last_summary_time = std::chrono::steady_clock::now();
        auto now_steady = std::chrono::steady_clock::now();
        if (now_steady - last_summary_time > std::chrono::seconds(2)) {
          last_summary_time = now_steady;
          for (const auto &[ns, robot] : state.robots) {
            auto staleness_s = std::chrono::duration_cast<std::chrono::seconds>(
                now_steady - robot.last_tf_update).count();
            RCLCPP_INFO(get_logger(), "ROBOT [%s]: staleness=%lds, pose=(%.2f, %.2f, %.2f), status=%s",
                        ns.c_str(), staleness_s, robot.pose.x, robot.pose.y, robot.pose.yaw,
                        robot.status == RobotStatus::Idle ? "Idle" :
                        robot.status == RobotStatus::Navigating ? "Nav" : "Offline");
          }
        }
      });

  RCLCPP_INFO(get_logger(), "Fleet commander RosBridge init");
}

void RosBridge::try_subscribe_map() {
  std::lock_guard<std::recursive_mutex> lock(state_mutex);
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
  std::lock_guard<std::recursive_mutex> lock(state_mutex);

  state.map = std::move(m);
  map_updated = true;
  RCLCPP_INFO(get_logger(), "Map received: %dx%d", state.map.width,
              state.map.height);
}

void RosBridge::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {

  std::lock_guard<std::recursive_mutex> lock(state_mutex);

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

  std::lock_guard<std::recursive_mutex> lock(state_mutex);

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
  robot.last_tf_update = std::chrono::steady_clock::now();

  if (robot.status == RobotStatus::Offline) {
    robot.status = RobotStatus::Idle;
  }
}

void RosBridge::discover_robots() {

  std::lock_guard<std::recursive_mutex> lock(state_mutex);

  for (const auto &[ns, robot] : state.robots) {

    if (action_clients_.count(ns))
      continue;

    std::string action_name = "/" + ns + "/navigate_to_pose";

    auto client =
        rclcpp_action::create_client<NavigateToPose>(this, action_name);
    action_clients_[ns] = client;

    RCLCPP_INFO(get_logger(),
                "Created an action client for %s (action: %s, ready: %s)",
                ns.c_str(), action_name.c_str(),
                client->action_server_is_ready() ? "yes" : "no");
  }

  auto topics = get_topic_names_and_types();

  for (const auto &[topic_name, types] : topics) {
    std::string ns;

    if ((topic_name.ends_with("/cmd_vel") && topic_name.size() > strlen("/cmd_vel")) ||
        (topic_name.ends_with("/scan") && topic_name.size() > strlen("/scan")) ||
        (topic_name.ends_with("/wheel_odom") && topic_name.size() > strlen("/wheel_odom"))) {
      size_t slash_pos = topic_name.rfind('/');
      if (slash_pos > 1) {
        ns = topic_name.substr(1, slash_pos - 1);
      }
    } else if (topic_name.ends_with("/navigate_to_pose/_action/send_goal")) {
      std::string action_topic =
          topic_name.substr(0, topic_name.size() - strlen("/_action/send_goal"));
      size_t nav_pos = action_topic.rfind("/navigate_to_pose");
      if (nav_pos != std::string::npos && nav_pos > 0) {
        ns = action_topic.substr(1, nav_pos - 1);
      }
    } else {
      continue;
    }

    if (ns.empty() || ns.find('/') != std::string::npos)
      continue;

    std::string action_name = "/" + ns + "/navigate_to_pose";

    if (!action_clients_.count(ns)) {
      auto client =
          rclcpp_action::create_client<NavigateToPose>(this, action_name);
      action_clients_[ns] = client;
    }

    if (!state.robots.count(ns)) {
      auto &r = state.robots[ns];
      r.id = ns;
      r.status = RobotStatus::Offline;
      r.last_tf_update = std::chrono::steady_clock::now();

      RCLCPP_INFO(get_logger(),
                  "Discovered robot %s (action: %s, ready: %s)",
                  ns.c_str(), action_name.c_str(),
                  action_clients_[ns]->action_server_is_ready() ? "yes" : "no");
    } else {
      auto &r = state.robots[ns];
      if (action_clients_[ns] && action_clients_[ns]->action_server_is_ready()) {
        r.last_tf_update = std::chrono::steady_clock::now();
        if (r.status == RobotStatus::Offline && r.pose_initialized) {
          r.status = RobotStatus::Idle;
        }
      }
    }
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

bool RosBridge::send_goal(const std::string &robot_id,
                          const std::string &goal_id, double x, double y,
                          double yaw) {

  auto it = action_clients_.find(robot_id);

  if (it == action_clients_.end()) {
    RCLCPP_WARN(get_logger(), "No action client for robot %s",
                robot_id.c_str());
    return false;
  }

  auto client = it->second;

  if (!client->action_server_is_ready()) {
    RCLCPP_INFO(get_logger(),
                "Action server for %s not immediately ready, waiting up to 1s...",
                robot_id.c_str());
    if (!client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(),
                  "Action server for %s is not ready, cannot send goal",
                  robot_id.c_str());
      return false;
    }
  }

  auto goal = NavigateToPose::Goal();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now(); // rclcpp::Node::now()
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = std::sin(yaw * 0.5);
  goal.pose.pose.orientation.w = std::cos(yaw * 0.5);

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  opts.goal_response_callback =
      [this, robot_id, goal_id](const GoalHandleNav::SharedPtr &goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(),
                       "Goal %s rejected by %s! Rolling back to Pending.",
                       goal_id.c_str(), robot_id.c_str());
          std::lock_guard<std::recursive_mutex> lock(state_mutex);
          for (auto &g : state.goals) {
            if (g.id == goal_id) {
              g.status = GoalStatus::Pending;
              break;
            }
          }
          for (auto &[_, r] : state.robots) {
            if (r.id == robot_id) {
              r.status = RobotStatus::Idle;
              r.current_goal_id.clear();
              break;
            }
          }
        }
      };

  opts.result_callback = [this, robot_id,
                          goal_id](const GoalHandleNav::WrappedResult &result) {
    bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    if (on_goal_result)
      on_goal_result(goal_id, robot_id, success);
  };
  client->async_send_goal(goal, opts);
  return true;
}

void RosBridge::publish_vla_prompt(const std::string &prompt) {
  if (vla_prompt_pub_) {
    std_msgs::msg::String msg;
    msg.data = prompt;
    vla_prompt_pub_->publish(msg);
    std::lock_guard<std::recursive_mutex> lock(state_mutex);
    state.vla_target.prompt = prompt;
  }
}

void RosBridge::vla_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lock(state_mutex);
  state.vla_target.active = true;
  state.vla_target.goal_pos.x() = msg->pose.position.x;
  state.vla_target.goal_pos.y() = msg->pose.position.y;

  double qz = msg->pose.orientation.z;
  double qw = msg->pose.orientation.w;
  state.vla_target.goal_yaw = 2.0 * std::atan2(qz, qw);
  state.vla_target.last_update = std::chrono::steady_clock::now();
}

void RosBridge::vla_status_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::recursive_mutex> lock(state_mutex);
  try {
    const std::string &s = msg->data;
    auto find_val = [&](const std::string &key) -> std::string {
      std::string pattern = "\"" + key + "\"";
      size_t pos = s.find(pattern);
      if (pos == std::string::npos)
        return "";
      pos += pattern.length();
      while (pos < s.size() && (s[pos] == ' ' || s[pos] == ':' || s[pos] == '\t'))
        pos++;
      if (pos < s.size() && s[pos] == '\"')
        pos++;
      size_t end = pos;
      while (end < s.size() && s[end] != ',' && s[end] != '}' &&
             s[end] != '\"' && s[end] != '\n')
        end++;
      return s.substr(pos, end - pos);
    };

    std::string cls = find_val("detected_class");
    if (!cls.empty())
      state.vla_target.detected_class = cls;
    std::string conf_str = find_val("confidence");
    if (!conf_str.empty())
      state.vla_target.confidence = std::stof(conf_str);
    std::string tx_str = find_val("target_x");
    std::string ty_str = find_val("target_y");
    if (!tx_str.empty() && !ty_str.empty()) {
      state.vla_target.target_pos.x() = std::stod(tx_str);
      state.vla_target.target_pos.y() = std::stod(ty_str);
    }
  } catch (...) {
  }
}

bool RosBridge::dispatch_vla_goal() {
  std::lock_guard<std::recursive_mutex> lock(state_mutex);
  if (!state.vla_target.is_fresh()) {
    RCLCPP_WARN(get_logger(), "No fresh VLA target to dispatch");
    return false;
  }
  std::string robot_id = "bcr_bot_1";
  if (!state.robots.count(robot_id)) {
    if (!state.robots.empty())
      robot_id = state.robots.begin()->first;
    else {
      RCLCPP_WARN(get_logger(), "No robot registered in state to dispatch VLA goal");
      return false;
    }
  }

  std::string goal_id = generate_goal_id();
  GoalState g;
  g.id = goal_id;
  g.target = state.vla_target.goal_pos;
  g.status = GoalStatus::Active;
  g.assigned_robot = robot_id;
  g.created_at = std::chrono::steady_clock::now();
  state.add_goal(g);

  RCLCPP_INFO(get_logger(), "Dispatching VLA action to %s: Goal (%s) at (%.2f, %.2f, yaw=%.2f)",
              robot_id.c_str(), goal_id.c_str(), state.vla_target.goal_pos.x(),
              state.vla_target.goal_pos.y(), state.vla_target.goal_yaw);

  return send_goal(robot_id, goal_id, state.vla_target.goal_pos.x(),
                   state.vla_target.goal_pos.y(), state.vla_target.goal_yaw);
}

