#include "ros_bridge.hpp"
#include <cmath>

RosBridge::RosBridge() : Node("fleet_commander"){
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&RosBridge::map_callback, this, std::placeholders::_1));
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf",  rclcpp::QoS(100), std::bind(&RosBridge::tf_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Fleet commander RosBridge init");
}
void RosBridge::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    
    MapData m;

    m.height = msg->info.height;
    m.width = msg->info.width;
    m.resolution = msg->info.resolution;
    m.origin = Eigen::Vector2d(msg->info.origin.position.x, msg->info.origin.position.y);
    m.pixels.resize(msg->data.size());

    for (size_t i = 0; i < msg->data.size(); ++i){
        int8_t cell = msg->data[i];
        m.pixels[i] = (cell == -1) ? 128 : static_cast<uint8_t>(255.0 * (1 - cell / 100));
    }
    std::lock_guard<std::mutex> lock(state_mutex);

    state.map = std::move(m);
    map_updated = true;
}

void RosBridge::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){

    std::lock_guard<std::mutex> lock(state_mutex);

    for (const auto &t : msg->transforms){
        const auto &parent = t.header.frame_id;
        const auto &child = t.child_frame_id;

        if (child.find("base_footprint") != std::string::npos && parent.find("/odom") != std::string::npos){

            std::string ns = child.substr(0, child.find("/base_footprint"));

            if (ns.empty()) continue;

            double x = t.transform.translation.x;
            double y = t.transform.translation.y;

            double qx = t.transform.rotation.x;
            double qy = t.transform.rotation.y;
            double qz = t.transform.rotation.z;
            double qw = t.transform.rotation.w;

            double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

            state.robots[ns] = RobotState{
                ns,
                RobotPose{x, y, yaw},
                RobotStatus::Idle,
                "",
                std::chrono::steady_clock::now()
            };

        }

    }

}
