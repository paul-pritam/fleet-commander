#pragma once

#include <memory>
#include <string>
#include <functional>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "state.hpp"

class RosBridge:public rclcpp::Node {
    public:
        RosBridge();
        std::mutex state_mutex;
        FleetState state;
        bool map_updated = false;

    private:
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
        void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
};