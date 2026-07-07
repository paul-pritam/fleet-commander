#include <cstdio>

#include <rclcpp/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros_bridge.hpp"
#include "app.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    App app;
    if (!app.init(800, 600, "Fleet Commander")) {
        return 1;
    }
    app.run();
    app.shutdown();

    rclcpp::shutdown();
    return 0;
}
