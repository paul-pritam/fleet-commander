# Fleet Commander

A C++20 multi-robot fleet management application and visual console for autonomous mobile robots (AMRs) running ROS 2 and Nav2.

Fleet Commander provides real-time occupancy grid visualization, dynamic multi-robot pose tracking across ROS namespaces, asynchronous Nav2 action dispatching, and cost-based task scheduling. The user interface is rendered using Dear ImGui and hardware-accelerated OpenGL 4.6.

## Why This Project Exists

RViz2 is an essential developer tool for inspecting TF trees and sensor topics, but it is resource-intensive and awkward to deploy as an operator console on a warehouse tablet or shop-floor terminal.

Fleet Commander was built from scratch in C++20 to provide a dedicated, lightweight fleet dispatch station. It initializes a native OpenGL 4.6 context, boots in milliseconds, and decouples the UI rendering loop from ROS 2 DDS network traffic using a mutex-guarded state machine.

### Design Trade-offs & Engineering Decisions

- **Dear ImGui over Qt**: Avoids heavy moc preprocessing and bulky runtime libraries, producing a lean static binary that runs at a solid 60 FPS even on low-power companion screens.
- **GL_NEAREST Texture Filtering**: Standard linear interpolation smooths out cell boundaries, turning crisp occupancy grids into blurry gradients. Nearest-neighbor sampling preserves exact costmap cell boundaries.
- **Heading-Aware Cost Scheduling**: Standard Euclidean distance assigns whichever robot is geographically closest, even if it is pointed in the opposite direction. On differential-drive robots, large in-place turns take time and scrub wheels. `HeadingAwareCost` accounts for initial heading alignment to reduce total mission duration.

## Architecture

Fleet Commander decouples ROS 2 asynchronous communication from the OpenGL rendering loop through a shared, mutex-protected state machine.

```
+-------------------------------------------------------------------+
|                        Fleet Commander GUI                        |
|             (GLFW 3.4 / OpenGL 4.6 / Dear ImGui v1.91.8)          |
+---------------------------------+---------------------------------+
                                  |
                   std::lock_guard<std::mutex>
                                  |
+---------------------------------v---------------------------------+
|                           FleetState                              |
|   - MapData (texture pixels, resolution, world origin)            |
|   - std::map<std::string, RobotState>                             |
|   - std::vector<GoalState>                                        |
+---------------------------------^---------------------------------+
                                  |
                   std::lock_guard<std::mutex>
                                  |
+---------------------------------+---------------------------------+
|                            RosBridge                              |
|                     (ROS 2 Jazzy rclcpp Node)                     |
+-------------------+-----------------------+-----------------------+
|  /map Subscriber  |    /tf Subscribers    |  Nav2 Action Clients  |
|  OccupancyGrid    |  Robot Pose Tracking  |   NavigateToPose      |
+-------------------+-----------------------+-----------------------+
```

### Core Components

- **OpenGL Occupancy Grid Ingestion (`App::update_map_texture`)**: Ingests `nav_msgs/msg/OccupancyGrid` messages and uploads rasterized costmap bytes directly into an OpenGL 2D texture. Uses `GL_NEAREST` texture filtering to preserve exact cell borders without interpolation blur. Provides bi-directional mapping between continuous world coordinates (meters) and discrete map pixel indices via `world_to_pixel` and `pixel_to_world`.
- **Multi-Robot Transform Tracking (`RosBridge::update_robot_pose`)**: Discovers robots under dynamic namespaces (such as `/bcr_bot_1`, `/bcr_bot_2`). Reconstructs each robot's world pose by composing `map -> odom` and `odom -> base_link` transform frames received over `/tf`. Tracks position (x, y), planar heading (yaw), and update recency to detect stale or disconnected robots via `is_reachable()`.
- **Asynchronous Nav2 Dispatch (`RosBridge::send_goal`)**: Maintains an active `rclcpp_action::Client<nav2_msgs::action::NavigateToPose>` client for every discovered robot. Dispatches goals asynchronously with non-blocking feedback and result callbacks, ensuring the UI rendering thread maintains 60 FPS without executor stutter.
- **Task Scheduling (`Scheduler`)**: Assigns queued targets to available robots using pluggable cost models:
  - `EuclideanCost`: Evaluates L2 metric distance between robot position and goal coordinates.
  - `HeadingAwareCost`: Evaluates metric distance penalized by angular misalignment (delta yaw) using `std::remainder(ryaw - angle_to_goal, 2 * pi)` with configurable heading weighting.
- **Thread Safety**: Inter-thread communication between the ROS 2 executor thread and the GLFW render loop is protected using `std::mutex` (`RosBridge::state_mutex`).

## Prerequisites

- **Operating System**: Ubuntu 24.04 (Noble) with ROS 2 Jazzy, or Ubuntu 22.04 (Jammy) with ROS 2 Humble
- **Compiler**: GCC 11+ or Clang 14+ supporting C++20
- **Build System**: CMake 3.20+, `ament_cmake`, `colcon`
- **ROS 2 Packages**:
  - `rclcpp`
  - `rclcpp_action`
  - `nav_msgs`
  - `nav2_msgs`
  - `tf2_msgs`
  - `action_msgs`
- **System Libraries**:
  - `libeigen3-dev`
  - `uuid-dev`
  - `libgl1-mesa-dev`
  - `libx11-dev` / `libwayland-dev`

Note: GLFW 3.4, Glad (OpenGL 4.6 Core), and Dear ImGui (v1.91.8) are downloaded and compiled automatically at build time using CMake `FetchContent`.

## Building

Source your ROS 2 environment and compile using `colcon`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/fleet_commander
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running Tests

Unit tests are implemented using GoogleTest and cover robot state updates, reachability timeouts, and goal lifecycle transitions:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/fleet_commander
colcon test --ctest-args -V
colcon test-result --all
```

## Usage

### 1. Launch Multi-Robot Simulation

Run a multi-robot simulation providing `/map`, `/tf`, and Nav2 action servers (for example, the `bcr_bot` warehouse simulation):

```bash
source /opt/ros/jazzy/setup.bash
source ~/bcr_ws/install/setup.bash
ros2 launch bcr_bot multi_robot.launch.py
```

### 2. Launch Fleet Commander

In a separate terminal, source the workspace and run the executable:

```bash
source /opt/ros/jazzy/setup.bash
source ~/fleet_commander/install/setup.bash
ros2 run fleet_commander fleet_commander
```

### 3. User Controls

- **Pan Map**: Left-click and drag on the map viewport.
- **Zoom Map**: Scroll mouse wheel.
- **Dispatch Goal**: Click the target coordinates on the occupancy grid to place a goal marker, or input coordinates manually in the Control Panel.
- **Scheduler Selection**: Select the cost assignment strategy (`EuclideanCost` or `HeadingAwareCost`) from the dropdown to assign queued goals to idle robots.
- **Goal Inspection**: View active goal status, elapsed run time, and completion history in the Goal Queue table.

## Project Structure

```
fleet_commander/
├── CMakeLists.txt              # CMake configuration and FetchContent declarations
├── package.xml                 # ROS 2 package manifest and dependencies
├── include/
│   ├── app.hpp                 # GLFW window, OpenGL context, and Dear ImGui layout
│   ├── ros_bridge.hpp          # ROS 2 node, topic subscriptions, and Nav2 action clients
│   ├── scheduler.hpp           # Task allocation interfaces and cost metrics
│   └── state.hpp               # Thread-safe data models for robots, goals, and map
├── src/
│   ├── app.cpp                 # OpenGL rendering loop and ImGui control panels
│   ├── main.cpp                # Application entrypoint and ROS 2 lifecycle initialization
│   ├── ros_bridge.cpp          # Subscription callbacks, TF math, and action handling
│   └── scheduler.cpp           # Cost metric implementations and assignment logic
└── tests/
    └── test_state.cpp          # GoogleTest suite for robot and goal state logic
```

## License

This project is licensed under the MIT License.
