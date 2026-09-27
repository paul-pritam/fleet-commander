#pragma once

#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "ros_bridge.hpp"

struct GLFWwindow;

struct GoalDispatch {
  std::string robot_id;
  std::string goal_id;
  double x = 0.0;
  double y = 0.0;
};

class App {
public:
  App();
  ~App();

  bool init(int width, int height, const char *title);
  void run();
  void shutdown();

private:
  std::shared_ptr<RosBridge> ros_;
  std::thread ros_spin_thread_;
  std::atomic<bool> running_{false};

  GLFWwindow *window_ = nullptr;
  unsigned int map_texture_ = 0;
  int map_tex_w_ = 0, map_tex_h_ = 0;

  int selected_cost_fn_ = 0;
  int max_goals_per_robot_ = 1;
  float map_zoom_ = 1.0f;
  std::string status_msg_;

  float panel_width_ = 300.0f;

  void process_ros_events();
  void try_assign_goals();
  void render_ui();
  void render_toolbar();
  void render_map_panel();
  void render_status_bar();
  void render_status_panel();
  void update_map_texture();
};
