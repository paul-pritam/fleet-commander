#pragma once

#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "ros_bridge.hpp"
#include "comparison_types.hpp"
#include "run_comparator.hpp"

struct GLFWwindow;

enum class ActivePage {
  LiveFleet,
  RunComparison
};

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

  // View Mode / Page State
  ActivePage active_page_{ActivePage::LiveFleet};

  // Run Comparison & Benchmark State
  char run_a_input_[256]{"/home/pritam/fleet_commander/bags/run_astar"};
  char run_b_input_[256]{"/home/pritam/fleet_commander/bags/run_hybrid_astar"};
  char bags_dir_input_[256]{"/home/pritam/fleet_commander/bags"};
  std::vector<std::string> discovered_runs_;
  int selected_combo_run_a_{-1};
  int selected_combo_run_b_{-1};
  bool show_run_a_path_{true};
  bool show_run_b_path_{true};
  bool show_planned_paths_{true};
  int selected_mission_filter_{-1}; // -1 = All missions
  float comparison_zoom_{1.0f};
  float comp_pan_x_{0.0f};
  float comp_pan_y_{0.0f};
  std::string comparison_status_msg_{"Select two ROS 2 bags or session logs and click 'Run Benchmark Comparison'."};
  std::optional<fleet_commander::ComparisonReport> comparison_report_;
  std::thread comparison_thread_;
  std::atomic<bool> is_comparing_{false};
  bool use_cached_json_{true};

  void scan_runs_directory();
  void unselect_runs();

  void process_ros_events();
  void try_assign_goals();
  void render_ui();
  void render_navbar();
  void render_live_fleet_page();
  void render_comparison_page();
  void render_comparison_map_canvas();
  void render_comparison_dashboard();
  void execute_run_comparison(bool force_reparse = false);

  void render_toolbar();
  void render_map_panel();
  void render_status_bar();
  void render_status_panel();
  void update_map_texture();
};
