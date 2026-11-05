#include "app.hpp"
#include "scheduler.hpp"
#include "state.hpp"
// clang-format off
#include <algorithm>
#include <chrono>
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <memory>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <thread>
#include <vector>

App::App() = default;
App::~App() { shutdown(); }

void App::shutdown() {
  if (window_) {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();

  running_ = false;

  if (ros_spin_thread_.joinable()) {
    ros_spin_thread_.join();
  }

  if (comparison_thread_.joinable()) {
    comparison_thread_.join();
  }

  if (map_texture_) {
    glDeleteTextures(1, &map_texture_);
    map_texture_ = 0;
  }
}

bool App::init(int height, int width, const char *title) {
  if (!glfwInit())
    return false;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
  if (!window_) {
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  int version = gladLoadGL(glfwGetProcAddress);
  if (!version)
    return false;

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  ImGui::StyleColorsDark();

  ImGuiStyle &style = ImGui::GetStyle();
  style.WindowRounding = 4.0f;
  style.FrameRounding = 2.0f;
  style.GrabRounding = 2.0f;
  style.WindowPadding = ImVec2(8.0f, 8.0f);
  style.FramePadding = ImVec2(8.0f, 4.0f);
  style.ItemSpacing = ImVec2(8.0f, 4.0f);
  style.ScrollbarSize = 14.0f;

  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 460");

  ros_ = std::make_shared<RosBridge>();

  ros_->on_goal_result = [this](const std::string &goal_id,
                                const std::string &robot_id, bool success) {
    std::lock_guard<std::mutex> lock(ros_->state_mutex);
    for (auto &g : ros_->state.goals) {
      if (g.id == goal_id && g.status == GoalStatus::Active) {
        g.status = success ? GoalStatus::Succeeded : GoalStatus::Failed;
        g.completed_at = std::chrono::steady_clock::now();
        break;
      }
    }
    for (auto &[_, r] : ros_->state.robots) {
      if (r.id == robot_id) {
        r.status = RobotStatus::Idle;
        r.current_goal_id.clear();
        break;
      }
    }
  };

  unsigned char placeholder[4] = {128, 128, 128, 255};
  glGenTextures(1, &map_texture_);
  glBindTexture(GL_TEXTURE_2D, map_texture_);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               placeholder);

  running_ = true;

  ros_spin_thread_ = std::thread([this]() {
    rclcpp::WallRate loop_rate(60);
    while (running_ && rclcpp::ok()) {
      rclcpp::spin_some(ros_);
      loop_rate.sleep();
    }
  });

  scan_runs_directory();
  execute_run_comparison();

  return true;
}

void App::run() {

  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    process_ros_events();
    try_assign_goals();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    render_ui();
    ImGui::Render();

    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window_);
  }
}

void App::process_ros_events() {

  std::lock_guard<std::mutex> lock(ros_->state_mutex);
  if (ros_->map_updated) {
    update_map_texture();
    ros_->map_updated = false;
  }
}

void App::try_assign_goals() {

  std::vector<GoalDispatch> dispatches;
  {
    std::lock_guard<std::mutex> lock(ros_->state_mutex);

    auto &s = ros_->state;

    auto pending = s.pending_goals();
    auto idle = s.idle_robots();

    if (pending.empty() || idle.empty())
      return;

    std::vector<AssignmentInput> robots;
    for (auto *r : idle) {
      robots.push_back({r->id, r->pose.x, r->pose.y, r->pose.yaw});
    }

    std::vector<GoalInput> goals;
    for (auto *g : pending) {
      goals.push_back({g->id, g->target.x(), g->target.y()});
    }

    std::unique_ptr<CostFunction> cost_fn;
    if (selected_cost_fn_ == 0) {
      cost_fn = std::make_unique<EuclideanCost>();
    } else {
      cost_fn = std::make_unique<HeadingAwareCost>();
    }

    Scheduler sched(std::move(cost_fn), max_goals_per_robot_);
    auto assignments = sched.assign_goals(robots, goals);

    for (const auto &a : assignments) {
      for (auto *g : pending) {
        if (g->id == a.goal) {
          g->status = GoalStatus::Active;
          g->assigned_robot = a.robot;
          break;
        }
      }

      for (auto &[_, r] : s.robots) {
        if (r.id == a.robot) {
          r.status = RobotStatus::Navigating;
          r.current_goal_id = a.goal;
          break;
        }
      }
    }
    dispatches.reserve(assignments.size());
    for (const auto &a : assignments) {
      for (const auto &g : s.goals) {
        if (g.id == a.goal) {
          dispatches.push_back({a.robot, g.id, g.target.x(), g.target.y()});
          break;
        }
      }
    }
  }
  for (const auto &d : dispatches) {
    bool sent = ros_->send_goal(d.robot_id, d.goal_id, d.x, d.y);
    if (!sent) {
      std::lock_guard<std::mutex> lock(ros_->state_mutex);
      for (auto &g : ros_->state.goals) {
        if (g.id == d.goal_id) {
          g.status = GoalStatus::Pending;
          break;
        }
      }
      for (auto &[_, r] : ros_->state.robots) {
        if (r.id == d.robot_id) {
          r.status = RobotStatus::Idle;
          r.current_goal_id.clear();
          break;
        }
      }
    }
  }
}

void App::update_map_texture() {
  const auto &map = ros_->state.map;
  if (map.empty())
    return;

  std::vector<unsigned char> rgba(map.pixels.size() * 4);
  for (size_t i = 0; i < map.pixels.size(); ++i) {
    rgba[i * 4 + 0] = map.pixels[i];
    rgba[i * 4 + 1] = map.pixels[i];
    rgba[i * 4 + 2] = map.pixels[i];
    rgba[i * 4 + 3] = 255;
  }

  glBindTexture(GL_TEXTURE_2D, map_texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, map.width, map.height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, rgba.data());

  map_tex_h_ = map.height;
  map_tex_w_ = map.width;
}

void App::render_ui() {
  const ImGuiViewport *vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(vp->WorkPos);
  ImGui::SetNextWindowSize(vp->WorkSize);

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                           ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoBringToFrontOnFocus;

  ImGui::Begin("Fleet Commander", nullptr, flags);

  render_navbar();

  if (active_page_ == ActivePage::LiveFleet) {
    render_live_fleet_page();
  } else if (active_page_ == ActivePage::RunComparison) {
    render_comparison_page();
  }

  ImGui::End();
}

void App::render_navbar() {
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(16.0f, 6.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(12.0f, 6.0f));

  ImGui::TextColored(ImVec4(0.2f, 0.75f, 1.0f, 1.0f), "FLEET COMMANDER");
  ImGui::SameLine();
  ImGui::TextDisabled("| AMR Mission & Benchmark System");
  ImGui::SameLine(0.0f, 30.0f);

  bool is_live = (active_page_ == ActivePage::LiveFleet);
  if (is_live) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.45f, 0.75f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.55f, 0.85f, 1.0f));
  } else {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.25f, 0.30f, 1.0f));
  }
  if (ImGui::Button(" [ Live Fleet Operations ] ")) {
    active_page_ = ActivePage::LiveFleet;
  }
  ImGui::PopStyleColor(2);

  ImGui::SameLine();

  bool is_comp = (active_page_ == ActivePage::RunComparison);
  if (is_comp) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.60f, 0.35f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.70f, 0.42f, 1.0f));
  } else {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.25f, 0.30f, 1.0f));
  }
  if (ImGui::Button(" [ Run Comparison & Benchmark ] ")) {
    active_page_ = ActivePage::RunComparison;
    if (!comparison_report_.has_value()) {
      scan_runs_directory();
  execute_run_comparison();
    }
  }
  ImGui::PopStyleColor(2);

  ImGui::PopStyleVar(2);
  ImGui::Separator();
  ImGui::Spacing();
}

void App::render_live_fleet_page() {
  float padding = ImGui::GetStyle().WindowPadding.y;
  float status_bar_h = ImGui::GetFrameHeight() + padding * 2.0f;

  render_toolbar();

  ImVec2 avail = ImGui::GetContentRegionAvail();
  float content_h = avail.y - status_bar_h - padding;
  float content_w = avail.x;

  float status_w = panel_width_;
  if (content_w < 650.0f) {
    status_w = std::max(content_w * 0.38f, 160.0f);
  }
  float map_w = std::max(content_w - status_w - ImGui::GetStyle().ItemSpacing.x, 150.0f);

  ImGui::BeginChild("Map Panel", ImVec2(map_w, content_h),
                    ImGuiChildFlags_Border);
  render_map_panel();
  ImGui::EndChild();

  ImGui::SameLine();

  ImGui::BeginChild("Status Panel", ImVec2(status_w, content_h),
                    ImGuiChildFlags_Border);
  render_status_panel();
  ImGui::EndChild();

  render_status_bar();
}

void App::render_toolbar() {
  // row1
  ImGui::TextDisabled("Fleet Commander");
  ImGui::SameLine(0.0f, 30.0f);
  ImGui::TextDisabled("Click on the map to send Goals");

  // row2
  ImGui::Text("Cost Function:");
  ImGui::SameLine();
  const char *cost_names[] = {"Euclidean", "Heading-Aware"};
  ImGui::SetNextItemWidth(130.0f);
  ImGui::Combo("##cost", &selected_cost_fn_, cost_names, 2);

  ImGui::SameLine(0.0f, 20.0f);

  ImGui::Text("Mex Goals/Robot");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(60.0f);
  ImGui::DragInt("##max", &max_goals_per_robot_, 0.1f, 1, 10);
}

void App::render_status_panel() {
  std::lock_guard<std::mutex> lock(ros_->state_mutex);
  auto &s = ros_->state;

  if (ImGui::CollapsingHeader("Robots", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (s.robots.empty()) {
      ImGui::TextDisabled("No Robots discovered");
    }

    for (const auto &[id, robot] : s.robots) {

      ImVec4 color = robot.status == RobotStatus::Idle ? ImVec4(0, 1, 0, 1)
                     : robot.status == RobotStatus::Navigating
                         ? ImVec4(1, 1, 0, 1)
                         : ImVec4(1, 0, 0, 1);

      ImGui::TextColored(color, "%s", id.c_str());

      ImGui::SameLine();
      ImGui::TextDisabled("[%s]", robot.status == RobotStatus::Idle ? "idle"
                                  : robot.status == RobotStatus::Navigating
                                      ? "nav"
                                      : "offline");

      if (!robot.current_goal_id.empty()) {
        ImGui::Text("Goal: %s", robot.current_goal_id.c_str());
      }

      auto staleness =
          std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::steady_clock::now() - robot.last_tf_update)
              .count();

      if (staleness > 3) {
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "Last seen : %lds ago",
                           staleness);
      }

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::Spacing();
    }
  }
  ImGui::Spacing();

  if (ImGui::CollapsingHeader("Goals", ImGuiTreeNodeFlags_DefaultOpen)) {

    if (s.goals.empty()) {
      ImGui::TextDisabled("No goals. Click on the map to set one");
    } else {

      auto sorted = s.goals;
      std::sort(sorted.begin(), sorted.end(),
                [](const GoalState &a, const GoalState &b) {
                  auto order = [](GoalStatus s) {
                    switch (s) {
                    case GoalStatus::Active:
                      return 0;
                    case GoalStatus::Pending:
                      return 1;
                    case GoalStatus::Succeeded:
                      return 2;
                    case GoalStatus::Failed:
                      return 3;
                    case GoalStatus::Cancelled:
                      return 4;
                    }
                    return 5;
                  };
                  return order(a.status) < order(b.status);
                });

      if (ImGui::BeginTable("Goals", 4,
                            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("ID");
        ImGui::TableSetupColumn("Co-ord");
        ImGui::TableSetupColumn("Status");
        ImGui::TableSetupColumn("Robot");
        ImGui::TableHeadersRow();

        for (const auto &g : sorted) {

          ImVec4 color =
              g.status == GoalStatus::Pending     ? ImVec4(1, 0.65f, 0, 1)
              : g.status == GoalStatus::Active    ? ImVec4(1, 1, 0, 1)
              : g.status == GoalStatus::Succeeded ? ImVec4(0, 1, 0, 1)
              : g.status == GoalStatus::Failed    ? ImVec4(1, 0, 0, 1)
                                                  : ImVec4(0.5f, 0.5f, 0.5f, 1);

          ImGui::TableNextRow();

          // column 0
          ImGui::TableSetColumnIndex(0);
          ImGui::Text("%s", g.id.c_str());

          // column 1
          ImGui::TableSetColumnIndex(1);
          ImGui::Text("(%.1f, %.1f)", g.target.x(), g.target.y());

          // column 2
          ImGui::TableSetColumnIndex(2);
          ImGui::TextColored(color, "%s", GoalState::status_icon(g.status));

          // column 3
          ImGui::TableSetColumnIndex(3);
          ImGui::Text(
              "%s", g.assigned_robot.empty() ? "-" : g.assigned_robot.c_str());
        }
        ImGui::EndTable();
      }
    }
  }
}

void App::render_status_bar() {

  std::lock_guard<std::mutex> lock(ros_->state_mutex);
  auto &s = ros_->state;

  int robot_count = s.robots.size();
  int active = 0, pending = 0;

  for (const auto &g : s.goals) {
    if (g.status == GoalStatus::Active)
      active++;
    if (g.status == GoalStatus::Pending)
      pending++;
  }

  ImGui::Text(" Robots: %d | Active Goals: %d| Pending Goals: %d| Map: %dx%d",
              robot_count, active, pending, s.map.width, s.map.height);

  if (!status_msg_.empty()) {
    ImGui::SameLine();
    ImGui::Text("|");
    ImGui::SameLine();
    ImGui::TextDisabled("%s", status_msg_.c_str());
  }
}

void App::render_map_panel() {
  std::lock_guard<std::mutex> lock(ros_->state_mutex);

  auto &s = ros_->state;
  if (s.map.empty()) {
    ImGui::TextDisabled("waiting for /map.......");
    return;
  }

  ImVec2 avail = ImGui::GetContentRegionAvail();
  float map_aspect = static_cast<float>(s.map.width) / s.map.height;
  float avail_aspect = avail.x / avail.y;

  float display_w, display_h;
  if (map_aspect > avail_aspect) {
    display_w = avail.x;
    display_h = avail.x / map_aspect;
  } else {
    display_w = avail.y * map_aspect;
    display_h = avail.y;
  }

  display_w *= map_zoom_;
  display_h *= map_zoom_;

  ImVec2 cursor = ImGui::GetCursorScreenPos();
  ImGui::Image((ImTextureID)(intptr_t)map_texture_,
               ImVec2(display_w, display_h), ImVec2(0.0f, 1.0f),
               ImVec2(1.0f, 0.0f));

  float scale_x = display_w / s.map.width;
  float scale_y = display_h / s.map.height;

  auto *draw_list = ImGui::GetWindowDrawList();

  for (const auto &[id, robot] : s.robots) {
    if (!robot.is_reachable())
      continue;

    auto px = s.map.world_to_pixel_continuous(robot.pose.x, robot.pose.y);
    float sx = cursor.x + static_cast<float>(px.x()) * scale_x;
    float sy = cursor.y + static_cast<float>(s.map.height - 1 - px.y()) * scale_y;

    ImVec4 col = robot.status == RobotStatus::Idle         ? ImVec4(0, 1, 0, 1)
                 : robot.status == RobotStatus::Navigating ? ImVec4(1, 1, 0, 1)
                                                           : ImVec4(1, 0, 0, 1);

    ImU32 color = ImGui::ColorConvertFloat4ToU32(col);

    draw_list->AddCircleFilled(ImVec2(sx, sy), 6.0f, color);
    draw_list->AddCircle(ImVec2(sx, sy), 8.0f, IM_COL32(255, 255, 255, 255), 0,
                         2.0f);
    draw_list->AddText(ImVec2(sx + 10.0f, sy - 10.0f),
                       IM_COL32(255, 255, 255, 255), id.c_str());

    float arrow_len = 15.0f;
    float end_x = sx + cosf(robot.pose.yaw) * arrow_len;
    float end_y = sy - sinf(robot.pose.yaw) * arrow_len;
    draw_list->AddLine(ImVec2(sx, sy), ImVec2(end_x, end_y),
                       IM_COL32(255, 255, 255, 255), 2.0f);
  }

  for (const auto &g : s.goals) {
    auto px = s.map.world_to_pixel_continuous(g.target.x(), g.target.y());
    float sx = cursor.x + static_cast<float>(px.x()) * scale_x;
    float sy = cursor.y + static_cast<float>(s.map.height - 1 - px.y()) * scale_y;

    ImVec4 col = g.status == GoalStatus::Pending     ? ImVec4(1, 0.65f, 0, 1)
                 : g.status == GoalStatus::Active    ? ImVec4(1, 1, 0, 1)
                 : g.status == GoalStatus::Succeeded ? ImVec4(0, 1, 0, 1)
                 : g.status == GoalStatus::Failed
                     ? ImVec4(1, 0, 0, 1)
                     : ImVec4(0.5f, 0.5f, 0.5f, 0.5f);
    ImU32 color = ImGui::ColorConvertFloat4ToU32(col);

    float size = 8.0f;
    ImVec2 points[] = {
        {sx, sy - size}, {sx + size, sy}, {sx, sy + size}, {sx - size, sy}};

    draw_list->AddConvexPolyFilled(points, 4, color);
    draw_list->AddPolyline(points, 4, IM_COL32(255, 255, 255, 255),
                           ImDrawFlags_Closed, 1.0f);

    char label[16];
    snprintf(label, sizeof(label), "%s", g.id.c_str());
    draw_list->AddText(ImVec2(sx + 12.0f, sy), IM_COL32(255, 255, 255, 255),
                       label);
  }

  if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0)) {
    ImVec2 mouse = ImGui::GetMousePos();
    float local_x = (mouse.x - cursor.x) / display_w;
    float local_y = (mouse.y - cursor.y) / display_h;

    if (local_x >= 0.0f && local_x <= 1.0f && local_y >= 0.0f &&
        local_y <= 1.0f) {
      int pixel_x = static_cast<int>(local_x * s.map.width);
      int pixel_y = static_cast<int>((1.0f - local_y) * s.map.height);

      auto world = s.map.pixel_to_world(pixel_x, pixel_y);

      GoalState goal;
      goal.id = generate_goal_id();
      goal.target = world;
      goal.status = GoalStatus::Pending;
      goal.created_at = std::chrono::steady_clock::now();
      s.add_goal(std::move(goal));

      status_msg_ = "Goal Created";
    }
  }

  float scroll = ImGui::GetIO().MouseWheel;
  if (scroll != 0.0) {
    if (ImGui::IsItemHovered()) {
      map_zoom_ *= (scroll > 0) ? 1.1f : 0.9f;
      map_zoom_ = std::clamp(map_zoom_, 0.1f, 5.0f);
    }
  }
}

void App::render_comparison_page() {
  ImVec2 avail = ImGui::GetContentRegionAvail();
  float canvas_w = avail.x * 0.60f;
  float dash_w = avail.x - canvas_w - ImGui::GetStyle().ItemSpacing.x;

  ImGui::BeginChild("BenchmarkCanvasChild", ImVec2(canvas_w, avail.y), ImGuiChildFlags_Border);
  render_comparison_map_canvas();
  ImGui::EndChild();

  ImGui::SameLine();

  ImGui::BeginChild("BenchmarkDashboardChild", ImVec2(dash_w, avail.y), ImGuiChildFlags_Border);
  render_comparison_dashboard();
  ImGui::EndChild();
}

void App::render_comparison_map_canvas() {
  ImGui::TextColored(ImVec4(0.2f, 0.75f, 1.0f, 1.0f), "DUAL RUN TRAJECTORY OVERLAY CANVAS");
  ImGui::SameLine(0.0f, 15.0f);
  if (ImGui::SmallButton("Zoom +")) {
    comparison_zoom_ = std::min(5.0f, comparison_zoom_ * 1.15f);
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Zoom -")) {
    comparison_zoom_ = std::max(0.2f, comparison_zoom_ / 1.15f);
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Reset View")) {
    comparison_zoom_ = 1.0f;
    comp_pan_x_ = 0.0f;
    comp_pan_y_ = 0.0f;
  }
  ImGui::SameLine(0.0f, 12.0f);
  ImGui::TextDisabled("Scale: %.1fx", comparison_zoom_);

  ImGui::Separator();

  double origin_x = -14.3;
  double origin_y = -15.5;
  double resolution = 0.05;
  int map_w = 587;
  int map_h = 624;

  bool has_live_map = false;
  {
    std::lock_guard<std::mutex> lock(ros_->state_mutex);
    if (!ros_->state.map.empty()) {
      has_live_map = true;
      origin_x = ros_->state.map.origin.x();
      origin_y = ros_->state.map.origin.y();
      resolution = ros_->state.map.resolution;
      map_w = static_cast<int>(ros_->state.map.width);
      map_h = static_cast<int>(ros_->state.map.height);
    }
  }

  ImVec2 avail = ImGui::GetContentRegionAvail();
  float map_aspect = static_cast<float>(map_w) / static_cast<float>(map_h);
  float avail_aspect = avail.x / avail.y;

  float display_w = 0.0f, display_h = 0.0f;
  if (map_aspect > avail_aspect) {
    display_w = avail.x;
    display_h = avail.x / map_aspect;
  } else {
    display_w = avail.y * map_aspect;
    display_h = avail.y;
  }

  display_w *= comparison_zoom_;
  display_h *= comparison_zoom_;

  ImVec2 cursor = ImGui::GetCursorScreenPos();
  float start_x = cursor.x + comp_pan_x_;
  float start_y = cursor.y + comp_pan_y_;

  ImDrawList *draw_list = ImGui::GetWindowDrawList();
  if (has_live_map && map_texture_ != 0) {
    ImGui::SetCursorScreenPos(ImVec2(start_x, start_y));
    ImGui::Image((ImTextureID)(intptr_t)map_texture_,
                 ImVec2(display_w, display_h),
                 ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
  } else {
    draw_list->AddRectFilled(ImVec2(start_x, start_y),
                             ImVec2(start_x + display_w, start_y + display_h),
                             IM_COL32(24, 26, 32, 255));
    float step_px = static_cast<float>(2.0 / resolution) * (display_w / static_cast<float>(map_w));
    if (step_px > 10.0f) {
      for (float gx = start_x; gx < start_x + display_w; gx += step_px) {
        draw_list->AddLine(ImVec2(gx, start_y), ImVec2(gx, start_y + display_h),
                           IM_COL32(40, 44, 55, 120), 1.0f);
      }
      for (float gy = start_y; gy < start_y + display_h; gy += step_px) {
        draw_list->AddLine(ImVec2(start_x, gy), ImVec2(start_x + display_w, gy),
                           IM_COL32(40, 44, 55, 120), 1.0f);
      }
    }
    draw_list->AddRect(ImVec2(start_x, start_y),
                       ImVec2(start_x + display_w, start_y + display_h),
                       IM_COL32(70, 75, 90, 255), 0.0f, 0, 1.5f);
  }

  float scale_x = display_w / static_cast<float>(map_w);
  float scale_y = display_h / static_cast<float>(map_h);
  auto to_screen = [&](double wx, double wy) -> ImVec2 {
    double px = (wx - origin_x) / resolution;
    double py = (wy - origin_y) / resolution;
    return ImVec2(start_x + static_cast<float>(px) * scale_x,
                  start_y + static_cast<float>(static_cast<double>(map_h - 1) - py) * scale_y);
  };

  if (!comparison_report_.has_value()) {
    float card_w = std::min(display_w * 0.85f, 480.0f);
    float card_h = 100.0f;
    float card_x = start_x + (display_w - card_w) * 0.5f;
    float card_y = start_y + (display_h - card_h) * 0.4f;

    draw_list->AddRectFilled(ImVec2(card_x, card_y),
                             ImVec2(card_x + card_w, card_y + card_h),
                             IM_COL32(18, 20, 26, 235), 6.0f);
    draw_list->AddRect(ImVec2(card_x, card_y),
                       ImVec2(card_x + card_w, card_y + card_h),
                       IM_COL32(70, 80, 100, 255), 6.0f, 0, 1.5f);

    draw_list->AddText(ImVec2(card_x + 18.0f, card_y + 14.0f),
                       IM_COL32(255, 215, 0, 255), "BENCHMARK STANDBY: NO RUNS ACTIVE");
    draw_list->AddText(ImVec2(card_x + 18.0f, card_y + 40.0f),
                       IM_COL32(210, 220, 235, 255),
                       "Select or browse any ROS 2 bag folders (.mcap/.db3) or JSON files");
    draw_list->AddText(ImVec2(card_x + 18.0f, card_y + 62.0f),
                       IM_COL32(160, 175, 195, 255),
                       "on the right, then click 'Run Benchmark Comparison'.");
  } else {
    const auto &run_a = comparison_report_->run_a;
    const auto &run_b = comparison_report_->run_b;

    if (show_run_a_path_ && !run_a.trajectory.empty()) {
      ImU32 col_a = IM_COL32(255, 130, 20, 230);
      for (size_t i = 1; i < run_a.trajectory.size(); ++i) {
        ImVec2 p1 = to_screen(run_a.trajectory[i - 1].x, run_a.trajectory[i - 1].y);
        ImVec2 p2 = to_screen(run_a.trajectory[i].x, run_a.trajectory[i].y);
        draw_list->AddLine(p1, p2, col_a, 2.5f);
      }
      if (show_planned_paths_ && !run_a.planned_path.empty()) {
        ImU32 col_a_plan = IM_COL32(255, 180, 70, 140);
        for (size_t i = 1; i < run_a.planned_path.size(); ++i) {
          ImVec2 p1 = to_screen(run_a.planned_path[i - 1].x, run_a.planned_path[i - 1].y);
          ImVec2 p2 = to_screen(run_a.planned_path[i].x, run_a.planned_path[i].y);
          draw_list->AddLine(p1, p2, col_a_plan, 1.2f);
        }
      }
    }

    if (show_run_b_path_ && !run_b.trajectory.empty()) {
      ImU32 col_b = IM_COL32(40, 220, 80, 240);
      for (size_t i = 1; i < run_b.trajectory.size(); ++i) {
        ImVec2 p1 = to_screen(run_b.trajectory[i - 1].x, run_b.trajectory[i - 1].y);
        ImVec2 p2 = to_screen(run_b.trajectory[i].x, run_b.trajectory[i].y);
        draw_list->AddLine(p1, p2, col_b, 3.2f);
      }
      if (show_planned_paths_ && !run_b.planned_path.empty()) {
        ImU32 col_b_plan = IM_COL32(100, 255, 150, 150);
        for (size_t i = 1; i < run_b.planned_path.size(); ++i) {
          ImVec2 p1 = to_screen(run_b.planned_path[i - 1].x, run_b.planned_path[i - 1].y);
          ImVec2 p2 = to_screen(run_b.planned_path[i].x, run_b.planned_path[i].y);
          draw_list->AddLine(p1, p2, col_b_plan, 1.2f);
        }
      }
    }

    if (!run_b.trajectory.empty()) {
      ImVec2 start_pt = to_screen(run_b.trajectory.front().x, run_b.trajectory.front().y);
      draw_list->AddCircleFilled(start_pt, 6.0f, IM_COL32(30, 144, 255, 255));
      draw_list->AddCircle(start_pt, 8.0f, IM_COL32(255, 255, 255, 255), 0, 2.0f);
      draw_list->AddText(ImVec2(start_pt.x + 10.0f, start_pt.y - 12.0f),
                         IM_COL32(255, 255, 255, 255), "START");
    }

    const auto &missions = !run_b.missions.empty() ? run_b.missions : run_a.missions;
    for (size_t i = 0; i < missions.size(); ++i) {
      if (missions[i].goal_pose.size() >= 2) {
        ImVec2 gp = to_screen(missions[i].goal_pose[0], missions[i].goal_pose[1]);
        float sz = 8.0f;
        ImVec2 diamond[4] = {
          {gp.x, gp.y - sz},
          {gp.x + sz, gp.y},
          {gp.x, gp.y + sz},
          {gp.x - sz, gp.y}
        };
        draw_list->AddConvexPolyFilled(diamond, 4, IM_COL32(255, 215, 0, 230));
        draw_list->AddPolyline(diamond, 4, IM_COL32(255, 255, 255, 255), ImDrawFlags_Closed, 1.5f);

        char glbl[64];
        snprintf(glbl, sizeof(glbl), "M%zu: %s", i + 1, missions[i].name.c_str());
        draw_list->AddText(ImVec2(gp.x + 11.0f, gp.y - 7.0f),
                           IM_COL32(255, 230, 120, 255), glbl);
      }
    }

    ImVec2 leg_pos(cursor.x + 12.0f, cursor.y + 40.0f);
    ImVec2 leg_size(340.0f, 100.0f);
    draw_list->AddRectFilled(leg_pos, ImVec2(leg_pos.x + leg_size.x, leg_pos.y + leg_size.y),
                             IM_COL32(20, 22, 28, 220), 4.0f);
    draw_list->AddRect(leg_pos, ImVec2(leg_pos.x + leg_size.x, leg_pos.y + leg_size.y),
                       IM_COL32(60, 65, 80, 220), 4.0f);

    float ly = leg_pos.y + 8.0f;
    draw_list->AddLine(ImVec2(leg_pos.x + 12.0f, ly + 8.0f), ImVec2(leg_pos.x + 36.0f, ly + 8.0f),
                       IM_COL32(40, 220, 80, 255), 3.0f);
    char buf_b[128];
    snprintf(buf_b, sizeof(buf_b), "Run B: %s (%.1f%% eff)", run_b.run_name.c_str(), run_b.path_efficiency_percent);
    draw_list->AddText(ImVec2(leg_pos.x + 44.0f, ly), IM_COL32(220, 255, 220, 255), buf_b);

    ly += 22.0f;
    draw_list->AddLine(ImVec2(leg_pos.x + 12.0f, ly + 8.0f), ImVec2(leg_pos.x + 36.0f, ly + 8.0f),
                       IM_COL32(255, 130, 20, 255), 2.5f);
    char buf_a[128];
    snprintf(buf_a, sizeof(buf_a), "Run A: %s (%.1f%% eff)", run_a.run_name.c_str(), run_a.path_efficiency_percent);
    draw_list->AddText(ImVec2(leg_pos.x + 44.0f, ly), IM_COL32(255, 220, 190, 255), buf_a);

    ly += 22.0f;
    float gsz = 5.0f;
    ImVec2 gpoly[4] = {
      {leg_pos.x + 24.0f, ly + 8.0f - gsz},
      {leg_pos.x + 24.0f + gsz, ly + 8.0f},
      {leg_pos.x + 24.0f, ly + 8.0f + gsz},
      {leg_pos.x + 24.0f - gsz, ly + 8.0f}
    };
    draw_list->AddConvexPolyFilled(gpoly, 4, IM_COL32(255, 215, 0, 255));
    draw_list->AddText(ImVec2(leg_pos.x + 44.0f, ly), IM_COL32(240, 240, 240, 255),
                       "Target Dispatch Goals");

    ly += 22.0f;
    draw_list->AddCircleFilled(ImVec2(leg_pos.x + 24.0f, ly + 8.0f), 4.5f, IM_COL32(30, 144, 255, 255));
    draw_list->AddText(ImVec2(leg_pos.x + 44.0f, ly), IM_COL32(200, 220, 255, 255),
                       "Initial AMCL Robot Origin");
  }

  if (ImGui::IsWindowHovered()) {
    float scroll = ImGui::GetIO().MouseWheel;
    if (scroll != 0.0f) {
      comparison_zoom_ *= (scroll > 0) ? 1.1f : 0.9f;
      comparison_zoom_ = std::clamp(comparison_zoom_, 0.2f, 5.0f);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
      ImVec2 delta = ImGui::GetIO().MouseDelta;
      comp_pan_x_ += delta.x;
      comp_pan_y_ += delta.y;
    }
  }
}

void App::render_comparison_dashboard() {
  ImGui::TextColored(ImVec4(0.2f, 0.75f, 1.0f, 1.0f), "BENCHMARK CONTROL & DIRECT ROSBAG SELECTION");
  ImGui::Separator();
  ImGui::Spacing();

  // Action Buttons: Unselect and Reset
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.18f, 0.18f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.70f, 0.24f, 0.24f, 1.0f));
  if (ImGui::Button(" [ Unselect / Clear Runs ] ", ImVec2(180.0f, 26.0f))) {
    unselect_runs();
  }
  ImGui::PopStyleColor(2);

  ImGui::SameLine();

  if (ImGui::Button("Load Default Benchmark Pair", ImVec2(200.0f, 26.0f))) {
    snprintf(run_a_input_, sizeof(run_a_input_), "/home/pritam/fleet_commander/bags/run_astar");
    snprintf(run_b_input_, sizeof(run_b_input_), "/home/pritam/fleet_commander/bags/run_hybrid_astar");
    execute_run_comparison();
  }

  ImGui::Spacing();

  // Section 1: Rosbag Directory Scanner & Dropdown Pickers
  if (ImGui::CollapsingHeader("Rosbag File & Folder Browser", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Bags Directory (recursive scan):");
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 70.0f);
    if (ImGui::InputText("##BagsDir", bags_dir_input_, sizeof(bags_dir_input_), ImGuiInputTextFlags_EnterReturnsTrue)) {
      scan_runs_directory();
    }
    ImGui::SameLine();
    if (ImGui::Button("Scan", ImVec2(60.0f, 0.0f))) {
      scan_runs_directory();
    }

    if (discovered_runs_.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f),
                         "0 rosbags or session files found in this directory.");
      ImGui::TextDisabled("Enter any folder containing ROS 2 bags and press Enter or 'Scan'.");
    } else {
      ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.4f, 1.0f),
                         "Discovered %zu rosbags / sessions:", discovered_runs_.size());

      // Combo for Run A
      std::string current_a = (selected_combo_run_a_ >= 0 && selected_combo_run_a_ < static_cast<int>(discovered_runs_.size()))
                            ? std::filesystem::path(discovered_runs_[selected_combo_run_a_]).filename().string()
                            : "Select Rosbag for Run A...";
      ImGui::Text("Pick Run A:");
      if (ImGui::BeginCombo("##ComboRunA", current_a.c_str())) {
        for (int i = 0; i < static_cast<int>(discovered_runs_.size()); ++i) {
          std::string item_name = std::filesystem::path(discovered_runs_[i]).filename().string();
          bool is_sel = (selected_combo_run_a_ == i);
          if (ImGui::Selectable(item_name.c_str(), is_sel)) {
            selected_combo_run_a_ = i;
            snprintf(run_a_input_, sizeof(run_a_input_), "%s", discovered_runs_[i].c_str());
          }
          if (is_sel) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
      }

      // Combo for Run B
      std::string current_b = (selected_combo_run_b_ >= 0 && selected_combo_run_b_ < static_cast<int>(discovered_runs_.size()))
                            ? std::filesystem::path(discovered_runs_[selected_combo_run_b_]).filename().string()
                            : "Select Rosbag for Run B...";
      ImGui::Text("Pick Run B:");
      if (ImGui::BeginCombo("##ComboRunB", current_b.c_str())) {
        for (int i = 0; i < static_cast<int>(discovered_runs_.size()); ++i) {
          std::string item_name = std::filesystem::path(discovered_runs_[i]).filename().string();
          bool is_sel = (selected_combo_run_b_ == i);
          if (ImGui::Selectable(item_name.c_str(), is_sel)) {
            selected_combo_run_b_ = i;
            snprintf(run_b_input_, sizeof(run_b_input_), "%s", discovered_runs_[i].c_str());
          }
          if (is_sel) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
      }
    }
  }

  ImGui::Spacing();

  // Section 2: Active Run File Inputs (Accepts Rosbag Folder, MCAP, or JSON)
  ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "ACTIVE RUN PATHS (ROSBAG FOLDER / MCAP / JSON):");
  ImGui::Text("Run A Path:");
  ImGui::SetNextItemWidth(-1.0f);
  ImGui::InputText("##RunAPath", run_a_input_, sizeof(run_a_input_));

  ImGui::Text("Run B Path:");
  ImGui::SetNextItemWidth(-1.0f);
  ImGui::InputText("##RunBPath", run_b_input_, sizeof(run_b_input_));

  ImGui::Spacing();
  ImGui::Checkbox("Use cached session if available (instant)", &use_cached_json_);
  ImGui::SameLine(0.0f, 15.0f);
  if (ImGui::SmallButton("Force Re-parse from Raw Rosbag")) {
    execute_run_comparison(true);
  }

  ImGui::Spacing();
  if (is_comparing_.load()) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.45f, 0.40f, 0.15f, 1.0f));
    ImGui::Button(" [ Parsing ROS 2 Bags & Calculating Curvature... Please Wait ] ", ImVec2(-1.0f, 34.0f));
    ImGui::PopStyleColor();
  } else {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.55f, 0.32f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.16f, 0.70f, 0.40f, 1.0f));
    if (ImGui::Button(" [ Run Benchmark Comparison ] ", ImVec2(-1.0f, 34.0f))) {
      execute_run_comparison(!use_cached_json_);
    }
    ImGui::PopStyleColor(2);
  }

  ImGui::Spacing();
  ImGui::TextDisabled("%s", comparison_status_msg_.c_str());
  ImGui::Spacing();
  ImGui::Separator();

  ImGui::Text("Trajectory Display Filters:");
  ImGui::Checkbox("Show Run A (Orange)", &show_run_a_path_);
  ImGui::SameLine();
  ImGui::Checkbox("Show Run B (Green)", &show_run_b_path_);
  ImGui::SameLine();
  ImGui::Checkbox("Show Waypoints", &show_planned_paths_);
  ImGui::Separator();
  ImGui::Spacing();

  if (comparison_report_.has_value()) {
    const auto &rep = *comparison_report_;

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.06f, 0.28f, 0.14f, 0.90f));
    ImGui::BeginChild("WinnerCard", ImVec2(-1.0f, 68.0f), ImGuiChildFlags_Border);
    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "WINNER: %s", rep.winner_name.c_str());
    ImGui::Text("Advantage: +%.1f%% Path Efficiency | %.1f%% Duration Reduction",
                rep.efficiency_delta_percent, rep.time_savings_percent);
    ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f),
                       "Re-plans Avoided: %d fewer controller aborts",
                       rep.replan_reduction_count);
    ImGui::EndChild();
    ImGui::PopStyleColor();

    ImGui::Spacing();

    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "HEAD-TO-HEAD COMPARISON METRICS");
    if (ImGui::BeginTable("MetricsTable", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupColumn("Metric");
      ImGui::TableSetupColumn("Run A");
      ImGui::TableSetupColumn("Run B");
      ImGui::TableSetupColumn("Delta / Advantage");
      ImGui::TableHeadersRow();

      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Total Execution Time");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.1f s", rep.run_a.total_duration_sec);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.1f s", rep.run_b.total_duration_sec);
      ImGui::TableSetColumnIndex(3);
      ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "%.1f%% faster (%.1fs saved)",
                         rep.time_savings_percent,
                         rep.run_a.total_duration_sec - rep.run_b.total_duration_sec);

      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Path Efficiency");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.1f%%", rep.run_a.path_efficiency_percent);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.1f%%", rep.run_b.path_efficiency_percent);
      ImGui::TableSetColumnIndex(3);
      ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "+%.1f%%",
                         rep.efficiency_delta_percent);

      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Mission Success Rate");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.0f%%", rep.run_a.success_rate_percent);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.0f%%", rep.run_b.success_rate_percent);
      ImGui::TableSetColumnIndex(3);
      ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Evaluated");

      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Controller Replans");
      ImGui::TableSetColumnIndex(1);
      ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%d replans", rep.run_a.num_replans);
      ImGui::TableSetColumnIndex(2);
      ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "%d replans", rep.run_b.num_replans);
      ImGui::TableSetColumnIndex(3);
      ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "%+d replans",
                         -rep.replan_reduction_count);

      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Max Traj Curvature");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.2f m^-1", rep.run_a.max_curvature);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.2f m^-1", rep.run_b.max_curvature);
      ImGui::TableSetColumnIndex(3);
      ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "Kinematic comparison");

      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Traveled Distance");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.2f m", rep.run_a.total_distance_m);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.2f m", rep.run_b.total_distance_m);
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.2f m optimal", rep.run_b.optimal_distance_m);

      ImGui::EndTable();
    }

    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Mission-by-Mission Breakdown", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::BeginTable("MissionsTable", 4,
                            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Mission Name");
        ImGui::TableSetupColumn("Run A Time / Eff");
        ImGui::TableSetupColumn("Run B Time / Eff");
        ImGui::TableSetupColumn("Speedup Factor");
        ImGui::TableHeadersRow();

        size_t count = std::max(rep.run_a.missions.size(), rep.run_b.missions.size());
        for (size_t i = 0; i < count; ++i) {
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          std::string mname = (i < rep.run_b.missions.size()) ? rep.run_b.missions[i].name
                            : (i < rep.run_a.missions.size()) ? rep.run_a.missions[i].name : "Mission";
          ImGui::Text("%s", mname.c_str());

          ImGui::TableSetColumnIndex(1);
          if (i < rep.run_a.missions.size()) {
            ImGui::Text("%.1fs (%.1f%%)", rep.run_a.missions[i].duration_sec,
                        rep.run_a.missions[i].efficiency_percent);
          } else {
            ImGui::Text("-");
          }

          ImGui::TableSetColumnIndex(2);
          if (i < rep.run_b.missions.size()) {
            ImGui::Text("%.1fs (%.1f%%)", rep.run_b.missions[i].duration_sec,
                        rep.run_b.missions[i].efficiency_percent);
          } else {
            ImGui::Text("-");
          }

          ImGui::TableSetColumnIndex(3);
          if (i < rep.run_a.missions.size() && i < rep.run_b.missions.size() &&
              rep.run_b.missions[i].duration_sec > 0.0) {
            float speedup = rep.run_a.missions[i].duration_sec / rep.run_b.missions[i].duration_sec;
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "%.2fx faster", speedup);
          } else {
            ImGui::Text("-");
          }
        }
        ImGui::EndTable();
      }
    }


  }
}

void App::execute_run_comparison(bool force_reparse) {
  if (is_comparing_.load()) {
    return;
  }

  if (run_a_input_[0] == '\0' || run_b_input_[0] == '\0') {
    comparison_status_msg_ = "Please provide both Run A and Run B paths.";
    return;
  }

  std::string path_a = run_a_input_;
  std::string path_b = run_b_input_;

  if (comparison_thread_.joinable()) {
    comparison_thread_.join();
  }

  is_comparing_ = true;
  comparison_status_msg_ = force_reparse
      ? "Parsing raw ROS 2 bag files from scratch (deserializing poses & curvature)..."
      : "Loading run telemetry...";

  comparison_thread_ = std::thread([this, path_a, path_b, force_reparse]() {
    fleet_commander::RunSession session_a;
    fleet_commander::RunSession session_b;

    bool ok_a = fleet_commander::RunComparator::load_run(path_a, session_a, force_reparse);
    bool ok_b = fleet_commander::RunComparator::load_run(path_b, session_b, force_reparse);

    if (!ok_a && !ok_b) {
      comparison_status_msg_ = "Failed to load both Run A and Run B. Check bag / file paths.";
      is_comparing_ = false;
      return;
    }
    if (!ok_a) {
      comparison_status_msg_ = std::string("Failed to load Run A from: ") + path_a;
      is_comparing_ = false;
      return;
    }
    if (!ok_b) {
      comparison_status_msg_ = std::string("Failed to load Run B from: ") + path_b;
      is_comparing_ = false;
      return;
    }

    auto report = fleet_commander::RunComparator::compare(session_a, session_b);
    comparison_report_ = report;

    char buf[256];
    snprintf(buf, sizeof(buf), "Loaded: %s (%zu poses) vs %s (%zu poses).%s",
             session_a.run_name.c_str(), session_a.trajectory.size(),
             session_b.run_name.c_str(), session_b.trajectory.size(),
             force_reparse ? " (Parsed from raw bag messages)" : " (Fast session)");
    comparison_status_msg_ = buf;
    is_comparing_ = false;
  });
}

void App::scan_runs_directory() {
  discovered_runs_ = fleet_commander::RunComparator::discover_runs(bags_dir_input_);
}

void App::unselect_runs() {
  comparison_report_.reset();
  run_a_input_[0] = '\0';
  run_b_input_[0] = '\0';
  selected_combo_run_a_ = -1;
  selected_combo_run_b_ = -1;
  comparison_status_msg_ = "Runs unselected. Pick any two rosbags or logs to benchmark.";
}

