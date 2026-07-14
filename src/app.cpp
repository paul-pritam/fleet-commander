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
#include <string>
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
  return true;
}

void App::run() {

  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    rclcpp::spin_some(ros_);
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

  for (const auto &a : assignments) {
    for (const auto &g : s.goals) {
      if (g.id == a.goal) {
        lock.~lock_guard();
        ros_->send_goal(a.robot, g.id, g.target.x(), g.target.y());
        new (&lock) std::lock_guard<std::mutex>(ros_->state_mutex);
        break;
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
  float padding = ImGui::GetStyle().WindowPadding.y;

  float toolbar_h = ImGui::GetFrameHeight() * 2.0f + padding * 3.0f;
  float status_bar_h = ImGui::GetFrameHeight() + padding * 2.0f;

  float content_h = vp->WorkSize.y - toolbar_h - status_bar_h - padding;

  render_toolbar();

  float content_y = vp->WorkPos.y + toolbar_h;
  float content_w = vp->WorkSize.x;

  ImGui::SetCursorScreenPos(ImVec2(vp->WorkPos.x, content_y));
  ImGui::BeginChild("Map Panel", ImVec2(content_w - panel_width_, content_h),
                    ImGuiChildFlags_Border);

  render_map_panel();
  ImGui::EndChild();

  ImGui::SetCursorScreenPos(
      ImVec2(vp->WorkPos.x + content_w - panel_width_, content_y));
  ImGui::BeginChild("Status Panel", ImVec2(panel_width_, content_h),
                    ImGuiChildFlags_Border);

  render_status_panel();
  ImGui::EndChild();

  ImGui::SetCursorScreenPos(
      ImVec2(vp->WorkPos.x, content_y + content_h + padding));
  render_status_bar();
  ImGui::End();
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
               ImVec2(display_w, display_h));

  float scale_x = display_w / s.map.width;
  float scale_y = display_h / s.map.height;

  auto *draw_list = ImGui::GetWindowDrawList();

  for (const auto &[id, robot] : s.robots) {
    if (!robot.is_reachable())
      continue;

    auto px = s.map.world_to_pixel(robot.pose.x, robot.pose.y);
    float sx = cursor.x + px.x() * scale_x;
    float sy = cursor.y + px.y() * scale_y;

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
    float end_y = sy + sinf(robot.pose.yaw) * arrow_len;
    draw_list->AddLine(ImVec2(sx, sy), ImVec2(end_x, end_y),
                       IM_COL32(255, 255, 255, 255), 2.0f);
  }

  for (const auto &g : s.goals) {
    auto px = s.map.world_to_pixel(g.target.x(), g.target.y());
    float sx = cursor.x + px.x() * scale_x;
    float sy = cursor.y + px.y() * scale_y;

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
      int pixel_y = static_cast<int>(local_y * s.map.height);

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
    float scroll = ImGui::GetIO().MouseWheel;

    if (scroll != 0.0f && ImGui::IsItemHovered()) {
      map_zoom_ *= (scroll > 0) ? 1.1f : 0.9f;
      map_zoom_ = std::clamp(map_zoom_, 0.1f, 5.0f);
    }
  }
}
