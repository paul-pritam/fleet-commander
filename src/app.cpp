#include "app.hpp"
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
  ImGui::StyleColorsDark();

  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::GetStyle().WindowRounding = 6.0f;
  ImGui::GetStyle().FrameRounding = 4.0f;
  ImGui::GetStyle().ScrollbarSize = 14.0f;

  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 460");

  ros_ = std::make_shared<RosBridge>();

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
  int w, h;
  glfwGetFramebufferSize(window_, &w, &h);
  ImVec2 size = {(float)w, (float)h};

  ImGui::SetNextWindowSize(size, ImGuiCond_Always);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::Begin("Map", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar);
  render_map_panel();
  ImGui::End();
  ImGui::PopStyleVar();

  render_status_bar();
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

    if (local_x >= 0.0f && local_x <= 1.0f && local_y >= 0.0f && local_y <= 1.0f) {
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

void App::render_status_bar() { return; }
