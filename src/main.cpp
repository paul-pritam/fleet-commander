#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <cstdio>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <rclcpp/utilities.hpp>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (!glfwInit()) {
    fprintf(stderr, "Failed to init GLFW\n");
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  GLFWwindow *window =
      glfwCreateWindow(1200, 800, "Fleet Commander", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "Failed to create GLFW window\n");
    glfwTerminate();
    rclcpp::shutdown();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  int version = gladLoadGL(glfwGetProcAddress);
  if (!version) {
    fprintf(stderr, "Failed to init GLAD\n");
    glfwTerminate();
    rclcpp::shutdown();
    return 1;
  }
  printf("OpenGL %d.%d\n", GLAD_VERSION_MAJOR(version),
         GLAD_VERSION_MINOR(version));

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

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 460");

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(1200, 800));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                             ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoResize;

    ImGui::Begin("Fleet Commander", nullptr, flags);
    ImGui::End();

    ImGui::Render();

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    glViewport(0, 0, display_w, display_h);

    glClearColor(0.1f, 0.1f, 0.1f, 0.1f);

    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  rclcpp::shutdown();
  return 0;
}
