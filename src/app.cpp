#include "app.hpp"
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

App::App() = default;
App::~App() {shutdown();}

void App::shutdown() {
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

bool App::init(int height, int width, const char *title){
    if (!glfwInit()) return false;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {glfwTerminate(); return false;}

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    int version = gladLoadGL(glfwGetProcAddress);
    if (!version) return false;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 460");

    ros_ = std::make_shared<RosBridge>();

    unsigned char placeholder[4] = {128, 128, 128, 255};
    glGenTextures(1, &map_texture_);
    glBindTexture(GL_TEXTURE_2D, map_texture_);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, placeholder);
    return true;    
}

void App::run(){

    while (!glfwWindowShouldClose(window_)){
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
        glClearColor(0.1f, 0.1f, 0.1f, 0.1f);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window_);
    }

}

void App::process_ros_events(){

    std::lock_guard<std::mutex> lock(ros_->state_mutex);
    if (ros_->map_updated){
        update_map_texture();
        ros_->map_updated = false;
    }
}

void App::update_map_texture(){
    const auto &map = ros_->state.map;
    if (map.empty()) return;

    std::vector<unsigned char> rgba(map.pixels.size() * 4);
    for (size_t i = 0; i<map.pixels.size(); ++i){
        rgba[i * 4 + 0] = map.pixels[i];
        rgba[i * 4 + 1] = map.pixels[i];
        rgba[i * 4 + 2] = map.pixels[i];
        rgba[i * 4 + 3] = 255;
    }
    
    glBindTexture(GL_TEXTURE_2D, map_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, map.width, map.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());

    map_tex_h_ = map.height;
    map_tex_w_ = map.width;
}

void App::render_ui(){
    // WHY: Get actual window size from GLFW.
    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    ImVec2 size = {(float)w, (float)h};

    // WHY: SetNextWindowSize makes ImGui window fill the entire application window.
    // WHY: No title bar, no padding — map fills the whole space.
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

void App::render_map_panel(){

    auto &s = ros_->state;
    if (s.map.empty()){
        ImGui::TextDisabled("waiting for /map.......");
        return;
    }

    ImVec2 avail = ImGui::GetContentRegionAvail();
    float map_aspect = static_cast<float>(s.map.width)/s.map.height;
    float avail_aspect = avail.x/avail.y;

    float display_w, display_h;
    if (map_aspect > avail_aspect){
        display_w = avail.x;
        display_h = avail.x/map_aspect;
    }
    else{
        display_w = avail.y * map_aspect;
        display_h = avail.y;
    }

    display_w *= map_zoom_;
    display_h *= map_zoom_;

    ImGui::Image((ImTextureID)(intptr_t)map_texture_, ImVec2(display_w, display_h));
}

void App::render_status_bar(){
    return;
}



