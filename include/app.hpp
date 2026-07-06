#pragma once
 
#include <memory>
#include <string>

#include "ros_bridge.hpp"

struct GLFWwindow;

class App{
    public:
        App();
        ~App();

        bool init (int height, int width, const char *title);
        void run();
        void shutdown();
    private:
        std::shared_ptr<RosBridge> ros_;

        GLFWwindow *window_ = nullptr;
        unsigned int map_texture_ = 0;
        int map_tex_w_ = 0, map_tex_h_ = 0;
        int selected_cost_fn_ = 0;
        float map_zoom_ = 1.0f;
        std::string status_msg_;

        void process_ros_events();
        void render_ui();
        void render_map_panel();
        void render_status_bar();
        void update_map_texture();
};