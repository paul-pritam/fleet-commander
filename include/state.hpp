#pragma once

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <cstdint>
#include <eigen3/Eigen/Dense>

struct MapData{
    std::vector<uint8_t> pixels;
    uint32_t width = 0;
    uint32_t height = 0;
    double resolution = 0.0;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();

    bool empty() const { return pixels.empty(); }
};

struct RobotPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

enum class RobotStatus { Idle, 
                        Navigating, 
                        Offline };

struct RobotState       
{
    std::string id;
    RobotPose pose;
    RobotStatus status = RobotStatus::Idle;
    std::string current_goal_id;
    std::chrono::steady_clock::time_point last_tf_update;

    bool is_reachable() const{
        return std::chrono::steady_clock::now() - last_tf_update < std::chrono::seconds(5);   
    }
};

enum class GoalStatus{
    Pending,
    Active,
    Succeeded,
    Failed,
    Cancelled
};

struct GoalState {
    std::string id;
    Eigen::Vector2d target = Eigen::Vector2d::Zero();
    GoalStatus status = GoalStatus::Pending;
    std::string assigned_robot;
    std::chrono::steady_clock::time_point created_at;
    std::chrono::steady_clock::time_point completed_at;

    bool is_terminal() const{
        return status == GoalStatus::Succeeded || 
               status == GoalStatus::Failed || 
               status == GoalStatus::Cancelled;
    }

    double age_ses() const{
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - created_at
        ).count();
    }
};

struct FleetState{
    std::map<std::string, RobotState> robots;
    std::vector<GoalState> goals;
    MapData map;
    void add_goal(GoalState goal){
        goals.push_back(std::move(goal)); // goal is moved instead of copy , saves some memory
    }
};
