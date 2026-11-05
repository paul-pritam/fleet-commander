#pragma once

#include <iostream>
#include <fstream>
#include <cmath>
#include <filesystem>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
#include "comparison_types.hpp"

namespace fleet_commander
{

class RunComparator {
public:
  static bool is_rosbag(const std::string & path) {
    namespace fs = std::filesystem;
    if (!fs::exists(path)) return false;
    if (fs::is_directory(path)) {
      if (fs::exists(fs::path(path) / "metadata.yaml")) return true;
      for (const auto & entry : fs::directory_iterator(path)) {
        if (entry.path().extension() == ".mcap" || entry.path().extension() == ".db3") return true;
      }
    } else {
      std::string ext = fs::path(path).extension().string();
      if (ext == ".mcap" || ext == ".db3") return true;
    }
    return false;
  }

  static bool load_run_from_json(const std::string & filepath, RunSession & session) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      return false;
    }
    try {
      nlohmann::json j;
      file >> j;
      session.file_path = filepath;
      session.run_name = j.value("run_name", "Unknown Run");
      session.planner_type = j.value("planner_type", "Standard");
      session.total_duration_sec = j.value("total_duration_sec", 0.0);
      session.total_distance_m = j.value("total_distance_m", 0.0);
      session.optimal_distance_m = j.value("optimal_distance_m", 0.0);
      session.path_efficiency_percent = j.value("path_efficiency_percent", 0.0);
      session.success_rate_percent = j.value("success_rate_percent", 0.0);
      session.max_curvature = j.value("max_curvature", 0.0);
      session.mean_curvature = j.value("mean_curvature", 0.0);
      session.num_replans = j.value("num_replans", 0);

      session.trajectory.clear();
      if (j.contains("trajectory") && j["trajectory"].is_array()) {
        for (const auto & item : j["trajectory"]) {
          Point2D pt;
          pt.x = item.value("x", 0.0);
          pt.y = item.value("y", 0.0);
          pt.yaw = item.value("yaw", 0.0);
          pt.time = item.value("time", 0.0);
          session.trajectory.push_back(pt);
        }
      }

      session.planned_path.clear();
      if (j.contains("planned_path") && j["planned_path"].is_array()) {
        for (const auto & item : j["planned_path"]) {
          Point2D pt;
          pt.x = item.value("x", 0.0);
          pt.y = item.value("y", 0.0);
          session.planned_path.push_back(pt);
        }
      }

      session.missions.clear();
      if (j.contains("missions") && j["missions"].is_array()) {
        for (const auto & item : j["missions"]) {
          MissionSummary m;
          m.name = item.value("name", "Mission");
          m.status = item.value("status", "Unknown");
          m.efficiency_percent = item.value("efficiency_percent", 0.0);
          m.duration_sec = item.value("duration_sec", 0.0);
          if (item.contains("goal_pose") && item["goal_pose"].is_array()) {
            for (const auto & v : item["goal_pose"]) {
              m.goal_pose.push_back(v.get<double>());
            }
          }
          session.missions.push_back(m);
        }
      }
      return true;
    } catch (const std::exception & e) {
      std::cerr << "Error parsing JSON session: " << e.what() << "\n";
      return false;
    }
  }

  static bool load_run(const std::string & filepath, RunSession & session, bool force_reparse = false) {
    namespace fs = std::filesystem;
    if (filepath.empty() || !fs::exists(filepath)) {
      return false;
    }

    if (!force_reparse && fs::path(filepath).extension() == ".json") {
      return load_run_from_json(filepath, session);
    }

    if (is_rosbag(filepath)) {
      std::string clean = filepath;
      while (!clean.empty() && clean.back() == '/') clean.pop_back();
      std::string json_path = clean + ".json";

      bool need_export = force_reparse;
      if (!need_export) {
        if (!fs::exists(json_path)) {
          need_export = true;
        } else {
          try {
            auto bag_time = fs::last_write_time(filepath);
            auto json_time = fs::last_write_time(json_path);
            if (json_time < bag_time) {
              need_export = true;
            }
          } catch (...) {
            need_export = true;
          }
        }
      }

      if (need_export) {
        std::string cmd = "python3 /home/pritam/fleet_commander/scripts/export_bag_to_json.py \"" +
                          clean + "\" \"" + json_path + "\"";
        int ret = std::system(cmd.c_str());
        if (ret != 0) {
          std::cerr << "Bag export command returned: " << ret << "\n";
        }
      }

      if (fs::exists(json_path)) {
        return load_run_from_json(json_path, session);
      }
    }

    return load_run_from_json(filepath, session);
  }

  static std::vector<std::string> discover_runs(const std::string & directory, int max_depth = 4) {
    namespace fs = std::filesystem;
    std::vector<std::string> runs;
    if (directory.empty() || !fs::exists(directory) || !fs::is_directory(directory)) {
      return runs;
    }

    try {
      auto it = fs::recursive_directory_iterator(directory, fs::directory_options::skip_permission_denied);
      for (auto entry = fs::begin(it); entry != fs::end(it); ++entry) {
        if (entry.depth() > max_depth) {
          entry.disable_recursion_pending();
          continue;
        }

        std::string fname = entry->path().filename().string();
        if (fname == ".git" || fname == "build" || fname == "install" || fname == ".cache" || fname == "node_modules") {
          entry.disable_recursion_pending();
          continue;
        }

        const auto & p = entry->path();
        if (entry->is_directory()) {
          if (fs::exists(p / "metadata.yaml")) {
            runs.push_back(p.string());
            entry.disable_recursion_pending();
          }
        } else if (p.extension() == ".mcap" || p.extension() == ".db3") {
          if (!fs::exists(p.parent_path() / "metadata.yaml")) {
            runs.push_back(p.string());
          }
        } else if (p.extension() == ".json") {
          std::string stem = p.stem().string();
          fs::path parent = p.parent_path();
          if (!fs::exists(parent / stem)) {
            runs.push_back(p.string());
          }
        }
      }
    } catch (const std::exception & e) {
      std::cerr << "Error scanning directory for runs: " << e.what() << "\n";
    }
    std::sort(runs.begin(), runs.end());
    runs.erase(std::unique(runs.begin(), runs.end()), runs.end());
    return runs;
  }

  static ComparisonReport compare(const RunSession & a, const RunSession & b) {
    ComparisonReport report;
    report.run_a = a;
    report.run_b = b;

    report.efficiency_delta_percent = b.path_efficiency_percent - a.path_efficiency_percent;
    report.success_rate_delta_percent = b.success_rate_percent - a.success_rate_percent;

    if (a.total_duration_sec > 0.0) {
      report.time_savings_percent = ((a.total_duration_sec - b.total_duration_sec) / a.total_duration_sec) * 100.0;
    }

    if (a.max_curvature > 0.0) {
      report.curvature_reduction_percent = ((a.max_curvature - b.max_curvature) / a.max_curvature) * 100.0;
    }

    report.replan_reduction_count = a.num_replans - b.num_replans;

    if (b.success_rate_percent > a.success_rate_percent + 0.1) {
      report.winner_name = b.run_name;
    } else if (a.success_rate_percent > b.success_rate_percent + 0.1) {
      report.winner_name = a.run_name;
    } else {
      report.winner_name = (b.path_efficiency_percent >= a.path_efficiency_percent)
                           ? b.run_name : a.run_name;
    }

    char buf[256];
    snprintf(buf, sizeof(buf), "WINNER: %s (+%.1f%% Path Efficiency, %.1f%% Time Savings)",
             report.winner_name.c_str(),
             std::abs(report.efficiency_delta_percent),
             std::abs(report.time_savings_percent));
    report.summary_verdict = buf;

    return report;
  }
};

} // namespace fleet_commander
