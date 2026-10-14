#include "waypoint_editor/io/waypoint_nav_yaml.hpp"

#include <fstream>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace waypoint_editor
{
namespace io
{

bool WaypointNavYaml::Save(const std::vector<Waypoint> &waypoints, const std::string &path, std::string &error)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        error = "Failed to open file for writing: " + path;
        return false;
    }

    // Determine frame_id from first waypoint or default to "map"
    std::string frame_id = "map";
    if (!waypoints.empty() && !waypoints.front().pose.header.frame_id.empty()) {
        frame_id = waypoints.front().pose.header.frame_id;
    }

    ofs << "frame_id: " << frame_id << "\n";
    ofs << "waypoints:\n";

    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const auto &wp = waypoints[i];
        const auto &pos = wp.pose.pose.position;
        const auto &ori = wp.pose.pose.orientation;

        ofs << "  - position: {x: " << pos.x << ", y: " << pos.y << ", z: " << pos.z << "}\n";
        ofs << "    orientation: {x: " << ori.x << ", y: " << ori.y << ", z: " << ori.z << ", w: " << ori.w << "}\n";

        if (!wp.function_command.empty()) {
            ofs << "    command: \"" << wp.function_command << "\"\n";
        }
    }

    error.clear();
    return true;
}

namespace
{

bool ReadDouble(const YAML::Node &node, const char *key, double &value)
{
    const auto child = node[key];
    if (!child) {
        return false;
    }
    try {
        value = child.as<double>();
        return true;
    } catch (const YAML::Exception &) {
        return false;
    }
}

}  // namespace

bool WaypointNavYaml::Load(const std::string &path, std::vector<Waypoint> &waypoints, std::string &error)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception &ex) {
        error = "Failed to parse yaml: " + std::string(ex.what());
        return false;
    }

    // Read top-level frame_id
    std::string frame_id = "map";
    const auto frame_node = root["frame_id"];
    if (frame_node && frame_node.IsScalar()) {
        frame_id = frame_node.as<std::string>();
    }

    const auto wp_node = root["waypoints"];
    if (!wp_node || !wp_node.IsSequence()) {
        error = "Missing 'waypoints' list in yaml";
        return false;
    }

    std::vector<Waypoint> parsed;
    for (const auto &entry : wp_node) {
        if (!entry.IsMap()) {
            continue;
        }

        const auto pos_node = entry["position"];
        const auto ori_node = entry["orientation"];
        if (!pos_node || !ori_node) {
            continue;
        }

        Waypoint wp;
        if (!ReadDouble(pos_node, "x", wp.pose.pose.position.x) ||
            !ReadDouble(pos_node, "y", wp.pose.pose.position.y) ||
            !ReadDouble(pos_node, "z", wp.pose.pose.position.z)) {
            continue;
        }
        if (!ReadDouble(ori_node, "x", wp.pose.pose.orientation.x) ||
            !ReadDouble(ori_node, "y", wp.pose.pose.orientation.y) ||
            !ReadDouble(ori_node, "z", wp.pose.pose.orientation.z) ||
            !ReadDouble(ori_node, "w", wp.pose.pose.orientation.w)) {
            continue;
        }

        wp.pose.header.frame_id = frame_id;

        const auto cmd_node = entry["command"];
        if (cmd_node && cmd_node.IsScalar()) {
            wp.function_command = cmd_node.as<std::string>();
        }

        parsed.emplace_back(std::move(wp));
    }

    if (parsed.empty()) {
        error = "No valid waypoints found in navigation yaml";
        return false;
    }

    waypoints = std::move(parsed);
    error.clear();
    return true;
}

}  // namespace io
}  // namespace waypoint_editor
