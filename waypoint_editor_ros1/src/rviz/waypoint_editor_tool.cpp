#include <ros/ros.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MenuEntry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <pluginlib/class_list_macros.h>

#include <QApplication>
#include <QFileDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind/bind.hpp>

#include "waypoint_editor/io/waypoint_csv.hpp"
#include "waypoint_editor/io/waypoint_nav_yaml.hpp"
#include "waypoint_editor/io/waypoint_yaml.hpp"
#include "waypoint_editor/rviz/waypoint_editor_tool.hpp"

namespace ph = boost::placeholders;

namespace waypoint_editor
{

namespace
{

constexpr double kWaypointMarkerScale = 0.75;
constexpr double kWaypointSphereScale = 0.28;
constexpr double kWaypointArrowLength = 0.36;
constexpr double kWaypointArrowWidth = 0.10;
constexpr double kWaypointArrowHeight = 0.10;
constexpr double kWaypointTextScale = 0.16;
constexpr double kWaypointTextOffsetX = -0.12;
constexpr double kWaypointTextOffsetY = -0.12;
constexpr double kWaypointTextOffsetZ = 0.16;
const char *kHiddenDescriptionToken = "";
constexpr double kMenuDummyAlpha = 0.01;
constexpr double kNavigationProgressReachDistance = 0.35;
constexpr double kNavigationProgressHysteresis = 0.05;

} // namespace

WaypointEditorTool::WaypointEditorTool()
  : rviz::PoseTool(),
    pose_dirty_(false),
    last_displayed_distance_(0.0),
    navigation_state_(NAV_IDLE),
    navigation_index_(0),
    navigation_goal_index_(0),
    navigation_action_status_("IDLE"),
    navigation_elapsed_(0.0),
    navigation_resume_started_at_()
{
}

WaypointEditorTool::~WaypointEditorTool()
{
    stopNavigationExecution(true, NAV_IDLE, "IDLE");
    if (service_spinner_) {
        service_spinner_->stop();
    }
}

void WaypointEditorTool::onInitialize()
{
    PoseTool::onInitialize();
    setName("Add Waypoint");

    nh_ = ros::NodeHandle("/waypoint_editor");
    srv_nh_ = ros::NodeHandle("/waypoint_editor");
    srv_nh_.setCallbackQueue(&service_queue_);
    service_spinner_.reset(new ros::AsyncSpinner(1, &service_queue_));
    service_spinner_->start();

    server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_marker_server"));
    move_base_client_.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));

    save_service_ = srv_nh_.advertiseService("save_waypoints", &WaypointEditorTool::handleSaveWaypoints, this);
    load_service_ = srv_nh_.advertiseService("load_waypoints", &WaypointEditorTool::handleLoadWaypoints, this);
    undo_service_ = srv_nh_.advertiseService("undo_waypoints", &WaypointEditorTool::handleUndoWaypoints, this);
    redo_service_ = srv_nh_.advertiseService("redo_waypoints", &WaypointEditorTool::handleRedoWaypoints, this);
    clear_service_ = srv_nh_.advertiseService("clear_waypoints", &WaypointEditorTool::handleClearWaypoints, this);
    start_navigation_service_ = srv_nh_.advertiseService("start_navigation", &WaypointEditorTool::handleStartNavigation, this);
    pause_navigation_service_ = srv_nh_.advertiseService("pause_navigation", &WaypointEditorTool::handlePauseNavigation, this);
    resume_navigation_service_ = srv_nh_.advertiseService("resume_navigation", &WaypointEditorTool::handleResumeNavigation, this);
    stop_navigation_service_ = srv_nh_.advertiseService("stop_navigation", &WaypointEditorTool::handleStopNavigation, this);

    line_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_line", 10, true);
    waypoint_path_pub_ = nh_.advertise<nav_msgs::Path>("waypoints_path", 1, true);
    total_wp_dist_pub_ = nh_.advertise<std_msgs::Float64>("total_wp_dist", 10, true);
    last_wp_dist_pub_ = nh_.advertise<std_msgs::Float64>("last_wp_dist", 10, true);
    waypoint_count_pub_ = nh_.advertise<std_msgs::Int32>("waypoint_count", 1, true);
    navigation_ready_pub_ = nh_.advertise<std_msgs::Bool>("navigation_ready", 1, true);
    undo_available_pub_ = nh_.advertise<std_msgs::Bool>("undo_available", 1, true);
    redo_available_pub_ = nh_.advertise<std_msgs::Bool>("redo_available", 1, true);
    nav_status_pub_ = nh_.advertise<std_msgs::String>("navigation_status", 1, true);
    nav_action_status_pub_ = nh_.advertise<std_msgs::String>("navigation_action_status", 1, true);
    nav_index_pub_ = nh_.advertise<std_msgs::Int32>("navigation_index", 1, true);
    nav_target_pub_ = nh_.advertise<std_msgs::String>("navigation_target", 1, true);
    nav_elapsed_seconds_pub_ = nh_.advertise<std_msgs::Float64>("navigation_elapsed_seconds", 1, true);
    nav_remaining_distance_pub_ = nh_.advertise<std_msgs::Float64>("navigation_remaining_distance", 1, true);

    navigation_timer_ = nh_.createTimer(
        ros::Duration(0.1),
        &WaypointEditorTool::processNavigationTimer,
        this,
        false,
        false);
    state_publish_timer_ = nh_.createTimer(
        ros::Duration(0.5),
        &WaypointEditorTool::refreshPublishedState,
        this);

    waypoint_sequence_.clear();
    pose_dirty_ = false;
    last_displayed_distance_ = 0.0;
    updateWaypointMarker();
    publishRangeMetrics();
    publishWaypointPath();
    stopNavigationExecution(false, NAV_IDLE, "IDLE");
    importConfiguredWaypointFile();
}

void WaypointEditorTool::activate()
{
    PoseTool::activate();
}

void WaypointEditorTool::deactivate()
{
    PoseTool::deactivate();
}

void WaypointEditorTool::onPoseSet(double x, double y, double theta)
{
    Waypoint wp;
    wp.pose.header.frame_id = "map";
    wp.pose.header.stamp = ros::Time::now();
    wp.pose.pose.position.x = x;
    wp.pose.pose.position.y = y;
    wp.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    wp.pose.pose.orientation.x = q.x();
    wp.pose.pose.orientation.y = q.y();
    wp.pose.pose.orientation.z = q.z();
    wp.pose.pose.orientation.w = q.w();

    const int new_id = appendWaypointAndRefresh(std::move(wp));
    ROS_INFO("Added waypoint %d", new_id);
    deactivate();
}

void WaypointEditorTool::updateWaypointMarker()
{
    server_->clear();
    for (std::size_t i = 0; i < waypoint_sequence_.size(); ++i) {
        visualization_msgs::InteractiveMarker marker = createWaypointMarker(static_cast<int>(i));
        server_->insert(marker, boost::bind(&WaypointEditorTool::processFeedback, this, ph::_1));
    }
    server_->applyChanges();
}

int WaypointEditorTool::appendWaypointAndRefresh(Waypoint wp)
{
    if (wp.pose.header.frame_id.empty()) {
        wp.pose.header.frame_id = "map";
    }
    if (wp.pose.header.stamp.isZero()) {
        wp.pose.header.stamp = ros::Time::now();
    }

    const int new_id = waypoint_sequence_.appendWaypoint(std::move(wp));
    visualization_msgs::InteractiveMarker marker = createWaypointMarker(new_id);
    server_->insert(marker, boost::bind(&WaypointEditorTool::processFeedback, this, ph::_1));
    server_->applyChanges();
    commitWaypointChanges(new_id);
    return new_id;
}

bool WaypointEditorTool::loadWaypointsFromPath(const std::string &path, std::vector<Waypoint> &loaded, std::string &error) const
{
    const QString qpath = QString::fromStdString(path).toLower();
    bool ok = false;

    if (qpath.endsWith(".nav.yaml")) {
        ok = io::WaypointNavYaml::Load(path, loaded, error);
    } else if (qpath.endsWith(".yaml")) {
        std::string nav_yaml_error;
        ok = io::WaypointNavYaml::Load(path, loaded, nav_yaml_error);
        if (!ok) {
            std::string waypoint_yaml_error;
            ok = io::WaypointYaml::Load(path, loaded, waypoint_yaml_error);
            if (!ok) {
                error = !waypoint_yaml_error.empty() ? waypoint_yaml_error : nav_yaml_error;
            }
        }
    } else {
        ok = io::WaypointCsv::Load(path, loaded, error);
        if (ok && loaded.empty()) {
            ok = false;
            error = "No valid waypoints found in csv";
        }
    }

    return ok;
}

void WaypointEditorTool::applyLoadedWaypoints(std::vector<Waypoint> loaded)
{
    stopNavigationExecution(true, NAV_IDLE, "IDLE");
    for (Waypoint &wp : loaded) {
        if (wp.pose.header.frame_id.empty()) {
            wp.pose.header.frame_id = "map";
        }
        wp.pose.header.stamp = ros::Time::now();
    }

    waypoint_sequence_.assign(std::move(loaded));
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1);
}

bool WaypointEditorTool::importConfiguredWaypointFile()
{
    if (!waypoint_sequence_.empty()) {
        return false;
    }

    std::string configured_path;
    if (!ros::param::get("/waypoint_editor/default_waypoint_file", configured_path) || configured_path.empty()) {
        if (!ros::param::get("/move_base/SmoothWaypointPlanner/waypoint_file", configured_path) || configured_path.empty()) {
            return false;
        }
    }

    std::vector<Waypoint> loaded;
    std::string error;
    if (!loadWaypointsFromPath(configured_path, loaded, error)) {
        ROS_WARN("WaypointEditorTool failed to auto-import configured waypoint file '%s': %s", configured_path.c_str(), error.c_str());
        return false;
    }

    applyLoadedWaypoints(std::move(loaded));
    ROS_INFO("WaypointEditorTool auto-imported %zu waypoints from configured file %s", waypoint_sequence_.size(), configured_path.c_str());
    return true;
}

visualization_msgs::InteractiveMarker WaypointEditorTool::createWaypointMarker(const int id)
{
    const Waypoint &wp = waypoint_sequence_.at(static_cast<std::size_t>(id));

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = wp.pose.header.frame_id.empty() ? "map" : wp.pose.header.frame_id;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = std::to_string(id);
    int_marker.description = kHiddenDescriptionToken;
    int_marker.scale = kWaypointMarkerScale;
    int_marker.pose = wp.pose.pose;

    visualization_msgs::InteractiveMarkerControl pos_control;
    pos_control.name = "move_position";
    pos_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    pos_control.always_visible = true;
    pos_control.orientation.w = 0.7071;
    pos_control.orientation.x = 0.0;
    pos_control.orientation.y = 0.7071;
    pos_control.orientation.z = 0.0;
    {
        visualization_msgs::Marker sphere;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.scale.x = kWaypointSphereScale;
        sphere.scale.y = kWaypointSphereScale;
        sphere.scale.z = kWaypointSphereScale;
        sphere.color.r = 0.0;
        sphere.color.g = 1.0;
        sphere.color.b = 0.0;
        sphere.color.a = 1.0;
        pos_control.markers.push_back(sphere);
    }
    int_marker.controls.push_back(pos_control);

    visualization_msgs::InteractiveMarkerControl rot_control;
    rot_control.name = "rotate_yaw";
    rot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    rot_control.orientation.w = 0.7071;
    rot_control.orientation.x = 0.0;
    rot_control.orientation.y = -0.7071;
    rot_control.orientation.z = 0.0;
    rot_control.always_visible = true;
    {
        visualization_msgs::Marker arrow;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.scale.x = kWaypointArrowLength;
        arrow.scale.y = kWaypointArrowWidth;
        arrow.scale.z = kWaypointArrowHeight;
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        rot_control.markers.push_back(arrow);
    }
    int_marker.controls.push_back(rot_control);

    visualization_msgs::InteractiveMarkerControl text_control;
    text_control.name = "display_text";
    text_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    text_control.always_visible = true;
    {
        visualization_msgs::Marker text;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.scale.z = kWaypointTextScale;
        text.color.r = 0.0;
        text.color.g = 0.0;
        text.color.b = 0.0;
        text.color.a = 1.0;
        text.pose.position.x = kWaypointTextOffsetX;
        text.pose.position.y = kWaypointTextOffsetY;
        text.pose.position.z = kWaypointTextOffsetZ;
        text.text = "ID:" + std::to_string(id + 1);
        if (!wp.function_command.empty()) {
            text.text += "\n" + wp.function_command;
        }
        text_control.markers.push_back(text);
    }
    int_marker.controls.push_back(text_control);

    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.name = "menu";
    menu_control.always_visible = true;
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    {
        visualization_msgs::Marker dummy;
        dummy.type = visualization_msgs::Marker::SPHERE;
        dummy.scale.x = 0.001;
        dummy.scale.y = 0.001;
        dummy.scale.z = 0.001;
        dummy.color.r = 0.0;
        dummy.color.g = 0.0;
        dummy.color.b = 0.0;
        dummy.color.a = kMenuDummyAlpha;
        menu_control.markers.push_back(dummy);
    }
    int_marker.controls.push_back(menu_control);

    visualization_msgs::MenuEntry delete_entry;
    delete_entry.id = 1;
    delete_entry.parent_id = 0;
    delete_entry.title = "Delete Waypoint";
    int_marker.menu_entries.push_back(delete_entry);

    visualization_msgs::MenuEntry change_id_entry;
    change_id_entry.id = 2;
    change_id_entry.parent_id = 0;
    change_id_entry.title = "Change Waypoint ID";
    int_marker.menu_entries.push_back(change_id_entry);

    visualization_msgs::MenuEntry command_entry;
    command_entry.id = 3;
    command_entry.parent_id = 0;
    command_entry.title = "Edit Function Command";
    int_marker.menu_entries.push_back(command_entry);

    return int_marker;
}

void WaypointEditorTool::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
    const int id = std::stoi(fb->marker_name);
    if (!isValidWaypointId(id)) {
        return;
    }

    switch (fb->event_type) {
        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            geometry_msgs::Pose new_pose = fb->pose;
            waypoint_sequence_.updatePose(static_cast<std::size_t>(id), new_pose);
            pose_dirty_ = true;

            server_->setPose(fb->marker_name, fb->pose);
            server_->applyChanges();
            updateLastDistanceFromWaypoint(id);
            publishRangeMetrics();
            break;
        }
        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            if (pose_dirty_) {
                commitWaypointChanges(id);
            }
            break;
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            processMenuControl(fb);
            break;
        default:
            break;
    }
}

void WaypointEditorTool::processMenuControl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
    if (fb->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
        return;
    }

    const int id = std::stoi(fb->marker_name);
    if (!isValidWaypointId(id)) {
        return;
    }

    switch (fb->menu_entry_id) {
        case 1:
            stopNavigationExecution(true, NAV_IDLE, "IDLE");
            waypoint_sequence_.eraseWaypoint(static_cast<std::size_t>(id));
            updateWaypointMarker();
            commitWaypointChanges(id);
            ROS_INFO("Deleted waypoint %d", id);
            break;

        case 2:
        {
            bool ok = false;
            QString current = QString::number(id);
            QString text = QInputDialog::getText(
                nullptr,
                tr("Change Waypoint ID"),
                tr("Enter new waypoint ID:"),
                QLineEdit::Normal,
                current,
                &ok);

            if (!ok) {
                break;
            }

            const int insert_id = text.toInt();
            if (insert_id < 0 || insert_id >= static_cast<int>(waypoint_sequence_.size())) {
                const int max_index = std::max(0, static_cast<int>(waypoint_sequence_.size()) - 1);
                QMessageBox::warning(
                    nullptr,
                    tr("Invalid Waypoint ID"),
                    tr("Waypoint ID %1 is out of range.\nPlease enter a value between %2 and %3.")
                        .arg(insert_id)
                        .arg(0)
                        .arg(max_index));
                break;
            }

            stopNavigationExecution(true, NAV_IDLE, "IDLE");
            Waypoint waypoint = waypoint_sequence_.at(static_cast<std::size_t>(id));
            waypoint_sequence_.eraseWaypoint(static_cast<std::size_t>(id));
            waypoint_sequence_.insertWaypoint(static_cast<std::size_t>(insert_id), std::move(waypoint));
            updateWaypointMarker();
            commitWaypointChanges(insert_id);
            ROS_INFO("Changed waypoint id %d to %d", id, insert_id);
            break;
        }

        case 3:
        {
            bool ok = false;
            QString current = QString::fromStdString(waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command);
            QString text = QInputDialog::getText(
                nullptr,
                tr("Edit Function Command"),
                tr("Enter command for waypoint ID: %1").arg(id),
                QLineEdit::Normal,
                current,
                &ok);

            if (!ok) {
                break;
            }

            waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command = text.toStdString();
            updateWaypointMarker();
            commitWaypointChanges(id);
            ROS_INFO("Updated command of waypoint %d to '%s'", id, waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command.c_str());
            break;
        }

        default:
            break;
    }
}

double WaypointEditorTool::computeSegmentDistance(std::size_t first, std::size_t second) const
{
    if (first >= waypoint_sequence_.size() || second >= waypoint_sequence_.size()) {
        return 0.0;
    }
    const geometry_msgs::Point &p0 = waypoint_sequence_.at(first).pose.pose.position;
    const geometry_msgs::Point &p1 = waypoint_sequence_.at(second).pose.pose.position;
    return std::hypot(p1.x - p0.x, p1.y - p0.y);
}

double WaypointEditorTool::computeDistanceToWaypoint(
    const geometry_msgs::PoseStamped &pose,
    const std::size_t waypoint_index) const
{
    if (waypoint_index >= waypoint_sequence_.size()) {
        return std::numeric_limits<double>::infinity();
    }

    const geometry_msgs::Point &robot = pose.pose.position;
    const geometry_msgs::Point &target = waypoint_sequence_.at(waypoint_index).pose.pose.position;
    return std::hypot(target.x - robot.x, target.y - robot.y);
}

void WaypointEditorTool::updateNavigationProgress(const geometry_msgs::PoseStamped &robot_pose)
{
    if (waypoint_sequence_.empty() || navigation_index_ >= waypoint_sequence_.size()) {
        return;
    }

    std::size_t updated_index = navigation_index_;
    while (updated_index + 1 < waypoint_sequence_.size()) {
        const double current_distance = computeDistanceToWaypoint(robot_pose, updated_index);
        const double next_distance = computeDistanceToWaypoint(robot_pose, updated_index + 1);
        const bool close_enough_to_next = next_distance <= kNavigationProgressReachDistance;
        const bool clearly_closer_to_next = next_distance + kNavigationProgressHysteresis < current_distance;
        if (!close_enough_to_next && !clearly_closer_to_next) {
            break;
        }
        ++updated_index;
    }

    navigation_index_ = std::min(updated_index, navigation_goal_index_);
}

void WaypointEditorTool::updateLastDistanceFromWaypoint(int waypoint_index)
{
    const std::size_t count = waypoint_sequence_.size();
    if (count < 2) {
        last_displayed_distance_ = 0.0;
        return;
    }

    const int max_index = static_cast<int>(count) - 1;
    const int clamped = std::max(0, std::min(waypoint_index, max_index));
    const std::size_t idx = static_cast<std::size_t>(clamped);

    if (idx > 0) {
        last_displayed_distance_ = computeSegmentDistance(idx - 1, idx);
    } else {
        last_displayed_distance_ = computeSegmentDistance(idx, idx + 1);
    }
}

void WaypointEditorTool::publishLineMarker()
{
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "waypoint_lines";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.025;
    line.color.r = 0.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    for (std::size_t i = 1; i < waypoint_sequence_.size(); ++i) {
        line.points.push_back(waypoint_sequence_.at(i - 1).pose.pose.position);
        line.points.push_back(waypoint_sequence_.at(i).pose.pose.position);
    }

    line_pub_.publish(line);
}

void WaypointEditorTool::publishWaypointPath()
{
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    if (!waypoint_sequence_.empty()) {
        const std::string frame_id = waypoint_sequence_.at(0).pose.header.frame_id.empty()
            ? "map"
            : waypoint_sequence_.at(0).pose.header.frame_id;
        path.header.frame_id = frame_id;

        path.poses.reserve(waypoint_sequence_.size());
        for (std::size_t i = 0; i < waypoint_sequence_.size(); ++i) {
            geometry_msgs::PoseStamped pose = waypoint_sequence_.at(i).pose;
            pose.header.frame_id = pose.header.frame_id.empty() ? frame_id : pose.header.frame_id;
            pose.header.stamp = path.header.stamp;
            path.poses.push_back(std::move(pose));
        }
    }

    waypoint_path_pub_.publish(path);
}

void WaypointEditorTool::publishTotalWpsDist()
{
    std_msgs::Float64 msg;
    msg.data = waypoint_sequence_.totalDistance();
    total_wp_dist_pub_.publish(msg);
}

void WaypointEditorTool::publishLastWpsDist()
{
    std_msgs::Float64 msg;
    msg.data = last_displayed_distance_;
    last_wp_dist_pub_.publish(msg);
}

void WaypointEditorTool::publishRangeMetrics()
{
    publishLineMarker();
    publishTotalWpsDist();
    publishLastWpsDist();
}

void WaypointEditorTool::commitWaypointChanges(int waypoint_index, bool snapshot_history)
{
    if (snapshot_history) {
        waypoint_sequence_.snapshotHistory();
    }
    pose_dirty_ = false;
    updateLastDistanceFromWaypoint(waypoint_index);
    publishRangeMetrics();
    publishWaypointPath();
    publishNavigationState();
}

double WaypointEditorTool::computeRemainingRouteDistanceLowerBound() const
{
    if (waypoint_sequence_.empty()) {
        return -1.0;
    }
    if (navigation_state_ == NAV_IDLE) {
        return -1.0;
    }
    if (navigation_state_ == NAV_FINISHED) {
        return 0.0;
    }
    if (navigation_index_ >= waypoint_sequence_.size()) {
        return -1.0;
    }

    double remaining_distance = 0.0;
    for (std::size_t idx = navigation_index_ + 1; idx < waypoint_sequence_.size(); ++idx) {
        remaining_distance += computeSegmentDistance(idx - 1, idx);
    }
    return remaining_distance;
}

ros::Duration WaypointEditorTool::currentNavigationElapsed() const
{
    ros::Duration elapsed = navigation_elapsed_;
    if (navigation_state_ == NAV_FOLLOWING && !navigation_resume_started_at_.isZero()) {
        elapsed += ros::Time::now() - navigation_resume_started_at_;
    }
    return elapsed;
}

void WaypointEditorTool::resetNavigationMetrics()
{
    navigation_elapsed_ = ros::Duration(0.0);
    navigation_resume_started_at_ = ros::Time();
}

bool WaypointEditorTool::isValidWaypointId(int id) const
{
    return id >= 0 && id < static_cast<int>(waypoint_sequence_.size());
}

bool WaypointEditorTool::requestFilePathForSaving(std::string &path, SaveFormat &format)
{
    QString qpath;
    QString selected_filter;
    bool accepted = false;

    QMetaObject::invokeMethod(qApp, [&]() {
        qpath = QFileDialog::getSaveFileName(
            nullptr,
            tr("Save Waypoints As"),
            QString(),
            tr("CSV Files (*.csv);;YAML Files (*.yaml);;Navigation YAML (*.nav.yaml)"),
            &selected_filter);
        accepted = !qpath.isEmpty();
    }, Qt::BlockingQueuedConnection);

    if (!accepted) {
        return false;
    }

    const QString lower = qpath.toLower();
    if (lower.endsWith(".nav.yaml")) {
        format = FORMAT_NAV_YAML;
    } else if (lower.endsWith(".yaml")) {
        if (selected_filter.contains("Navigation", Qt::CaseInsensitive)) {
            qpath = qpath.left(qpath.lastIndexOf('.')) + ".nav.yaml";
            format = FORMAT_NAV_YAML;
        } else {
            format = FORMAT_YAML;
        }
    } else if (lower.endsWith(".csv")) {
        format = FORMAT_CSV;
    } else if (selected_filter.contains("Navigation", Qt::CaseInsensitive)) {
        qpath += ".nav.yaml";
        format = FORMAT_NAV_YAML;
    } else if (selected_filter.contains("yaml", Qt::CaseInsensitive)) {
        qpath += ".yaml";
        format = FORMAT_YAML;
    } else {
        qpath += ".csv";
        format = FORMAT_CSV;
    }

    path = qpath.toStdString();
    return true;
}

bool WaypointEditorTool::requestFilePathForLoading(std::string &path, bool &load_yaml)
{
    QString qpath;
    QString selected_filter;
    bool accepted = false;

    QMetaObject::invokeMethod(qApp, [&]() {
        qpath = QFileDialog::getOpenFileName(
            nullptr,
            tr("Open Waypoints"),
            QString(),
            tr("CSV Files (*.csv);;YAML Files (*.yaml *.nav.yaml)"),
            &selected_filter);
        accepted = !qpath.isEmpty();
    }, Qt::BlockingQueuedConnection);

    if (!accepted) {
        return false;
    }

    const QString lower = qpath.toLower();
    if (lower.endsWith(".yaml") || lower.endsWith(".nav.yaml")) {
        load_yaml = true;
    } else if (lower.endsWith(".csv")) {
        load_yaml = false;
    } else {
        load_yaml = selected_filter.contains("yaml", Qt::CaseInsensitive);
    }

    path = qpath.toStdString();
    return true;
}

bool WaypointEditorTool::handleSaveWaypoints(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    std::string path;
    SaveFormat format = FORMAT_CSV;
    if (!requestFilePathForSaving(path, format)) {
        res.success = false;
        res.message = "Save canceled by user";
        return true;
    }

    std::string error;
    bool ok = false;
    switch (format) {
        case FORMAT_YAML:
            ok = io::WaypointYaml::Save(waypoint_sequence_.waypoints(), path, error);
            break;
        case FORMAT_NAV_YAML:
            ok = io::WaypointNavYaml::Save(waypoint_sequence_.waypoints(), path, error);
            break;
        case FORMAT_CSV:
        default:
            ok = io::WaypointCsv::Save(waypoint_sequence_.waypoints(), path, error);
            break;
    }

    if (!ok) {
        const QString error_path = QString::fromStdString(path);
        QMetaObject::invokeMethod(qApp, [error_path]() {
            QMessageBox::warning(nullptr, QObject::tr("Error"), QObject::tr("Cannot save file:\n%1").arg(error_path));
        }, Qt::BlockingQueuedConnection);
        res.success = false;
        res.message = error.empty() ? "Failed to save waypoint file" : error;
        return true;
    }

    res.success = true;
    res.message = "Saved " + std::to_string(waypoint_sequence_.size()) + " waypoints to " + path;
    return true;
}

bool WaypointEditorTool::handleLoadWaypoints(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    std::string path;
    bool load_yaml = false;
    if (!requestFilePathForLoading(path, load_yaml)) {
        res.success = false;
        res.message = "Load canceled by user";
        return true;
    }

    std::vector<Waypoint> loaded;
    std::string error;
    if (!loadWaypointsFromPath(path, loaded, error)) {
        const QString error_path = QString::fromStdString(path);
        const QString error_text = QString::fromStdString(error.empty() ? "Failed to load waypoint file" : error);
        QMetaObject::invokeMethod(qApp, [error_path, error_text]() {
            QMessageBox::warning(nullptr, QObject::tr("Error"), QObject::tr("Cannot open file:\n%1\n\nReason: %2").arg(error_path, error_text));
        }, Qt::BlockingQueuedConnection);
        res.success = false;
        res.message = error.empty() ? "Failed to load waypoint file" : error;
        return true;
    }

    applyLoadedWaypoints(std::move(loaded));

    res.success = true;
    res.message = "Loaded " + std::to_string(waypoint_sequence_.size()) + " waypoints from " + path;
    return true;
}

bool WaypointEditorTool::handleUndoWaypoints(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    if (!waypoint_sequence_.undo()) {
        publishNavigationState();
        res.success = false;
        res.message = "No more actions to undo";
        return true;
    }

    stopNavigationExecution(true, NAV_IDLE, "IDLE");
    pose_dirty_ = false;
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1, false);
    res.success = true;
    res.message = "Undid waypoint change";
    return true;
}

bool WaypointEditorTool::handleRedoWaypoints(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    if (!waypoint_sequence_.redo()) {
        publishNavigationState();
        res.success = false;
        res.message = "No more actions to redo";
        return true;
    }

    stopNavigationExecution(true, NAV_IDLE, "IDLE");
    pose_dirty_ = false;
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1, false);
    res.success = true;
    res.message = "Redid waypoint change";
    return true;
}

bool WaypointEditorTool::handleClearWaypoints(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    stopNavigationExecution(true, NAV_IDLE, "IDLE");
    waypoint_sequence_.clear();
    pose_dirty_ = false;
    updateWaypointMarker();
    publishRangeMetrics();
    publishWaypointPath();
    publishNavigationState();
    res.success = true;
    res.message = "Cleared all waypoints";
    return true;
}

bool WaypointEditorTool::handleStartNavigation(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    if (waypoint_sequence_.empty()) {
        res.success = false;
        res.message = "No waypoints to navigate";
        return true;
    }

    if (!move_base_client_ || !move_base_client_->waitForServer(ros::Duration(5.0))) {
        navigation_state_ = NAV_ERROR;
        navigation_action_status_ = "MOVE_BASE_UNAVAILABLE";
        publishNavigationState();
        res.success = false;
        res.message = "move_base action server not available";
        return true;
    }

    stopNavigationExecution(true, NAV_IDLE, "IDLE");
    resetNavigationMetrics();
    navigation_index_ = 0;
    navigation_goal_index_ = waypoint_sequence_.size() - 1;
    navigation_state_ = NAV_FOLLOWING;
    navigation_action_status_ = "STARTING";
    navigation_resume_started_at_ = ros::Time::now();
    navigation_timer_.start();
    sendNavigationGoal(navigation_goal_index_);
    publishNavigationState();

    res.success = true;
    res.message = "Started waypoint navigation";
    return true;
}

bool WaypointEditorTool::handlePauseNavigation(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    if (navigation_state_ != NAV_FOLLOWING) {
        res.success = false;
        res.message = "Navigation is not running";
        return true;
    }

    stopNavigationExecution(true, NAV_PAUSED, "PAUSED");
    res.success = true;
    res.message = "Paused waypoint navigation";
    return true;
}

bool WaypointEditorTool::handleResumeNavigation(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    if (navigation_state_ != NAV_PAUSED) {
        res.success = false;
        res.message = "Navigation is not paused";
        return true;
    }
    if (waypoint_sequence_.empty()) {
        res.success = false;
        res.message = "No remaining waypoint to resume";
        return true;
    }
    if (!move_base_client_ || !move_base_client_->waitForServer(ros::Duration(5.0))) {
        navigation_state_ = NAV_ERROR;
        navigation_action_status_ = "MOVE_BASE_UNAVAILABLE";
        publishNavigationState();
        res.success = false;
        res.message = "move_base action server not available";
        return true;
    }

    navigation_goal_index_ = waypoint_sequence_.size() - 1;
    navigation_state_ = NAV_FOLLOWING;
    navigation_action_status_ = "RESUMING";
    navigation_resume_started_at_ = ros::Time::now();
    navigation_timer_.start();
    sendNavigationGoal(navigation_goal_index_);
    publishNavigationState();

    res.success = true;
    res.message = "Resumed waypoint navigation";
    return true;
}

bool WaypointEditorTool::handleStopNavigation(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    stopNavigationExecution(true, NAV_IDLE, "IDLE");
    res.success = true;
    res.message = "Stopped waypoint navigation";
    return true;
}

void WaypointEditorTool::processNavigationTimer(const ros::TimerEvent &)
{
    if (navigation_state_ != NAV_FOLLOWING || !move_base_client_) {
        return;
    }

    const actionlib::SimpleClientGoalState state = move_base_client_->getState();
    navigation_action_status_ = state.toString();

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        if (!waypoint_sequence_.empty()) {
            navigation_index_ = waypoint_sequence_.size() - 1;
        }
        stopNavigationExecution(false, NAV_FINISHED, state.toString());
        return;
    }

    if (state == actionlib::SimpleClientGoalState::ABORTED ||
        state == actionlib::SimpleClientGoalState::REJECTED ||
        state == actionlib::SimpleClientGoalState::LOST)
    {
        stopNavigationExecution(false, NAV_ERROR, state.toString());
        return;
    }

    publishNavigationState();
}

void WaypointEditorTool::refreshPublishedState(const ros::TimerEvent &)
{
    if (navigation_state_ != NAV_FOLLOWING) {
        publishNavigationState();
    }
}

void WaypointEditorTool::sendNavigationGoal(std::size_t waypoint_index)
{
    if (!move_base_client_ || waypoint_index >= waypoint_sequence_.size()) {
        return;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = waypoint_sequence_.at(waypoint_index).pose;
    goal.target_pose.header.stamp = ros::Time::now();
    if (goal.target_pose.header.frame_id.empty()) {
        goal.target_pose.header.frame_id = "map";
    }

    move_base_client_->sendGoal(
        goal,
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleDoneCallback(),
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
        boost::bind(&WaypointEditorTool::handleNavigationFeedback, this, ph::_1));
    navigation_action_status_ = "PENDING";
    publishNavigationState();

    ROS_INFO(
        "WaypointEditorTool sent final goal %zu/%zu: (%.3f, %.3f)",
        waypoint_index + 1,
        waypoint_sequence_.size(),
        goal.target_pose.pose.position.x,
        goal.target_pose.pose.position.y);
}

void WaypointEditorTool::handleNavigationFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    if (feedback == nullptr || navigation_state_ != NAV_FOLLOWING) {
        return;
    }

    updateNavigationProgress(feedback->base_position);
    publishNavigationState();
}

void WaypointEditorTool::stopNavigationExecution(bool cancel_goal, NavigationState next_state, const std::string &action_status)
{
    if (navigation_state_ == NAV_FOLLOWING && !navigation_resume_started_at_.isZero()) {
        navigation_elapsed_ += ros::Time::now() - navigation_resume_started_at_;
        navigation_resume_started_at_ = ros::Time();
    }

    if (navigation_timer_.hasStarted()) {
        navigation_timer_.stop();
    }
    if (cancel_goal && move_base_client_) {
        move_base_client_->cancelAllGoals();
    }

    navigation_state_ = next_state;
    navigation_action_status_ = action_status;
    if (next_state == NAV_IDLE) {
        navigation_index_ = 0;
        navigation_goal_index_ = 0;
        resetNavigationMetrics();
    }
    publishNavigationState();
}

void WaypointEditorTool::publishNavigationState()
{
    std_msgs::Int32 waypoint_count_msg;
    waypoint_count_msg.data = static_cast<int>(waypoint_sequence_.size());
    waypoint_count_pub_.publish(waypoint_count_msg);

    std_msgs::Bool ready_msg;
    ready_msg.data = isNavigationBackendReady();
    navigation_ready_pub_.publish(ready_msg);

    std_msgs::Bool undo_available_msg;
    undo_available_msg.data = waypoint_sequence_.canUndo();
    undo_available_pub_.publish(undo_available_msg);

    std_msgs::Bool redo_available_msg;
    redo_available_msg.data = waypoint_sequence_.canRedo();
    redo_available_pub_.publish(redo_available_msg);

    std_msgs::String nav_msg;
    nav_msg.data = navigationStateToString();
    nav_status_pub_.publish(nav_msg);

    std_msgs::String action_msg;
    action_msg.data = navigation_action_status_.empty() ? "IDLE" : navigation_action_status_;
    nav_action_status_pub_.publish(action_msg);

    std_msgs::Float64 elapsed_msg;
    elapsed_msg.data = navigation_state_ == NAV_IDLE ? -1.0 : std::max(0.0, currentNavigationElapsed().toSec());
    nav_elapsed_seconds_pub_.publish(elapsed_msg);

    std_msgs::Float64 remaining_distance_msg;
    remaining_distance_msg.data = computeRemainingRouteDistanceLowerBound();
    nav_remaining_distance_pub_.publish(remaining_distance_msg);

    std_msgs::Int32 index_msg;
    const bool has_waypoints = !waypoint_sequence_.empty();
    const bool has_active_target = has_waypoints && navigation_state_ != NAV_IDLE;
    std::size_t capped = 0;
    if (has_active_target) {
        capped = std::min(navigation_index_, waypoint_sequence_.size() - 1);
        index_msg.data = static_cast<int>(capped);
    } else {
        index_msg.data = -1;
    }
    nav_index_pub_.publish(index_msg);

    std_msgs::String target_msg;
    if (!has_waypoints) {
        target_msg.data = "WP - / 0";
    } else if (!has_active_target) {
        target_msg.data = "WP - / " + std::to_string(waypoint_sequence_.size());
    } else {
        target_msg.data = "WP " + std::to_string(capped + 1) + " / " + std::to_string(waypoint_sequence_.size());
    }
    nav_target_pub_.publish(target_msg);
}

bool WaypointEditorTool::isNavigationBackendReady()
{
    return move_base_client_ && move_base_client_->isServerConnected();
}

std::string WaypointEditorTool::navigationStateToString() const
{
    switch (navigation_state_) {
        case NAV_IDLE:
            return "IDLE";
        case NAV_FOLLOWING:
            return "FOLLOWING";
        case NAV_PAUSED:
            return "PAUSED";
        case NAV_FINISHED:
            return "FINISHED";
        case NAV_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

} // namespace waypoint_editor

PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorTool, rviz::Tool)
