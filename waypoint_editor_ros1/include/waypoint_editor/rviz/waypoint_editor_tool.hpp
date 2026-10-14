#ifndef WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_
#define WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <memory>
#include <string>

#include "waypoint_editor/core/waypoint_sequence.hpp"

namespace waypoint_editor
{

class WaypointEditorTool : public rviz::PoseTool
{
public:
    enum NavigationState { NAV_IDLE, NAV_FOLLOWING, NAV_PAUSED, NAV_FINISHED, NAV_ERROR };

    WaypointEditorTool();
    ~WaypointEditorTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    void onPoseSet(double x, double y, double theta) override;
    void updateWaypointMarker();
    visualization_msgs::InteractiveMarker createWaypointMarker(int id);
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb);
    void processMenuControl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb);

    bool handleSaveWaypoints(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool handleLoadWaypoints(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool handleUndoWaypoints(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool handleRedoWaypoints(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool handleClearWaypoints(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool handleStartNavigation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool handlePauseNavigation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool handleResumeNavigation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool handleStopNavigation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void publishLineMarker();
    void publishWaypointPath();
    void publishTotalWpsDist();
    void publishLastWpsDist();
    void publishRangeMetrics();

    enum SaveFormat { FORMAT_CSV, FORMAT_YAML, FORMAT_NAV_YAML };
    bool requestFilePathForSaving(std::string &path, SaveFormat &format);
    bool requestFilePathForLoading(std::string &path, bool &load_yaml);

private:
    bool loadWaypointsFromPath(const std::string &path, std::vector<Waypoint> &loaded, std::string &error) const;
    bool importConfiguredWaypointFile();
    int appendWaypointAndRefresh(Waypoint wp);
    void processNavigationTimer(const ros::TimerEvent &event);
    void sendNavigationGoal(std::size_t waypoint_index);
    void handleNavigationFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
    void stopNavigationExecution(bool cancel_goal, NavigationState next_state, const std::string &action_status);
    void publishNavigationState();
    void refreshPublishedState(const ros::TimerEvent &event);
    double computeRemainingRouteDistanceLowerBound() const;
    ros::Duration currentNavigationElapsed() const;
    void resetNavigationMetrics();
    std::string navigationStateToString() const;
    bool isNavigationBackendReady();
    double computeDistanceToWaypoint(const geometry_msgs::PoseStamped &pose, std::size_t waypoint_index) const;
    void updateNavigationProgress(const geometry_msgs::PoseStamped &robot_pose);
    void applyLoadedWaypoints(std::vector<Waypoint> loaded);

    ros::CallbackQueue service_queue_;
    std::shared_ptr<ros::AsyncSpinner> service_spinner_;

    ros::NodeHandle nh_;
    ros::NodeHandle srv_nh_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    ros::Publisher line_pub_;
    ros::Publisher waypoint_path_pub_;
    ros::Publisher total_wp_dist_pub_;
    ros::Publisher last_wp_dist_pub_;
    ros::Publisher waypoint_count_pub_;
    ros::Publisher navigation_ready_pub_;
    ros::Publisher undo_available_pub_;
    ros::Publisher redo_available_pub_;
    ros::Publisher nav_status_pub_;
    ros::Publisher nav_action_status_pub_;
    ros::Publisher nav_index_pub_;
    ros::Publisher nav_target_pub_;
    ros::Publisher nav_elapsed_seconds_pub_;
    ros::Publisher nav_remaining_distance_pub_;
    ros::ServiceServer save_service_;
    ros::ServiceServer load_service_;
    ros::ServiceServer undo_service_;
    ros::ServiceServer redo_service_;
    ros::ServiceServer clear_service_;
    ros::ServiceServer start_navigation_service_;
    ros::ServiceServer pause_navigation_service_;
    ros::ServiceServer resume_navigation_service_;
    ros::ServiceServer stop_navigation_service_;
    ros::Timer navigation_timer_;
    ros::Timer state_publish_timer_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;

    WaypointSequence waypoint_sequence_;
    bool pose_dirty_;
    double last_displayed_distance_;
    NavigationState navigation_state_;
    std::size_t navigation_index_;
    std::size_t navigation_goal_index_;
    std::string navigation_action_status_;
    ros::Duration navigation_elapsed_;
    ros::Time navigation_resume_started_at_;

    void updateLastDistanceFromWaypoint(int waypoint_index);
    double computeSegmentDistance(std::size_t first, std::size_t second) const;
    void commitWaypointChanges(int waypoint_index, bool snapshot_history = true);
    bool isValidWaypointId(int id) const;
};

} // namespace waypoint_editor

#endif // WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_
