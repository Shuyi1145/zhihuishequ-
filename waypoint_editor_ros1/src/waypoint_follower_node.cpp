#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <string>
#include <vector>

#include "waypoint_editor/core/waypoint.hpp"
#include "waypoint_editor/io/waypoint_nav_yaml.hpp"

namespace waypoint_editor
{

class WaypointFollower
{
public:
    enum State { IDLE, FOLLOWING, PAUSED, FINISHED };

    WaypointFollower()
      : nh_("~"),
        ac_("move_base", true),
        state_(IDLE),
        current_index_(0),
        loop_(false)
    {
        // Parameters
        nh_.param<std::string>("waypoint_file", waypoint_file_, "");
        nh_.param<bool>("loop", loop_, false);

        // Publishers
        index_pub_ = nh_.advertise<std_msgs::Int32>("current_index", 1, true);
        status_pub_ = nh_.advertise<std_msgs::String>("status", 1, true);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 1, true);

        // Services
        start_srv_ = nh_.advertiseService("start", &WaypointFollower::handleStart, this);
        pause_srv_ = nh_.advertiseService("pause", &WaypointFollower::handlePause, this);
        resume_srv_ = nh_.advertiseService("resume", &WaypointFollower::handleResume, this);
        stop_srv_ = nh_.advertiseService("stop", &WaypointFollower::handleStop, this);

        publishStatus();

        // Load waypoints if file specified
        if (!waypoint_file_.empty()) {
            loadWaypoints();
        }

        ROS_INFO("WaypointFollower initialized. Waiting for start service call...");
    }

    void spin()
    {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();

            if (state_ == FOLLOWING) {
                processFollowing();
            }

            rate.sleep();
        }
    }

private:
    bool loadWaypoints()
    {
        std::string error;
        if (!io::WaypointNavYaml::Load(waypoint_file_, waypoints_, error)) {
            ROS_ERROR("Failed to load waypoints: %s", error.c_str());
            return false;
        }
        ROS_INFO("Loaded %zu waypoints from %s", waypoints_.size(), waypoint_file_.c_str());
        return true;
    }

    bool handleStart(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res)
    {
        if (waypoints_.empty()) {
            if (waypoint_file_.empty()) {
                res.success = false;
                res.message = "No waypoint file specified";
                return true;
            }
            if (!loadWaypoints()) {
                res.success = false;
                res.message = "Failed to load waypoints";
                return true;
            }
        }

        if (waypoints_.empty()) {
            res.success = false;
            res.message = "No waypoints loaded";
            return true;
        }

        ROS_INFO("Waiting for move_base action server...");
        if (!ac_.waitForServer(ros::Duration(5.0))) {
            res.success = false;
            res.message = "move_base action server not available";
            return true;
        }

        current_index_ = 0;
        state_ = FOLLOWING;
        sendGoal(current_index_);
        publishStatus();

        res.success = true;
        res.message = "Started waypoint following";
        ROS_INFO("Started waypoint following with %zu waypoints", waypoints_.size());
        return true;
    }

    bool handlePause(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res)
    {
        if (state_ != FOLLOWING) {
            res.success = false;
            res.message = "Not currently following";
            return true;
        }

        state_ = PAUSED;
        ac_.cancelGoal();
        publishStatus();

        res.success = true;
        res.message = "Paused";
        ROS_INFO("Paused waypoint following");
        return true;
    }

    bool handleResume(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res)
    {
        if (state_ != PAUSED) {
            res.success = false;
            res.message = "Not paused";
            return true;
        }

        state_ = FOLLOWING;
        sendGoal(current_index_);
        publishStatus();

        res.success = true;
        res.message = "Resumed";
        ROS_INFO("Resumed waypoint following");
        return true;
    }

    bool handleStop(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response &res)
    {
        if (state_ == IDLE) {
            res.success = false;
            res.message = "Already idle";
            return true;
        }

        ac_.cancelAllGoals();
        state_ = IDLE;
        current_index_ = 0;
        publishStatus();

        res.success = true;
        res.message = "Stopped";
        ROS_INFO("Stopped waypoint following");
        return true;
    }

    void sendGoal(size_t index)
    {
        if (index >= waypoints_.size()) {
            return;
        }

        const Waypoint &wp = waypoints_[index];

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = wp.pose;
        if (goal.target_pose.header.stamp.isZero()) {
            goal.target_pose.header.stamp = ros::Time::now();
        }
        if (goal.target_pose.header.frame_id.empty()) {
            goal.target_pose.header.frame_id = "map";
        }

        ROS_INFO("Sending goal %zu/%zu: (%.2f, %.2f)",
                 index + 1, waypoints_.size(),
                 wp.pose.pose.position.x, wp.pose.pose.position.y);

        ac_.sendGoal(goal);

        // Publish current goal
        goal_pub_.publish(goal.target_pose);

        // Publish index
        std_msgs::Int32 idx_msg;
        idx_msg.data = static_cast<int>(index);
        index_pub_.publish(idx_msg);
    }

    void processFollowing()
    {
        if (current_index_ >= waypoints_.size()) {
            return;
        }

        actionlib::SimpleClientGoalState goal_state = ac_.getState();

        if (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Reached waypoint %zu/%zu", current_index_ + 1, waypoints_.size());

            // Process command
            const std::string &cmd = waypoints_[current_index_].function_command;
            if (!processCommand(cmd)) {
                return;  // Command handling (e.g., stop) changed state
            }

            // Move to next waypoint
            current_index_++;
            if (current_index_ >= waypoints_.size()) {
                if (loop_) {
                    ROS_INFO("Looping back to first waypoint");
                    current_index_ = 0;
                    sendGoal(current_index_);
                    publishStatus();
                } else {
                    state_ = FINISHED;
                    publishStatus();
                    ROS_INFO("All waypoints reached!");
                }
            } else {
                sendGoal(current_index_);
                publishStatus();
            }
        } else if (goal_state == actionlib::SimpleClientGoalState::ABORTED ||
                   goal_state == actionlib::SimpleClientGoalState::REJECTED) {
            ROS_WARN("Goal %zu was %s. Moving to next waypoint.",
                     current_index_ + 1,
                     goal_state == actionlib::SimpleClientGoalState::ABORTED ? "aborted" : "rejected");

            current_index_++;
            if (current_index_ >= waypoints_.size()) {
                if (loop_) {
                    current_index_ = 0;
                    sendGoal(current_index_);
                } else {
                    state_ = FINISHED;
                    ROS_INFO("All waypoints processed (some failed).");
                }
            } else {
                sendGoal(current_index_);
            }
            publishStatus();
        }
        // ACTIVE or PENDING: still navigating, do nothing
    }

    bool processCommand(const std::string &cmd)
    {
        if (cmd.empty()) {
            return true;
        }

        // "wait:N" - wait N seconds
        if (cmd.substr(0, 5) == "wait:") {
            double seconds = 0.0;
            try {
                seconds = std::stod(cmd.substr(5));
            } catch (const std::exception &) {
                ROS_WARN("Invalid wait command: %s", cmd.c_str());
                return true;
            }
            ROS_INFO("Waiting %.1f seconds at waypoint %zu", seconds, current_index_ + 1);
            ros::Duration(seconds).sleep();
            return true;
        }

        // "stop" - stop following
        if (cmd == "stop") {
            ROS_INFO("Stop command at waypoint %zu", current_index_ + 1);
            state_ = FINISHED;
            publishStatus();
            return false;
        }

        ROS_WARN("Unknown command: %s", cmd.c_str());
        return true;
    }

    void publishStatus()
    {
        std_msgs::String msg;
        switch (state_) {
            case IDLE:      msg.data = "IDLE"; break;
            case FOLLOWING: msg.data = "FOLLOWING"; break;
            case PAUSED:    msg.data = "PAUSED"; break;
            case FINISHED:  msg.data = "FINISHED"; break;
        }
        status_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    State state_;
    size_t current_index_;
    bool loop_;
    std::string waypoint_file_;
    std::vector<Waypoint> waypoints_;

    ros::Publisher index_pub_;
    ros::Publisher status_pub_;
    ros::Publisher goal_pub_;

    ros::ServiceServer start_srv_;
    ros::ServiceServer pause_srv_;
    ros::ServiceServer resume_srv_;
    ros::ServiceServer stop_srv_;
};

}  // namespace waypoint_editor

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_follower");
    waypoint_editor::WaypointFollower follower;
    follower.spin();
    return 0;
}
