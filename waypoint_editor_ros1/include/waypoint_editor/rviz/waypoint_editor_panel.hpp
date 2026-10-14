#ifndef WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_PANEL_HPP_
#define WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_PANEL_HPP_

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rviz/panel.h>

#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <memory>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace waypoint_editor
{

class WaypointEditorPanel : public rviz::Panel
{
    Q_OBJECT

public:
    explicit WaypointEditorPanel(QWidget *parent = nullptr);
    ~WaypointEditorPanel() override;

    void onInitialize() override;
    void load(const rviz::Config &config) override;
    void save(rviz::Config config) const override;

protected Q_SLOTS:
    void onLoad2DMapButtonClick();
    void onLoad3DMapButtonClick();
    void onLoadWaypointsButtonClick();
    void onSaveWaypointsButtonClick();
    void onUndoWaypointsButtonClick();
    void onRedoWaypointsButtonClick();
    void onClearWaypointsButtonClick();
    void onStartNavigationButtonClick();
    void onPauseNavigationButtonClick();
    void onResumeNavigationButtonClick();
    void onStopNavigationButtonClick();

private Q_SLOTS:
    void applyStatusMessage(const QString &msg);
    void setWaypointCountValue(int count);
    void setNavigationReadyValue(bool ready);
    void setUndoAvailableValue(bool available);
    void setRedoAvailableValue(bool available);
    void setNavigationStateValue(const QString &state);
    void setNavigationActionStatusValue(const QString &status);
    void setNavigationTargetValue(const QString &target);
    void setNavigationIndexValue(int index);
    void setNavigationElapsedSeconds(double seconds);
    void setNavigationRemainingDistance(double distance);
    void refreshUiState();

private:
    void initializeRosInterfaces();
    void postStatusMessage(const QString &msg);
    void callTriggerService(ros::ServiceClient &client, const QString &success_msg, const QString &fail_msg);
    void callTriggerServiceAsync(ros::ServiceClient &client, const QString &success_msg, const QString &fail_msg);
    void updateSummaryLabels();
    void updateButtonStates();
    bool serviceAvailable(ros::ServiceClient &client) const;
    bool isNavigationActive() const;
    bool canStartNavigation() const;
    bool canPauseNavigation() const;
    bool canResumeNavigation() const;
    bool canStopNavigation() const;

    QLabel *status_value_label_;
    QLabel *waypoint_count_value_label_;
    QLabel *edit_state_value_label_;
    QLabel *nav_status_value_label_;
    QLabel *localization_value_label_;
    QLabel *nav_action_value_label_;
    QLabel *nav_target_value_label_;
    QLabel *poses_remaining_value_label_;
    QLabel *eta_value_label_;
    QLabel *distance_remaining_value_label_;
    QLabel *time_taken_value_label_;
    QLabel *recoveries_value_label_;
    QVBoxLayout *layout_;
    QPushButton *load_2d_map_button_;
    QPushButton *load_3d_map_button_;
    QPushButton *load_waypoints_button_;
    QPushButton *save_waypoints_button_;
    QPushButton *undo_button_;
    QPushButton *redo_button_;
    QPushButton *clear_button_;
    QPushButton *start_navigation_button_;
    QPushButton *pause_navigation_button_;
    QPushButton *resume_navigation_button_;
    QPushButton *stop_navigation_button_;
    QTimer *service_refresh_timer_;

    ros::NodeHandle nh_;
    ros::NodeHandle sub_nh_;
    ros::CallbackQueue callback_queue_;
    std::shared_ptr<ros::AsyncSpinner> callback_spinner_;
    bool ros_initialized_{false};
    ros::ServiceClient load_client_;
    ros::ServiceClient save_client_;
    ros::ServiceClient undo_client_;
    ros::ServiceClient redo_client_;
    ros::ServiceClient clear_client_;
    ros::ServiceClient start_navigation_client_;
    ros::ServiceClient pause_navigation_client_;
    ros::ServiceClient resume_navigation_client_;
    ros::ServiceClient stop_navigation_client_;
    ros::Subscriber waypoint_count_sub_;
    ros::Subscriber navigation_ready_sub_;
    ros::Subscriber undo_available_sub_;
    ros::Subscriber redo_available_sub_;
    ros::Subscriber nav_status_sub_;
    ros::Subscriber nav_action_status_sub_;
    ros::Subscriber nav_target_sub_;
    ros::Subscriber nav_index_sub_;
    ros::Subscriber nav_elapsed_seconds_sub_;
    ros::Subscriber nav_remaining_distance_sub_;

    QString status_message_{"Start Waypoint Editor!"};
    QString navigation_state_{"IDLE"};
    QString navigation_action_status_{"IDLE"};
    QString navigation_target_{"WP - / 0"};
    int waypoint_count_{0};
    int navigation_index_{-1};
    double navigation_elapsed_seconds_{-1.0};
    double navigation_remaining_distance_{-1.0};
    bool navigation_ready_{false};
    bool undo_available_{false};
    bool redo_available_{false};
    bool panel_can_manage_map_server_{true};
};

} // namespace waypoint_editor

#endif // WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_PANEL_HPP_
