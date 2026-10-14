#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <rviz/load_resource.h>
#include <std_srvs/Trigger.h>

#include <QFileDialog>
#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QStringList>
#include <QPixmap>
#include <QPushButton>
#include <QSizePolicy>
#include <QtConcurrent/QtConcurrent>

#include <algorithm>
#include <cstdlib>

#include "waypoint_editor/rviz/waypoint_editor_panel.hpp"

namespace waypoint_editor
{
namespace
{

QString formatDistance(const double meters, const bool lower_bound = false)
{
    const QString prefix = lower_bound ? QStringLiteral(">= ") : QString();
    return prefix + QString::number(std::max(0.0, meters), 'f', 3) + QStringLiteral(" m");
}

QString shellQuote(const QString &value)
{
    QString escaped = value;
    escaped.replace(QLatin1Char('\''), QStringLiteral("'\\''"));
    return QStringLiteral("'") + escaped + QStringLiteral("'");
}

QString humanizeStatusToken(QString value)
{
    value = value.trimmed();
    if (value.isEmpty()) {
        return QObject::tr("Unknown");
    }

    value.replace(QLatin1Char('_'), QLatin1Char(' '));
    const QString lower = value.toLower();
    const QStringList parts = lower.split(QLatin1Char(' '), QString::SkipEmptyParts);
    QStringList titled;
    titled.reserve(parts.size());
    for (const QString &part : parts) {
        titled.push_back(part.left(1).toUpper() + part.mid(1));
    }
    return titled.join(QLatin1Char(' '));
}

QString formatDurationSeconds(const double seconds)
{
    const int rounded_seconds = std::max(0, static_cast<int>(seconds + 0.5));
    const int hours = rounded_seconds / 3600;
    const int minutes = (rounded_seconds % 3600) / 60;
    const int secs = rounded_seconds % 60;

    if (hours > 0) {
        return QStringLiteral("%1:%2:%3")
            .arg(hours)
            .arg(minutes, 2, 10, QLatin1Char('0'))
            .arg(secs, 2, 10, QLatin1Char('0'));
    }

    return QStringLiteral("%1:%2")
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(secs, 2, 10, QLatin1Char('0'));
}

QString formatRichValue(const QString &text, const QString &color)
{
    return QStringLiteral("<span style=\"color:%1; font-weight:600;\">%2</span>")
        .arg(color, text.toHtmlEscaped());
}

QString formatNavigationStateValue(const QString &state)
{
    if (state == QLatin1String("FOLLOWING")) {
        return formatRichValue(QObject::tr("Active"), QStringLiteral("#1b7f3b"));
    }
    if (state == QLatin1String("PAUSED")) {
        return formatRichValue(QObject::tr("Paused"), QStringLiteral("#b36b00"));
    }
    if (state == QLatin1String("FINISHED")) {
        return formatRichValue(QObject::tr("Finished"), QStringLiteral("#2563eb"));
    }
    if (state == QLatin1String("ERROR")) {
        return formatRichValue(QObject::tr("Error"), QStringLiteral("#c62828"));
    }
    if (state == QLatin1String("IDLE")) {
        return formatRichValue(QObject::tr("Inactive"), QStringLiteral("#6b7280"));
    }
    return formatRichValue(QObject::tr("Unknown"), QStringLiteral("#6b7280"));
}

QString formatLocalizationValue(const bool ready)
{
    if (ready) {
        return formatRichValue(QObject::tr("Proxy ready (move_base)"), QStringLiteral("#1b7f3b"));
    }
    return formatRichValue(QObject::tr("Proxy inactive (move_base)"), QStringLiteral("#6b7280"));
}

QString navigationMetricColor(const QString &navigation_state)
{
    if (navigation_state == QLatin1String("FOLLOWING")) {
        return QStringLiteral("#2563eb");
    }
    if (navigation_state == QLatin1String("PAUSED")) {
        return QStringLiteral("#b36b00");
    }
    if (navigation_state == QLatin1String("FINISHED")) {
        return QStringLiteral("#1b7f3b");
    }
    if (navigation_state == QLatin1String("ERROR")) {
        return QStringLiteral("#c62828");
    }
    return QStringLiteral("#6b7280");
}

QString formatFeedbackValue(const QString &navigation_state, const QString &status)
{
    const QString token = status.trimmed().toUpper();
    if (navigation_state == QLatin1String("IDLE") && (token.isEmpty() || token == QLatin1String("IDLE"))) {
        return formatRichValue(QObject::tr("Inactive"), QStringLiteral("#6b7280"));
    }

    const QString label = humanizeStatusToken(status);
    if (token == QLatin1String("ACTIVE") || token == QLatin1String("SUCCEEDED")) {
        return formatRichValue(label, QStringLiteral("#1b7f3b"));
    }
    if (token == QLatin1String("PENDING") ||
        token == QLatin1String("STARTING") ||
        token == QLatin1String("RESUMING") ||
        token == QLatin1String("PREEMPTING") ||
        token == QLatin1String("RECALLING")) {
        return formatRichValue(label, QStringLiteral("#2563eb"));
    }
    if (token == QLatin1String("PAUSED")) {
        return formatRichValue(label, QStringLiteral("#b36b00"));
    }
    if (token == QLatin1String("ABORTED") ||
        token == QLatin1String("REJECTED") ||
        token == QLatin1String("LOST") ||
        token == QLatin1String("MOVE_BASE_UNAVAILABLE")) {
        return formatRichValue(label, QStringLiteral("#c62828"));
    }
    return formatRichValue(label, navigationMetricColor(navigation_state));
}

QString formatTargetWaypointValue(const QString &navigation_state, const QString &target)
{
    if (navigation_state == QLatin1String("IDLE")) {
        return formatRichValue(QObject::tr("Inactive"), QStringLiteral("#6b7280"));
    }
    if (navigation_state == QLatin1String("FINISHED")) {
        return formatRichValue(QObject::tr("Done"), QStringLiteral("#1b7f3b"));
    }
    if (target.trimmed().isEmpty() || target.contains(QLatin1String("WP -"))) {
        return formatRichValue(QObject::tr("Unknown"), QStringLiteral("#6b7280"));
    }
    return formatRichValue(target, navigationMetricColor(navigation_state));
}

QString formatPosesRemainingValue(const QString &navigation_state, const int waypoint_count, const int navigation_index)
{
    if (waypoint_count <= 0) {
        return formatRichValue(QStringLiteral("0"), QStringLiteral("#6b7280"));
    }
    if (navigation_state == QLatin1String("FINISHED")) {
        return formatRichValue(QStringLiteral("0"), QStringLiteral("#1b7f3b"));
    }
    if (navigation_state == QLatin1String("IDLE")) {
        return formatRichValue(QObject::tr("Inactive"), QStringLiteral("#6b7280"));
    }
    if (navigation_index < 0 || navigation_index >= waypoint_count) {
        return formatRichValue(QObject::tr("Unknown"), QStringLiteral("#6b7280"));
    }
    return formatRichValue(QString::number(std::max(0, waypoint_count - navigation_index)), navigationMetricColor(navigation_state));
}

QString formatEtaValue(const QString &navigation_state)
{
    if (navigation_state == QLatin1String("IDLE")) {
        return formatRichValue(QObject::tr("Inactive"), QStringLiteral("#6b7280"));
    }
    if (navigation_state == QLatin1String("FINISHED")) {
        return formatRichValue(QObject::tr("Done"), QStringLiteral("#1b7f3b"));
    }
    return formatRichValue(QObject::tr("Unknown"), QStringLiteral("#6b7280"));
}

QString formatDistanceRemainingValue(const QString &navigation_state, const double remaining_distance)
{
    if (navigation_state == QLatin1String("IDLE")) {
        return formatRichValue(QObject::tr("Inactive"), QStringLiteral("#6b7280"));
    }
    if (navigation_state == QLatin1String("FINISHED")) {
        return formatRichValue(formatDistance(0.0), QStringLiteral("#1b7f3b"));
    }
    if (remaining_distance < 0.0) {
        return formatRichValue(QObject::tr("Unknown"), QStringLiteral("#6b7280"));
    }
    return formatRichValue(formatDistance(remaining_distance, true), navigationMetricColor(navigation_state));
}

QString formatTimeTakenValue(const QString &navigation_state, const double elapsed_seconds)
{
    if (navigation_state == QLatin1String("IDLE") && elapsed_seconds < 0.0) {
        return formatRichValue(QObject::tr("Inactive"), QStringLiteral("#6b7280"));
    }
    if (elapsed_seconds < 0.0) {
        return formatRichValue(QObject::tr("Unknown"), QStringLiteral("#6b7280"));
    }
    return formatRichValue(formatDurationSeconds(elapsed_seconds), navigationMetricColor(navigation_state));
}

QString formatRecoveriesValue(const QString &navigation_state)
{
    if (navigation_state == QLatin1String("IDLE")) {
        return formatRichValue(QObject::tr("Inactive"), QStringLiteral("#6b7280"));
    }
    return formatRichValue(QObject::tr("Unknown"), QStringLiteral("#6b7280"));
}

QLabel *createValueLabel(QWidget *parent, const QFont &font, const bool wrap = false)
{
    QLabel *label = new QLabel(parent);
    label->setFont(font);
    label->setWordWrap(wrap);
    label->setContentsMargins(0, 0, 0, 0);
    label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    return label;
}

QHBoxLayout *createInfoRow(const QString &title, QLabel *value, const QFont &label_font)
{
    QLabel *title_label = new QLabel(title);
    title_label->setFont(label_font);
    title_label->setContentsMargins(0, 0, 0, 0);
    title_label->setMinimumWidth(112);
    title_label->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);

    QHBoxLayout *row = new QHBoxLayout;
    row->setContentsMargins(0, 0, 0, 0);
    row->setSpacing(6);
    row->addWidget(title_label);
    row->addWidget(value, 1);
    return row;
}

QGroupBox *createGroupBox(const QString &title, const QFont &font, QWidget *parent)
{
    QGroupBox *group = new QGroupBox(title, parent);
    group->setFont(font);
    group->setStyleSheet(QStringLiteral("QGroupBox::title { font-weight: bold; }") );
    return group;
}

} // namespace

WaypointEditorPanel::WaypointEditorPanel(QWidget *parent)
  : rviz::Panel(parent),
    status_value_label_(nullptr),
    waypoint_count_value_label_(nullptr),
    edit_state_value_label_(nullptr),
    nav_status_value_label_(nullptr),
    localization_value_label_(nullptr),
    nav_action_value_label_(nullptr),
    poses_remaining_value_label_(nullptr),
    eta_value_label_(nullptr),
    distance_remaining_value_label_(nullptr),
    time_taken_value_label_(nullptr),
    recoveries_value_label_(nullptr),
    layout_(nullptr),
    load_2d_map_button_(nullptr),
    load_3d_map_button_(nullptr),
    load_waypoints_button_(nullptr),
    save_waypoints_button_(nullptr),
    undo_button_(nullptr),
    redo_button_(nullptr),
    clear_button_(nullptr),
    start_navigation_button_(nullptr),
    pause_navigation_button_(nullptr),
    resume_navigation_button_(nullptr),
    stop_navigation_button_(nullptr),
    service_refresh_timer_(nullptr)
{
    QFont body_font = font();
    body_font.setPointSize(10);
    QFont label_font = body_font;
    label_font.setBold(true);

    status_value_label_ = createValueLabel(this, body_font, true);
    waypoint_count_value_label_ = createValueLabel(this, body_font);
    edit_state_value_label_ = createValueLabel(this, body_font, true);
    nav_status_value_label_ = createValueLabel(this, body_font);
    localization_value_label_ = createValueLabel(this, body_font, true);
    nav_action_value_label_ = createValueLabel(this, body_font, true);
    nav_target_value_label_ = createValueLabel(this, body_font, true);
    poses_remaining_value_label_ = createValueLabel(this, body_font);
    eta_value_label_ = createValueLabel(this, body_font);
    distance_remaining_value_label_ = createValueLabel(this, body_font);
    time_taken_value_label_ = createValueLabel(this, body_font);
    recoveries_value_label_ = createValueLabel(this, body_font);
    eta_value_label_->hide();
    recoveries_value_label_->hide();

    load_2d_map_button_ = new QPushButton(tr("Load 2D Map"), this);
    load_3d_map_button_ = new QPushButton(tr("Load 3D PCD"), this);
    load_waypoints_button_ = new QPushButton(tr("Load WPs"), this);
    save_waypoints_button_ = new QPushButton(tr("Save WPs"), this);
    undo_button_ = new QPushButton(tr("Undo"), this);
    redo_button_ = new QPushButton(tr("Redo"), this);
    clear_button_ = new QPushButton(tr("Clear All"), this);
    start_navigation_button_ = new QPushButton(tr("Start Nav"), this);
    pause_navigation_button_ = new QPushButton(tr("Pause"), this);
    resume_navigation_button_ = new QPushButton(tr("Resume"), this);
    stop_navigation_button_ = new QPushButton(tr("Stop"), this);

    for (QPushButton *button : {load_2d_map_button_, load_3d_map_button_, load_waypoints_button_, save_waypoints_button_, undo_button_, redo_button_, clear_button_, start_navigation_button_, pause_navigation_button_, resume_navigation_button_, stop_navigation_button_}) {
        button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    }

    QLabel *logo_label = new QLabel(this);
    QPixmap logo = rviz::loadPixmap("package://waypoint_editor/icons/classes/waypoint_editor_logo.png");
    if (!logo.isNull()) {
        QPixmap compact_logo = logo.scaledToWidth(58, Qt::SmoothTransformation);
        logo_label->setPixmap(compact_logo);
        logo_label->setFixedSize(compact_logo.size());
    }
    logo_label->setAlignment(Qt::AlignCenter);
    logo_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QVBoxLayout *editor_info_layout = new QVBoxLayout;
    editor_info_layout->setContentsMargins(0, 0, 0, 0);
    editor_info_layout->setSpacing(6);
    editor_info_layout->addLayout(createInfoRow(tr("Status:"), status_value_label_, label_font));
    editor_info_layout->addLayout(createInfoRow(tr("Waypoints:"), waypoint_count_value_label_, label_font));
    editor_info_layout->addLayout(createInfoRow(tr("Edit State:"), edit_state_value_label_, label_font));

    QVBoxLayout *logo_layout = new QVBoxLayout;
    logo_layout->setContentsMargins(0, 0, 0, 0);
    logo_layout->setSpacing(0);
    logo_layout->addStretch();
    logo_layout->addWidget(logo_label, 0, Qt::AlignCenter);
    logo_layout->addStretch();

    QHBoxLayout *editor_content_layout = new QHBoxLayout;
    editor_content_layout->setContentsMargins(6, 6, 6, 6);
    editor_content_layout->setSpacing(6);
    editor_content_layout->addLayout(editor_info_layout, 1);
    editor_content_layout->addLayout(logo_layout, 0);

    QGroupBox *editor_group = createGroupBox(tr("Editor"), body_font, this);
    editor_group->setLayout(editor_content_layout);

    QVBoxLayout *navigation_layout = new QVBoxLayout;
    navigation_layout->setContentsMargins(6, 6, 6, 6);
    navigation_layout->setSpacing(4);
    navigation_layout->addLayout(createInfoRow(tr("Navigation:"), nav_status_value_label_, label_font));
    navigation_layout->addLayout(createInfoRow(tr("Localization:"), localization_value_label_, label_font));
    navigation_layout->addLayout(createInfoRow(tr("Feedback:"), nav_action_value_label_, label_font));
    navigation_layout->addLayout(createInfoRow(tr("Target waypoint:"), nav_target_value_label_, label_font));
    navigation_layout->addSpacing(4);
    navigation_layout->addLayout(createInfoRow(tr("Poses remaining:"), poses_remaining_value_label_, label_font));
    navigation_layout->addLayout(createInfoRow(tr("Distance remaining:"), distance_remaining_value_label_, label_font));
    navigation_layout->addLayout(createInfoRow(tr("Time taken:"), time_taken_value_label_, label_font));
    navigation_layout->addSpacing(6);

    QHBoxLayout *navigation_row1 = new QHBoxLayout;
    navigation_row1->setContentsMargins(0, 0, 0, 0);
    navigation_row1->setSpacing(8);
    navigation_row1->addWidget(start_navigation_button_, 1);
    navigation_row1->addWidget(pause_navigation_button_, 1);

    QHBoxLayout *navigation_row2 = new QHBoxLayout;
    navigation_row2->setContentsMargins(0, 0, 0, 0);
    navigation_row2->setSpacing(8);
    navigation_row2->addWidget(resume_navigation_button_, 1);
    navigation_row2->addWidget(stop_navigation_button_, 1);

    navigation_layout->addLayout(navigation_row1);
    navigation_layout->addLayout(navigation_row2);

    QGroupBox *navigation_group = createGroupBox(tr("Navigation"), body_font, this);
    navigation_group->setLayout(navigation_layout);

    QHBoxLayout *map_row = new QHBoxLayout;
    map_row->setContentsMargins(6, 6, 6, 6);
    map_row->setSpacing(8);
    map_row->addWidget(load_2d_map_button_, 1);
    map_row->addWidget(load_3d_map_button_, 1);

    QGroupBox *map_group = createGroupBox(tr("Map"), body_font, this);
    map_group->setLayout(map_row);

    QHBoxLayout *waypoint_row1 = new QHBoxLayout;
    waypoint_row1->setContentsMargins(6, 6, 6, 0);
    waypoint_row1->setSpacing(8);
    waypoint_row1->addWidget(undo_button_, 1);
    waypoint_row1->addWidget(redo_button_, 1);
    waypoint_row1->addWidget(clear_button_, 1);

    QHBoxLayout *waypoint_row2 = new QHBoxLayout;
    waypoint_row2->setContentsMargins(6, 0, 6, 6);
    waypoint_row2->setSpacing(8);
    waypoint_row2->addWidget(load_waypoints_button_, 1);
    waypoint_row2->addWidget(save_waypoints_button_, 1);

    QVBoxLayout *waypoint_layout = new QVBoxLayout;
    waypoint_layout->setContentsMargins(0, 0, 0, 0);
    waypoint_layout->setSpacing(4);
    waypoint_layout->addLayout(waypoint_row1);
    waypoint_layout->addLayout(waypoint_row2);

    QGroupBox *waypoint_group = createGroupBox(tr("Waypoints"), body_font, this);
    waypoint_group->setLayout(waypoint_layout);

    for (QGroupBox *group : {editor_group, navigation_group, map_group, waypoint_group}) {
        group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    }

    layout_ = new QVBoxLayout;
    layout_->setContentsMargins(4, 4, 4, 4);
    layout_->setSpacing(6);
    layout_->setAlignment(Qt::AlignTop);
    layout_->addWidget(editor_group);
    layout_->addWidget(navigation_group);
    layout_->addWidget(map_group);
    layout_->addWidget(waypoint_group);
    layout_->addStretch();
    setLayout(layout_);

    connect(load_2d_map_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoad2DMapButtonClick);
    connect(load_3d_map_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoad3DMapButtonClick);
    connect(load_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoadWaypointsButtonClick);
    connect(save_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onSaveWaypointsButtonClick);
    connect(undo_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onUndoWaypointsButtonClick);
    connect(redo_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onRedoWaypointsButtonClick);
    connect(clear_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onClearWaypointsButtonClick);
    connect(start_navigation_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onStartNavigationButtonClick);
    connect(pause_navigation_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onPauseNavigationButtonClick);
    connect(resume_navigation_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onResumeNavigationButtonClick);
    connect(stop_navigation_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onStopNavigationButtonClick);

    service_refresh_timer_ = new QTimer(this);
    service_refresh_timer_->setInterval(250);
    connect(service_refresh_timer_, &QTimer::timeout, this, &WaypointEditorPanel::refreshUiState);
    service_refresh_timer_->start();

    initializeRosInterfaces();
    refreshUiState();
}

WaypointEditorPanel::~WaypointEditorPanel()
{
    if (service_refresh_timer_) {
        service_refresh_timer_->stop();
    }
    if (callback_spinner_) {
        callback_spinner_->stop();
    }
}

void WaypointEditorPanel::initializeRosInterfaces()
{
    if (ros_initialized_) {
        return;
    }

    nh_ = ros::NodeHandle("/waypoint_editor");
    sub_nh_ = ros::NodeHandle("/waypoint_editor");
    nh_.param("panel_can_manage_map_server", panel_can_manage_map_server_, true);
    sub_nh_.setCallbackQueue(&callback_queue_);
    callback_spinner_.reset(new ros::AsyncSpinner(1, &callback_queue_));
    callback_spinner_->start();

    load_client_ = nh_.serviceClient<std_srvs::Trigger>("load_waypoints");
    save_client_ = nh_.serviceClient<std_srvs::Trigger>("save_waypoints");
    undo_client_ = nh_.serviceClient<std_srvs::Trigger>("undo_waypoints");
    redo_client_ = nh_.serviceClient<std_srvs::Trigger>("redo_waypoints");
    clear_client_ = nh_.serviceClient<std_srvs::Trigger>("clear_waypoints");
    start_navigation_client_ = nh_.serviceClient<std_srvs::Trigger>("start_navigation");
    pause_navigation_client_ = nh_.serviceClient<std_srvs::Trigger>("pause_navigation");
    resume_navigation_client_ = nh_.serviceClient<std_srvs::Trigger>("resume_navigation");
    stop_navigation_client_ = nh_.serviceClient<std_srvs::Trigger>("stop_navigation");

    waypoint_count_sub_ = sub_nh_.subscribe<std_msgs::Int32>("waypoint_count", 1, [this](const std_msgs::Int32::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setWaypointCountValue", Qt::QueuedConnection, Q_ARG(int, msg->data));
    });

    navigation_ready_sub_ = sub_nh_.subscribe<std_msgs::Bool>("navigation_ready", 1, [this](const std_msgs::Bool::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setNavigationReadyValue", Qt::QueuedConnection, Q_ARG(bool, msg->data));
    });

    undo_available_sub_ = sub_nh_.subscribe<std_msgs::Bool>("undo_available", 1, [this](const std_msgs::Bool::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setUndoAvailableValue", Qt::QueuedConnection, Q_ARG(bool, msg->data));
    });

    redo_available_sub_ = sub_nh_.subscribe<std_msgs::Bool>("redo_available", 1, [this](const std_msgs::Bool::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setRedoAvailableValue", Qt::QueuedConnection, Q_ARG(bool, msg->data));
    });

    nav_status_sub_ = sub_nh_.subscribe<std_msgs::String>("navigation_status", 1, [this](const std_msgs::String::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setNavigationStateValue", Qt::QueuedConnection, Q_ARG(QString, QString::fromStdString(msg->data)));
    });

    nav_action_status_sub_ = sub_nh_.subscribe<std_msgs::String>("navigation_action_status", 1, [this](const std_msgs::String::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setNavigationActionStatusValue", Qt::QueuedConnection, Q_ARG(QString, QString::fromStdString(msg->data)));
    });

    nav_target_sub_ = sub_nh_.subscribe<std_msgs::String>("navigation_target", 1, [this](const std_msgs::String::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setNavigationTargetValue", Qt::QueuedConnection, Q_ARG(QString, QString::fromStdString(msg->data)));
    });

    nav_index_sub_ = sub_nh_.subscribe<std_msgs::Int32>("navigation_index", 1, [this](const std_msgs::Int32::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setNavigationIndexValue", Qt::QueuedConnection, Q_ARG(int, msg->data));
    });

    nav_elapsed_seconds_sub_ = sub_nh_.subscribe<std_msgs::Float64>("navigation_elapsed_seconds", 1, [this](const std_msgs::Float64::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setNavigationElapsedSeconds", Qt::QueuedConnection, Q_ARG(double, msg->data));
    });

    nav_remaining_distance_sub_ = sub_nh_.subscribe<std_msgs::Float64>("navigation_remaining_distance", 1, [this](const std_msgs::Float64::ConstPtr &msg) {
        QMetaObject::invokeMethod(this, "setNavigationRemainingDistance", Qt::QueuedConnection, Q_ARG(double, msg->data));
    });

    ros_initialized_ = true;
}

void WaypointEditorPanel::onInitialize()
{
    initializeRosInterfaces();
    refreshUiState();
}

void WaypointEditorPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
}

void WaypointEditorPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}

void WaypointEditorPanel::postStatusMessage(const QString &msg)
{
    QMetaObject::invokeMethod(this, "applyStatusMessage", Qt::QueuedConnection, Q_ARG(QString, msg));
}

void WaypointEditorPanel::applyStatusMessage(const QString &msg)
{
    status_message_ = msg;
    updateSummaryLabels();
}

void WaypointEditorPanel::setWaypointCountValue(const int count)
{
    waypoint_count_ = std::max(0, count);
    if (waypoint_count_ == 0) {
        navigation_index_ = -1;
    }
    refreshUiState();
}

void WaypointEditorPanel::setNavigationReadyValue(const bool ready)
{
    navigation_ready_ = ready;
    refreshUiState();
}

void WaypointEditorPanel::setUndoAvailableValue(const bool available)
{
    undo_available_ = available;
    refreshUiState();
}

void WaypointEditorPanel::setRedoAvailableValue(const bool available)
{
    redo_available_ = available;
    refreshUiState();
}

void WaypointEditorPanel::setNavigationStateValue(const QString &state)
{
    navigation_state_ = state;
    refreshUiState();
}

void WaypointEditorPanel::setNavigationActionStatusValue(const QString &status)
{
    navigation_action_status_ = status;
    refreshUiState();
}

void WaypointEditorPanel::setNavigationTargetValue(const QString &target)
{
    navigation_target_ = target;
    refreshUiState();
}

void WaypointEditorPanel::setNavigationIndexValue(const int index)
{
    navigation_index_ = index;
    refreshUiState();
}

void WaypointEditorPanel::setNavigationElapsedSeconds(const double seconds)
{
    navigation_elapsed_seconds_ = seconds;
    refreshUiState();
}

void WaypointEditorPanel::setNavigationRemainingDistance(const double distance)
{
    navigation_remaining_distance_ = distance;
    refreshUiState();
}

void WaypointEditorPanel::refreshUiState()
{
    updateSummaryLabels();
    updateButtonStates();
}

void WaypointEditorPanel::updateSummaryLabels()
{
    status_value_label_->setText(status_message_);
    waypoint_count_value_label_->setText(QString::number(waypoint_count_));
    nav_status_value_label_->setText(formatNavigationStateValue(navigation_state_));
    localization_value_label_->setText(formatLocalizationValue(navigation_ready_));
    nav_action_value_label_->setText(formatFeedbackValue(navigation_state_, navigation_action_status_));
    nav_target_value_label_->setText(formatTargetWaypointValue(navigation_state_, navigation_target_));
    poses_remaining_value_label_->setText(formatPosesRemainingValue(navigation_state_, waypoint_count_, navigation_index_));
    eta_value_label_->setText(formatEtaValue(navigation_state_));
    distance_remaining_value_label_->setText(formatDistanceRemainingValue(navigation_state_, navigation_remaining_distance_));
    time_taken_value_label_->setText(formatTimeTakenValue(navigation_state_, navigation_elapsed_seconds_));
    recoveries_value_label_->setText(formatRecoveriesValue(navigation_state_));

    if (isNavigationActive()) {
        edit_state_value_label_->setText(tr("Locked during active navigation"));
    } else if (waypoint_count_ > 0) {
        edit_state_value_label_->setText(tr("Ready for edits"));
    } else {
        edit_state_value_label_->setText(tr("Add or load waypoints"));
    }
}

void WaypointEditorPanel::updateButtonStates()
{
    const bool has_waypoints = waypoint_count_ > 0;
    const bool navigation_active = isNavigationActive();

    load_2d_map_button_->setEnabled(panel_can_manage_map_server_ && !navigation_active);
    load_3d_map_button_->setEnabled(!navigation_active);
    load_waypoints_button_->setEnabled(!navigation_active && serviceAvailable(load_client_));
    save_waypoints_button_->setEnabled(!navigation_active && has_waypoints && serviceAvailable(save_client_));
    undo_button_->setEnabled(!navigation_active && undo_available_);
    redo_button_->setEnabled(!navigation_active && redo_available_);
    clear_button_->setEnabled(!navigation_active && has_waypoints && serviceAvailable(clear_client_));
    start_navigation_button_->setEnabled(canStartNavigation() && serviceAvailable(start_navigation_client_));
    pause_navigation_button_->setEnabled(canPauseNavigation() && serviceAvailable(pause_navigation_client_));
    resume_navigation_button_->setEnabled(canResumeNavigation() && serviceAvailable(resume_navigation_client_));
    stop_navigation_button_->setEnabled(canStopNavigation() && serviceAvailable(stop_navigation_client_));
}

bool WaypointEditorPanel::serviceAvailable(ros::ServiceClient &client) const
{
    return client.isValid() && client.exists();
}

bool WaypointEditorPanel::isNavigationActive() const
{
    return navigation_state_ == QLatin1String("FOLLOWING") || navigation_state_ == QLatin1String("PAUSED");
}

bool WaypointEditorPanel::canStartNavigation() const
{
    if (!navigation_ready_ || waypoint_count_ <= 0) {
        return false;
    }
    return navigation_state_ == QLatin1String("IDLE") ||
           navigation_state_ == QLatin1String("FINISHED") ||
           navigation_state_ == QLatin1String("ERROR");
}

bool WaypointEditorPanel::canPauseNavigation() const
{
    return navigation_state_ == QLatin1String("FOLLOWING");
}

bool WaypointEditorPanel::canResumeNavigation() const
{
    return navigation_ready_ &&
           navigation_state_ == QLatin1String("PAUSED") &&
           waypoint_count_ > 0 &&
           navigation_index_ >= 0 &&
           navigation_index_ < waypoint_count_;
}

bool WaypointEditorPanel::canStopNavigation() const
{
    return isNavigationActive();
}

void WaypointEditorPanel::callTriggerService(ros::ServiceClient &client, const QString &success_msg, const QString &fail_msg)
{
    if (!client.waitForExistence(ros::Duration(0.2))) {
        postStatusMessage(tr("Service not available: %1").arg(QString::fromStdString(client.getService())));
        QMetaObject::invokeMethod(this, "refreshUiState", Qt::QueuedConnection);
        return;
    }

    std_srvs::Trigger srv;
    if (client.call(srv)) {
        if (!srv.response.message.empty()) {
            postStatusMessage(QString::fromStdString(srv.response.message));
        } else {
            postStatusMessage(srv.response.success ? success_msg : fail_msg);
        }
    } else {
        postStatusMessage(tr("Service call failed: %1").arg(QString::fromStdString(client.getService())));
    }

    QMetaObject::invokeMethod(this, "refreshUiState", Qt::QueuedConnection);
}

void WaypointEditorPanel::callTriggerServiceAsync(ros::ServiceClient &client, const QString &success_msg, const QString &fail_msg)
{
    ros::ServiceClient client_copy = client;
    QtConcurrent::run([this, client_copy, success_msg, fail_msg]() mutable {
        if (!client_copy.waitForExistence(ros::Duration(0.2))) {
            postStatusMessage(tr("Service not available: %1").arg(QString::fromStdString(client_copy.getService())));
            QMetaObject::invokeMethod(this, "refreshUiState", Qt::QueuedConnection);
            return;
        }

        std_srvs::Trigger srv;
        if (client_copy.call(srv)) {
            if (!srv.response.message.empty()) {
                postStatusMessage(QString::fromStdString(srv.response.message));
            } else {
                postStatusMessage(srv.response.success ? success_msg : fail_msg);
            }
        } else {
            postStatusMessage(tr("Service call failed: %1").arg(QString::fromStdString(client_copy.getService())));
        }

        QMetaObject::invokeMethod(this, "refreshUiState", Qt::QueuedConnection);
    });
}

void WaypointEditorPanel::onLoad2DMapButtonClick()
{
    initializeRosInterfaces();
    if (!panel_can_manage_map_server_) {
        postStatusMessage(tr("Load 2D Map is disabled while another launch owns /map_server"));
        return;
    }

    const QString qpath = QFileDialog::getOpenFileName(this, tr("Open 2D Map YAML"), QString(), tr("YAML Files (*.yaml)"));
    if (qpath.isEmpty()) {
        return;
    }

    const std::string cmd =
        "rosnode kill /map_server 2>/dev/null; sleep 0.5; rosrun map_server map_server " +
        shellQuote(qpath).toStdString() + " __name:=map_server &";
    const int ret = std::system(cmd.c_str());
    postStatusMessage(ret == 0 ? tr("Loading 2D Map...") : tr("Failed to load 2D Map"));
}

void WaypointEditorPanel::onLoad3DMapButtonClick()
{
    initializeRosInterfaces();

    const QString qpath = QFileDialog::getOpenFileName(this, tr("Open 3D PCD"), QString(), tr("Point Cloud Files (*.pcd)"));
    if (qpath.isEmpty()) {
        return;
    }

    const std::string cmd =
        "rosnode kill /waypoint_editor_pcd_map 2>/dev/null; sleep 0.3; rosrun pcl_ros pcd_to_pointcloud " +
        shellQuote(qpath).toStdString() +
        " 0.033 _frame_id:=map __name:=waypoint_editor_pcd_map cloud_pcd:=/waypoint_editor/pcd_map &";
    const int ret = std::system(cmd.c_str());
    postStatusMessage(ret == 0 ? tr("Loading 3D PCD...") : tr("Failed to load 3D PCD"));
}

void WaypointEditorPanel::onLoadWaypointsButtonClick()
{
    initializeRosInterfaces();
    callTriggerServiceAsync(load_client_, tr("Loaded waypoints"), tr("Failed to load waypoints"));
}

void WaypointEditorPanel::onSaveWaypointsButtonClick()
{
    initializeRosInterfaces();
    callTriggerServiceAsync(save_client_, tr("Saved waypoints"), tr("Failed to save waypoints"));
}

void WaypointEditorPanel::onUndoWaypointsButtonClick()
{
    initializeRosInterfaces();
    if (!undo_available_) {
        postStatusMessage(tr("Nothing to undo"));
        return;
    }
    callTriggerService(undo_client_, tr("Reverted waypoint change"), tr("Nothing to undo"));
}

void WaypointEditorPanel::onRedoWaypointsButtonClick()
{
    initializeRosInterfaces();
    if (!redo_available_) {
        postStatusMessage(tr("Nothing to redo"));
        return;
    }
    callTriggerService(redo_client_, tr("Reapplied waypoint change"), tr("Nothing to redo"));
}

void WaypointEditorPanel::onClearWaypointsButtonClick()
{
    initializeRosInterfaces();
    callTriggerService(clear_client_, tr("Cleared all waypoints"), tr("Failed to clear waypoints"));
}

void WaypointEditorPanel::onStartNavigationButtonClick()
{
    initializeRosInterfaces();
    callTriggerServiceAsync(start_navigation_client_, tr("Started navigation"), tr("Failed to start navigation"));
}

void WaypointEditorPanel::onPauseNavigationButtonClick()
{
    initializeRosInterfaces();
    callTriggerServiceAsync(pause_navigation_client_, tr("Paused navigation"), tr("Failed to pause navigation"));
}

void WaypointEditorPanel::onResumeNavigationButtonClick()
{
    initializeRosInterfaces();
    callTriggerServiceAsync(resume_navigation_client_, tr("Resumed navigation"), tr("Failed to resume navigation"));
}

void WaypointEditorPanel::onStopNavigationButtonClick()
{
    initializeRosInterfaces();
    callTriggerServiceAsync(stop_navigation_client_, tr("Stopped navigation"), tr("Failed to stop navigation"));
}

} // namespace waypoint_editor

PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorPanel, rviz::Panel)
