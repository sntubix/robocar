/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "visualization/dashboard_component.h"
#include "common/common.h"

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <QHBoxLayout>
#include <QVBoxLayout>

using namespace robocar::visualization;

DashboardComponent::DashboardComponent(const cycle::Params &params, QApplication *qt_app, QWidget *parent)
    : QWidget(parent), cycle::Module(params), _qt_app(qt_app)
{
    // params
    auto rviz_config = params.get("rviz_config").to_string();
    if (!std::filesystem::exists(rviz_config))
    {
        throw std::invalid_argument("invalid rviz configuration file: '" + rviz_config + "'");
    }

    // publishers
    _pub_ad_toggle = this->create_publisher<msg::AdToggle>("vehicle/ad_toggle", 1);
    _pub_input = this->create_publisher<msg::ActCmd>("vehicle/input", 1);
    _pub_tfl = this->create_publisher<msg::TrafficLight>("planning/traffic_light/input", 1);
    _pub_mapping_toggle = this->create_publisher<msg::Mapping>("mapping/toggle", 1);
    // subscribers
    _sub_vehicle = this->create_subscription<msg::Vehicle>("vehicle/status", 1,
                                                           std::bind(&DashboardComponent::on_vehicle,
                                                                     this, std::placeholders::_1));
    _sub_planning = this->create_subscription<msg::Planning>("planning/trajectory", 1,
                                                             std::bind(&DashboardComponent::on_trajectory,
                                                                       this, std::placeholders::_1));
    _sub_tfl = this->create_subscription<msg::Object3d>("planning/traffic_light", 1,
                                                        std::bind(&DashboardComponent::on_traffic_light,
                                                                  this, std::placeholders::_1));
    _sub_act_cmd = this->create_subscription<msg::ActCmd>("actuator/command", 1,
                                                          std::bind(&DashboardComponent::on_act_cmd,
                                                                    this, std::placeholders::_1));
    _sub_loc = this->create_subscription<msg::Localization>("localization/position", 1,
                                                            std::bind(&DashboardComponent::on_localization,
                                                                      this, std::placeholders::_1));
    _sub_mapping_status = this->create_subscription<msg::Mapping>("mapping/status", 1,
                                                                  std::bind(&DashboardComponent::on_mapping_status,
                                                                            this, std::placeholders::_1));

    // main window
    this->setWindowTitle("RoboCar");
    this->setLayout(new QVBoxLayout(this));
    this->resize(1800, 900);
    this->grabKeyboard();

    // rviz panel
    auto node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz");
    _rviz_panel = new rviz_common::VisualizationFrame(node);
    this->layout()->addWidget(_rviz_panel);
    _rviz_panel->setApp(_qt_app);
    _rviz_panel->setStyleSheet("background-color: rgb(255, 255, 255)");
    _rviz_panel->setObjectName(QString::fromUtf8("RvizViewer"));
    _rviz_panel->initialize(node, QString(rviz_config.c_str()));
    _rviz_panel->activateWindow();
    _rviz_panel->lower();

    // dash panel
    auto dash_panel = new QWidget(this);
    auto dash_layout = new QHBoxLayout(dash_panel);
    dash_panel->setLayout(dash_layout);
    dash_panel->setMaximumHeight(150);
    this->layout()->addWidget(dash_panel);

    // status panel
    auto status_panel = new QWidget(dash_panel);
    status_panel->setLayout(new QHBoxLayout(status_panel));
    dash_layout->addWidget(status_panel, 33);

    // left status panel
    auto left_status = new QWidget(status_panel);
    left_status->setLayout(new QVBoxLayout(left_status));
    status_panel->layout()->addWidget(left_status);
    // AD status
    _ad_status = new QLabel("AD ENGAGED", left_status);
    _ad_status->setStyleSheet("font: 14pt; color: rgb(0, 0, 0)");
    _ad_status->setAlignment(Qt::AlignCenter);
    left_status->layout()->addWidget(_ad_status);
    connect(this, &DashboardComponent::vehicle_changed, this, &DashboardComponent::on_vehicle_changed);
    // mapping status
    _mapping_status = new QLabel("MAPPING", left_status);
    _mapping_status->setStyleSheet("font: 14pt; color: rgb(0, 0, 0)");
    _mapping_status->setAlignment(Qt::AlignCenter);
    left_status->layout()->addWidget(_mapping_status);
    connect(this, &DashboardComponent::mapping_status_changed, this, &DashboardComponent::on_mapping_status_changed);

    // right status panel
    auto right_status = new QWidget(status_panel);
    right_status->setLayout(new QVBoxLayout(right_status));
    status_panel->layout()->addWidget(right_status);

    // vehicle status panel
    auto vehicle_status_panel = new QWidget(right_status);
    vehicle_status_panel->setLayout(new QHBoxLayout(vehicle_status_panel));
    right_status->layout()->addWidget(vehicle_status_panel);
    // vehicle status label
    auto vehicle_status_l = new QLabel("VEHICLE : ", vehicle_status_panel);
    vehicle_status_l->setStyleSheet("color: rgb(255, 255, 255)");
    vehicle_status_panel->layout()->addWidget(vehicle_status_l);
    // vehicle status value
    _vehicle_status = new QLabel("N/A", vehicle_status_panel);
    _vehicle_status->setStyleSheet("color: rgb(255, 255, 255)");
    vehicle_status_panel->layout()->addWidget(_vehicle_status);

    // localization status panel
    auto loc_status_panel = new QWidget(right_status);
    loc_status_panel->setLayout(new QHBoxLayout(loc_status_panel));
    right_status->layout()->addWidget(loc_status_panel);
    // localization status label
    auto localization_status_l = new QLabel("LOCALIZATION : ", loc_status_panel);
    localization_status_l->setStyleSheet("color: rgb(255, 255, 255)");
    loc_status_panel->layout()->addWidget(localization_status_l);
    // localization status value
    _localization_status = new QLabel("N/A", loc_status_panel);
    _localization_status->setStyleSheet("color: rgb(255, 255, 255)");
    loc_status_panel->layout()->addWidget(_localization_status);

    // perception status panel
    auto perception_status_panel = new QWidget(right_status);
    perception_status_panel->setLayout(new QHBoxLayout(perception_status_panel));
    right_status->layout()->addWidget(perception_status_panel);
    // perception status label
    auto perception_status_l = new QLabel("PERCEPTION : ", perception_status_panel);
    perception_status_l->setStyleSheet("color: rgb(255, 255, 255)");
    perception_status_panel->layout()->addWidget(perception_status_l);
    // perception status value
    _perception_status = new QLabel("N/A", perception_status_panel);
    _perception_status->setStyleSheet("color: rgb(255, 255, 255)");
    perception_status_panel->layout()->addWidget(_perception_status);

    // info panel
    auto info_panel = new QWidget(dash_panel);
    info_panel->setLayout(new QHBoxLayout(info_panel));
    dash_layout->addWidget(info_panel, 33);

    // speed limit panel
    auto speed_limit_panel = new QWidget(info_panel);
    speed_limit_panel->setLayout(new QHBoxLayout(speed_limit_panel));
    info_panel->layout()->addWidget(speed_limit_panel);
    // speed limit label
    auto speed_limit_l = new QLabel("SPEED LIMIT", speed_limit_panel);
    speed_limit_l->setStyleSheet("color: rgb(255, 255, 255)");
    speed_limit_l->setAlignment(Qt::AlignCenter);
    speed_limit_panel->layout()->addWidget(speed_limit_l);
    // speed limit value
    _speed_limit = new QLabel("N/A", speed_limit_panel);
    _speed_limit->setStyleSheet("font: 11pt; color: rgb(255, 255, 255)");
    _speed_limit->setAlignment(Qt::AlignCenter);
    speed_limit_panel->layout()->addWidget(_speed_limit);

    // center panel
    auto center_panel = new QWidget(info_panel);
    center_panel->setLayout(new QVBoxLayout(center_panel));
    info_panel->layout()->addWidget(center_panel);

    // planning
    _planning = new QLabel("N/A", center_panel);
    _planning->setStyleSheet("font: 18pt; color: rgb(255, 255, 255)");
    _planning->setAlignment(Qt::AlignCenter);
    center_panel->layout()->addWidget(_planning);
    connect(this, &DashboardComponent::planning_changed, this, &DashboardComponent::on_planning_changed);

    // speed panel
    auto speed_panel = new QWidget(center_panel);
    speed_panel->setLayout(new QVBoxLayout(speed_panel));
    center_panel->layout()->addWidget(speed_panel);
    // speed value
    _speed = new QLabel("N/A", speed_panel);
    _speed->setStyleSheet("font: 17pt; color: rgb(255, 255, 255)");
    _speed->setAlignment(Qt::AlignCenter);
    speed_panel->layout()->addWidget(_speed);
    connect(this, &DashboardComponent::position_changed, this, &DashboardComponent::on_position_changed);
    // speed unit
    auto speed_unit = new QLabel("km/h", speed_panel);
    speed_unit->setStyleSheet("font: 9pt; color: rgb(255, 255, 255)");
    speed_unit->setAlignment(Qt::AlignCenter);
    speed_panel->layout()->addWidget(speed_unit);

    // traffic light panel
    auto traffic_light_panel = new QWidget(info_panel);
    traffic_light_panel->setLayout(new QVBoxLayout(traffic_light_panel));
    info_panel->layout()->addWidget(traffic_light_panel);
    connect(this, &DashboardComponent::traffic_light_changed, this, &DashboardComponent::on_traffic_light_changed);
    // traffic light label
    auto traffic_light_l = new QLabel("TRAFFIC LIGHT", traffic_light_panel);
    traffic_light_l->setStyleSheet("color: rgb(255, 255, 255)");
    traffic_light_l->setAlignment(Qt::AlignCenter);
    traffic_light_panel->layout()->addWidget(traffic_light_l);
    // green light
    _green_light = new QLabel("G", traffic_light_panel);
    _green_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
    _green_light->setAlignment(Qt::AlignCenter);
    traffic_light_panel->layout()->addWidget(_green_light);
    // yellow light
    _yellow_light = new QLabel("Y", traffic_light_panel);
    _yellow_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
    _yellow_light->setAlignment(Qt::AlignCenter);
    traffic_light_panel->layout()->addWidget(_yellow_light);
    // red light
    _red_light = new QLabel("R", traffic_light_panel);
    _red_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
    _red_light->setAlignment(Qt::AlignCenter);
    traffic_light_panel->layout()->addWidget(_red_light);

    // actuation panel
    auto actuation_panel = new QWidget(dash_panel);
    actuation_panel->setLayout(new QHBoxLayout(actuation_panel));
    dash_layout->addWidget(actuation_panel, 33);
    connect(this, &DashboardComponent::actuation_changed, this, &DashboardComponent::on_actuation_changed);

    // steering panel
    auto steering_panel = new QWidget(actuation_panel);
    steering_panel->setLayout(new QHBoxLayout(steering_panel));
    actuation_panel->layout()->addWidget(steering_panel);
    // steering label
    auto steering_l = new QLabel("STEERING : ", steering_panel);
    steering_l->setStyleSheet("color: rgb(255, 255, 255)");
    steering_panel->layout()->addWidget(steering_l);
    // steering value
    _steering = new QLabel("N/A", steering_panel);
    _steering->setStyleSheet("color: rgb(255, 255, 255)");
    steering_panel->layout()->addWidget(_steering);

    // throttle panel
    auto throttle_panel = new QWidget(actuation_panel);
    throttle_panel->setLayout(new QHBoxLayout(throttle_panel));
    actuation_panel->layout()->addWidget(throttle_panel);
    // throttle label
    auto throttle_l = new QLabel("THROTTLE : ", throttle_panel);
    throttle_l->setStyleSheet("color: rgb(255, 255, 255)");
    throttle_panel->layout()->addWidget(throttle_l);
    // throttle value
    _throttle = new QLabel("N/A", throttle_panel);
    _throttle->setStyleSheet("color: rgb(255, 255, 255)");
    throttle_panel->layout()->addWidget(_throttle);

    // brake panel
    auto brake_panel = new QWidget(actuation_panel);
    brake_panel->setLayout(new QHBoxLayout(brake_panel));
    actuation_panel->layout()->addWidget(brake_panel);
    // brake label
    auto brake_l = new QLabel("BRAKE : ", brake_panel);
    brake_l->setStyleSheet("color: rgb(255, 255, 255)");
    brake_panel->layout()->addWidget(brake_l);
    // brake value
    _brake = new QLabel("N/A", brake_panel);
    _brake->setStyleSheet("color: rgb(255, 255, 255)");
    brake_panel->layout()->addWidget(_brake);

    // show main window
    this->setStyleSheet("background-color: rgb(30, 30, 30)");
    this->show();
}

void DashboardComponent::on_vehicle(const msg::Vehicle &vehicle)
{
    _ad_engaged = vehicle.ad_engaged;
    emit vehicle_changed(vehicle.ad_engaged, vehicle.steering_status,
                         vehicle.throttle_status, vehicle.brake_status,
                         vehicle.gnss, vehicle.lidar, vehicle.camera);
}

void DashboardComponent::on_localization(const msg::Localization &loc)
{
    emit position_changed(loc.x, loc.y, loc.yaw, loc.vel * 3.6);
}

void DashboardComponent::on_trajectory(const msg::Planning &trajectory)
{
    emit planning_changed(trajectory.state, trajectory.target_velocity * 3.6,
                          trajectory.obstacle_type);
}

void DashboardComponent::on_traffic_light(const msg::Object3d &tfl)
{
    emit traffic_light_changed(tfl.type);
}

void DashboardComponent::on_act_cmd(const msg::ActCmd &act_cmd)
{
    emit actuation_changed(act_cmd.steering * 180.0 / M_PI, act_cmd.throttle, act_cmd.brake);
}

void DashboardComponent::on_mapping_status(const msg::Mapping &mapping)
{
    _mapping = mapping.status;
    emit mapping_status_changed(mapping.status);
}

void DashboardComponent::on_vehicle_changed(bool ad_engaged, int steering, int throttle,
                                            int brake, int gnss, int lidar, int camera)
{
    if (_ad_engaged)
        _ad_status->setStyleSheet("font: 14pt; color: rgb(0, 255, 0)");
    else
        _ad_status->setStyleSheet("font: 14pt; color: rgb(0, 0, 0)");

    // vehicle
    bool v_error = false;
    v_error |= (steering == STATUS_ERROR) || (steering == STATUS_TIMEOUT);
    v_error |= (throttle == STATUS_ERROR) || (throttle == STATUS_TIMEOUT);
    v_error |= (brake == STATUS_ERROR) || (brake == STATUS_TIMEOUT);
    if (v_error)
    {
        _vehicle_status->setStyleSheet("color: rgb(255, 0, 0)");
        _vehicle_status->setText("ERROR");
    }
    else
    {
        _vehicle_status->setStyleSheet("color: rgb(0, 255, 0)");
        _vehicle_status->setText("OK");
    }

    // localization
    if (gnss == STATUS_OK)
    {
        _localization_status->setStyleSheet("color: rgb(0, 255, 0)");
        _localization_status->setText("OK");
    }
    else
    {
        _localization_status->setStyleSheet("color: rgb(255, 0, 0)");
        _localization_status->setText("ERROR");
    }

    // perception
    if (lidar != STATUS_OK)
    {
        _perception_status->setStyleSheet("color: rgb(255, 0, 0)");
        _perception_status->setText("ERROR");
    }
    else if (camera != STATUS_OK)
    {
        _perception_status->setStyleSheet("color: rgb(255, 0, 0)");
        _perception_status->setText("ERROR");
    }
    else
    {
        _perception_status->setStyleSheet("color: rgb(0, 255, 0)");
        _perception_status->setText("OK");
    }
}

void DashboardComponent::on_mapping_status_changed(bool mapping)
{
    if (mapping)
    {
        _mapping_status->setStyleSheet("font: 14pt; color: rgb(0, 255, 0)");
    }
    else
    {
        _mapping_status->setStyleSheet("font: 14pt; color: rgb(0, 0, 0)");
    }
}

void DashboardComponent::on_position_changed(double x, double y, double yaw, double speed)
{
    _speed->setText(cycle::utils::round_str((speed), 0).c_str());
}

void DashboardComponent::on_planning_changed(int state, double target_speed, int obstacle_type)
{
    if (state == STATE_STANDBY)
    {
        _planning->setStyleSheet("font: 18pt; color: rgb(0, 255, 0)");
        _planning->setText("STANDBY");
    }
    if (state == STATE_DRIVE)
    {
        _planning->setStyleSheet("font: 18pt; color: rgb(0, 255, 255)");
        _planning->setText("DRIVE");
    }
    if (state == STATE_KEEP_DIST)
    {
        if ((obstacle_type == OBJ_TFL_RED) || (obstacle_type == OBJ_TFL_YELLOW) || (obstacle_type == OBJ_TFL_NONE))
        {
            _planning->setStyleSheet("font: 18pt; color: rgb(255, 0, 0)");
            _planning->setText("STOP");
        }
        else
        {
            _planning->setStyleSheet("font: 18pt; color: rgb(255, 127, 0)");
            _planning->setText("KEEP DIST");
        }
    }
    _speed_limit->setText(cycle::utils::round_str((target_speed), 0).c_str());
}

void DashboardComponent::on_traffic_light_changed(int status)
{
    if (status == robocar::OBJ_NONE)
    {
        _green_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
        _yellow_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
        _red_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
    }
    if (status == robocar::OBJ_TFL_NONE)
    {
        _green_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
        _yellow_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
        _red_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
    }
    if (status == robocar::OBJ_TFL_GREEN)
    {
        _green_light->setStyleSheet("font: 12pt; color: rgb(0, 255, 0)");
        _yellow_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
        _red_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
    }
    if (status == robocar::OBJ_TFL_YELLOW)
    {
        _green_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
        _yellow_light->setStyleSheet("font: 12pt; color: rgb(255, 255, 0)");
        _red_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
    }
    if (status == robocar::OBJ_TFL_RED)
    {
        _green_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
        _yellow_light->setStyleSheet("font: 12pt; color: rgb(0, 0, 0)");
        _red_light->setStyleSheet("font: 12pt; color: rgb(255, 0, 0)");
    }
}

void DashboardComponent::on_actuation_changed(double steering, double throttle, double brake)
{
    _steering->setText(cycle::utils::round_str(steering, 2).c_str());
    _throttle->setText(cycle::utils::round_str(throttle, 2).c_str());
    _brake->setText(cycle::utils::round_str(brake, 2).c_str());
}

void DashboardComponent::on_tfl_timer()
{
    std::unique_lock<std::mutex> lock(_m_tfl);
    _tfl_counter--;
    if (_tfl_counter < 1)
    {
        msg::TrafficLight tfl;
        tfl.header.stamp = cycle::utils::unix_ms_to_ros_time(cycle::Time::now().ms());
        tfl.type = OBJ_TFL_NONE;
        _pub_tfl->publish(tfl);
    }
}

void DashboardComponent::closeEvent(QCloseEvent *event)
{
    msg::AdToggle ad_toggle;
    ad_toggle.toggle = false;
    // disengage autonomous driving
    while (_ad_engaged)
    {
        _pub_ad_toggle->publish(ad_toggle);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // stop mapping
    while (_mapping)
    {
        msg::Mapping mapping;
        mapping.header.stamp = this->get_clock()->now();
        mapping.status = false;
        mapping.filename.data = "";
        _pub_mapping_toggle->publish(mapping);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    this->msg_cycle("shutdown");
    event->accept();
}