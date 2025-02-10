/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef VISUALIZATION_DASHBOARD_COMPONENT_H
#define VISUALIZATION_DASHBOARD_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include <rviz_common/visualization_frame.hpp>

#include <QApplication>
#include <QWidget>
#include <QCloseEvent>
#include <QLabel>
#include <QInputDialog>
#include <QTimer>

namespace robocar::visualization
{
    class DashboardComponent : public QWidget, public cycle::Service
    {
        Q_OBJECT
    public:
        explicit DashboardComponent(const cycle::Params &params, QApplication *qt_app, QWidget *parent = 0);

        void serve() override;

    signals:
        void vehicle_changed(bool ad_engaged, int steering, int throttle,
                             int brake, int gnss, int lidar, int camera);
        void mapping_status_changed(bool mapping);
        void localization_changed(double x, double y, double pitch, double yaw, double speed);
        void planning_changed(int state, double target_speed, int obstacle_type);
        void traffic_light_changed(int status);
        void actuation_changed(int mode, double steering, double throttle, double brake);

    public slots:
        void on_vehicle_changed(bool ad_engaged, int steering, int throttle,
                                int brake, int gnss, int lidar, int camera);
        void on_mapping_status_changed(bool mapping);
        void on_localization_changed(double x, double y, double pitch, double yaw, double speed);
        void on_planning_changed(int state, double target_speed, int obstacle_type);
        void on_traffic_light_changed(int status);
        void on_actuation_changed(int mode, double steering, double throttle, double brake);

    private:
        bool _ad_engaged = false;
        bool _mapping = false;

        // input actuation
        std::mutex _m_ai;
        msg::ActCmd _act_input;
        int _prev_act_mode = ACT_OVERRIDE_NONE;
        int _nb_inputs = 0;

        // input tfl
        std::mutex _m_tfl;
        int _tfl_counter = 0;

        QApplication *_qt_app;
        // rviz panel
        rviz_common::VisualizationFrame *_rviz_panel;
        // status panel
        QLabel *_ad_status;
        QLabel *_mapping_status;
        QLabel *_vehicle_status;
        QLabel *_localization_status;
        QLabel *_perception_status;
        // center panel
        QLabel *_planning;
        QLabel *_speed;
        QLabel *_speed_limit;
        QLabel *_green_light;
        QLabel *_yellow_light;
        QLabel *_red_light;
        // actuation panel
        QLabel *_steering;
        QLabel *_throttle;
        QLabel *_brake;
        // override panel
        QLabel *_override_none;
        QLabel *_override_partial;
        QLabel *_override_full;

        // publishers
        rclcpp::Publisher<msg::AdToggle>::SharedPtr _pub_ad_toggle;
        rclcpp::Publisher<msg::ActCmd>::SharedPtr _pub_input;
        rclcpp::Publisher<msg::TrafficLight>::SharedPtr _pub_tfl;
        rclcpp::Publisher<msg::Mapping>::SharedPtr _pub_mapping_toggle;
        // subscribers
        rclcpp::Subscription<msg::Vehicle>::SharedPtr _sub_vehicle;
        rclcpp::Subscription<msg::Localization>::SharedPtr _sub_loc;
        rclcpp::Subscription<msg::Planning>::SharedPtr _sub_planning;
        rclcpp::Subscription<msg::Object3d>::SharedPtr _sub_tfl;
        rclcpp::Subscription<msg::ActCmd>::SharedPtr _sub_act_cmd;
        rclcpp::Subscription<msg::Mapping>::SharedPtr _sub_mapping_status;

        void on_vehicle(const msg::Vehicle &vehicle);
        void on_localization(const msg::Localization &loc);
        void on_trajectory(const msg::Planning &trajectory);
        void on_traffic_light(const msg::Object3d &tfl);
        void on_act_cmd(const msg::ActCmd &act_cmd);
        void on_mapping_status(const msg::Mapping &mapping);
        void on_tfl_timer();

        void keyPressEvent(QKeyEvent *event) override
        {
            auto now = this->get_clock()->now();

            if (!event->isAutoRepeat())
            {
                std::unique_lock<std::mutex> lock_ai(_m_ai);
                if (event->key() == Qt::Key_A)
                {
                    _act_input.steering = 0.05;
                    _nb_inputs++;
                }
                if (event->key() == Qt::Key_D)
                {
                    _act_input.steering = -0.05;
                    _nb_inputs++;
                }
                if (event->key() == Qt::Key_W)
                {
                    _act_input.throttle = 0.25;
                    _nb_inputs++;
                }
                if (event->key() == Qt::Key_Space)
                {
                    _act_input.brake = 0.4;
                    _nb_inputs++;
                }

                if (_nb_inputs > 0)
                {
                    _act_input.header.stamp = now;
                    _act_input.mode = ACT_OVERRIDE_PARTIAL;
                }
                lock_ai.unlock();
            }
        }

        void keyReleaseEvent(QKeyEvent *event) override
        {
            auto now = this->get_clock()->now();

            if (!event->isAutoRepeat())
            {
                if (event->key() == Qt::Key_Shift)
                {
                    msg::AdToggle ad_toggle;
                    if (!_ad_engaged)
                    {
                        ad_toggle.toggle = true;
                    }
                    else
                    {
                        ad_toggle.toggle = false;
                    }
                    _pub_ad_toggle->publish(ad_toggle);
                }

                if (event->key() == Qt::Key_M)
                {
                    msg::Mapping mapping;
                    if (!_mapping)
                    {
                        // retrieve map filename from input dialog
                        this->releaseKeyboard();
                        auto input = QInputDialog(this);
                        input.resize(400, 200);
                        input.setStyleSheet("font: 11pt; color: rgb(255, 255, 255)");
                        input.setWindowTitle("Mapping");
                        input.setLabelText("Enter filename :");
                        input.setTextValue("map.geojson");
                        auto ret = input.exec();
                        auto text = input.textValue();
                        this->grabKeyboard();

                        if (ret && !text.isEmpty())
                        {
                            mapping.header.stamp = now;
                            mapping.status = true;
                            mapping.filename.data = text.toStdString();
                            _pub_mapping_toggle->publish(mapping);
                        }
                    }
                    else
                    {
                        mapping.header.stamp = now;
                        mapping.status = false;
                        mapping.filename.data = "";
                        _pub_mapping_toggle->publish(mapping);
                    }
                }

                std::unique_lock<std::mutex> lock_ai(_m_ai);
                if (event->key() == Qt::Key_A)
                {
                    _act_input.steering = 0.0;
                    _nb_inputs--;
                }
                if (event->key() == Qt::Key_D)
                {
                    _act_input.steering = 0.0;
                    _nb_inputs--;
                }
                if (event->key() == Qt::Key_W)
                {
                    _act_input.throttle = 0.0;
                    _nb_inputs--;
                }
                if (event->key() == Qt::Key_Space)
                {
                    _act_input.brake = 0.0;
                    _nb_inputs--;
                }

                if (_nb_inputs == 0)
                {
                    _act_input.header.stamp = now;
                    _act_input.mode = ACT_OVERRIDE_NONE;
                }
                lock_ai.unlock();

                std::unique_lock<std::mutex> lock_tfl(_m_tfl);
                if (event->key() == Qt::Key_T)
                {
                    msg::TrafficLight tfl;
                    tfl.header.stamp = now;
                    tfl.type = OBJ_TFL_NONE;
                    _pub_tfl->publish(tfl);
                    _tfl_counter++;
                    QTimer::singleShot(10000, this, &DashboardComponent::on_tfl_timer);
                }
                if (event->key() == Qt::Key_G)
                {
                    msg::TrafficLight tfl;
                    tfl.header.stamp = now;
                    tfl.type = OBJ_TFL_GREEN;
                    _pub_tfl->publish(tfl);
                    _tfl_counter++;
                    QTimer::singleShot(10000, this, &DashboardComponent::on_tfl_timer);
                }
                if (event->key() == Qt::Key_Y)
                {
                    msg::TrafficLight tfl;
                    tfl.header.stamp = now;
                    tfl.type = OBJ_TFL_YELLOW;
                    _pub_tfl->publish(tfl);
                    _tfl_counter++;
                    QTimer::singleShot(10000, this, &DashboardComponent::on_tfl_timer);
                }
                if (event->key() == Qt::Key_R)
                {
                    msg::TrafficLight tfl;
                    tfl.header.stamp = now;
                    tfl.type = OBJ_TFL_RED;
                    _pub_tfl->publish(tfl);
                    _tfl_counter++;
                    QTimer::singleShot(10000, this, &DashboardComponent::on_tfl_timer);
                }
                lock_tfl.unlock();
            }
        }

        void closeEvent(QCloseEvent *event) override;
    };
}

#endif // VISUALIZATION_DASHBOARD_COMPONENT_H