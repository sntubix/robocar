/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/simulation/sim_objects_component.h"

namespace robocar
{
    namespace simulation
    {

        SimObjectsComponent::SimObjectsComponent(const robocar::Params &params) : robocar::Component(params)
        {
            // params
            bool follow_enable = params.get("follow_enable").to_bool();
            double follow_dim_x = params.get("follow_dim_x").to_double();
            double follow_dim_y = params.get("follow_dim_y").to_double();
            double follow_dim_z = params.get("follow_dim_z").to_double();
            double follow_speed = params.get("follow_speed").to_double();
            double follow_accel = params.get("follow_accel").to_double();
            double follow_max_dist = params.get("follow_max_dist").to_double();
            uint64_t follow_drive_period = params.get("follow_drive_period").to_double();
            uint64_t follow_brake_period = params.get("follow_brake_period").to_double();

            bool close_enable = params.get("close_enable").to_bool();
            double close_dim_x = params.get("close_dim_x").to_double();
            double close_dim_y = params.get("close_dim_y").to_double();
            double close_dim_z = params.get("close_dim_z").to_double();
            double close_offset_x = params.get("close_offset_x").to_double();
            double close_offset_y = params.get("close_offset_y").to_double();
            double close_speed = params.get("close_speed").to_double();
            double close_accel = params.get("close_accel").to_double();
            double close_max_dist = params.get("close_max_dist").to_double();
            uint64_t close_wait_period = params.get("close_wait_period").to_double();
            uint64_t close_drive_period = params.get("close_drive_period").to_double();

            bool static_enable = params.get("static_enable").to_bool();
            double static_dim_x = params.get("static_dim_x").to_double();
            double static_dim_y = params.get("static_dim_y").to_double();
            double static_dim_z = params.get("static_dim_z").to_double();
            double static_x = params.get("static_x").to_double();
            double static_y = params.get("static_y").to_double();
            double static_max_dist = params.get("static_max_dist").to_double();

            // init object agents
            if (follow_enable)
            {
                _agents.push_back(std::shared_ptr<SimObjectAgent>{
                    new SimObjectFollow(follow_dim_x, follow_dim_y, follow_dim_z, 0.0, 0.0,
                                        follow_speed, follow_accel, follow_max_dist,
                                        follow_drive_period, follow_brake_period)});
            }
            if (close_enable)
            {
                _agents.push_back(std::shared_ptr<SimObjectAgent>{
                    new SimObjectClose(close_dim_x, close_dim_y, close_dim_z, 0.0, 0.0,
                                       close_offset_x, close_offset_y,
                                       close_speed, close_accel, close_max_dist,
                                       close_wait_period, close_drive_period)});
            }
            if (static_enable)
            {
                _agents.push_back(std::shared_ptr<SimObjectAgent>{
                    new SimObjectStatic(static_dim_x, static_dim_y, static_dim_z,
                                        static_x, static_y, static_max_dist)});
            }

            // publisher
            _pub_objects = this->create_publisher<msg::Objects3d>("perception/lidar/objects3d", 1);
            // subscribers
            _sub_loc = this->create_subscription<msg::Localization>("localization/position", 1,
                                                                    std::bind(&SimObjectsComponent::on_localization,
                                                                              this, std::placeholders::_1));
            _sub_waypoints = this->create_subscription<msg::Path>("map/waypoints", 1,
                                                                  std::bind(&SimObjectsComponent::on_waypoints,
                                                                            this, std::placeholders::_1));
        }

        void SimObjectsComponent::serve()
        {
            std::unique_lock<std::mutex> lock(_mtx);

            uint64_t now = robocar::Time::now().ms();
            double dt = (now - _prev_time) / 1000.0;
            _prev_time = now;

            // get updated objects from agents
            msg::Objects3d objects;
            size_t i = 0;
            for (auto &agent : _agents)
            {
                auto obj = agent->update(_loc, _waypoints, dt);
                obj.header.stamp = robocar::utils::unix_ms_to_ros_time(now);
                objects.objects.push_back(obj);
                i++;
            }

            // transform into vehicle frame
            for (msg::Object3d &object : objects.objects)
            {
                // header
                object.header.frame_id = "vehicle";

                double cos_theta = cos(-_loc.yaw);
                double sin_theta = sin(-_loc.yaw);

                // direction
                double dir_x = object.direction.x * cos_theta - object.direction.y * sin_theta;
                double dir_y = object.direction.x * sin_theta + object.direction.y * cos_theta;
                object.direction.x = dir_x;
                object.direction.y = dir_y;

                cos_theta = cos(_loc.yaw);
                sin_theta = sin(_loc.yaw);

                // ground center
                double gc_x = cos_theta * object.ground_center.x + sin_theta * object.ground_center.y - cos_theta * _loc.x - sin_theta * _loc.y;
                double gc_y = -sin_theta * object.ground_center.x + cos_theta * object.ground_center.y + sin_theta * _loc.x - cos_theta * _loc.y;
                object.ground_center.x = gc_x;
                object.ground_center.y = gc_y;

                // bbox points
                for (msg::Vector3d &point : object.points)
                {
                    double x = cos_theta * point.x + sin_theta * point.y - cos_theta * _loc.x - sin_theta * _loc.y;
                    double y = -sin_theta * point.x + cos_theta * point.y + sin_theta * _loc.x - cos_theta * _loc.y;
                    point.x = x;
                    point.y = y;
                }
            }

            // publish objects
            _pub_objects->publish(objects);
        }

        void SimObjectsComponent::on_localization(const msg::Localization &loc)
        {
            _mtx.lock();
            _loc = loc;
            _mtx.unlock();
        }

        void SimObjectsComponent::on_waypoints(const msg::Path &waypoints)
        {
            _mtx.lock();
            _waypoints = waypoints;
            _mtx.unlock();
        }

    } // namespace simulation
} // namespace robocar