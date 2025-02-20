/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/planning/planning_component.h"

using namespace robocar::utils;

namespace robocar
{
    namespace planning
    {

        PlanningComponent::PlanningComponent(const robocar::Params &params) : robocar::Component(params)
        {
            // params
            _target_velocity = params.get("target_speed").to_double() / 3.6;
            if (_target_velocity <= 0.0)
            {
                throw std::invalid_argument("'target_speed' must be > 0.0");
            }
            _vehicle_length = params.get("vehicle_length").to_double();
            if (_vehicle_length <= 0.0)
            {
                throw std::invalid_argument("'vehicle_length' must be > 0.0");
            }
            _vehicle_width = params.get("vehicle_width").to_double();
            if (_vehicle_width <= 0.0)
            {
                throw std::invalid_argument("'vehicle_width' must be > 0.0");
            }
            _wheel_base = params.get("wheel_base").to_double();
            if (_wheel_base <= 0.0)
            {
                throw std::invalid_argument("'wheel_base' must be > 0.0");
            }
            _margin_up = params.get("margin_up").to_double();
            if (_margin_up <= 0.0)
            {
                throw std::invalid_argument("'margin_up' must be > 0.0");
            }
            _margin_down = params.get("margin_down").to_double();
            if (_margin_down <= 0.0)
            {
                throw std::invalid_argument("'margin_down' must be > 0.0");
            }
            if (_margin_up <= _margin_down)
            {
                throw std::invalid_argument("'margin_up' must be greater than 'margin_down'");
            }
            _objects_window_size = params.get("objects_window").to_int();
            if (_objects_window_size <= 0)
            {
                throw std::invalid_argument("'objects_window' must be > 0");
            }
            _objects_window.resize(_objects_window_size);
            _obdv_window = params.get("obdv_window").to_int();
            if (_obdv_window <= 0)
            {
                throw std::invalid_argument("'obdv_window' must be > 0");
            }
            for (size_t i = 0; i < _obdv_window; i++)
            {
                _prev_obdv.push_back(0.0);
            }
            _max_accel = params.get("max_accel").to_double();
            if (_max_accel <= 0.0)
            {
                throw std::invalid_argument("'max_accel' must be > 0.0");
            }
            _min_accel = params.get("min_accel").to_double();
            if (_min_accel >= 0.0)
            {
                throw std::invalid_argument("'min_accel' must be < 0.0");
            }
            _rss_min_dist = params.get("rss_min_dist").to_double();
            if (_rss_min_dist <= 0.0)
            {
                throw std::invalid_argument("'rss_min_dist' must be > 0.0");
            }
            _rss_reaction_time = params.get("rss_reaction_time").to_double();
            if (_rss_reaction_time <= 0.0)
            {
                throw std::invalid_argument("'rss_reaction_time' must be > 0.0");
            }
            _rss_max_accel = params.get("rss_max_accel").to_double();
            if (_rss_max_accel <= 0.0)
            {
                throw std::invalid_argument("'rss_max_accel' must be > 0.0");
            }
            _rss_min_brake = params.get("rss_min_brake").to_double();
            if (_rss_min_brake <= 0.0)
            {
                throw std::invalid_argument("'rss_min_brake' must be > 0.0");
            }
            _rss_max_brake = params.get("rss_max_brake").to_double();
            if (_rss_max_brake <= 0.0)
            {
                throw std::invalid_argument("'rss_max_brake' must be > 0.0");
            }

            // init MPPI
            mppi::Params mppi_params;
            mppi_params.cl_file = params.get("cl_file").to_string();
            mppi_params.cl_platform_id = params.get("cl_platform_id").to_int();
            mppi_params.local_size = params.get("local_size").to_int();
            mppi_params.multiplier = params.get("multiplier").to_int();
            mppi_params.N = params.get("N").to_int();
            mppi_params.delta = params.get("delta").to_double();
            mppi_params.lambda = params.get("lambda").to_double();
            mppi_params.sigma_s = params.get("sigma_s").to_double();
            mppi_params.sigma_a = params.get("sigma_a").to_double();
            mppi_params.wheel_base = _wheel_base;
            mppi_params.max_steering = params.get("max_steering").to_double();
            mppi_params.max_steering_rate = params.get("max_steering_rate").to_double();
            mppi_params.max_accel = _max_accel;
            mppi_params.min_accel = _min_accel;
            mppi_params.diverge_threshold = params.get("diverge_threshold").to_double();
            mppi_params.diverge_left = params.get("diverge_left").to_double();
            mppi_params.diverge_right = params.get("diverge_right").to_double();
            mppi_params.rss_min_dist = params.get("rss_min_dist").to_double();
            mppi_params.rss_reaction_time = params.get("rss_reaction_time").to_double();
            mppi_params.rss_max_accel = params.get("rss_max_accel").to_double();
            mppi_params.rss_min_brake = params.get("rss_min_brake").to_double();
            mppi_params.rss_max_brake = params.get("rss_max_brake").to_double();
            mppi_params.stop_velocity = params.get("stop_velocity").to_double();
            _mppi.reset(new mppi::MPPI(mppi_params));

            _prev_planning.header.stamp = robocar::utils::unix_ms_to_ros_time(0);
            _prev_planning.header.frame_id = "map";
            _prev_planning.state = STATE_STANDBY;
            _prev_planning.target_velocity = _target_velocity;
            _prev_planning.obstacle_type = OBJ_NONE;
            _prev_planning.obstacle_d = -1.0;
            _prev_planning.obstacle_dv = 0.0;

            _prev_tfl.header.stamp = robocar::utils::unix_ms_to_ros_time(0);
            _prev_tfl.header.frame_id = "vehicle";
            _prev_tfl.type = OBJ_NONE;
            _prev_tfl.ground_center.x = 0.0;
            _prev_tfl.ground_center.y = 0.0;
            _prev_tfl.ground_center.z = 0.0;
            _prev_tfl.direction.x = 1.0;
            _prev_tfl.direction.y = 0.0;
            _prev_tfl.direction.z = 0.0;
            _prev_tfl.dims.x = 0.0;
            _prev_tfl.dims.y = 0.0;
            _prev_tfl.dims.z = 0.0;
            _prev_tfl.velocity.x = 0.0;
            _prev_tfl.velocity.y = 0.0;
            _prev_tfl.velocity.z = 0.0;

            _tfl_input.header.stamp = robocar::utils::unix_ms_to_ros_time(0);
            _tfl_input.type = OBJ_TFL_NONE;

            // publishers
            _pub_trajectory = this->create_publisher<msg::Planning>("planning/trajectory", 1);
            _pub_objects_circles = this->create_publisher<msg::ObjectsCircles>("planning/objects_circles", 1);
            _pub_tfl = this->create_publisher<msg::Object3d>("planning/traffic_light", 1);
            // subscribers
            _sub_status = this->create_subscription<msg::Vehicle>("vehicle/status", 1,
                                                                  std::bind(&PlanningComponent::on_vehicle_status,
                                                                            this, std::placeholders::_1));
            _sub_loc = this->create_subscription<msg::Localization>("localization/position", 1,
                                                                    std::bind(&PlanningComponent::on_localization,
                                                                              this, std::placeholders::_1));
            _sub_waypoints = this->create_subscription<msg::Path>("map/waypoints", 1,
                                                                  std::bind(&PlanningComponent::on_waypoints,
                                                                            this, std::placeholders::_1));
            _sub_objects3d = this->create_subscription<msg::Objects3d>("perception/lidar/objects3d", 1,
                                                                       std::bind(&PlanningComponent::on_objects3d,
                                                                                 this, std::placeholders::_1));
            _sub_objects2d = this->create_subscription<msg::Objects2d>("perception/camera/objects2d", 1,
                                                                       std::bind(&PlanningComponent::on_objects2d,
                                                                                 this, std::placeholders::_1));
            _sub_tfl_input = this->create_subscription<msg::TrafficLight>("planning/traffic_light/input", 1,
                                                                          std::bind(&PlanningComponent::on_tfl_input,
                                                                                    this, std::placeholders::_1));
            _sub_target_speed = this->create_subscription<msg::TargetSpeed>("voice/target_speed", 1,
                                                                            std::bind(&PlanningComponent::on_target_speed,
                                                                                      this, std::placeholders::_1));
        }

        void PlanningComponent::serve()
        {
            _mtx.lock();
            auto loc = _loc;
            auto loc_stamp = loc.header.stamp;
            auto waypoints = _waypoints;
            auto objects3d = _objects3d;
            auto objects2d = _objects2d;
            auto tfl_input = _tfl_input;
            auto target_velocity = _target_velocity;
            _mtx.unlock();

            msg::Planning planning;
            planning.header.stamp = loc_stamp;
            planning.header.frame_id = "map";
            planning.state = _state;
            planning.target_velocity = target_velocity;
            planning.obstacle_type = OBJ_NONE;
            planning.obstacle_d = -1.0;
            planning.obstacle_dv = 0.0;

            // compute dt
            uint64_t now = ros_to_unix_ms_time(loc_stamp);
            double dt = (now - _prev_time) / 1000.0;
            _prev_time = now;

            // match traffic light to map and apply FSM
            auto tfl = traffic_light_match(now, tfl_input, objects2d, waypoints);
            tfl = traffic_light_fsm(loc, _prev_tfl, tfl);
            // transform traffic light into vehicle frame
            auto v_tfl = tfl;
            v_tfl.header.frame_id = "vehicle";
            double cos_theta = cos(loc.yaw);
            double sin_theta = sin(loc.yaw);
            double v_tfl_x = cos_theta * tfl.ground_center.x + sin_theta * tfl.ground_center.y - cos_theta * loc.x - sin_theta * loc.y;
            double v_tfl_y = -sin_theta * tfl.ground_center.x + cos_theta * tfl.ground_center.y + sin_theta * loc.x - cos_theta * loc.y;
            double v_tfl_yaw = atan2(tfl.direction.y, tfl.direction.x) + M_PI_2 - loc.yaw;
            v_tfl.ground_center.x = v_tfl_x;
            v_tfl.ground_center.y = v_tfl_y;
            v_tfl.direction.x = cos(v_tfl_yaw);
            v_tfl.direction.y = sin(v_tfl_yaw);
            // add to 3D objects
            objects3d.objects.push_back(v_tfl);

            // build object circles
            auto objects = get_object_circles(loc, objects3d, waypoints);

            // objects window
            msg::ObjectsCircles __objects;
            _objects_window[_objects_window_index] = std::move(objects);
            if (_objects_window_index < (_objects_window.size() - 1))
            {
                _objects_window_index++;
            }
            else
            {
                _objects_window_index = 0;
            }
            for (size_t i = 0; i < _objects_window.size(); i++)
            {
                if (!_objects_window[i].empty())
                {
                    __objects.data.reserve(__objects.data.size() + _objects_window[i].size());
                    __objects.data.insert(__objects.data.end(), _objects_window[i].begin(),
                                          _objects_window[i].end());
                }
            }
            size_t nb_objects = __objects.data.size() / _object_dim;

            // MPPI state
            std::vector<float> __state = {static_cast<float>(loc.x),
                                          static_cast<float>(loc.y),
                                          static_cast<float>(loc.yaw),
                                          static_cast<float>(_vehicle_steering),
                                          static_cast<float>(loc.vel),
                                          static_cast<float>(dt)};

            // MPPI waypoints
            size_t offset_w = 0;
            size_t waypoint_dim = 4;
            std::vector<float> __waypoints;
            __waypoints.resize(waypoints.waypoints.size() * waypoint_dim);
            for (auto &waypoint : waypoints.waypoints)
            {
                __waypoints[offset_w] = waypoint.x;
                __waypoints[offset_w + 1] = waypoint.y;
                __waypoints[offset_w + 2] = waypoint.yaw;
                __waypoints[offset_w + 3] = waypoint.vel;
                offset_w += waypoint_dim;
            }

            // run MPPI
            std::vector<mppi::Waypoint> trajectory = _mppi->run(__state, __waypoints,
                                                                __objects.data);
            for (auto &w : trajectory)
            {
                msg::Waypoint waypoint;
                waypoint.x = w.x();
                waypoint.y = w.y();
                waypoint.yaw = w.yaw();
                waypoint.yaw_rate = w.yaw_rate();
                waypoint.vel = w.vel();
                waypoint.accel = w.accel();
                planning.trajectory.waypoints.push_back(waypoint);
            }

            // collision checker using direction
            double dir_x = loc.x;
            double dir_y = loc.y;
            double dir_yaw = loc.yaw;
            if (planning.trajectory.waypoints.size() > 1)
            {
                size_t index = planning.trajectory.waypoints.size() - 1;
                dir_x = planning.trajectory.waypoints[index].x;
                dir_y = planning.trajectory.waypoints[index].y;
                dir_yaw = planning.trajectory.waypoints[index].yaw;
            }
            size_t offset_o = 0;
            for (size_t i = 0; i < nb_objects; i++)
            {
                double obj_x = __objects.data[offset_o];
                double obj_y = __objects.data[offset_o + 1];
                double obj_r = __objects.data[offset_o + 2];
                double dx = obj_x - loc.x;
                double dy = obj_y - loc.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                // check collision using direction
                double dir_dist = dir_obj_dist(dir_x, dir_y, dir_yaw, obj_x, obj_y, obj_r);
                if ((dir_dist != -1.0f) && (dir_dist <= obj_r))
                {
                    if ((dist < planning.obstacle_d) || (planning.obstacle_d == -1.0))
                    {
                        planning.obstacle_type = __objects.data[offset_o + 3];
                        planning.obstacle_d = dist;
                    }
                }

                offset_o += _object_dim;
            }

            // collision checker on trajectory
            for (auto &waypoint : planning.trajectory.waypoints)
            {
                size_t offset_o = 0;
                for (size_t i = 0; i < nb_objects; i++)
                {
                    double dx = __objects.data[offset_o] - waypoint.x;
                    double dy = __objects.data[offset_o + 1] - waypoint.y;
                    double dist = std::sqrt(dx * dx + dy * dy) - __objects.data[offset_o + 2];

                    if (dist <= 0.0)
                    {
                        dx = __objects.data[offset_o] - loc.x;
                        dy = __objects.data[offset_o + 1] - loc.y;
                        dist = std::sqrt(dx * dx + dy * dy);

                        if ((dist < planning.obstacle_d) || (planning.obstacle_d == -1.0))
                        {
                            planning.obstacle_type = __objects.data[offset_o + 3];
                            planning.obstacle_d = dist;
                        }
                    }

                    offset_o += _object_dim;
                }
            }

            // compute obstacle dv
            double obstacle_dv = 0.0;
            if ((_prev_planning.obstacle_d != -1.0) && (planning.obstacle_d != -1.0) && (dt > 0.0))
            {
                obstacle_dv = (planning.obstacle_d - _prev_planning.obstacle_d) / dt;
            }

            // moving average of obstacle dv
            _prev_obdv.push_back(obstacle_dv);
            _prev_obdv_sum += obstacle_dv;
            _prev_obdv_sum -= _prev_obdv.front();
            _prev_obdv.pop_front();
            obstacle_dv = _prev_obdv_sum / _obdv_window;
            planning.obstacle_dv = obstacle_dv;

            // compute state
            bool trajectory_valid = planning.trajectory.waypoints.size() > 0;
            bool obstacle = planning.obstacle_d != -1.0;
            if (_state == STATE_STANDBY)
            {
                if (trajectory_valid && obstacle)
                {
                    _state = STATE_KEEP_DIST;
                }
                else if (trajectory_valid && !obstacle)
                {
                    _state = STATE_DRIVE;
                }
            }
            else if (_state == STATE_DRIVE)
            {
                if (!trajectory_valid)
                {
                    _state = STATE_STANDBY;
                }
                else if (obstacle)
                {
                    _state = STATE_KEEP_DIST;
                }
            }
            else if (_state == STATE_KEEP_DIST)
            {
                if (trajectory_valid && !obstacle)
                {
                    _state = STATE_DRIVE;
                }
                else if (!trajectory_valid)
                {
                    _state = STATE_STANDBY;
                }
            }
            planning.state = _state;

            // publish
            _pub_trajectory->publish(planning);
            _pub_objects_circles->publish(__objects);
            _pub_tfl->publish(tfl);
            _prev_planning = planning;
            _prev_tfl = tfl;
        }

        msg::Object3d PlanningComponent::traffic_light_match(uint64_t now,
                                                             const msg::TrafficLight &tfl_input,
                                                             const msg::Objects2d &objects,
                                                             const msg::Path &waypoints)
        {
            uint64_t tfl_stamp = now;
            int tfl_type = OBJ_NONE;
            bool tfl_right = true;
            double tfl_x = 0.0;
            double tfl_y = 0.0;
            double tfl_yaw = 0.0;
            double tfl_dim_x = 0.0;
            double tfl_dim_y = 0.0;
            double tfl_dim_z = 0.0;

            // search for closest traffic light from map
            for (auto &waypoint : waypoints.waypoints)
            {
                if (waypoint.tfl)
                {
                    tfl_right = waypoint.tfl_right;
                    tfl_type = OBJ_TFL_NONE;
                    tfl_x = waypoint.x;
                    tfl_y = waypoint.y;
                    tfl_yaw = waypoint.yaw;
                    tfl_dim_x = 4.0;
                    tfl_dim_y = 2.0;
                    tfl_dim_z = 3.0;
                    break;
                }
            }

            // get traffic light status from perception
            double last_img_x = 0.0;
            uint64_t obj_stamp = 0;
            if (objects.objects.size() > 0)
            {
                last_img_x = objects.objects[0].x1;
                obj_stamp = ros_to_unix_ms_time(objects.objects[0].header.stamp);
            }
            for (auto &obj : objects.objects)
            {
                bool relevant = (tfl_type != OBJ_NONE) && ((obj.type == OBJ_TFL_GREEN) || (obj.type == OBJ_TFL_YELLOW) || (obj.type == OBJ_TFL_RED));
                bool match = (tfl_right && (obj.x1 >= last_img_x)) || (!tfl_right && (obj.x1 <= last_img_x));
                if (relevant && match)
                {
                    last_img_x = obj.x1;
                    tfl_stamp = obj_stamp;
                    tfl_type = obj.type;
                }
            }

            // traffic light input override
            if (((tfl_input.type == OBJ_TFL_GREEN) || (tfl_input.type == OBJ_TFL_YELLOW) || (tfl_input.type == OBJ_TFL_RED)) && (tfl_type != OBJ_NONE))
            {
                tfl_stamp = now;
                tfl_type = tfl_input.type;
            }

            msg::Object3d tfl;
            tfl.header.stamp = unix_ms_to_ros_time(tfl_stamp);
            tfl.header.frame_id = "map";
            tfl.type = tfl_type;
            tfl.ground_center.x = tfl_x;
            tfl.ground_center.y = tfl_y;
            tfl.ground_center.z = 0.0;
            tfl.direction.x = cos(tfl_yaw);
            tfl.direction.y = sin(tfl_yaw);
            tfl.direction.z = 0.0;
            tfl.dims.x = tfl_dim_x;
            tfl.dims.y = tfl_dim_y;
            tfl.dims.z = tfl_dim_z;
            tfl.velocity.x = 0.0;
            tfl.velocity.y = 0.0;
            tfl.velocity.z = 0.0;
            return tfl;
        }

        msg::Object3d PlanningComponent::traffic_light_fsm(const msg::Localization &loc,
                                                           const msg::Object3d &p_tfl,
                                                           const msg::Object3d &n_tfl)
        {
            auto time_diff = ros_to_unix_ms_time(n_tfl.header.stamp) - ros_to_unix_ms_time(p_tfl.header.stamp);

            // was none
            if (p_tfl.type == OBJ_NONE)
            {
                return n_tfl;
            }
            // was unknown
            if (p_tfl.type == OBJ_TFL_NONE)
            {
                return n_tfl;
            }
            // was green
            else if (p_tfl.type == OBJ_TFL_GREEN)
            {
                // to green
                if (n_tfl.type == OBJ_TFL_GREEN)
                {
                    return n_tfl;
                }
                // to yellow
                if ((n_tfl.type == OBJ_TFL_YELLOW) && (time_diff > 1000))
                {
                    return n_tfl;
                }
                // to red
                if ((n_tfl.type == OBJ_TFL_RED) && (time_diff > 100))
                {
                    return n_tfl;
                }
                // to unknown
                if ((n_tfl.type == OBJ_TFL_NONE) && (time_diff > 2000))
                {
                    return n_tfl;
                }
                // to none
                if (n_tfl.type == OBJ_NONE)
                {
                    return n_tfl;
                }
            }
            // was yellow
            else if (p_tfl.type == OBJ_TFL_YELLOW)
            {
                // to green
                if ((n_tfl.type == OBJ_TFL_GREEN) && (time_diff > 500))
                {
                    return n_tfl;
                }
                // to yellow
                if (n_tfl.type == OBJ_TFL_YELLOW)
                {
                    return n_tfl;
                }
                // to red
                if ((n_tfl.type == OBJ_TFL_RED) && (time_diff > 100))
                {
                    return n_tfl;
                }
                // to unknown
                if ((n_tfl.type == OBJ_TFL_NONE) && (time_diff > 1500))
                {
                    return n_tfl;
                }
                // to none
                if (n_tfl.type == OBJ_NONE)
                {
                    return n_tfl;
                }
            }
            // was red
            else if (p_tfl.type == OBJ_TFL_RED)
            {
                // to green
                if ((n_tfl.type == OBJ_TFL_GREEN) && (time_diff > 500))
                {
                    return n_tfl;
                }
                // to yellow
                if ((n_tfl.type == OBJ_TFL_YELLOW) && (time_diff > 500))
                {
                    return n_tfl;
                }
                // to red
                if (n_tfl.type == OBJ_TFL_RED)
                {
                    return n_tfl;
                }
                // to unknown
                if ((n_tfl.type == OBJ_TFL_NONE) && (time_diff > 1500))
                {
                    return n_tfl;
                }
                // to none
                if (n_tfl.type == OBJ_NONE)
                {
                    return n_tfl;
                }
            }

            return p_tfl;
        }

        std::vector<float> PlanningComponent::get_object_circles(const msg::Localization &loc,
                                                                 const msg::Objects3d &objects,
                                                                 const msg::Path &waypoints)
        {
            std::vector<float> obj_circles;
            obj_circles.reserve(objects.objects.size() * 2 * _object_dim);

            for (auto &object : objects.objects)
            {
                if (object.type == OBJ_NONE)
                {
                    continue;
                }
                if (object.type == OBJ_TFL_GREEN)
                {
                    continue;
                }
                if (object.dims.z < 0.3)
                {
                    continue;
                }

                // direction angle along max dimension
                double min_dim = 0.0;
                double max_dim = 0.0;
                double theta = 0.0;
                if (object.dims.x > object.dims.y)
                {
                    min_dim = object.dims.y;
                    max_dim = object.dims.x;
                    theta = atan2(object.direction.y, object.direction.x);
                }
                else
                {
                    min_dim = object.dims.x;
                    max_dim = object.dims.y;
                    theta = atan2(object.direction.y, object.direction.x) + M_PI_2;
                }

                // dimensions bounds
                min_dim = std::min(min_dim, 0.1);
                max_dim = std::min(max_dim, 3.5);

                // circle radius
                double R = (min_dim / 2.0) + (_vehicle_width / 2.0) + _margin_up;
                // minimum height covered by the circle
                double h = (min_dim / 2.0) + (_vehicle_width / 2.0) + _margin_down;
                // interdistance between circles centers
                double l = 2.0 * std::sqrt(R * R - h * h);

                // edge point margin
                double edge_m = (min_dim / 2.0) + _margin_up - _margin_down;
                // edge offset for circle center
                double edge_l = std::sqrt((edge_m * edge_m) - (min_dim / 2.0) * (min_dim / 2.0));

                // bottom edge circle center
                double d = 0.0;
                double c_x = object.ground_center.x - (max_dim / 2.0) * cos(theta) + edge_l * cos(theta);
                double c_y = object.ground_center.y - (max_dim / 2.0) * sin(theta) + edge_l * sin(theta);
                d += edge_l;

                while (d < max_dim)
                {
                    // rotation and translation into map frame
                    double x = c_x * cos(loc.yaw) - c_y * sin(loc.yaw) + loc.x;
                    double y = c_x * sin(loc.yaw) + c_y * cos(loc.yaw) + loc.y;

                    // for (auto& waypoint : waypoints.waypoint()) {
                    //     double dx = x - waypoint.x();
                    //     double dy = y - waypoint.y();
                    //     double dist = std::sqrt(dx * dx + dy * dy) - R;

                    //     if (dist <= 0.0) {
                    //         obj_circles.push_back(x);
                    //         obj_circles.push_back(y);
                    //         obj_circles.push_back(R);
                    //         obj_circles.push_back(object.type);
                    //         break;
                    //     }
                    // }

                    obj_circles.push_back(x);
                    obj_circles.push_back(y);
                    obj_circles.push_back(R);
                    obj_circles.push_back(object.type);

                    c_x += l * cos(theta);
                    c_y += l * sin(theta);
                    d += l;
                }

                if (d >= max_dim)
                {
                    // top edge circle center
                    c_x = object.ground_center.x + (max_dim / 2.0) * cos(theta) - edge_l * cos(theta);
                    c_y = object.ground_center.y + (max_dim / 2.0) * sin(theta) - edge_l * sin(theta);

                    // rotation and translation into map frame
                    double x = c_x * cos(loc.yaw) - c_y * sin(loc.yaw) + loc.x;
                    double y = c_x * sin(loc.yaw) + c_y * cos(loc.yaw) + loc.y;

                    // for (auto& waypoint : waypoints.waypoint()) {
                    //     double dx = x - waypoint.x();
                    //     double dy = y - waypoint.y();
                    //     double dist = std::sqrt(dx * dx + dy * dy) - R;

                    //     if (dist <= 0.0) {
                    //         obj_circles.push_back(x);
                    //         obj_circles.push_back(y);
                    //         obj_circles.push_back(R);
                    //         obj_circles.push_back(object.type);
                    //         break;
                    //     }
                    // }

                    obj_circles.push_back(x);
                    obj_circles.push_back(y);
                    obj_circles.push_back(R);
                    obj_circles.push_back(object.type);
                }
            }

            return obj_circles;
        }

        double PlanningComponent::dir_obj_dist(double x, double y, double yaw,
                                               double obj_x, double obj_y, double obj_r)
        {
            // check object is in front
            double cos_theta = cos(yaw);
            double sin_theta = sin(yaw);
            double w_x = cos_theta * obj_x + sin_theta * obj_y - cos_theta * x - sin_theta * y;
            if (w_x < -obj_r)
            {
                return -1.0f;
            }

            // distance between direction line and object
            double x_0 = x;
            double y_0 = y;
            double x_1 = x + cos_theta;
            double y_1 = y + sin_theta;
            double a = y_0 - y_1;
            double b = x_1 - x_0;
            double c = x_0 * y_1 - x_1 * y_0;
            double dist = std::abs(a * obj_x + b * obj_y + c) / std::sqrt(a * a + b * b);

            return dist;
        }

        void PlanningComponent::on_tfl_input(const msg::TrafficLight &tfl)
        {
            _mtx.lock();
            _tfl_input = tfl;
            _mtx.unlock();
        }

        void PlanningComponent::on_vehicle_status(const msg::Vehicle &status)
        {
            _mtx.lock();
            _vehicle_steering = status.steering;
            _mtx.unlock();
        }

        void PlanningComponent::on_localization(const msg::Localization &loc)
        {
            _mtx.lock();
            _loc = loc;
            _mtx.unlock();
        }

        void PlanningComponent::on_waypoints(const msg::Path &waypoints)
        {
            _mtx.lock();
            _waypoints = waypoints;
            _mtx.unlock();
        }

        void PlanningComponent::on_objects3d(const msg::Objects3d &objects)
        {
            _mtx.lock();
            _objects3d = objects;
            _mtx.unlock();
        }

        void PlanningComponent::on_objects2d(const msg::Objects2d &objects)
        {
            _mtx.lock();
            _objects2d = objects;
            _mtx.unlock();
        }

        void PlanningComponent::on_target_speed(const msg::TargetSpeed &target_speed)
        {
            _mtx.lock();
            _target_velocity = target_speed.speed;
            _mtx.unlock();
        }

    } // namespace planning
} // namespace robocar