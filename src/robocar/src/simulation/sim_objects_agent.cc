/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/simulation/sim_objects_agent.h"

namespace robocar
{
    namespace simulation
    {

        SimObjectAgent::SimObjectAgent(double dim_x, double dim_y, double dim_z,
                                       double gc_x, double gc_y)
        {
            // header
            _object.header.stamp = robocar::utils::unix_ms_to_ros_time(0);
            _object.header.frame_id = "map";
            // type
            _object.type = OBJ_OBSTACLE;
            // ground center
            _object.ground_center.x = gc_x;
            _object.ground_center.y = gc_y;
            _object.ground_center.z = 0.0;
            // direction
            _object.direction.x = 1.0;
            _object.direction.y = 0.0;
            _object.direction.z = 0.0;
            // dims
            _object.dims.x = dim_x;
            _object.dims.y = dim_y;
            _object.dims.z = dim_z;
            // velocity
            _object.velocity.x = 0.0;
            _object.velocity.y = 0.0;
            _object.velocity.z = 0.0;

            // distances from ground center
            double l_2 = _object.dims.x / 2;
            double w_2 = _object.dims.y / 2;
            double h = _object.dims.z;
            // bbox points in object frame
            _bbox = std::vector<Eigen::Vector3d>(8);
            _bbox[0] = {l_2, w_2, 0};
            _bbox[1] = {-l_2, w_2, 0};
            _bbox[2] = {-l_2, -w_2, 0};
            _bbox[3] = {l_2, -w_2, 0};
            _bbox[4] = {l_2, w_2, h};
            _bbox[5] = {-l_2, w_2, h};
            _bbox[6] = {-l_2, -w_2, h};
            _bbox[7] = {l_2, -w_2, h};

            // direction angle
            double theta = atan2(_object.direction.y, _object.direction.x);
            double cos_theta = cos(theta);
            double sin_theta = sin(theta);
            // bbox points in map frame
            for (int j = 0; j < _bbox.size(); j++)
            {
                msg::Vector3d point;
                point.x = (_bbox[j][0] * cos_theta) - (_bbox[j][1] * sin_theta) + _object.ground_center.x;
                point.y = (_bbox[j][0] * sin_theta) + (_bbox[j][1] * cos_theta) + _object.ground_center.y;
                point.z = _bbox[j][2] + _object.ground_center.z;
                _object.points.push_back(point);
            }
        }

        void SimObjectAgent::update_bbox()
        {
            // direction angle
            double theta = atan2(_object.direction.y, _object.direction.x);
            double cos_theta = cos(theta);
            double sin_theta = sin(theta);

            // update bbox points
            for (int j = 0; j < _bbox.size(); j++)
            {
                double x = (_bbox[j][0] * cos_theta) - (_bbox[j][1] * sin_theta);
                double y = (_bbox[j][0] * sin_theta) + (_bbox[j][1] * cos_theta);
                _object.points[j].x = x + _object.ground_center.x;
                _object.points[j].y = y + _object.ground_center.y;
            }
        }

        msg::Object3d SimObjectFollow::update(msg::Localization loc, msg::Path waypoints, double dt)
        {
            double dx = _object.ground_center.x - loc.x;
            double dy = _object.ground_center.y - loc.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            double vel = _object.velocity.x;

            _state_dt += dt;

            if (_state == 0)
            {
                vel = std::min(vel + _accel * dt, _vel);

                // reset if too far
                if (dist >= _max_dist)
                {
                    _object.ground_center.x = loc.x;
                    _object.ground_center.y = loc.y;
                }

                // state fsm
                if (_state_dt > _state_0_period)
                {
                    _state_dt = 0.0;
                    _state = 1;
                }
            }

            else if (_state == 1)
            {
                vel = std::max(vel - _accel * dt, 0.0);

                // state fsm
                if (_state_dt > _state_1_period)
                {
                    _state_dt = 0.0;
                    _state = 0;
                }
            }
            _object.velocity.x = vel;

            if (waypoints.waypoints.size() > 0)
            {
                double gc_x = _object.ground_center.x;
                double gc_y = _object.ground_center.y;

                // find closest waypoint
                size_t index = 0;
                double min_dist = -1.0;
                for (int i = 0; i < waypoints.waypoints.size(); i++)
                {
                    dx = gc_x - waypoints.waypoints[i].x;
                    dy = gc_y - waypoints.waypoints[i].y;
                    dist = std::sqrt(dx * dx + dy * dy);

                    if ((dist < min_dist) || (min_dist == -1.0))
                    {
                        min_dist = dist;
                        index = i;
                    }
                }

                // always aim at the next closest waypoint
                if ((index + 1) < waypoints.waypoints.size())
                {
                    index++;
                }

                // move along waypoints
                dx = 0.0;
                dy = 0.0;
                double disp = 0.0;
                double max_disp = _object.velocity.x * dt;
                for (size_t i = index; i < waypoints.waypoints.size(); i++)
                {
                    if (i == index)
                    {
                        dx = waypoints.waypoints[index].x - gc_x;
                        dy = waypoints.waypoints[index].y - gc_y;
                    }
                    else
                    {
                        dx = waypoints.waypoints[i].x - waypoints.waypoints[i - 1].x;
                        dy = waypoints.waypoints[i].y - waypoints.waypoints[i - 1].y;
                    }
                    dist = std::sqrt(dx * dx + dy * dy);
                    disp += dist;

                    if (disp >= max_disp)
                    {
                        double theta = atan2(dy, dx);
                        double diff = max_disp - (disp - dist);
                        gc_x += std::cos(theta) * diff;
                        gc_y += std::sin(theta) * diff;
                        break;
                    }

                    gc_x += dx;
                    gc_y += dy;
                }

                // update direction
                if ((dx != 0.0) || (dy != 0.0))
                {
                    _object.direction.x = dx;
                    _object.direction.y = dy;
                }
                // update ground center
                _object.ground_center.x = gc_x;
                _object.ground_center.y = gc_y;

                // update bbox
                this->update_bbox();
            }

            return _object;
        }

        msg::Object3d SimObjectClose::update(msg::Localization loc, msg::Path waypoints, double dt)
        {
            _state_dt += dt;

            if (_state == 0)
            {
                if (waypoints.waypoints.size() > 0)
                {
                    double dx = 0.0;
                    double dy = 0.0;
                    double dist = 0.0;

                    // find closest waypoint to vehicle
                    size_t index_c = 0;
                    double min_dist = -1.0;
                    for (int i = 0; i < waypoints.waypoints.size(); i++)
                    {
                        dx = loc.x - waypoints.waypoints[i].x;
                        dy = loc.y - waypoints.waypoints[i].y;
                        dist = std::sqrt(dx * dx + dy * dy);

                        if ((dist < min_dist) || (min_dist == -1.0))
                        {
                            min_dist = dist;
                            index_c = i;
                        }
                    }

                    // find waypoint at correct offset distance from vehicle
                    size_t index_o = 0;
                    double prev_x = loc.x;
                    double prev_y = loc.y;
                    dist = 0.0;
                    for (int i = index_c; i < waypoints.waypoints.size(); i++)
                    {
                        dx = waypoints.waypoints[i].x - prev_x;
                        dy = waypoints.waypoints[i].y - prev_y;
                        dist += std::sqrt(dx * dx + dy * dy);

                        if (dist >= _offset_x)
                        {
                            index_o = i;
                            break;
                        }

                        prev_x = waypoints.waypoints[i].x;
                        prev_y = waypoints.waypoints[i].y;
                    }

                    // align to waypoint direction and vehicle speed
                    _object.direction.x = cos(waypoints.waypoints[index_o].yaw);
                    _object.direction.y = sin(waypoints.waypoints[index_o].yaw);
                    _object.velocity.x = loc.vel;

                    // move to offset position
                    double pos_yaw = waypoints.waypoints[index_o].yaw + M_PI_2;
                    _object.ground_center.x = waypoints.waypoints[index_o].x + _offset_y * cos(pos_yaw);
                    _object.ground_center.y = waypoints.waypoints[index_o].y + _offset_y * sin(pos_yaw);

                    // update bbox
                    this->update_bbox();

                    // state fsm
                    if (_state_dt > _state_0_period)
                    {
                        _state_dt = 0.0;
                        _state = 1;
                    }
                }
            }

            else if (_state == 1)
            {
                double dx = _object.ground_center.x - loc.x;
                double dy = _object.ground_center.y - loc.y;
                double dist = std::sqrt(dx * dx + dy * dy);

                // reset if too far
                if (dist >= _max_dist)
                {
                    _object.ground_center.x = loc.x;
                    _object.ground_center.y = loc.y;
                    _object.velocity.x = 0.0;
                }

                if (waypoints.waypoints.size() > 0)
                {
                    double gc_x = _object.ground_center.x;
                    double gc_y = _object.ground_center.y;

                    // find closest waypoint
                    size_t index_c = 0;
                    double min_dist = -1.0;
                    for (int i = 0; i < waypoints.waypoints.size(); i++)
                    {
                        dx = gc_x - waypoints.waypoints[i].x;
                        dy = gc_y - waypoints.waypoints[i].y;
                        dist = std::sqrt(dx * dx + dy * dy);

                        if ((dist < min_dist) || (min_dist == -1.0))
                        {
                            min_dist = dist;
                            index_c = i;
                        }
                    }

                    // find waypoint at correct offset distance
                    size_t index_o = 0;
                    double prev_x = waypoints.waypoints[index_c].x;
                    double prev_y = waypoints.waypoints[index_c].y;
                    dist = 0.0;
                    for (int i = index_c; i < waypoints.waypoints.size(); i++)
                    {
                        dx = waypoints.waypoints[i].x - prev_x;
                        dy = waypoints.waypoints[i].y - prev_y;
                        dist += std::sqrt(dx * dx + dy * dy);
                        index_o = i;

                        if (dist >= _offset_x)
                        {
                            break;
                        }

                        prev_x = waypoints.waypoints[i].x;
                        prev_y = waypoints.waypoints[i].y;
                    }

                    // check object is near closest waypoint and adjust speed
                    dx = _object.ground_center.x - waypoints.waypoints[index_c].x;
                    dy = _object.ground_center.y - waypoints.waypoints[index_c].y;
                    dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < 1.0)
                    {
                        _object.velocity.x = std::min(_object.velocity.x + _accel * dt, _vel);
                    }
                    else
                    {
                        _object.velocity.x = loc.vel * 1.2;
                    }

                    // move along waypoints
                    dx = 0.0;
                    dy = 0.0;
                    double disp = 0.0;
                    double max_disp = _object.velocity.x * dt;
                    for (size_t i = index_o; i < waypoints.waypoints.size(); i++)
                    {
                        if (i == index_o)
                        {
                            dx = waypoints.waypoints[index_o].x - gc_x;
                            dy = waypoints.waypoints[index_o].y - gc_y;
                        }
                        else
                        {
                            dx = waypoints.waypoints[i].x - waypoints.waypoints[i - 1].x;
                            dy = waypoints.waypoints[i].y - waypoints.waypoints[i - 1].y;
                        }
                        dist = std::sqrt(dx * dx + dy * dy);
                        disp += dist;

                        if (disp >= max_disp)
                        {
                            double theta = atan2(dy, dx);
                            double diff = max_disp - (disp - dist);
                            gc_x += std::cos(theta) * diff;
                            gc_y += std::sin(theta) * diff;
                            break;
                        }

                        gc_x += dx;
                        gc_y += dy;
                    }

                    // update direction
                    if ((dx != 0.0) || (dy != 0.0))
                    {
                        _object.direction.x = dx;
                        _object.direction.y = dy;
                    }
                    // update ground center
                    _object.ground_center.x = gc_x;
                    _object.ground_center.y = gc_y;
                }

                // update bbox
                this->update_bbox();

                // state fsm
                if (_state_dt > _state_1_period)
                {
                    _state_dt = 0.0;
                    _state = 0;
                }
            }

            return _object;
        }

        SimObjectStatic::SimObjectStatic(double dim_x, double dim_y, double dim_z,
                                         double gc_x, double gc_y, double max_dist)
            : SimObjectAgent(dim_x, dim_y, dim_z, gc_x, gc_y)
        {
            _offset_x = gc_x;
            _offset_y = gc_y;
            _max_dist = max_dist;
        }

        msg::Object3d SimObjectStatic::update(msg::Localization loc, msg::Path waypoints, double dt)
        {
            // object distance from the vehicle
            double dx = _object.ground_center.x - loc.x;
            double dy = _object.ground_center.y - loc.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            // object position along x in vehicle frame
            double cos_theta = cos(loc.yaw);
            double sin_theta = sin(loc.yaw);
            double gc_x = cos_theta * _object.ground_center.x + sin_theta * _object.ground_center.y - cos_theta * loc.x - sin_theta * loc.y;

            if (((gc_x < -5.0) || (dist >= _max_dist)) && (waypoints.waypoints.size() > 0))
            {
                auto &waypoint = waypoints.waypoints[waypoints.waypoints.size() - 1];

                // update direction
                _object.direction.x = cos(waypoint.yaw);
                _object.direction.y = sin(waypoint.yaw);

                // update ground center
                double theta = waypoint.yaw;
                _object.ground_center.x = waypoint.x + _offset_x * cos(theta) + _offset_y * cos(theta + M_PI_2);
                _object.ground_center.y = waypoint.y + _offset_x * sin(theta) + _offset_y * sin(theta + M_PI_2);

                // update bbox
                this->update_bbox();
            }

            return _object;
        }

    } // namespace simulation
} // namespace robocar