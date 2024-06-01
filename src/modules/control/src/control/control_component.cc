/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "control/control_component.h"

namespace robocar
{
	namespace control
	{

		ControlComponent::ControlComponent(const cycle::Params &params) : cycle::Service(params)
		{
			// params
			_wheel_base = params.get("wheel_base").to_double();
			if (_wheel_base <= 0.0)
			{
				throw std::invalid_argument("'wheel_base' must be > 0.0");
			}
			_minimum_distance = params.get("minimum_distance").to_double();
			if (_minimum_distance <= 0.0)
			{
				throw std::invalid_argument("'minimum_distance' must be > 0.0");
			}
			_target_time = params.get("target_time").to_double();
			if (_target_time <= 0.0)
			{
				throw std::invalid_argument("'target_time' must be > 0.0");
			}
			_curv_theta = params.get("curvature_theta").to_double();
			if (_curv_theta <= 0.0)
			{
				throw std::invalid_argument("'curvature_theta' must be > 0.0");
			}
			_steering_kp = params.get("steering_kp").to_double();
			if (_steering_kp <= 0.0)
			{
				throw std::invalid_argument("'steering_kp' must be > 0.0");
			}
			_max_steering_kp = params.get("max_steering_kp").to_double();
			if (_max_steering_kp <= 0.0)
			{
				throw std::invalid_argument("'max_steering_kp' must be > 0.0");
			}
			_max_steering = params.get("max_steering").to_double();
			if (_max_steering <= 0.0)
			{
				throw std::invalid_argument("'max_steering' must be > 0.0");
			}
			_velocity_kp_a = params.get("velocity_kp_a").to_double();
			if (_velocity_kp_a <= 0.0)
			{
				throw std::invalid_argument("'velocity_kp_a' must be > 0.0");
			}
			_velocity_kp_b = params.get("velocity_kp_b").to_double();
			if (_velocity_kp_b <= 0.0)
			{
				throw std::invalid_argument("'velocity_kp_b' must be > 0.0");
			}
			_velocity_ki_a = params.get("velocity_ki_a").to_double();
			if (_velocity_ki_a <= 0.0)
			{
				throw std::invalid_argument("'velocity_ki_a' must be > 0.0");
			}
			_velocity_ki_b = params.get("velocity_ki_b").to_double();
			if (_velocity_ki_b <= 0.0)
			{
				throw std::invalid_argument("'velocity_ki_b' must be > 0.0");
			}
			_velocity_ki_window = params.get("velocity_ki_window").to_int();
			if (_velocity_ki_window <= 0)
			{
				throw std::invalid_argument("'velocity_ki_window' must be > 0");
			}
			for (int i = 0; i < _velocity_ki_window; i++)
			{
				_prev_velocity_e.push_back(0.0);
			}
			_velocity_kd_a = params.get("velocity_kd_a").to_double();
			if (_velocity_kd_a <= 0.0)
			{
				throw std::invalid_argument("'velocity_kd_a' must be > 0.0");
			}
			_velocity_kd_b = params.get("velocity_kd_b").to_double();
			if (_velocity_kd_b <= 0.0)
			{
				throw std::invalid_argument("'velocity_kd_b' must be > 0.0");
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
			_safe_dist_kp = params.get("safe_dist_kp").to_double();
			if (_safe_dist_kp <= 0.0)
			{
				throw std::invalid_argument("'safe_dist_kp' must be > 0.0");
			}
			_safe_dist_ki = params.get("safe_dist_ki").to_double();
			if (_safe_dist_ki <= 0.0)
			{
				throw std::invalid_argument("'safe_dist_ki' must be > 0.0");
			}
			_safe_dist_ki_window = params.get("safe_dist_ki_window").to_int();
			if (_safe_dist_ki_window <= 0)
			{
				throw std::invalid_argument("'safe_dist_ki_window' must be > 0");
			}
			for (int i = 0; i < _safe_dist_ki_window; i++)
			{
				_prev_safe_dist_e.push_back(0.0);
			}
			_safe_dist_kd = params.get("safe_dist_kd").to_double();
			if (_safe_dist_kd <= 0.0)
			{
				throw std::invalid_argument("'safe_dist_kd' must be > 0.0");
			}
			_smooth_brake = params.get("smooth_brake").to_double();
			if (_smooth_brake <= 0.0)
			{
				throw std::invalid_argument("'smooth_brake' must be > 0.0");
			}
			_smooth_brake_min_dist = params.get("smooth_brake_min_dist").to_double();
			if (_smooth_brake_min_dist <= 0.0)
			{
				throw std::invalid_argument("'smooth_brake_min_dist' must be > 0.0");
			}
			_smooth_brake_duration = params.get("smooth_brake_duration").to_int();
			if (_smooth_brake_duration <= 0)
			{
				throw std::invalid_argument("'smooth_brake_duration' must be > 0");
			}
			_th_filter_coef = params.get("th_filter_coef").to_double();
			if ((_th_filter_coef <= 0.0) || (_th_filter_coef >= 1.0))
			{
				throw std::invalid_argument("'th_filter_coef' must belong to ]0.0, 1.0[");
			}
			_th_filter_duration = params.get("th_filter_duration").to_int();
			if (_th_filter_duration <= 0)
			{
				throw std::invalid_argument("'th_filter_duration' must be > 0");
			}
			_stop_velocity = params.get("stop_velocity").to_double();
			if (_stop_velocity <= 0.0)
			{
				throw std::invalid_argument("'stop_velocity' must be > 0.0");
			}
			_max_throttle = params.get("max_throttle").to_double();
			if ((_max_throttle <= 0.0) || (_max_throttle > 1.0))
			{
				throw std::invalid_argument("'max_throttle' must belong to ]0.0, 1.0]");
			}
			_max_brake = params.get("max_brake").to_double();
			if ((_max_brake <= 0.0) || (_max_brake > 1.0))
			{
				throw std::invalid_argument("'max_brake' must belong to ]0.0, 1.0]");
			}

			// publishers
			_pub_act = this->create_publisher<msg::ActCmd>("vehicle/control", 1);
			_pub_target = this->create_publisher<msg::Waypoint>("control/target", 1);
			_pub_log = this->create_publisher<msg::LogEntry>("logging/entry", 1);
			// subscribers
			_sub_loc = this->create_subscription<msg::Localization>("localization/position", 1,
																	std::bind(&ControlComponent::on_localization,
																			  this, std::placeholders::_1));
			_sub_planning = this->create_subscription<msg::Planning>("planning/trajectory", 1,
																	 std::bind(&ControlComponent::on_planning,
																			   this, std::placeholders::_1));
			_sub_status = this->create_subscription<msg::Vehicle>("vehicle/status", 1,
																  std::bind(&ControlComponent::on_vehicle,
																			this, std::placeholders::_1));
		}

		void ControlComponent::serve()
		{
			msg::LogEntry log_entry;
			std_msgs::msg::String name;
			_mtx.lock();
			// retrieve latest control data
			bool ad_engaged = _ad_engaged;
			double curr_steering = _vehicle_steering;
			uint64_t stamp = cycle::utils::ros_to_unix_ms_time(_loc.header.stamp);
			double curr_theta = _loc.yaw;
			double curr_vel = _loc.vel;
			double target_velocity = _planning.target_velocity;
			double obstacle_d = _planning.obstacle_d;
			double obstacle_dv = _planning.obstacle_dv;
			// transform points into vehicle frame
			msg::Trajectory t;
			if (_planning.trajectory.waypoints.size() > 0)
			{
				t = transform(_planning.trajectory, _loc);
			}
			_mtx.unlock();

			// dt
			double dt = (stamp - _prev_stamp) / 1000.0;
			_prev_stamp = stamp;

			// init actuation
			msg::ActCmd act;
			act.header.stamp = cycle::utils::unix_ms_to_ros_time(stamp);
			act.steering = 0.0;
			act.throttle = 0.0;
			act.brake = 0.0;

			// init target point
			msg::Waypoint target_point;
			target_point.x = 0.0;
			target_point.y = 0.0;
			target_point.yaw = 0.0;
			target_point.yaw_rate = 0.0;
			target_point.vel = 0.0;
			target_point.accel = 0.0;

			// other init
			double velocity_dt_e = 0.0;
			double safe_dist = 0.0;
			double safe_dist_e = 0.0;
			double safe_dist_dt_e = 0.0;

			// pure pursuit
			double pp_steering = 0.0;
			double throttle = 0.0;
			if (t.waypoints.size() > 0)
			{
				// estimate waypoints curvature
				int nb = 0;
				double prev_theta = curr_theta;
				double curvature = 0.0;
				double max_d = (_target_time * curr_vel);
				for (auto &waypoint : t.waypoints)
				{
					double d = std::sqrt((waypoint.x * waypoint.x) + (waypoint.y * waypoint.y));
					if (d > max_d)
					{
						break;
					}
					if (waypoint.yaw != prev_theta)
					{
						double diff = waypoint.yaw - prev_theta;
						if (diff > M_PI)
						{
							diff -= (2 * M_PI);
						}
						if (diff < -M_PI)
						{
							diff += (2 * M_PI);
						}
						diff = std::abs(diff);
						curvature += diff;
						prev_theta = waypoint.yaw;
						nb++;
					}
				}
				if (nb != 0)
				{
					curvature = curvature / nb;
				}
				else
				{
					curvature = _curv_theta;
				}
				if (curvature > _curv_theta)
				{
					curvature = _curv_theta;
				}
				curvature = curvature / _curv_theta;
				curvature = ((1.0 - _curv_smoothing) * curvature) + (_curv_smoothing * _prev_curv);
				_prev_curv = curvature;

				// compute target point
				double dist = 0.0;
				double path_dist = 0.0;
				double prev_x = 0.0;
				double prev_y = 0.0;
				int max_index = 0;
				double max_dist = 0.0;
				double max_path_dist = 0.0;
				double target_distance = std::max(_target_time * curr_vel * (1.0 - curvature),
												  _minimum_distance);
				for (int i = 0; i < t.waypoints.size(); i++)
				{
					// path distance
					double dx = t.waypoints[i].x - prev_x;
					double dy = t.waypoints[i].y - prev_y;
					path_dist += std::sqrt((dx * dx) + (dy * dy));
					prev_x = t.waypoints[i].x;
					prev_y = t.waypoints[i].y;
					// euclidean distance
					dist = std::sqrt((t.waypoints[i].x * t.waypoints[i].x) + (t.waypoints[i].y * t.waypoints[i].y));

					// point closest to target distance
					if (dist >= target_distance)
					{
						target_point = t.waypoints[i];
						target_distance = dist;
						break;
					}
					// farthest point
					if (dist > max_dist)
					{
						max_index = i;
						max_dist = dist;
						max_path_dist = path_dist;
					}
					// choose farthest point as target if none is found
					if (i == (t.waypoints.size() - 1))
					{
						target_point = t.waypoints[max_index];
						target_distance = max_dist;
						path_dist = max_path_dist;
					}
				}

				// compute steering
				if (target_distance > 0.0)
				{
					// pure pursuit
					double alpha = atan2(target_point.y, target_point.x);
					pp_steering = atan2(2 * _wheel_base * sin(alpha), target_distance);
					// steering P
					double steering_e = (1.0 - curvature) * (pp_steering - curr_steering);
					double steering_kp = std::min(_steering_kp * curr_vel, _max_steering_kp);
					pp_steering += (steering_kp * steering_e);
				}
				if (pp_steering > _max_steering)
				{
					pp_steering = _max_steering;
				}
				if (pp_steering < -_max_steering)
				{
					pp_steering = -_max_steering;
				}

				// velocity P error
				target_velocity = std::min(target_velocity, target_point.vel);
				double velocity_e = (target_velocity - curr_vel);
				// velocity I error
				if (ad_engaged)
				{
					_prev_velocity_e.push_back(velocity_e);
					_velocity_sum_e += velocity_e;
					if (_prev_velocity_e.size() > _velocity_ki_window)
					{
						_velocity_sum_e -= _prev_velocity_e.front();
						_prev_velocity_e.pop_front();
					}
				}
				else
				{
					_velocity_sum_e = 0.0;
					_prev_velocity_e.clear();
				}
				// velocity D error
				velocity_dt_e = 0.0;
				if (dt > 0.0)
				{
					velocity_dt_e = (velocity_e - _p_velocity_e) / dt;
				}
				_p_velocity_e = velocity_e;

				// throttle PID
				if (target_point.accel > 0.0)
				{
					throttle = (_velocity_kp_a * target_point.accel) + (_velocity_ki_a * _velocity_sum_e) + (_velocity_kd_a * velocity_dt_e);
				}
				else
				{
					throttle = (_velocity_kp_b * target_point.accel) + (_velocity_ki_b * _velocity_sum_e) + (_velocity_kd_b * velocity_dt_e);
				}

				// RSS safe distance
				double v_r = curr_vel;
				double v_f = curr_vel + obstacle_dv;
				if (v_f < 0.0)
				{
					v_r += std::abs(v_f);
					v_f = 0.0;
				}
				safe_dist = _rss_min_dist + (v_r * _rss_reaction_time) + (0.5 * _rss_max_accel * _rss_reaction_time * _rss_reaction_time) + (std::pow(v_r + _rss_reaction_time * _rss_max_accel, 2.0) / (2 * _rss_min_brake)) - ((v_f * v_f) / (2 * _rss_max_brake));
				if (safe_dist < 0.0)
				{
					safe_dist = 0.0;
				}

				// safe distance P error
				safe_dist_e = 0.0;
				if ((obstacle_d < safe_dist) && (obstacle_d != -1.0))
				{
					safe_dist_e = obstacle_d - safe_dist;
				}
				// safe distance I error
				_prev_safe_dist_e.push_back(safe_dist_e);
				_safe_dist_sum_e += safe_dist_e;
				if (_prev_safe_dist_e.size() > _safe_dist_ki_window)
				{
					_safe_dist_sum_e -= _prev_safe_dist_e.front();
					_prev_safe_dist_e.pop_front();
				}
				// safe distance D error
				safe_dist_dt_e = 0.0;
				if ((dt > 0.0) && (_p_obstacle_d != -1.0))
				{
					safe_dist_dt_e = (safe_dist_e - _p_safe_dist_e) / dt;
				}

				if (safe_dist_e < 0.0)
				{
					_velocity_sum_e = 0.0;
					_prev_velocity_e.clear();

					// safe distance PID
					throttle = (_safe_dist_kp * safe_dist_e) + (_safe_dist_ki * _safe_dist_sum_e) + (_safe_dist_kd * safe_dist_dt_e);

					// smooth braking
					if (_smooth_braking == 0)
					{
						_smooth_braking = 1;
						_smooth_braking_stamp = stamp;
						_safe_dist_sum_e = 0.0;
						_prev_safe_dist_e.clear();
					}
					if (_smooth_braking == 1)
					{
						if ((stamp - _smooth_braking_stamp) > _smooth_brake_duration)
						{
							_smooth_braking = 2;
							_safe_dist_sum_e = 0.0;
							_prev_safe_dist_e.clear();
						}

						if (safe_dist > _smooth_brake_min_dist)
						{
							throttle = std::max(-_smooth_brake, throttle);
						}
					}
				}
				else
				{
					_smooth_braking = 0;
				}

				// trigger throttle filter
				if (!_th_filter && (_p_safe_dist_e < 0.0) && (safe_dist_e >= 0.0))
				{
					_th_filter = true;
					_th_filter_stamp = stamp;
				}
				// check disable throttle filter
				if ((_th_filter && ((stamp - _th_filter_stamp) > _th_filter_duration)) || (safe_dist_e < 0.0))
				{
					_th_filter = false;
				}
				// apply throttle filter
				if (_th_filter)
				{
					throttle = _th_filter_coef * _prev_throttle + (1.0 - _th_filter_coef) * throttle;
				}
				_p_obstacle_d = obstacle_d;
				_p_safe_dist_e = safe_dist_e;
				_prev_throttle = throttle;

				// throttle and brake bounds
				if (throttle > _max_throttle)
				{
					throttle = _max_throttle;
				}
				if (throttle < -_max_brake)
				{
					throttle = -_max_brake;
				}
				if ((obstacle_d >= 0.0) && (obstacle_d < _rss_min_dist) && (curr_vel < _stop_velocity))
				{
					target_velocity = 0.0;
					throttle = -_smooth_brake;
				}

				// set actuation
				act.steering = pp_steering;
				if (throttle >= 0.0)
				{
					act.throttle = throttle;
					act.brake = 0.0;
				}
				else
				{
					act.throttle = 0.0;
					act.brake = -throttle;
				}
			}
			else
			{
				target_velocity = 0.0;
				act.steering = 0.0;
				act.throttle = 0.0;
				act.brake = 0.0;
			}

			// logs
			name.data = "steering";
			log_entry.name.push_back(name);
			log_entry.value.push_back(curr_steering);
			name.data = "steering_control";
			log_entry.name.push_back(name);
			log_entry.value.push_back(pp_steering);
			// name.data = "curvature";
			// log_entry.name.push_back(name);
			// log_entry.value.push_back(curvature);
			// name.data = "target_distance";
			// log_entry.name.push_back(name);
			// log_entry.value.push_back(target_distance);
			// name.data = "steering_e";
			// log_entry.name.push_back(name);
			// log_entry.value.push_back(steering_e);
			// name.data = "steering_kp";
			// log_entry.name.push_back(name);
			// log_entry.value.push_back(steering_kp);
			name.data = "velocity";
			log_entry.name.push_back(name);
			log_entry.value.push_back(curr_vel);
			name.data = "target_velocity";
			log_entry.name.push_back(name);
			log_entry.value.push_back(target_velocity);
			name.data = "target_accel";
			log_entry.name.push_back(name);
			log_entry.value.push_back(target_point.accel);
			name.data = "throttle_control";
			log_entry.name.push_back(name);
			log_entry.value.push_back(throttle);
			if (target_point.accel > 0.0)
			{
				name.data = "velocity_kp";
				log_entry.name.push_back(name);
				log_entry.value.push_back(_velocity_kp_a * target_point.accel);
				name.data = "velocity_ki";
				log_entry.name.push_back(name);
				log_entry.value.push_back(_velocity_ki_a * _velocity_sum_e);
				name.data = "velocity_kd";
				log_entry.name.push_back(name);
				log_entry.value.push_back(_velocity_kd_a * velocity_dt_e);
			}
			else
			{
				name.data = "velocity_kp";
				log_entry.name.push_back(name);
				log_entry.value.push_back(_velocity_kp_b * target_point.accel);
				name.data = "velocity_ki";
				log_entry.name.push_back(name);
				log_entry.value.push_back(_velocity_ki_b * _velocity_sum_e);
				name.data = "velocity_kd";
				log_entry.name.push_back(name);
				log_entry.value.push_back(_velocity_kd_b * velocity_dt_e);
			}
			name.data = "safe_dist";
			log_entry.name.push_back(name);
			log_entry.value.push_back(safe_dist);
			name.data = "safe_dist_kp";
			log_entry.name.push_back(name);
			log_entry.value.push_back(_safe_dist_kp * safe_dist_e);
			name.data = "safe_dist_ki";
			log_entry.name.push_back(name);
			log_entry.value.push_back(_safe_dist_ki * _safe_dist_sum_e);
			name.data = "safe_dist_kd";
			log_entry.name.push_back(name);
			log_entry.value.push_back(_safe_dist_kd * safe_dist_dt_e);
			name.data = "obstacle_d";
			log_entry.name.push_back(name);
			log_entry.value.push_back(obstacle_d);
			name.data = "obstacle_dv";
			log_entry.name.push_back(name);
			log_entry.value.push_back(obstacle_dv);

			// publish target point and actuation
			_pub_target->publish(target_point);
			_pub_act->publish(act);
			_pub_log->publish(log_entry);
		}

		msg::Trajectory ControlComponent::transform(msg::Trajectory const &t, msg::Localization const &loc)
		{
			msg::Trajectory _t;
			double cos_theta = std::cos(loc.yaw);
			double sin_theta = std::sin(loc.yaw);

			for (auto &waypoint : t.waypoints)
			{
				double x = cos_theta * waypoint.x + sin_theta * waypoint.y - cos_theta * loc.x - sin_theta * loc.y;
				if (x <= 0.0)
				{
					continue;
				}

				double y = -sin_theta * waypoint.x + cos_theta * waypoint.y + sin_theta * loc.x - cos_theta * loc.y;

				msg::Waypoint w = waypoint;
				w.x = x;
				w.y = y;
				_t.waypoints.push_back(w);
			}

			return _t;
		}

		void ControlComponent::on_localization(const msg::Localization &loc)
		{
			_mtx.lock();
			_loc = loc;
			_mtx.unlock();
		}

		void ControlComponent::on_planning(const msg::Planning &planning)
		{
			_mtx.lock();
			_planning = planning;
			_mtx.unlock();
		}

		void ControlComponent::on_vehicle(const msg::Vehicle &vehicle)
		{
			_mtx.lock();
			_ad_engaged = vehicle.ad_engaged;
			_vehicle_steering = vehicle.steering;
			_mtx.unlock();
		}

	} // namespace control
} // namespace robocar