/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef CONTROL_CONTROL_COMPONENT_H
#define CONTROL_CONTROL_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include <deque>

namespace robocar
{
	namespace control
	{

		class ControlComponent : public cycle::Service
		{
		public:
			ControlComponent(const cycle::Params &params);

			void serve() override;

		private:
			// params
			double _wheel_base = 2.57;
			double _minimum_distance = 3.5;
			double _target_time = 2.0;
			double _curv_theta = 0.14;
			double _curv_smoothing = 0.95;
			double _steering_kp = 0.63;
			double _max_steering_kp = 4.5;
			double _max_steering = 0.57;
			double _velocity_kp_a = 0.2;
			double _velocity_kp_b = 0.2;
			double _velocity_ki_a = 0.01;
			double _velocity_ki_b = 0.01;
			int _velocity_ki_window = 40;
			double _velocity_kd_a = 0.02;
			double _velocity_kd_b = 0.02;
			double _rss_min_dist = 7.0;
			double _rss_reaction_time = 0.3;
			double _rss_max_accel = 2.5;
			double _rss_min_brake = 2.0;
			double _rss_max_brake = 9.0;
			double _safe_dist_kp = 0.25;
			double _safe_dist_ki = 0.005;
			double _safe_dist_kd = 0.02;
			int _safe_dist_ki_window = 40;
			double _smooth_brake = 0.3;
			double _smooth_brake_min_dist = 10.0;
			int _smooth_brake_duration = 2000;
			double _th_filter_coef = 0.95;
			int _th_filter_duration = 1000;
			double _stop_velocity = 1.5;
			double _max_throttle = 0.6;
			double _max_brake = 0.6;

			std::mutex _mtx;
			bool _ad_engaged = false;
			double _vehicle_steering = 0.0;
			msg::Localization _loc;
			msg::Planning _planning;
			uint64_t _prev_stamp = 0;
			double _prev_curv = 0.0;

			// velocity PID
			double _velocity_sum_e = 0.0;
			std::deque<double> _prev_velocity_e;
			double _p_velocity_e = 0.0;

			// smooth braking
			int _smooth_braking = 0;
			uint64_t _smooth_braking_stamp = 0;

			// safe dist PID
			double _safe_dist_sum_e = 0.0;
			std::deque<double> _prev_safe_dist_e;
			double _p_obstacle_d = -1.0;
			double _p_safe_dist_e = 0.0;

			// throttle filter
			bool _th_filter = false;
			uint64_t _th_filter_stamp = 0;
			double _prev_throttle = 0.0;

			// publishers
			rclcpp::Publisher<msg::ActCmd>::SharedPtr _pub_act;
			rclcpp::Publisher<msg::Waypoint>::SharedPtr _pub_target;
			rclcpp::Publisher<msg::LogEntry>::SharedPtr _pub_log;
			// subscribers
			rclcpp::Subscription<msg::Localization>::SharedPtr _sub_loc;
			rclcpp::Subscription<msg::Planning>::SharedPtr _sub_planning;
			rclcpp::Subscription<msg::Vehicle>::SharedPtr _sub_status;

			msg::Trajectory transform(const msg::Trajectory &t, const msg::Localization &loc);

			void on_localization(const msg::Localization &loc);
			void on_planning(const msg::Planning &planning);
			void on_vehicle(const msg::Vehicle &vehicle);
		};

	} // namespace control
} // namespace robocar

#endif // CONTROL_CONTROL_COMPONENT_H
