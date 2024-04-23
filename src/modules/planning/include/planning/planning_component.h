/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef PLANNING_PLANNING_COMPONENT_H
#define PLANNING_PLANNING_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include "planning/mppi/mppi.h"

#include <deque>

namespace robocar {
namespace planning {

class PlanningComponent : public cycle::Service {
	public:
		PlanningComponent(const cycle::Params& params);

		void serve() override;

	private:
		// params
		double _target_velocity = 0.0;
		double _vehicle_length = 4.14;
		double _vehicle_width = 1.8;
		double _wheel_base = 2.57;
		double _margin_up = 0.4;
		double _margin_down = 0.25;
		int _objects_window_size = 5;
		int _obdv_window = 10;
		double _max_accel = 1.1;
		double _min_accel = -2.0;
		double _rss_min_dist = 7.0;
		double _rss_reaction_time = 0.3;
		double _rss_max_accel = 2.5;
		double _rss_min_brake = 2.0;
		double _rss_max_brake = 9.0;

		uint64_t _prev_time = 0;
		std::unique_ptr<mppi::MPPI> _mppi;
		int _state = STATE_STANDBY;
		msg::Planning _prev_planning;
		msg::Object3d _prev_tfl;

		// objects
		size_t _object_dim = 4;
		int _objects_window_index = 0;
		std::vector<std::vector<float>> _objects_window;

		// obstacle dv
		double _prev_obdv_sum = 0.0;
		std::deque<double> _prev_obdv;

		std::mutex _mtx;
		double _vehicle_steering = 0.0;
		msg::Localization _loc;
		msg::Path _waypoints;
		msg::Objects3d _objects3d;
		msg::Objects2d _objects2d;
		msg::TrafficLight _tfl_input;

		// publishers
		rclcpp::Publisher<msg::Planning>::SharedPtr _pub_trajectory;
		rclcpp::Publisher<msg::ObjectsCircles>::SharedPtr _pub_objects_circles;
		rclcpp::Publisher<msg::Object3d>::SharedPtr _pub_tfl;
		// subscribers
		rclcpp::Subscription<msg::Vehicle>::SharedPtr _sub_status;
		rclcpp::Subscription<msg::Localization>::SharedPtr _sub_loc;
		rclcpp::Subscription<msg::Path>::SharedPtr _sub_waypoints;
		rclcpp::Subscription<msg::Objects3d>::SharedPtr _sub_objects3d;
		rclcpp::Subscription<msg::Objects2d>::SharedPtr _sub_objects2d;
		rclcpp::Subscription<msg::TrafficLight>::SharedPtr _sub_tfl_input;
		rclcpp::Subscription<msg::TargetSpeed>::SharedPtr _sub_target_speed;

		msg::Object3d traffic_light_match(uint64_t now,
										  const msg::TrafficLight& tfl_input,
										  const msg::Objects2d& objects,
										  const msg::Path& waypoints);
		msg::Object3d traffic_light_fsm(const msg::Localization& loc,
										const msg::Object3d& p_tfl,
										const msg::Object3d& n_tfl);

		std::vector<float> get_object_circles(const msg::Localization& loc,
											  const msg::Objects3d& objects,
											  const msg::Path& waypoints);

		double dir_obj_dist(double x, double y, double yaw,
							double obj_x, double obj_y, double obj_r);

		void on_vehicle_status(const msg::Vehicle& status);
		void on_localization(const msg::Localization& loc);
		void on_waypoints(const msg::Path& waypoints);
		void on_objects3d(const msg::Objects3d& objects);
		void on_objects2d(const msg::Objects2d& objects);
		void on_tfl_input(const msg::TrafficLight& tfl);
		void on_target_speed(const msg::TargetSpeed& target_speed);
};

}	// namespace planning
}	// namespace robocar

#endif // PLANNING_PLANNING_COMPONENT_H