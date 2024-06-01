/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef VEHICLE_VEHICLE_COMPONENT_H
#define VEHICLE_VEHICLE_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include "vehicle/ma_filters.h"

namespace robocar::vehicle
{
	class VehicleComponent : public cycle::Service
	{
	public:
		VehicleComponent(const cycle::Params &params);

		void serve() override;

	private:
		// params
		uint64_t _can_timeout = 100;
		uint64_t _gnss_timeout = 500;
		uint64_t _pc_timeout = 500;
		uint64_t _camera_timeout = 500;
		uint64_t _planning_timeout = 500;
		double _steering_ratio = 15.7;
		double _steering_speed = 200.0;
		double _steering_safety_coef = 0.16;
		double _max_gnss_uncertainty = 0.5;

		// status
		std::atomic<uint64_t> _gnss_stamp = 0;
		std::atomic<uint64_t> _pc_stamp = 0;
		std::atomic<uint64_t> _camera_stamp = 0;
		std::atomic<uint64_t> _planning_stamp = 0;
		std::atomic<double> _gnss_uncertainty = 0.0;
		std::atomic<double> _velocity = 0.0;
		double _min_steering_speed = 0.0;
		double _max_steering_speed = 0.0;
		int _sd_index = 0;
		std::vector<double> _steering_delta;
		uint64_t _prev_steering_stamp = 0;
		double _prev_steering = 0.0;

		// actuation
		std::mutex _m_a;
		msg::ActStatus _act_status;
		msg::ActCmd _act_control;
		msg::ActCmd _act_input;
		std::unique_ptr<MA> _ma_steering;
		std::unique_ptr<MA> _ma_throttle;
		std::unique_ptr<MA> _ma_brake;

		// publishers
		rclcpp::Publisher<msg::Vehicle>::SharedPtr _pub_vehicle;
		rclcpp::Publisher<msg::ActToggle>::SharedPtr _pub_act_toggle;
		rclcpp::Publisher<msg::ActCmd>::SharedPtr _pub_act_cmd;
		// subscribers
		rclcpp::Subscription<msg::AdToggle>::SharedPtr _sub_ad_toggle;
		rclcpp::Subscription<msg::ActStatus>::SharedPtr _sub_act_status;
		rclcpp::Subscription<msg::ActCmd>::SharedPtr _sub_input;
		rclcpp::Subscription<msg::GNSS>::SharedPtr _sub_gnss;
		rclcpp::Subscription<msg::PointCloud>::SharedPtr _sub_pc;
		rclcpp::Subscription<msg::CompressedImage>::SharedPtr _sub_image;
		rclcpp::Subscription<msg::Planning>::SharedPtr _sub_planning;
		rclcpp::Subscription<msg::ActCmd>::SharedPtr _sub_control;

		bool status_check();
		void on_ad_toggle(const msg::AdToggle &ad_toggle);
		void on_act_status(const msg::ActStatus &act_status);
		void on_input(const msg::ActCmd &act_cmd);
		void on_gnss(const msg::GNSS &gnss);
		void on_point_cloud(const msg::PointCloud::ConstSharedPtr &pc);
		void on_image(const msg::CompressedImage::ConstSharedPtr &img);
		void on_planning(const msg::Planning &planning);
		void on_control(const msg::ActCmd &act_cmd);
	};
}

#endif // VEHICLE_VEHICLE_COMPONENT_H