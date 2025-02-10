/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef DRIVERS_TOD_COMPONENT_H
#define DRIVERS_TOD_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include <mutex>

#include "rapidjson/stringbuffer.h"
#include <zmq.hpp>

namespace robocar::drivers::tod
{
	class VehicleData {
	public:
		VehicleData() {}

		uint64_t stamp = 0;
		bool ad_engaged = false;
		double steering = 0.0;
		double lat = 0.0;
		double lon = 0.0;
		double alt = 0.0;
		double roll = 0.0;
		double pitch = 0.0;
		double yaw = 0.0;
		// double velocity_north = 0.0;
		// double velocity_east = 0.0;
		// double velocity_down = 0.0;
		double vel = 0.0;
		// double angular_rate_x = 0.0;
		// double angular_rate_y = 0.0;
		// double angular_rate_z = 0.0;
		// double accel_x = 0.0;
		// double accel_y = 0.0;
		// double accel_z = 0.0;
		// double sigma_x = 0.0;
		// double sigma_y = 0.0;

		const char* to_bytes();

	private:
		rapidjson::StringBuffer _str_buffer;
	};

	class TODComponent : public cycle::Service
	{
	public:
		TODComponent(const cycle::Params &params);

		void serve() override;

	private:
		// params
		double _target_velocity = 0.0;
		double _max_steering = 0.57;
		double _max_throttle = 1.0;
		double _max_brake = 1.0;

    	zmq::context_t _context;
		zmq::socket_t _sock_data;
		zmq::socket_t _sock_cmd;

		std::mutex _m_vd;
		VehicleData _vehicle_data;
		std::atomic<bool> _ad_engaged{false};

		// publishers
		rclcpp::Publisher<msg::AdToggle>::SharedPtr _pub_ad_toggle;
		rclcpp::Publisher<msg::ActCmd>::SharedPtr _pub_input;

		// subscribers
		rclcpp::Subscription<msg::Vehicle>::SharedPtr _sub_vehicle;
		rclcpp::Subscription<msg::GNSS>::SharedPtr _sub_gnss;
		rclcpp::Subscription<msg::Localization>::SharedPtr _sub_loc;
		rclcpp::Subscription<msg::Image>::SharedPtr _sub_camera;

		void on_tod_msg(const std::string& tod_msg);

		void on_vehicle(const msg::Vehicle &vehicle);
		void on_gnss(const msg::GNSS &gnss);
		void on_loc(const msg::Localization &loc);
		void on_camera(const msg::Image::ConstSharedPtr &img);
	};
}

#endif // DRIVERS_TOD_COMPONENT_H