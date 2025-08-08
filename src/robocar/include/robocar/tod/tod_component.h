/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_TOD_TOD_COMPONENT_H
#define ROBOCAR_TOD_TOD_COMPONENT_H

#include "robocar/core/robocar.h"

#include <mutex>

#include <rapidjson/stringbuffer.h>
#include <zmq.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

namespace robocar::tod
{
	class VehicleData {
	public:
		VehicleData() {}

		uint64_t stamp = 0;
		uint64_t tod_msg_stamp = 0;
		bool ad_engaged = false;
		double steering = 0.0;
		double steering_n = 0.0;
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

	class TODComponent : public robocar::Component
	{
	public:
		TODComponent(const robocar::Params &params);

		void serve() override;

	private:
		// params
		double _target_velocity = 0.0;
		double _sl_threshold = 0.95;
		double _sl_smoothing = 0.75;
		double _max_steering = 0.57;
		double _max_throttle = 1.0;
		double _max_brake = 1.0;
		int _stream_width = 1280;
		int _stream_height = 720;
		int _stream_framerate = 30;
		int _stream_bitrate = 2500;

    	zmq::context_t _context;
		zmq::socket_t _sock_data;
		zmq::socket_t _sock_cmd;

		int _gst_argc = 0;
		char** _gst_argv = NULL;
		GstRTSPServer* _gst_rtsp_server;
		GstRTSPMountPoints* _gst_rtsp_mounts;
		GstRTSPMediaFactory* _gst_rtsp_factory;
		GMainLoop* _gst_loop;

		double _prev_throttle = 0.0;
		std::mutex _m_av;
		int _act_mode = ACT_OVERRIDE_NONE;
		VehicleData _vehicle_data;
		std::mutex _m_c;
		cv_bridge::CvImagePtr _cv_cam_1;

		// publishers
		rclcpp::Publisher<msg::AdToggle>::SharedPtr _pub_ad_toggle;
		rclcpp::Publisher<msg::ActCmd>::SharedPtr _pub_input;

		// subscribers
		rclcpp::Subscription<msg::ActCmd>::SharedPtr _sub_act_cmd;
		rclcpp::Subscription<msg::Vehicle>::SharedPtr _sub_vehicle;
		rclcpp::Subscription<msg::GNSS>::SharedPtr _sub_gnss;
		rclcpp::Subscription<msg::Image>::SharedPtr _sub_camera_0;
		rclcpp::Subscription<msg::Image>::SharedPtr _sub_camera_1;

		void on_tod_msg(const std::string& tod_msg);

		void on_act_cmd(const msg::ActCmd &act_cmd);
		void on_vehicle(const msg::Vehicle &vehicle);
		void on_gnss(const msg::GNSS &gnss);
		void on_camera_0(const msg::Image::ConstSharedPtr &img);
		void on_camera_1(const msg::Image::ConstSharedPtr &img);
	};
}

#endif // ROBOCAR_TOD_TOD_COMPONENT_H