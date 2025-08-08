/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_DRIVERS_CAMERA_CAMERA_COMPONENT_H
#define ROBOCAR_DRIVERS_CAMERA_CAMERA_COMPONENT_H

#include "robocar/core/robocar.h"

#include <opencv2/opencv.hpp>

namespace robocar::drivers::camera
{
	class CameraComponent : public robocar::Component
	{
	public:
		CameraComponent(const robocar::Params &params);

		void serve() override;

	private:
		// params
		int _cam_0_id = 0;
		int _cam_0_width = 1920;
		int _cam_0_height = 1080;
		bool _cam_0_flip = false;
		int _cam_1_id = 1;
		int _cam_1_width = 1920;
		int _cam_1_height = 1080;
		bool _cam_1_flip = false;

		cv::VideoCapture _cam_0;
		cv::VideoCapture _cam_1;
		cv::Mat _cam_0_mat;
		cv::Mat _cam_1_mat;

		// publishers
		rclcpp::Publisher<msg::Image>::SharedPtr _pub_cam_0;
		rclcpp::Publisher<msg::CompressedImage>::SharedPtr _pub_cam_0_c;
		rclcpp::Publisher<msg::Image>::SharedPtr _pub_cam_1;
		rclcpp::Publisher<msg::CompressedImage>::SharedPtr _pub_cam_1_c;
	};
}

#endif // ROBOCAR_DRIVERS_CAMERA_CAMERA_COMPONENT_H