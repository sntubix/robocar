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
		int _device_id = 0;
		int _width = 1920;
		int _height = 1080;
		bool _flip = false;

		cv::VideoCapture _camera;
		cv::Mat _cv_mat;

		// publishers
		rclcpp::Publisher<msg::Image>::SharedPtr _pub_img;
		rclcpp::Publisher<msg::CompressedImage>::SharedPtr _pub_c_img;
	};
}

#endif // ROBOCAR_DRIVERS_CAMERA_CAMERA_COMPONENT_H