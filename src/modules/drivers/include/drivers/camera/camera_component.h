/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef DRIVERS_CAMERA_CAMERA_COMPONENT_H
#define DRIVERS_CAMERA_CAMERA_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include <opencv2/opencv.hpp>

namespace robocar::drivers::camera {
	class CameraComponent : public cycle::Service {
	public:
		CameraComponent(const cycle::Params& params);

		void serve() override;

	private:
		// params
		int _device_id = 0;
		int _width = 1920;
		int _height = 1080;
		bool _flip = false;

		cv::VideoCapture _camera;
		cv::Mat _cv_mat;

		// publisher
		rclcpp::Publisher<msg::CompressedImage>::SharedPtr _pub_img;
	};
}

#endif // DRIVERS_CAMERA_CAMERA_COMPONENT_H