/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "drivers/camera/camera_component.h"

#include <cv_bridge/cv_bridge.h>

using namespace robocar::drivers::camera;

CameraComponent::CameraComponent(const cycle::Params &params) : cycle::Service(params)
{
    // params
    _device_id = params.get("device_id").to_int();
    _width = params.get("width").to_int();
    if (_width < 1)
    {
        throw std::invalid_argument("'width' must be > 1");
    }
    _height = params.get("height").to_int();
    if (_height < 1)
    {
        throw std::invalid_argument("'height' must be > 1");
    }
    _flip = params.get("flip").to_bool();

    // open device
    _camera.open(_device_id, cv::CAP_V4L2);
    if (!_camera.isOpened())
    {
        throw std::runtime_error("unable to open camera using id: " + std::to_string(_device_id));
    }
    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _width);
    _camera.set(cv::CAP_PROP_FRAME_HEIGHT, _height);

    // publishers
    _pub_img = this->create_publisher<msg::Image>("sensors/camera",
                                                  rclcpp::SensorDataQoS().keep_last(1));
    _pub_c_img = this->create_publisher<msg::CompressedImage>("sensors/camera/compressed",
                                                              rclcpp::SensorDataQoS().keep_last(1));
}

void CameraComponent::serve()
{
    _camera.read(_cv_mat);
    if (!_cv_mat.empty())
    {
        auto now = this->get_clock()->now();

        if (_flip)
        {
            cv::flip(_cv_mat, _cv_mat, -1);
        }

        // header
        std_msgs::msg::Header header;
        header.stamp = now;
        header.frame_id = "vehicle";

        auto img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, _cv_mat);

        // publish image
        _pub_img->publish(*img_bridge.toImageMsg());
        // publish compressed image
        _pub_c_img->publish(*img_bridge.toCompressedImageMsg());
    }
}