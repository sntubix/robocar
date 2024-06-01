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
        throw std::runtime_error("unable to open camera using id:" + std::to_string(_device_id));
    }
    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _width);
    _camera.set(cv::CAP_PROP_FRAME_HEIGHT, _height);

    // publisher
    _pub_img = this->create_publisher<msg::CompressedImage>("sensors/camera/compressed", 1);
}

void CameraComponent::serve()
{
    _camera.read(_cv_mat);
    if (!_cv_mat.empty())
    {
        if (_flip)
        {
            cv::flip(_cv_mat, _cv_mat, -1);
        }

        // convert to ROS msg
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "vehicle";
        auto img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, _cv_mat);
        auto img = img_bridge.toCompressedImageMsg();

        _pub_img->publish(*img);
    }
}