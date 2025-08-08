/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/drivers/camera/camera_component.h"

#include <cv_bridge/cv_bridge.hpp>

using namespace robocar::drivers::camera;

CameraComponent::CameraComponent(const robocar::Params &params) : robocar::Component(params)
{
    // params
    _cam_0_id = params.get("cam_0_id").to_int();
    _cam_0_width = params.get("cam_0_width").to_int();
    if (_cam_0_width < 1)
    {
        throw std::invalid_argument("'cam_0_width' must be > 1");
    }
    _cam_0_height = params.get("cam_0_height").to_int();
    if (_cam_0_height < 1)
    {
        throw std::invalid_argument("'cam_0_height' must be > 1");
    }
    _cam_0_flip = params.get("cam_0_flip").to_bool();
    _cam_1_id = params.get("cam_1_id").to_int();
    _cam_1_width = params.get("cam_1_width").to_int();
    if (_cam_1_width < 1)
    {
        throw std::invalid_argument("'cam_1_width' must be > 1");
    }
    _cam_1_height = params.get("cam_1_height").to_int();
    if (_cam_1_height < 1)
    {
        throw std::invalid_argument("'cam_1_height' must be > 1");
    }
    _cam_1_flip = params.get("cam_1_flip").to_bool();

    // open camera 0
    if (_cam_0_id >= 0)
    {
        _cam_0.open(_cam_0_id, cv::CAP_V4L2);
        if (!_cam_0.isOpened())
        {
            throw std::runtime_error("unable to open camera using id: " + std::to_string(_cam_0_id));
        }
        _cam_0.set(cv::CAP_PROP_FRAME_WIDTH, _cam_0_width);
        _cam_0.set(cv::CAP_PROP_FRAME_HEIGHT, _cam_0_height);
    }

    // open camera 1
    if (_cam_1_id >= 0)
    {
        _cam_1.open(_cam_1_id, cv::CAP_V4L2);
        if (!_cam_1.isOpened())
        {
            throw std::runtime_error("unable to open camera using id: " + std::to_string(_cam_1_id));
        }
        _cam_1.set(cv::CAP_PROP_FRAME_WIDTH, _cam_1_width);
        _cam_1.set(cv::CAP_PROP_FRAME_HEIGHT, _cam_1_height);
    }

    // publishers
    _pub_cam_0 = this->create_publisher<msg::Image>("sensors/camera_0",
                                                    rclcpp::SensorDataQoS().keep_last(1));
    _pub_cam_0_c = this->create_publisher<msg::CompressedImage>("sensors/camera_0/compressed",
                                                                rclcpp::SensorDataQoS().keep_last(1));
    _pub_cam_1 = this->create_publisher<msg::Image>("sensors/camera_1",
                                                    rclcpp::SensorDataQoS().keep_last(1));
    _pub_cam_1_c = this->create_publisher<msg::CompressedImage>("sensors/camera_1/compressed",
                                                                rclcpp::SensorDataQoS().keep_last(1));
}

void CameraComponent::serve()
{
    auto now = this->get_clock()->now();

    if (_cam_0_id >= 0)
    {
        _cam_0.read(_cam_0_mat);

        if (!_cam_0_mat.empty())
        {
            if ((_cam_0_mat.cols != _cam_0_width) || (_cam_0_mat.rows != _cam_0_height)) {
                cv::resize(_cam_0_mat, _cam_0_mat, cv::Size(_cam_0_width, _cam_0_height));
            }

            if (_cam_0_flip)
            {
                cv::flip(_cam_0_mat, _cam_0_mat, -1);
            }

            // header
            std_msgs::msg::Header header;
            header.stamp = now;
            header.frame_id = "vehicle";

            auto img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, _cam_0_mat);

            // publish image
            _pub_cam_0->publish(*img_bridge.toImageMsg());
            // publish compressed image
            _pub_cam_0_c->publish(*img_bridge.toCompressedImageMsg());
        }
    }

    if (_cam_1_id >= 0)
    {
        _cam_1.read(_cam_1_mat);

        if (!_cam_1_mat.empty())
        {
            if ((_cam_1_mat.cols != _cam_1_width) || (_cam_1_mat.rows != _cam_1_height)) {
                cv::resize(_cam_1_mat, _cam_1_mat, cv::Size(_cam_1_width, _cam_1_height));
            }

            if (_cam_1_flip)
            {
                cv::flip(_cam_1_mat, _cam_1_mat, -1);
            }

            // header
            std_msgs::msg::Header header;
            header.stamp = now;
            header.frame_id = "vehicle";

            auto img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, _cam_1_mat);

            // publish image
            _pub_cam_1->publish(*img_bridge.toImageMsg());
            // publish compressed image
            _pub_cam_1_c->publish(*img_bridge.toCompressedImageMsg());
        }
    }
}