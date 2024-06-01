/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_MESSAGES_H
#define ROBOCAR_MESSAGES_H

#include "msg_interfaces/msg/act_cmd.hpp"
#include "msg_interfaces/msg/act_status.hpp"
#include "msg_interfaces/msg/act_toggle.hpp"
#include "msg_interfaces/msg/ad_toggle.hpp"
#include "msg_interfaces/msg/gnss.hpp"
#include "msg_interfaces/msg/localization.hpp"
#include "msg_interfaces/msg/log_entry.hpp"
#include "msg_interfaces/msg/mapping.hpp"
#include "msg_interfaces/msg/map_waypoint.hpp"
#include "msg_interfaces/msg/object2d.hpp"
#include "msg_interfaces/msg/object3d.hpp"
#include "msg_interfaces/msg/objects2d.hpp"
#include "msg_interfaces/msg/objects3d.hpp"
#include "msg_interfaces/msg/objects_circles.hpp"
#include "msg_interfaces/msg/path.hpp"
#include "msg_interfaces/msg/planning.hpp"
#include "msg_interfaces/msg/target_speed.hpp"
#include "msg_interfaces/msg/traffic_light.hpp"
#include "msg_interfaces/msg/trajectory.hpp"
#include "msg_interfaces/msg/vehicle.hpp"
#include "msg_interfaces/msg/waypoint.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace msg_interfaces::msg
{
    typedef sensor_msgs::msg::PointCloud2 PointCloud;
    typedef sensor_msgs::msg::CompressedImage CompressedImage;
    typedef sensor_msgs::msg::Image Image;
}

using namespace msg_interfaces;

#endif // ROBOCAR_MESSAGES_H