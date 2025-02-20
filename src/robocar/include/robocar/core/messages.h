/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_CORE_MESSAGES_H
#define ROBOCAR_CORE_MESSAGES_H

#include "robocar_msgs/msg/act_cmd.hpp"
#include "robocar_msgs/msg/act_status.hpp"
#include "robocar_msgs/msg/act_toggle.hpp"
#include "robocar_msgs/msg/ad_toggle.hpp"
#include "robocar_msgs/msg/gnss.hpp"
#include "robocar_msgs/msg/localization.hpp"
#include "robocar_msgs/msg/log_entry.hpp"
#include "robocar_msgs/msg/mapping.hpp"
#include "robocar_msgs/msg/map_waypoint.hpp"
#include "robocar_msgs/msg/object2d.hpp"
#include "robocar_msgs/msg/object3d.hpp"
#include "robocar_msgs/msg/objects2d.hpp"
#include "robocar_msgs/msg/objects3d.hpp"
#include "robocar_msgs/msg/objects_circles.hpp"
#include "robocar_msgs/msg/path.hpp"
#include "robocar_msgs/msg/planning.hpp"
#include "robocar_msgs/msg/target_speed.hpp"
#include "robocar_msgs/msg/traffic_light.hpp"
#include "robocar_msgs/msg/trajectory.hpp"
#include "robocar_msgs/msg/vehicle.hpp"
#include "robocar_msgs/msg/waypoint.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace robocar_msgs::msg
{
    typedef sensor_msgs::msg::PointCloud2 PointCloud;
    typedef sensor_msgs::msg::CompressedImage CompressedImage;
    typedef sensor_msgs::msg::Image Image;
}

using namespace robocar_msgs;

#endif // ROBOCAR_CORE_MESSAGES_H