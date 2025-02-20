/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/visualization/visualization_component.h"
#include "robocar/core/utils/geodesy.h"

#include <filesystem>

#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace robocar;
using namespace robocar::visualization;
using namespace robocar::utils;

VisualizationComponent::VisualizationComponent(const robocar::Params &params) : robocar::Component(params)
{
    // params
    _vehicle_mesh = "file://" + std::filesystem::canonical(params.get("vehicle_mesh").to_string()).string();
    _map_geofeatures = std::filesystem::canonical(params.get("map_geofeatures").to_string()).string();
    _map_virtual_geofeatures = std::filesystem::canonical(params.get("map_virtual_geofeatures").to_string()).string();

    // init node
    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    // publishers
    _pub_vehicle = this->create_publisher<visualization_msgs::msg::Marker>("viz/vehicle/mesh", 1);
    _pub_trace = this->create_publisher<visualization_msgs::msg::Marker>("viz/map/trace", 1);
    _pub_geofeatures = this->create_publisher<visualization_msgs::msg::MarkerArray>("viz/map/geofeatures", 1);
    _pub_virtual_geofeatures = this->create_publisher<visualization_msgs::msg::MarkerArray>("viz/map/virtual_geofeatures", 1);
    _pub_waypoints = this->create_publisher<visualization_msgs::msg::Marker>("viz/map/waypoints", 1);
    _pub_trajectory = this->create_publisher<visualization_msgs::msg::Marker>("viz/planning/trajectory", 1);
    _pub_target = this->create_publisher<visualization_msgs::msg::Marker>("viz/control/target", 1);
    _pub_objects3d = this->create_publisher<visualization_msgs::msg::MarkerArray>("viz/perception/lidar/objects3d", 2);
    _pub_objects_circles = this->create_publisher<visualization_msgs::msg::MarkerArray>("viz/planning/objects_circles", 2);
    _pub_image = this->create_publisher<msg::Image>("viz/perception/camera/objects2d",
                                                    rclcpp::SensorDataQoS().keep_last(1));

    // subscribers
    _sub_loc = this->create_subscription<msg::Localization>("localization/position", 1,
                                                            std::bind(&VisualizationComponent::on_localization,
                                                                      this, std::placeholders::_1));
    _sub_trace = this->create_subscription<msg::Path>("map/trace", 1,
                                                      std::bind(&VisualizationComponent::on_trace,
                                                                this, std::placeholders::_1));
    _sub_waypoints = this->create_subscription<msg::Path>("map/waypoints", 1,
                                                          std::bind(&VisualizationComponent::on_waypoints,
                                                                    this, std::placeholders::_1));
    _sub_trajectory = this->create_subscription<msg::Planning>("planning/trajectory", 1,
                                                               std::bind(&VisualizationComponent::on_trajectory,
                                                                         this, std::placeholders::_1));
    _sub_target = this->create_subscription<msg::Waypoint>("control/target", 1,
                                                           std::bind(&VisualizationComponent::on_control_target,
                                                                     this, std::placeholders::_1));
    _sub_objects3d = this->create_subscription<msg::Objects3d>("perception/lidar/objects3d", 1,
                                                               std::bind(&VisualizationComponent::on_objects3d,
                                                                         this, std::placeholders::_1));
    _sub_objects2d = this->create_subscription<msg::Objects2d>("perception/camera/objects2d", 1,
                                                               std::bind(&VisualizationComponent::on_objects2d,
                                                                         this, std::placeholders::_1));
    _sub_objects_circles = this->create_subscription<msg::ObjectsCircles>("planning/objects_circles", 1,
                                                                          std::bind(&VisualizationComponent::on_objects_circles,
                                                                                    this, std::placeholders::_1));
    _sub_image = this->create_subscription<msg::CompressedImage>("sensors/camera/compressed",
                                                                 rclcpp::SensorDataQoS().keep_last(1),
                                                                 std::bind(&VisualizationComponent::on_image,
                                                                           this, std::placeholders::_1));

    // vehicle marker
    _vehicle_marker.id = 0;
    _vehicle_marker.header.stamp = rclcpp::Time(0);
    _vehicle_marker.header.frame_id = "vehicle";
    _vehicle_marker.lifetime = rclcpp::Duration(0, 0);
    _vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
    _vehicle_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    _vehicle_marker.mesh_use_embedded_materials = true;
    _vehicle_marker.mesh_resource = _vehicle_mesh;
    _vehicle_marker.scale.x = 0.006;
    _vehicle_marker.scale.y = 0.006;
    _vehicle_marker.scale.z = 0.006;
    _vehicle_marker.pose.orientation.x = 0.5;
    _vehicle_marker.pose.orientation.y = 0.5;
    _vehicle_marker.pose.orientation.z = 0.5;
    _vehicle_marker.pose.orientation.w = 0.5;

    // path marker
    _path_marker.id = 0;
    _path_marker.header.stamp = rclcpp::Time(0);
    _path_marker.header.frame_id = "map";
    _path_marker.action = visualization_msgs::msg::Marker::ADD;
    _path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    _path_marker.pose.orientation.w = 1.0;
    _path_marker.scale.x = 0.4;
    _path_marker.scale.y = 0.4;
    _path_marker.color.a = 0.5f;
    _path_marker.color.r = 0.7f;
    _path_marker.color.g = 0.7f;
    _path_marker.color.b = 0.7f;

    // load map markers
    _geofeature_markers = get_geofeature_markers();
    _virtual_geofeature_markers = get_virtual_geofeature_markers();

    // waypoints marker
    _waypoints_marker.id = 0;
    _waypoints_marker.header.stamp = rclcpp::Time(0);
    _waypoints_marker.header.frame_id = "map";

    // trajectory marker
    _trajectory_marker.id = 1;
    _trajectory_marker.header.stamp = rclcpp::Time(0);
    _trajectory_marker.header.frame_id = "map";
}

void VisualizationComponent::serve()
{
    try
    {
        // wait for transform
        auto tf = _tf_buffer->lookupTransform("map", "vehicle", tf2::TimePointZero);

        std::unique_lock<std::mutex> lock(_m_r);

        // update stamps
        auto tf_stamp = tf.header.stamp;
        _vehicle_marker.header.stamp = tf_stamp;
        _path_marker.header.stamp = tf_stamp;
        for (size_t i = 0; i < _geofeature_markers.markers.size(); i++)
        {
            _geofeature_markers.markers.at(i).header.stamp = tf_stamp;
        }
        for (size_t i = 0; i < _virtual_geofeature_markers.markers.size(); i++)
        {
            _virtual_geofeature_markers.markers.at(i).header.stamp = tf_stamp;
        }
        _waypoints_marker.header.stamp = tf_stamp;
        _trajectory_marker.header.stamp = tf_stamp;
        for (size_t i = 0; i < _circle_markers.markers.size(); i++)
        {
            _circle_markers.markers[i].header.stamp = tf_stamp;
        }

        // publish markers
        _pub_vehicle->publish(_vehicle_marker);
        _pub_trace->publish(_path_marker);
        _pub_geofeatures->publish(_geofeature_markers);
        _pub_virtual_geofeatures->publish(_virtual_geofeature_markers);
        _pub_waypoints->publish(_waypoints_marker);
        _pub_trajectory->publish(_trajectory_marker);
        _pub_objects_circles->publish(_circle_markers);
    }
    catch (tf2::TransformException &e)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", e.what());
    }
}

void VisualizationComponent::on_localization(const msg::Localization &loc)
{
    tf2::Quaternion quaternion;
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = loc.header.stamp;
    tf_stamped.header.frame_id = "map";
    tf_stamped.child_frame_id = "vehicle";
    tf_stamped.transform.translation.x = loc.x;
    tf_stamped.transform.translation.y = loc.y;
    tf_stamped.transform.translation.z = 0.0;
    quaternion.setRPY(0, 0, loc.yaw);
    tf_stamped.transform.rotation.x = quaternion.x();
    tf_stamped.transform.rotation.y = quaternion.y();
    tf_stamped.transform.rotation.z = quaternion.z();
    tf_stamped.transform.rotation.w = quaternion.w();
    _tf_broadcaster->sendTransform(tf_stamped);
}

void VisualizationComponent::on_trace(const msg::Path &trace)
{
    std::unique_lock<std::mutex> lock(_m_r);

    _path_marker.header.stamp = this->get_clock()->now();
    for (size_t i = 0; i < trace.waypoints.size(); i++)
    {
        geometry_msgs::msg::Point point;
        point.x = trace.waypoints[i].x;
        point.y = trace.waypoints[i].y;
        _path_marker.points.push_back(point);
    }
}

void VisualizationComponent::on_waypoints(const msg::Path &waypoints)
{
    std::unique_lock<std::mutex> lock(_m_r);
    _waypoints_marker = get_path_marker(waypoints, "map", 0, 0.0, 0.0, 0.5);
}

void VisualizationComponent::on_trajectory(const msg::Planning &planning)
{
    std::unique_lock<std::mutex> lock(_m_r);

    msg::Path p;
    for (auto &w : planning.trajectory.waypoints)
    {
        msg::MapWaypoint waypoint;
        waypoint.x = w.x;
        waypoint.y = w.y;
        waypoint.yaw = w.yaw;
        waypoint.vel = w.vel;
        waypoint.tfl = false;
        waypoint.tfl_right = false;
        p.waypoints.push_back(waypoint);
    }

    _trajectory_marker = get_path_marker(p, planning.header.frame_id, 1, 0.2, 1.0, 0.2);
}

void VisualizationComponent::on_control_target(const msg::Waypoint &target)
{
    msg::MapWaypoint waypoint;
    waypoint.x = target.x;
    waypoint.y = target.y;
    waypoint.yaw = target.yaw;
    waypoint.vel = target.vel;
    waypoint.tfl = false;
    waypoint.tfl_right = false;

    msg::Path p;
    p.waypoints.push_back(waypoint);

    _pub_target->publish(get_path_marker(p, "vehicle", 2, 0.8, 0.0, 0.2));
}

visualization_msgs::msg::Marker VisualizationComponent::get_path_marker(const msg::Path &p, std::string frame,
                                                                        int id, float r, float g, float b)
{
    visualization_msgs::msg::Marker path_marker;
    path_marker.id = id;
    path_marker.header.stamp = this->get_clock()->now();
    path_marker.header.frame_id = frame;
    if (p.waypoints.size() == 0)
    {
        return path_marker;
    }

    path_marker.action = visualization_msgs::msg::Marker::ADD;
    if (p.waypoints.size() > 1)
    {
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    }
    else
    {
        path_marker.type = visualization_msgs::msg::Marker::POINTS;
    }
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.5;
    path_marker.scale.y = 0.5;
    path_marker.color.a = 0.5f;
    path_marker.color.r = r;
    path_marker.color.g = g;
    path_marker.color.b = b;

    geometry_msgs::msg::Point waypoint;
    path_marker.points.clear();

    for (uint32_t i = 0; i < p.waypoints.size(); i++)
    {
        waypoint.x = p.waypoints[i].x;
        waypoint.y = p.waypoints[i].y;
        path_marker.points.push_back(waypoint);
    }

    return path_marker;
}

void VisualizationComponent::on_objects3d(const msg::Objects3d &objects)
{
    auto now = this->get_clock()->now();
    size_t nb_objects = objects.objects.size();
    size_t max_nb_objects = std::max(_prev_nb_objects, nb_objects);
    visualization_msgs::msg::MarkerArray object_markers;

    for (size_t i = 0; i < max_nb_objects; i++)
    {
        visualization_msgs::msg::Marker bbox;
        bbox.id = i;
        bbox.header.stamp = now;
        bbox.header.frame_id = "vehicle";
        bbox.ns = "objects3d";

        if (i < nb_objects)
        {
            auto &object = objects.objects[i];
            bbox.header = object.header;

            // create bbox
            geometry_msgs::msg::Point p[24];
            // ground area
            // p0 -> p1
            p[0].x = object.points[0].x;
            p[0].y = object.points[0].y;
            p[0].z = object.points[0].z;
            p[1].x = object.points[1].x;
            p[1].y = object.points[1].y;
            p[1].z = object.points[1].z;
            // p1 -> p2
            p[2].x = object.points[1].x;
            p[2].y = object.points[1].y;
            p[2].z = object.points[1].z;
            p[3].x = object.points[2].x;
            p[3].y = object.points[2].y;
            p[3].z = object.points[2].z;
            // p2 -> p3
            p[4].x = object.points[2].x;
            p[4].y = object.points[2].y;
            p[4].z = object.points[2].z;
            p[5].x = object.points[3].x;
            p[5].y = object.points[3].y;
            p[5].z = object.points[3].z;
            // p3 -> p0
            p[6].x = object.points[3].x;
            p[6].y = object.points[3].y;
            p[6].z = object.points[3].z;
            p[7].x = object.points[0].x;
            p[7].y = object.points[0].y;
            p[7].z = object.points[0].z;

            // top area
            // p4 -> p5
            p[8].x = object.points[4].x;
            p[8].y = object.points[4].y;
            p[8].z = object.points[4].z;
            p[9].x = object.points[5].x;
            p[9].y = object.points[5].y;
            p[9].z = object.points[5].z;
            // p5 -> p6
            p[10].x = object.points[5].x;
            p[10].y = object.points[5].y;
            p[10].z = object.points[5].z;
            p[11].x = object.points[6].x;
            p[11].y = object.points[6].y;
            p[11].z = object.points[6].z;
            // p6 -> p7
            p[12].x = object.points[6].x;
            p[12].y = object.points[6].y;
            p[12].z = object.points[6].z;
            p[13].x = object.points[7].x;
            p[13].y = object.points[7].y;
            p[13].z = object.points[7].z;
            // p7 -> p4
            p[14].x = object.points[7].x;
            p[14].y = object.points[7].y;
            p[14].z = object.points[7].z;
            p[15].x = object.points[4].x;
            p[15].y = object.points[4].y;
            p[15].z = object.points[4].z;

            // side areas
            // p0 -> p4
            p[16].x = object.points[0].x;
            p[16].y = object.points[0].y;
            p[16].z = object.points[0].z;
            p[17].x = object.points[4].x;
            p[17].y = object.points[4].y;
            p[17].z = object.points[4].z;
            // p3 -> p7
            p[18].x = object.points[3].x;
            p[18].y = object.points[3].y;
            p[18].z = object.points[3].z;
            p[19].x = object.points[7].x;
            p[19].y = object.points[7].y;
            p[19].z = object.points[7].z;
            // p2 -> p6
            p[20].x = object.points[2].x;
            p[20].y = object.points[2].y;
            p[20].z = object.points[2].z;
            p[21].x = object.points[6].x;
            p[21].y = object.points[6].y;
            p[21].z = object.points[6].z;
            // p1 -> p5
            p[22].x = object.points[1].x;
            p[22].y = object.points[1].y;
            p[22].z = object.points[1].z;
            p[23].x = object.points[5].x;
            p[23].y = object.points[5].y;
            p[23].z = object.points[5].z;

            // marker
            bbox.type = visualization_msgs::msg::Marker::LINE_LIST;
            bbox.scale.x = 0.2;
            bbox.color = std_msgs::msg::ColorRGBA();
            bbox.color.a = 1.0f;
            bbox.color.r = 0.2f;
            bbox.color.g = 0.6f;
            bbox.color.b = 1.0f;
            for (int j = 0; j < 24; ++j)
                bbox.points.push_back(p[j]);
            bbox.pose.orientation.x = 0;
            bbox.pose.orientation.y = 0;
            bbox.pose.orientation.z = 0;
            bbox.pose.orientation.w = 1;
        }
        else
        {
            bbox.action = bbox.DELETE;
        }

        object_markers.markers.push_back(bbox);
    }

    _pub_objects3d->publish(object_markers);
    _prev_nb_objects = nb_objects;
}

void VisualizationComponent::on_objects2d(const msg::Objects2d &objects)
{
    _m_o.lock();
    _objects2d = objects;
    _m_o.unlock();
}

void VisualizationComponent::on_objects_circles(const msg::ObjectsCircles &objects_circles)
{
    std::unique_lock<std::mutex> lock(_m_r);

    auto now = this->get_clock()->now();
    size_t object_dim = 4;
    size_t nb_circles = objects_circles.data.size() / object_dim;
    _circle_markers.markers.clear();

    if (_prev_nb_circles > nb_circles)
    {
        visualization_msgs::msg::Marker marker;
        marker.id = -1;
        marker.header.stamp = now;
        marker.header.frame_id = "map";
        marker.ns = "object_circle";
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        _circle_markers.markers.push_back(marker);
    }
    _prev_nb_circles = nb_circles;

    size_t offset = 0;
    for (size_t i = 0; i < nb_circles; i++)
    {
        visualization_msgs::msg::Marker marker;
        marker.id = i;
        marker.header.stamp = now;
        marker.header.frame_id = "map";
        marker.ns = "object_circle";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.pose.position.x = objects_circles.data[offset];
        marker.pose.position.y = objects_circles.data[offset + 1];
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0 * objects_circles.data[offset + 2]; // diameter of the circle
        marker.scale.y = 2.0 * objects_circles.data[offset + 2]; // diameter of the circle
        marker.scale.z = 0.05;                                   // thickness of the circle
        marker.color.a = 0.7f;
        marker.color.r = 0.95f;
        marker.color.g = 0.95f;
        marker.color.b = 0.95f;
        _circle_markers.markers.push_back(marker);

        offset += object_dim;
    }
}

void VisualizationComponent::on_image(const msg::CompressedImage::ConstSharedPtr &image)
{
    // check image
    if (image->format.empty() || image->data.empty())
    {
        return;
    }

    std::unique_lock<std::mutex> lock(_m_o);

    // convert to cv image
    auto cv_mat = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::BGR8);

    // draw objects
    for (auto &obj : _objects2d.objects)
    {
        auto p1 = cv::Point(obj.x1, obj.y1);
        auto p2 = cv::Point(obj.x2, obj.y2);

        if (obj.type == OBJ_TFL_GREEN)
        {
            cv::rectangle(cv_mat->image, p1, p2, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
        }
        if (obj.type == OBJ_TFL_YELLOW)
        {
            cv::rectangle(cv_mat->image, p1, p2, cv::Scalar(0, 255, 255), 2, cv::LINE_8);
        }
        if (obj.type == OBJ_TFL_RED)
        {
            cv::rectangle(cv_mat->image, p1, p2, cv::Scalar(0, 0, 255), 2, cv::LINE_8);
        }
        if (obj.type == OBJ_TFL_NONE)
        {
            cv::rectangle(cv_mat->image, p1, p2, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        }
    }

    // convert to ROS msg
    auto img_bridge = cv_bridge::CvImage(image->header,
                                         sensor_msgs::image_encodings::BGR8,
                                         cv_mat->image);
    auto img = img_bridge.toImageMsg();

    _pub_image->publish(*img);
}

visualization_msgs::msg::MarkerArray VisualizationComponent::get_geofeature_markers()
{
    visualization_msgs::msg::MarkerArray geofeatures;

    if (!std::filesystem::exists(_map_geofeatures))
        throw std::runtime_error("map geofeatures file does not exist '" + _map_geofeatures + "'");

    std::ifstream fs(_map_geofeatures, std::ios::in);
    std::string json((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
    rapidjson::Document d;
    d.Parse(json.c_str());
    if (!d.IsObject())
        throw std::runtime_error("unable to parse geofeatures map file '" + _map_geofeatures + "'");

    visualization_msgs::msg::Marker feature_marker;
    feature_marker.id = 0;
    feature_marker.header.stamp = this->get_clock()->now();
    feature_marker.header.frame_id = "map";
    feature_marker.lifetime = rclcpp::Duration(0, 0);
    feature_marker.action = visualization_msgs::msg::Marker::ADD;
    feature_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    feature_marker.pose.orientation.w = 1.0;
    feature_marker.scale.x = 0.25;
    feature_marker.scale.y = 0.25;
    feature_marker.color.a = 0.5f;
    feature_marker.color.r = 66.0f / 255.0f;
    feature_marker.color.g = 245.0f / 255.0f;
    feature_marker.color.b = 221.0f / 255.0f;

    rapidjson::Value &features = d["features"];
    for (int i = 0; i < static_cast<int>(features.Size()); i++)
    {
        rapidjson::Value &feature = features[i];
        rapidjson::Value &coordinates = feature["geometry"]["coordinates"];

        feature_marker.id = i;
        feature_marker.points.clear();
        for (size_t j = 0; j < static_cast<int>(coordinates.Size()); j++)
        {
            double longitude = coordinates[j][0].GetDouble();
            double latitude = coordinates[j][1].GetDouble();
            auto pos = geodesy::Geodesy::get_pos(latitude * M_PI / 180.0,
                                                 longitude * M_PI / 180.0);
            geometry_msgs::msg::Point point;
            point.x = pos.x();
            point.y = pos.y();
            feature_marker.points.push_back(point);
        }

        if (!feature_marker.points.empty())
        {
            geofeatures.markers.push_back(feature_marker);
        }
    }
    fs.close();

    return geofeatures;
}

visualization_msgs::msg::MarkerArray VisualizationComponent::get_virtual_geofeature_markers()
{
    visualization_msgs::msg::MarkerArray virtual_geofeatures;

    if (!std::filesystem::exists(_map_virtual_geofeatures))
        throw std::runtime_error("map virtual geofeatures file does not exist '" + _map_virtual_geofeatures + "'");

    std::ifstream fs(_map_virtual_geofeatures, std::ios::in);
    std::string json((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
    rapidjson::Document d;
    d.Parse(json.c_str());
    if (!d.IsObject())
        throw std::runtime_error("unable to parse virtual geofeatures map file '" + _map_virtual_geofeatures + "'");

    visualization_msgs::msg::Marker feature_marker;
    feature_marker.id = 0;
    feature_marker.header.stamp = this->get_clock()->now();
    feature_marker.header.frame_id = "map";
    feature_marker.lifetime = rclcpp::Duration(0, 0);
    feature_marker.action = visualization_msgs::msg::Marker::ADD;
    feature_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    feature_marker.pose.orientation.w = 1.0;
    feature_marker.scale.x = 0.25;
    feature_marker.scale.y = 0.25;
    feature_marker.color.a = 0.5f;
    feature_marker.color.r = 215.0f / 255.0f;
    feature_marker.color.g = 191.0f / 255.0f;
    feature_marker.color.b = 69.0f / 255.0f;

    rapidjson::Value &features = d["features"];
    for (int i = 0; i < static_cast<int>(features.Size()); ++i)
    {
        rapidjson::Value &feature = features[i];
        rapidjson::Value &coordinates = feature["geometry"]["coordinates"];

        feature_marker.id = i;
        feature_marker.points.clear();
        for (size_t j = 0; j < static_cast<int>(coordinates.Size()); j++)
        {
            double longitude = coordinates[j][0].GetDouble();
            double latitude = coordinates[j][1].GetDouble();
            auto pos = geodesy::Geodesy::get_pos(latitude * M_PI / 180.0,
                                                 longitude * M_PI / 180.0);
            geometry_msgs::msg::Point point;
            point.x = pos.x();
            point.y = pos.y();
            feature_marker.points.push_back(point);
        }

        if (!feature_marker.points.empty())
        {
            virtual_geofeatures.markers.push_back(feature_marker);
        }
    }
    fs.close();

    return virtual_geofeatures;
}