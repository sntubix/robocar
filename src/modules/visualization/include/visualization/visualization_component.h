/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef VISUALIZATION_VISUALIZATION_COMPONENT_H
#define VISUALIZATION_VISUALIZATION_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace robocar::visualization
{
	class VisualizationComponent : public cycle::Service
	{
	public:
		VisualizationComponent(const cycle::Params &params);

		void serve() override;

	private:
		// params
		std::string _vehicle_mesh;
		std::string _map_geofeatures;
		std::string _map_virtual_geofeatures;

		size_t _prev_nb_objects = 0;
		size_t _prev_nb_circles = 0;
		std::mutex _m_o;
		msg::Objects2d _objects2d;

		// publishers
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_vehicle;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_trace;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_geofeatures;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_virtual_geofeatures;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_waypoints;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_trajectory;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_target;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_objects3d;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_objects_circles;
		rclcpp::Publisher<msg::Image>::SharedPtr _pub_image;

		// subscribers
		rclcpp::Subscription<msg::Localization>::SharedPtr _sub_loc;
		rclcpp::Subscription<msg::Path>::SharedPtr _sub_trace;
		rclcpp::Subscription<msg::Path>::SharedPtr _sub_waypoints;
		rclcpp::Subscription<msg::Planning>::SharedPtr _sub_trajectory;
		rclcpp::Subscription<msg::Waypoint>::SharedPtr _sub_target;
		rclcpp::Subscription<msg::Objects3d>::SharedPtr _sub_objects3d;
		rclcpp::Subscription<msg::Objects2d>::SharedPtr _sub_objects2d;
		rclcpp::Subscription<msg::ObjectsCircles>::SharedPtr _sub_objects_circles;
		rclcpp::Subscription<msg::CompressedImage>::SharedPtr _sub_image;

		std::mutex _m_r;
		// transform
		std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
		std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
		std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
		// markers
		visualization_msgs::msg::Marker _vehicle_marker;
		visualization_msgs::msg::Marker _path_marker;
		visualization_msgs::msg::MarkerArray _geofeature_markers;
		visualization_msgs::msg::MarkerArray _virtual_geofeature_markers;
		visualization_msgs::msg::Marker _waypoints_marker;
		visualization_msgs::msg::Marker _trajectory_marker;
		visualization_msgs::msg::MarkerArray _circle_markers;

		void on_localization(const msg::Localization &loc);
		void on_trace(const msg::Path &trace);
		void on_waypoints(const msg::Path &waypoints);
		void on_trajectory(const msg::Planning &trajectory);
		void on_control_target(const msg::Waypoint &target);
		void on_objects3d(const msg::Objects3d &objects);
		void on_objects2d(const msg::Objects2d &objects);
		void on_objects_circles(const msg::ObjectsCircles &objects_circles);
		void on_image(const msg::CompressedImage::ConstSharedPtr &image);

		visualization_msgs::msg::MarkerArray get_geofeature_markers();
		visualization_msgs::msg::MarkerArray get_virtual_geofeature_markers();
		visualization_msgs::msg::Marker get_path_marker(const msg::Path &p, std::string frame,
														int id, float r, float g, float b);
	};
}

#endif // VISUALIZATION_VISUALIZATION_COMPONENT_H