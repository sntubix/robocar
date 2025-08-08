/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_MAP_MAP_COMPONENT_H
#define ROBOCAR_MAP_MAP_COMPONENT_H

#include "robocar/core/robocar.h"

#include <visualization_msgs/msg/marker_array.hpp>

namespace robocar
{
	namespace map
	{

		class MapComponent : public robocar::Component
		{
		public:
			MapComponent(const robocar::Params &params);

			void serve() override;

		private:
			// params
			std::string _map_geofeatures;
			std::string _map_virtual_geofeatures;
			size_t _waypoints_length = 10;
			double _waypoints_interdistance = 1.0;
			double _waypoints_interdistance_max = 8.0;
			double _waypoints_delta = 0.5;
			double _x_overlap_offset = 0.7;

			msg::Path _trace;
			std::mutex _mtx;
			msg::Localization _loc;
			// markers
			visualization_msgs::msg::MarkerArray _geofeature_markers;
			visualization_msgs::msg::MarkerArray _virtual_geofeature_markers;
			visualization_msgs::msg::Marker _trace_marker;
			visualization_msgs::msg::Marker _waypoints_marker;

			// publishers
			rclcpp::Publisher<msg::Path>::SharedPtr _pub_waypoints;
			rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_geofeatures;
			rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_virtual_geofeatures;
			rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_trace_marker;
			rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_waypoints_marker;
			// subscribers
			rclcpp::Subscription<msg::Localization>::SharedPtr _sub_loc;

			msg::Path load_waypoints(const std::string &filename);
			size_t get_closest_waypoint(size_t frame_counter);
			// markers
			visualization_msgs::msg::MarkerArray get_geofeature_markers();
			visualization_msgs::msg::MarkerArray get_virtual_geofeature_markers();

			void on_localization(const msg::Localization &loc);
		};

	} // namespace map
} // namespace robocar

#endif // ROBOCAR_MAP_MAP_COMPONENT_H