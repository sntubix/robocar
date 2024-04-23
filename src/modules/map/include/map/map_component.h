/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef MAP_MAP_COMPONENT_H
#define MAP_MAP_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

namespace robocar {
namespace map {

class MapComponent : public cycle::Service {
	public:
		MapComponent(const cycle::Params& params);

		void serve() override;

	private:
		// params
		size_t _waypoints_length = 10;
		double _waypoints_interdistance = 1.0;
		double _waypoints_interdistance_max = 8.0;
		double _waypoints_delta = 0.5;
		double _x_overlap_offset = 0.7;

		bool _pub_trace = true;
		msg::Path _trace;
		std::mutex _mtx;
		msg::Localization _loc;

		// publishers
		rclcpp::Publisher<msg::Path>::SharedPtr _pub_path_trace;
		rclcpp::Publisher<msg::Path>::SharedPtr _pub_path_waypoints;
		// subscribers
		rclcpp::Subscription<msg::Localization>::SharedPtr _sub_loc;

		msg::Path load_waypoints(const std::string &filename);
		size_t get_closest_waypoint(size_t frame_counter);
		void on_localization(const msg::Localization& loc);
};

}	// namespace map
}	// namespace robocar

#endif // MAP_MAP_COMPONENT_H