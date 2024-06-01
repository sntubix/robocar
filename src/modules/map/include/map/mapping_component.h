/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef MAP_MAPPING_COMPONENT_H
#define MAP_MAPPING_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include <deque>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace robocar
{
	namespace map
	{

		class MappingComponent : public cycle::Service
		{
		public:
			MappingComponent(const cycle::Params &params);

			void serve() override;

		private:
			// params
			double _waypoints_interval = 1.0;

			bool _mapping = false;
			msg::GNSS _prev_gnss;
			std::mutex _m;
			size_t _counter = 0;
			std::deque<msg::GNSS> _queue;
			std::FILE *_file;
			std::string _filename = "";
			std::unique_ptr<rapidjson::StringBuffer> _buffer;
			std::unique_ptr<rapidjson::Writer<rapidjson::StringBuffer>> _writer;

			// publishers
			rclcpp::Publisher<msg::Mapping>::SharedPtr _pub_toggle;
			rclcpp::Publisher<msg::Mapping>::SharedPtr _pub_status;
			// subscribers
			rclcpp::Subscription<msg::Mapping>::SharedPtr _sub_toggle;
			rclcpp::Subscription<msg::Vehicle>::SharedPtr _sub_vehicle;
			rclcpp::Subscription<msg::GNSS>::SharedPtr _sub_gnss;

			void flush_queue();
			double compute_dist(const msg::GNSS &gnss_a, const msg::GNSS &gnss_b);

			void on_toggle(const msg::Mapping &mapping);
			void on_vehicle(const msg::Vehicle &vehicle);
			void on_gnss(const msg::GNSS &gnss);
		};

	} // namespace map
} // namespace robocar

#endif // MAP_MAPPING_COMPONENT_H