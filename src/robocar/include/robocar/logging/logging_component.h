/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_LOGGING_LOGGING_COMPONENT_H
#define ROBOCAR_LOGGING_LOGGING_COMPONENT_H

#include "robocar/core/robocar.h"

namespace robocar
{
	namespace logging
	{

		class LoggingComponent : public robocar::Component
		{
		public:
			LoggingComponent(const robocar::Params &params);
			~LoggingComponent();

			void serve() override;

		private:
			std::FILE *_file;
			std::mutex _mtx;
			std::vector<std::string> _names;
			std::unordered_map<std::string, double> _values;
			std::atomic<bool> _ad_engaged{false};

			// subscribers
			rclcpp::Subscription<msg::LogEntry>::SharedPtr _sub_entry;
			rclcpp::Subscription<msg::Vehicle>::SharedPtr _sub_vehicle;

			void write_entry(bool flush);
			void on_entry(const msg::LogEntry &entry);
			void on_vehicle(const msg::Vehicle &vehicle);
		};

	} // namespace logging
} // namespace robocar

#endif // ROBOCAR_LOGGING_LOGGING_COMPONENT_H