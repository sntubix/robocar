/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_LOCALIZATION_LOCALIZATION_COMPONENT_H
#define ROBOCAR_LOCALIZATION_LOCALIZATION_COMPONENT_H

#include "robocar/core/robocar.h"

namespace robocar::localization
{
	class LocalizationComponent : public robocar::Component
	{
	public:
		LocalizationComponent(const robocar::Params &params);

	private:
		// publishers
		rclcpp::Publisher<msg::Localization>::SharedPtr _pub_loc;
		rclcpp::Publisher<msg::LogEntry>::SharedPtr _pub_log;
		// subscriber
		rclcpp::Subscription<msg::GNSS>::SharedPtr _sub_gnss;

		void on_gnss(const msg::GNSS &gnss);
	};
}

#endif // ROBOCAR_LOCALIZATION_LOCALIZATION_COMPONENT_H