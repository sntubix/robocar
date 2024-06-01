/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef LOCALIZATION_LOCALIZATION_COMPONENT_H
#define LOCALIZATION_LOCALIZATION_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

namespace robocar::localization
{
	class LocalizationComponent : public cycle::Module
	{
	public:
		LocalizationComponent(const cycle::Params &params);

	private:
		// publishers
		rclcpp::Publisher<msg::Localization>::SharedPtr _pub_loc;
		rclcpp::Publisher<msg::LogEntry>::SharedPtr _pub_log;
		// subscriber
		rclcpp::Subscription<msg::GNSS>::SharedPtr _sub_gnss;

		void on_gnss(const msg::GNSS &gnss);
	};
}

#endif // LOCALIZATION_LOCALIZATION_COMPONENT_H