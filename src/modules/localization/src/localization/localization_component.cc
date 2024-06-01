/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "localization/localization_component.h"
#include "common/geodesy.h"

using namespace robocar::localization;

LocalizationComponent::LocalizationComponent(const cycle::Params &params) : cycle::Module(params)
{
	// publishers
	_pub_loc = this->create_publisher<msg::Localization>("localization/position", 1);
	_pub_log = this->create_publisher<msg::LogEntry>("logging/entry", 1);
	// subscriber
	_sub_gnss = this->create_subscription<msg::GNSS>("sensors/gnss", 1,
													 std::bind(&LocalizationComponent::on_gnss,
															   this, std::placeholders::_1));
}

void LocalizationComponent::on_gnss(const msg::GNSS &gnss)
{
	msg::Localization loc;
	loc.header.stamp = gnss.header.stamp;
	loc.header.frame_id = "map";

	// projection
	auto pos = geodesy::Geodesy::get_pos(gnss.lat * M_PI / 180.0,
										 gnss.lon * M_PI / 180.0);
	loc.x = pos.x();
	loc.y = pos.y();

	// compute yaw
	double yaw = (5 * M_PI / 2) - (gnss.heading * M_PI / 180.0);
	if (yaw >= 2 * M_PI)
		yaw -= (2 * M_PI);
	loc.yaw = yaw;

	loc.vel = gnss.velocity;
	loc.accel = gnss.accel_x;
	loc.sigma_x = gnss.sigma_x;
	loc.sigma_y = gnss.sigma_y;

	_pub_loc->publish(loc);

	// log entry
	msg::LogEntry log_entry;
	std_msgs::msg::String name;

	name.data = "latitude";
	log_entry.name.push_back(name);
	log_entry.value.push_back(gnss.lat);

	name.data = "longitude";
	log_entry.name.push_back(name);
	log_entry.value.push_back(gnss.lon);

	_pub_log->publish(log_entry);
}