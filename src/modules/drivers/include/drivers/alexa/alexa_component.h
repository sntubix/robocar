/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef DRIVERS_ALEXA_COMPONENT_H
#define DRIVERS_ALEXA_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include <libserial/SerialPort.h>

namespace robocar::drivers::alexa
{
	class AlexaComponent : public cycle::Service
	{
	public:
		AlexaComponent(const cycle::Params &params);
		~AlexaComponent();

		void serve() override;

	private:
		// params
		std::string _serial_port = "/dev/ttyUSB0";
		double _target_velocity = 0.0;

		LibSerial::SerialPort _serial;
		LibSerial::DataBuffer _buffer;

		// publishers
		rclcpp::Publisher<msg::AdToggle>::SharedPtr _pub_ad_toggle;
		rclcpp::Publisher<msg::TargetSpeed>::SharedPtr _pub_target_speed;
	};

}

#endif // DRIVERS_ALEXA_COMPONENT_H