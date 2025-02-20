/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_DRIVERS_ALEXA_ALEXA_COMPONENT_H
#define ROBOCAR_DRIVERS_ALEXA_ALEXA_COMPONENT_H

#include "robocar/core/robocar.h"

#include <libserial/SerialPort.h>

namespace robocar::drivers::alexa
{
	class AlexaComponent : public robocar::Component
	{
	public:
		AlexaComponent(const robocar::Params &params);
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

#endif // ROBOCAR_DRIVERS_ALEXA_ALEXA_COMPONENT_H