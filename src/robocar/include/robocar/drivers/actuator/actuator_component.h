/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_DRIVERS_ACTUATOR_ACTUATOR_COMPONENT_H
#define ROBOCAR_DRIVERS_ACTUATOR_ACTUATOR_COMPONENT_H

#include "robocar/core/robocar.h"

namespace robocar::drivers::actuator
{
	class ActuatorComponent : public robocar::Component
	{
	public:
		ActuatorComponent(const robocar::Params &params);
		~ActuatorComponent();

		void serve() override;

	private:
		// params
		std::string _can_dev = "can0";
		double _steering_ratio = 15.7;
		double _steering_speed = 320.0;

		void on_act_toggle(const msg::ActToggle &act_toggle);
		void on_act_cmd(const msg::ActCmd &act_cmd);

		// publisher
		rclcpp::Publisher<msg::ActStatus>::SharedPtr _pub_act_status;
		// subscribers
		rclcpp::Subscription<msg::ActToggle>::SharedPtr _sub_act_toggle;
		rclcpp::Subscription<msg::ActCmd>::SharedPtr _sub_act_cmd;
	};
}

#endif // ROBOCAR_DRIVERS_ACTUATOR_ACTUATOR_COMPONENT_H