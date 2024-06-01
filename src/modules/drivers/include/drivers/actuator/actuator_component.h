/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef DRIVERS_ACTUATOR_ACTUATOR_COMPONENT_H
#define DRIVERS_ACTUATOR_ACTUATOR_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

namespace robocar::drivers::actuator
{
	class ActuatorComponent : public cycle::Service
	{
	public:
		ActuatorComponent(const cycle::Params &params);
		~ActuatorComponent();

		void serve() override;

	private:
		// params
		std::string _can_dev = "can0";

		bool is_engaged();
		bool publish_steering_angle(double angle, double velocity);
		bool publish_throttle_position(double throttle);
		bool publish_brake_position(double brake);

		void on_act_toggle(const msg::ActToggle &act_toggle);
		void on_act_cmd(const msg::ActCmd &act_cmd);

		// publisher
		rclcpp::Publisher<msg::ActStatus>::SharedPtr _pub_act_status;
		// subscribers
		rclcpp::Subscription<msg::ActToggle>::SharedPtr _sub_act_toggle;
		rclcpp::Subscription<msg::ActCmd>::SharedPtr _sub_act_cmd;
	};
}

#endif // DRIVERS_ACTUATOR_ACTUATOR_COMPONENT_H