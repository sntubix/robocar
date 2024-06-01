/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef SIMULATION_SIM_VEHICLE_COMPONENT_H
#define SIMULATION_SIM_VEHICLE_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

namespace robocar
{
	namespace simulation
	{

		class SimVehicleComponent : public cycle::Service
		{
		public:
			SimVehicleComponent(const cycle::Params &params);

			void serve() override;

		private:
			// params
			double _delta = 0.05;
			double _wheel_base = 2.57;
			double _steering_speed = 0.3557;
			double _steering_kp = 0.63;
			double _min_accel = -9.2;
			double _max_accel = 2.52;

			// state
			std::mutex _mtx;
			msg::Vehicle _vehicle;
			msg::ActCmd _act_cmd;
			msg::Localization _loc;
			double _x_init = 0.0;
			double _y_init = 0.0;
			double _yaw_init = 0.0;
			double _x = 0.0;
			double _y = 0.0;
			double _yaw = 0.0;
			double _steering = 0.0;
			double _v = 0.0;
			double _accel = 0.0;

			// publishers
			rclcpp::Publisher<msg::Vehicle>::SharedPtr _pub_vehicle;
			rclcpp::Publisher<msg::Localization>::SharedPtr _pub_loc;
			rclcpp::Publisher<msg::ActCmd>::SharedPtr _pub_act_cmd;
			// subscribers
			rclcpp::Subscription<msg::AdToggle>::SharedPtr _sub_ad_toggle;
			rclcpp::Subscription<msg::ActCmd>::SharedPtr _sub_control;

			void on_ad_toggle(const msg::AdToggle &ad_toggle);
			void on_control(const msg::ActCmd &act_cmd);
		};

	} // namespace simulation
} // namespace robocar

#endif // SIMULATION_SIM_VEHICLE_COMPONENT_H