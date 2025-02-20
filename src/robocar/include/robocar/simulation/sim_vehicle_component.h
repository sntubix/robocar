/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_SIMULATION_SIM_VEHICLE_COMPONENT_H
#define ROBOCAR_SIMULATION_SIM_VEHICLE_COMPONENT_H

#include "robocar/core/robocar.h"

namespace robocar
{
	namespace simulation
	{

		class SimVehicleComponent : public robocar::Component
		{
		public:
			SimVehicleComponent(const robocar::Params &params);

			void serve() override;

		private:
			// params
			double _delta = 0.05;
			double _wheel_base = 2.57;
			double _gravity = 9.81;
			double _rolling_resistance = 0.02;
			double _drag = 0.00029;
			double _max_steering = 0.57;
			double _max_steering_speed = 0.356;
			double _steering_kp = 0.63;
			double _min_accel = -9.2;
			double _max_accel = 2.52;

			// state
			std::mutex _mtx;
			msg::ActStatus _act_status;
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
			rclcpp::Publisher<msg::ActStatus>::SharedPtr _pub_act_status;
			rclcpp::Publisher<msg::Localization>::SharedPtr _pub_loc;
			rclcpp::Publisher<msg::GNSS>::SharedPtr _pub_gnss;
			rclcpp::Publisher<msg::PointCloud>::SharedPtr _pub_pc;
			rclcpp::Publisher<msg::CompressedImage>::SharedPtr _pub_c_img;
			// subscribers
			rclcpp::Subscription<msg::ActToggle>::SharedPtr _sub_act_toggle;
			rclcpp::Subscription<msg::ActCmd>::SharedPtr _sub_act_cmd;

			void on_act_toggle(const msg::ActToggle &act_toggle);
			void on_act_cmd(const msg::ActCmd &act_cmd);
		};

	} // namespace simulation
} // namespace robocar

#endif // ROBOCAR_SIMULATION_SIM_VEHICLE_COMPONENT_H