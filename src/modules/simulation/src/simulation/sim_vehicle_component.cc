/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "simulation/sim_vehicle_component.h"

namespace robocar
{
	namespace simulation
	{

		SimVehicleComponent::SimVehicleComponent(const cycle::Params &params) : cycle::Service(params)
		{
			// params
			_delta = params.get("period").to_int() / 1000.0;
			if (_delta <= 0.0)
			{
				throw std::invalid_argument("'period' must be > 0");
			}
			_wheel_base = params.get("wheel_base").to_double();
			if (_wheel_base <= 0.0)
			{
				throw std::invalid_argument("'wheel_base' must be > 0.0");
			}
			_steering_speed = params.get("steering_speed").to_double();
			if (_steering_speed <= 0.0)
			{
				throw std::invalid_argument("'steering_speed' must be > 0.0");
			}
			_steering_kp = params.get("steering_kp").to_double();
			if (_steering_kp <= 0.0)
			{
				throw std::invalid_argument("'steering_kp' must be > 0.0");
			}
			_min_accel = params.get("min_accel").to_double();
			if (_min_accel >= 0.0)
			{
				throw std::invalid_argument("'min_accel' must be < 0.0");
			}
			_max_accel = params.get("max_accel").to_double();
			if (_max_accel <= 0.0)
			{
				throw std::invalid_argument("'max_accel' must be > 0.0");
			}
			_x_init = params.get("x").to_double();
			_y_init = params.get("y").to_double();
			_yaw_init = params.get("yaw").to_double();

			// init vehicle
			_vehicle.header.stamp = rclcpp::Time(0);
			_vehicle.ad_engaged = false;
			_vehicle.steering_status_stamp = 0;
			_vehicle.throttle_status_stamp = 0;
			_vehicle.brake_status_stamp = 0;
			_vehicle.steering_status = STATUS_DISABLED;
			_vehicle.throttle_status = STATUS_DISABLED;
			_vehicle.brake_status = STATUS_DISABLED;
			_vehicle.steering_stamp = 0;
			_vehicle.velocity_stamp = 0;
			_vehicle.brake_stamp = 0;
			_vehicle.steering = 0.0;
			_vehicle.velocity = 0.0;
			_vehicle.brake = 0.0;
			_vehicle.gnss = STATUS_OK;
			_vehicle.lidar = STATUS_OK;
			_vehicle.camera = STATUS_OK;

			// init position
			_x = _x_init;
			_y = _y_init;
			_yaw = _yaw_init;
			_v = 0.0;
			_accel = 0.0;
			_loc.header.stamp = rclcpp::Time(0);
			_loc.header.frame_id = "map";
			_loc.x = _x;
			_loc.y = _y;
			_loc.yaw = _yaw;
			_loc.vel = _v;
			_loc.accel = _accel;
			_loc.sigma_x = 0.0;
			_loc.sigma_y = 0.0;

			// publishers
			_pub_vehicle = this->create_publisher<msg::Vehicle>("vehicle/status", 1);
			_pub_loc = this->create_publisher<msg::Localization>("localization/position", 1);
			_pub_act_cmd = this->create_publisher<msg::ActCmd>("actuator/command", 1);
			// subscribers
			_sub_ad_toggle = this->create_subscription<msg::AdToggle>("vehicle/ad_toggle", 1,
																	  std::bind(&SimVehicleComponent::on_ad_toggle,
																				this, std::placeholders::_1));
			_sub_control = this->create_subscription<msg::ActCmd>("vehicle/control", 1,
																  std::bind(&SimVehicleComponent::on_control,
																			this, std::placeholders::_1));
		}

		void SimVehicleComponent::serve()
		{
			std::unique_lock<std::mutex> lock(_mtx);

			if (_vehicle.ad_engaged)
			{
				// update speed
				if (_act_cmd.throttle > 0.0)
				{
					_accel = _act_cmd.throttle * _max_accel;
				}
				if (_act_cmd.brake > 0.0)
				{
					_accel = _act_cmd.brake * _min_accel;
				}
				_v += _accel * _delta;
				if (_v < 0.0)
				{
					_v = 0.0;
				}

				// update steering
				double steering_e = std::abs(_act_cmd.steering - _steering);
				double steering_speed = std::min(steering_e * _steering_kp, _steering_speed);
				if (_act_cmd.steering > _steering)
				{
					_steering += steering_speed * _delta;
				}
				else
				{
					_steering -= steering_speed * _delta;
				}

				// update position
				_yaw += std::tan(_steering) * _v * _delta / _wheel_base;
				_x += std::cos(_yaw) * _v * _delta;
				_y += std::sin(_yaw) * _v * _delta;
			}

			// publish vehicle
			auto now = cycle::Time::now().ms();
			_vehicle.header.stamp = cycle::utils::unix_ms_to_ros_time(now);
			_vehicle.steering_status_stamp = now;
			_vehicle.throttle_status_stamp = now;
			_vehicle.brake_status_stamp = now;
			_vehicle.steering_stamp = now;
			_vehicle.velocity_stamp = now;
			_vehicle.brake_stamp = now;
			_vehicle.steering = _steering;
			_vehicle.velocity = _v;
			_pub_vehicle->publish(_vehicle);

			// publish localization
			_loc.header.stamp = cycle::utils::unix_ms_to_ros_time(now);
			_loc.x = _x;
			_loc.y = _y;
			_loc.yaw = _yaw;
			_loc.vel = _v;
			_loc.accel = _accel;
			_pub_loc->publish(_loc);

			// publish applied actuation command
			_pub_act_cmd->publish(_act_cmd);
		}

		void SimVehicleComponent::on_ad_toggle(const msg::AdToggle &ad_toggle)
		{
			_mtx.lock();
			if (!_vehicle.ad_engaged)
			{
				_x = _x_init;
				_y = _y_init;
				_yaw = _yaw_init;
				_vehicle.ad_engaged = true;
				_vehicle.steering_status = STATUS_ENABLED;
				_vehicle.throttle_status = STATUS_ENABLED;
				_vehicle.brake_status = STATUS_ENABLED;
			}
			else
			{
				_v = 0.0;
				_accel = 0.0;
				_vehicle.ad_engaged = false;
				_vehicle.steering_status = STATUS_DISABLED;
				_vehicle.throttle_status = STATUS_DISABLED;
				_vehicle.brake_status = STATUS_DISABLED;
			}
			_mtx.unlock();
		}

		void SimVehicleComponent::on_control(const msg::ActCmd &act_cmd)
		{
			_mtx.lock();
			_act_cmd = act_cmd;
			_mtx.unlock();
		}

	} // namespace simulation
} // namespace robocar