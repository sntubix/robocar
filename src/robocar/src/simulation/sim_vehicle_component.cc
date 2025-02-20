/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/simulation/sim_vehicle_component.h"

namespace robocar
{
	namespace simulation
	{

		SimVehicleComponent::SimVehicleComponent(const robocar::Params &params) : robocar::Component(params)
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
			_gravity = params.get("gravity").to_double();
			if (_gravity <= 0.0)
			{
				throw std::invalid_argument("'gravity' must be > 0.0");
			}
			_rolling_resistance = params.get("rolling_resistance").to_double();
			if (_rolling_resistance <= 0.0)
			{
				throw std::invalid_argument("'rolling_resistance' must be > 0.0");
			}
			_drag = params.get("drag").to_double();
			if (_drag <= 0.0)
			{
				throw std::invalid_argument("'drag' must be > 0.0");
			}
			_max_steering = params.get("max_steering").to_double();
			if (_max_steering <= 0.0)
			{
				throw std::invalid_argument("'max_steering' must be > 0.0");
			}
			_max_steering_speed = params.get("max_steering_speed").to_double();
			if (_max_steering_speed <= 0.0)
			{
				throw std::invalid_argument("'max_steering_speed' must be > 0.0");
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

			auto now = this->get_clock()->now();
			auto now_stamp = robocar::utils::ros_to_unix_ms_time(now);

			// init actuator status
			_act_status.header.stamp = now;
			_act_status.ad_engaged = false;
			_act_status.steering_status_stamp = now_stamp;
			_act_status.steering_status = STATUS_DISABLED;
			_act_status.steering_stamp = now_stamp;
			_act_status.steering = 0.0;
			_act_status.throttle_status_stamp = now_stamp;
			_act_status.throttle_status = STATUS_DISABLED;
			_act_status.velocity_stamp = now_stamp;
			_act_status.velocity = 0.0;
			_act_status.brake_status_stamp = now_stamp;
			_act_status.brake_status = STATUS_DISABLED;
			_act_status.brake_stamp = now_stamp;
			_act_status.brake = 0.0;

			// init position
			_x = _x_init;
			_y = _y_init;
			_yaw = _yaw_init;
			_v = 0.0;
			_accel = 0.0;
			_loc.header.stamp = now;
			_loc.header.frame_id = "map";
			_loc.x = _x;
			_loc.y = _y;
			_loc.pitch = 0.0;
			_loc.yaw = _yaw;
			_loc.vel = _v;
			_loc.accel = _accel;
			_loc.sigma_x = 0.0;
			_loc.sigma_y = 0.0;

			// publishers
			_pub_act_status = this->create_publisher<msg::ActStatus>("actuator/status", 1);
			_pub_loc = this->create_publisher<msg::Localization>("localization/position", 1);
			_pub_gnss = this->create_publisher<msg::GNSS>("sensors/gnss", 1);
			_pub_pc = this->create_publisher<msg::PointCloud>("sensors/lidar/points",
															  rclcpp::SensorDataQoS().keep_last(1));
			_pub_c_img = this->create_publisher<msg::CompressedImage>("sensors/camera/compressed",
																	  rclcpp::SensorDataQoS().keep_last(1));

			// subscribers
			_sub_act_toggle = this->create_subscription<msg::ActToggle>("actuator/toggle", 1,
																		std::bind(&SimVehicleComponent::on_act_toggle,
																				  this, std::placeholders::_1));
			_sub_act_cmd = this->create_subscription<msg::ActCmd>("actuator/command", 1,
																  std::bind(&SimVehicleComponent::on_act_cmd,
																			this, std::placeholders::_1));
		}

		void SimVehicleComponent::serve()
		{
			std::unique_lock<std::mutex> lock(_mtx);

			if (_act_status.ad_engaged)
			{
				//TODO improve vehicle model

				_accel = 0.0;

				// rolling resistance
				_accel -= _rolling_resistance * _gravity;

				// drag
				_accel -= _drag * (_v * _v);

				// throttle and brake
				if (_act_cmd.throttle > 0.0)
				{
					_accel += _act_cmd.throttle * _max_accel;
				}
				if (_act_cmd.brake > 0.0)
				{
					_accel += _act_cmd.brake * _min_accel;
				}

				// engine braking
				if ((_act_cmd.throttle == 0.0) && (_act_cmd.brake == 0.0))
				{
					_accel += 0.1 * _min_accel;
				}

				// update speed
				_v += _accel * _delta;
				if (_v < 0.0)
				{
					_v = 0.0;
				}

				// update steering
				double steering_e = _act_cmd.steering - _steering;
				double steering_speed = std::min(std::abs(steering_e) * _steering_kp,
												 _max_steering_speed);
				if (steering_e > 0.0)
				{
					_steering += steering_speed * _delta;
				}
				else
				{
					_steering -= steering_speed * _delta;
				}
				if (_steering > _max_steering)
				{
					_steering = _max_steering;
				}
				if (_steering < -_max_steering)
				{
					_steering = -_max_steering;
				}

				// update position
				_yaw += std::tan(_steering) * _v * _delta / _wheel_base;
				_x += std::cos(_yaw) * _v * _delta;
				_y += std::sin(_yaw) * _v * _delta;
			}

			auto now = this->get_clock()->now();
			auto now_stamp = robocar::utils::ros_to_unix_ms_time(now);

			// publish actuator status
			_act_status.header.stamp = now;
			_act_status.steering_status_stamp = now_stamp;
			_act_status.throttle_status_stamp = now_stamp;
			_act_status.brake_status_stamp = now_stamp;
			_act_status.steering_stamp = now_stamp;
			_act_status.velocity_stamp = now_stamp;
			_act_status.brake_stamp = now_stamp;
			_act_status.steering = _steering;
			_act_status.velocity = _v;
			_act_status.brake = _act_cmd.brake;
			_pub_act_status->publish(_act_status);

			// publish localization
			_loc.header.stamp = now;
			_loc.x = _x;
			_loc.y = _y;
			_loc.yaw = _yaw;
			_loc.vel = _v;
			_loc.accel = _accel;
			_pub_loc->publish(_loc);

			// publish virtual GNSS
			msg::GNSS gnss;
			gnss.header.stamp = now;
			gnss.header.frame_id = "vehicle";
			gnss.sigma_x = 0.0;
			gnss.sigma_y = 0.0;
			_pub_gnss->publish(gnss);

			// publish virtual point cloud
			msg::PointCloud pc_msg;
			pc_msg.header.stamp = now;
			pc_msg.header.frame_id = "vehicle";
			_pub_pc->publish(pc_msg);

			// publish virtual camera image
			msg::CompressedImage img_msg;
			img_msg.header.stamp = now;
			img_msg.header.frame_id = "vehicle";
			_pub_c_img->publish(img_msg);
		}

		void SimVehicleComponent::on_act_toggle(const msg::ActToggle &act_toggle)
		{
			_mtx.lock();
			if ((!_act_status.ad_engaged) && (act_toggle.toggle))
			{
				_x = _x_init;
				_y = _y_init;
				_yaw = _yaw_init;
				_act_status.ad_engaged = true;
				_act_status.steering_status = STATUS_ENABLED;
				_act_status.throttle_status = STATUS_ENABLED;
				_act_status.brake_status = STATUS_ENABLED;
			}
			if (!act_toggle.toggle)
			{
				_v = 0.0;
				_accel = 0.0;
				_act_status.ad_engaged = false;
				_act_status.steering_status = STATUS_DISABLED;
				_act_status.throttle_status = STATUS_DISABLED;
				_act_status.brake_status = STATUS_DISABLED;
			}
			_mtx.unlock();
		}

		void SimVehicleComponent::on_act_cmd(const msg::ActCmd &act_cmd)
		{
			_mtx.lock();
			_act_cmd = act_cmd;
			_mtx.unlock();
		}

	} // namespace simulation
} // namespace robocar