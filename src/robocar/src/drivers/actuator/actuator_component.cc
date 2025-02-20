/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/drivers/actuator/actuator_component.h"

extern "C"
{
#include "oscc/oscc.h"
#include "oscc/vehicles/kia_soul_ev.h"
}

namespace robocar::drivers::actuator
{

	static std::atomic<uint64_t> _steering_status_stamp{0};
	static std::atomic<uint64_t> _throttle_status_stamp{0};
	static std::atomic<uint64_t> _brake_status_stamp{0};
	static std::atomic<bool> _steering_enabled{false};
	static std::atomic<bool> _throttle_enabled{false};
	static std::atomic<bool> _brake_enabled{false};

	static std::atomic<uint64_t> _fault_stamp{0};
	static std::atomic<bool> _fault_steering{false};
	static std::atomic<bool> _fault_throttle{false};
	static std::atomic<bool> _fault_brake{false};

	static std::vector<double> _vel(4);
	static std::atomic<uint64_t> _steering_stamp{0};
	static std::atomic<uint64_t> _speed_stamp{0};
	static std::atomic<uint64_t> _brake_stamp{0};
	static std::atomic<double> _steering{-1.0};
	static std::atomic<double> _speed{-1.0};
	static std::atomic<double> _brake{-1.0};

	static void steering_callback(oscc_steering_report_s *report);
	static void throttle_callback(oscc_throttle_report_s *report);
	static void brake_callback(oscc_brake_report_s *report);
	static void fault_callback(oscc_fault_report_s *report);
	static void obd_callback(struct can_frame *frame);

	static inline double get_wheel_speed(struct can_frame const *const frame, const size_t offset)
	{
		if (frame == nullptr)
			return OSCC_ERROR;
		if (frame->can_id != KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID)
			return OSCC_ERROR;
		uint16_t raw = ((frame->data[offset + 1] & 0x0F) << 8) | frame->data[offset];
		return ((int)(raw / 3.2) / 10.0);
	}

	static inline double get_steering_wheel_angle(struct can_frame const *const frame)
	{
		if (frame == nullptr)
			return OSCC_ERROR;
		if (frame->can_id != KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID)
			return OSCC_ERROR;
		int16_t raw = (frame->data[1] << 8) | frame->data[0];
		return ((double)raw * KIA_SOUL_OBD_STEERING_ANGLE_SCALAR);
	}

	static inline double get_brake_pressure(struct can_frame const *const frame)
	{
		if (frame == nullptr)
			return OSCC_ERROR;
		if (frame->can_id != KIA_SOUL_OBD_BRAKE_PRESSURE_CAN_ID)
			return OSCC_ERROR;
		double scale = 10.0;
		uint16_t raw = ((frame->data[5] & 0x0F) << 8) | frame->data[4];
		return (double)raw / scale;
	}

	static void steering_callback(oscc_steering_report_s *report)
	{
		_steering_enabled = report->enabled;
		_steering_status_stamp = robocar::Time::now().ms();
	}

	static void throttle_callback(oscc_throttle_report_s *report)
	{
		_throttle_enabled = report->enabled;
		_throttle_status_stamp = robocar::Time::now().ms();
	}

	static void brake_callback(oscc_brake_report_s *report)
	{
		_brake_enabled = report->enabled;
		_brake_status_stamp = robocar::Time::now().ms();
	}

	static void fault_callback(oscc_fault_report_s *report)
	{
		_fault_stamp = robocar::Time::now().ms();
		_fault_steering = false;
		_fault_throttle = false;
		_fault_brake = false;

		if (report->fault_origin_id == FAULT_ORIGIN_STEERING)
		{
			_fault_steering = true;
		}
		if (report->fault_origin_id == FAULT_ORIGIN_THROTTLE)
		{
			_fault_throttle = true;
		}
		if (report->fault_origin_id == FAULT_ORIGIN_BRAKE)
		{
			_fault_brake = true;
		}
	}

	static double median(std::vector<double> &v)
	{
		size_t n = v.size() / 2;
		std::nth_element(v.begin(), v.begin() + n, v.end());
		double vn = v[n];
		if (v.size() % 2 == 1)
		{
			return vn;
		}
		else
		{
			std::nth_element(v.begin(), v.begin() + n - 1, v.end());
			return 0.5 * (vn + v[n - 1]);
		}
	}

	static void obd_callback(struct can_frame *frame)
	{
		auto now = robocar::Time::now().ms();
		switch (frame->can_id)
		{
		case KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID:
		{
			_steering_stamp = now;
			_steering = get_steering_wheel_angle(frame) * M_PI / 180;
			break;
		}
		case KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID:
		{
			_speed_stamp = now;
			_vel[0] = get_wheel_speed(frame, 0); // front left
			_vel[1] = get_wheel_speed(frame, 2); // front right
			_vel[2] = get_wheel_speed(frame, 4); // rear left
			_vel[3] = get_wheel_speed(frame, 6); // rear right
			_speed = median(_vel);
			break;
		}
		case KIA_SOUL_OBD_BRAKE_PRESSURE_CAN_ID:
		{
			_brake_stamp = now;
			_brake = get_brake_pressure(frame);
			break;
		}
		default:
			break;
		}
	}

	ActuatorComponent::ActuatorComponent(const robocar::Params &params) : robocar::Component(params)
	{
		// params
		_can_dev = params.get("can_dev").to_string();
		if (_can_dev.empty())
		{
			throw std::invalid_argument("invalid 'can_dev'");
		}
		_steering_ratio = params.get("steering_ratio").to_double();
		if (_steering_ratio <= 0.0)
		{
			throw std::invalid_argument("'steering_ratio' must be > 0.0");
		}
		_steering_speed = (params.get("steering_speed").to_double() * _steering_ratio) * 180.0 / M_PI;
		if (_steering_speed <= 0.0)
		{
			throw std::invalid_argument("'steering_speed' must be > 0.0");
		}

		// connect to CAN device
		if (oscc_open(_can_dev.c_str()) != OSCC_ERROR)
		{
			// register callback handlers
			oscc_subscribe_to_steering_reports(steering_callback);
			oscc_subscribe_to_throttle_reports(throttle_callback);
			oscc_subscribe_to_brake_reports(brake_callback);
			oscc_subscribe_to_fault_reports(fault_callback);
			oscc_subscribe_to_obd_messages(obd_callback);
			RCLCPP_INFO(this->get_logger(), "OSCC controls initialized");
		}

		// publisher
		_pub_act_status = this->create_publisher<msg::ActStatus>("actuator/status", 1);
		// subscribers
		_sub_act_toggle = this->create_subscription<msg::ActToggle>("actuator/toggle", 1,
																	std::bind(&ActuatorComponent::on_act_toggle,
																			  this, std::placeholders::_1));
		_sub_act_cmd = this->create_subscription<msg::ActCmd>("actuator/command", 1,
															  std::bind(&ActuatorComponent::on_act_cmd,
																		this, std::placeholders::_1));
	}

	ActuatorComponent::~ActuatorComponent()
	{
		oscc_disable();
		oscc_close();
	}

	void ActuatorComponent::serve()
	{
		auto now = robocar::Time::now().ms();
		bool fault_recent = (now - _fault_stamp) < 500;

		msg::ActStatus status;
		status.header.stamp = robocar::utils::unix_ms_to_ros_time(now);
		status.ad_engaged = (_steering_enabled || _throttle_enabled || _brake_enabled);
		status.steering_status_stamp = _steering_status_stamp;
		status.throttle_status_stamp = _throttle_status_stamp;
		status.brake_status_stamp = _brake_status_stamp;

		// steering status
		if (_steering_enabled)
		{
			status.steering_status = STATUS_ENABLED;
		}
		else
		{
			status.steering_status = STATUS_DISABLED;
		}
		if (_fault_steering && fault_recent)
		{
			status.steering_status = STATUS_ERROR;
		}

		// throttle status
		if (_throttle_enabled)
		{
			status.throttle_status = STATUS_ENABLED;
		}
		else
		{
			status.throttle_status = STATUS_DISABLED;
		}
		if (_fault_throttle && fault_recent)
		{
			status.throttle_status = STATUS_ERROR;
		}

		// brake status
		if (_brake_enabled)
		{
			status.brake_status = STATUS_ENABLED;
		}
		else
		{
			status.brake_status = STATUS_DISABLED;
		}
		if (_fault_brake && fault_recent)
		{
			status.brake_status = STATUS_ERROR;
		}

		status.steering_stamp = _steering_stamp;
		status.velocity_stamp = _speed_stamp;
		status.brake_stamp = _brake_stamp;
		status.steering = _steering / _steering_ratio;
		status.velocity = _speed;
		status.brake = _brake;

		_pub_act_status->publish(status);
	}

	void ActuatorComponent::on_act_toggle(const msg::ActToggle &act_toggle)
	{
		if (act_toggle.toggle)
		{
			if (oscc_enable() == OSCC_OK)
			{
				RCLCPP_INFO(this->get_logger(), "OSCC controls engaged");
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "unable to engage OSCC controls");
			}
		}
		else
		{
			if (oscc_disable() == OSCC_OK)
			{
				RCLCPP_INFO(this->get_logger(), "OSCC controls disengaged");
			}
			else
			{
				RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "unable to disengage OSCC controls");
			}
		}
	}

	void ActuatorComponent::on_act_cmd(const msg::ActCmd &act_cmd)
	{
		double brake = act_cmd.brake;
		if (brake < 0.0)
		{
			brake = 0.0;
		}
		if (brake > 1.0)
		{
			brake = 1.0;
		}

		double throttle = act_cmd.throttle;
		if (throttle < 0.0)
		{
			throttle = 0.0;
		}
		if (throttle > 1.0)
		{
			throttle = 1.0;
		}

		double steering = (-act_cmd.steering * _steering_ratio) * 180.0 / M_PI;
		if (steering < -550.0)
		{
			steering = -550.0;
		}
		if (steering > 550.0)
		{
			steering = 550.0;
		}

		oscc_publish_brake_position(brake);
		oscc_publish_throttle_position(throttle);
		oscc_publish_steering_angle(steering, _steering_speed);
	}

} // namespace robocar::drivers::actuator