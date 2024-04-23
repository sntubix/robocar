/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#include "vehicle/vehicle_component.h"
#include "common/common.h"

using namespace robocar::vehicle;

VehicleComponent::VehicleComponent(const cycle::Params& params) : cycle::Service(params) {
	// params
	_can_timeout = params.get("can_timeout").to_int();
	_gnss_timeout = params.get("gnss_timeout").to_int();
	_pc_timeout = params.get("lidar_timeout").to_int();
	_camera_timeout = params.get("camera_timeout").to_int();
	_planning_timeout = params.get("planning_timeout").to_int();
	_steering_ratio = params.get("steering_ratio").to_double();
	if (_steering_ratio <= 0.0) {
        throw std::invalid_argument("'steering_ratio' must be > 0.0");
    }
	_steering_speed = params.get("steering_speed").to_double();
	if (_steering_speed <= 0.0) {
        throw std::invalid_argument("'steering_speed' must be > 0.0");
    }
	_steering_safety_coef = params.get("steering_safety_coef").to_double();
	if (_steering_safety_coef <= 0.0) {
        throw std::invalid_argument("'steering_safety_coef' must be > 0.0");
    }
	auto ma_steering_window = params.get("ma_steering_window").to_int();
	if (ma_steering_window <= 0) {
        throw std::invalid_argument("'ma_steering_window' must be > 0");
    }
	_ma_steering.reset(new MA(ma_steering_window));
	auto ma_throttle_window = params.get("ma_throttle_window").to_int();
	if (ma_throttle_window <= 0) {
        throw std::invalid_argument("'ma_throttle_window' must be > 0");
    }
	_ma_throttle.reset(new MA(ma_throttle_window));
	auto ma_brake_window = params.get("ma_brake_window").to_int();
	if (ma_brake_window <= 0) {
        throw std::invalid_argument("'ma_brake_window' must be > 0");
    }
	_ma_brake.reset(new MA(ma_brake_window));
	_max_gnss_uncertainty = params.get("gnss_uncertainty").to_double();
	if (_max_gnss_uncertainty <= 0.0) {
        throw std::invalid_argument("'max_gnss_uncertainty' must be > 0.0");
    }

	// init actuation
	_act_control.steering = 0.0;
	_act_control.steering_speed = 0.0;
    _act_control.throttle = 0.0;
    _act_control.brake = 0.0;
	_act_input.steering = 0.0;
    _act_input.throttle = 0.0;
    _act_input.brake = 0.0;
	_min_steering_speed = (0.2 * _steering_speed * M_PI) / (180.0 * _steering_ratio);
	_max_steering_speed = (1.1 * _steering_speed * M_PI) / (180.0 * _steering_ratio);
	_steering_delta.resize(params.get("ma_steering_window").to_int());
	for (int i=0; i < _steering_delta.size(); i++) {
		_steering_delta[i] = 0.0;
	}

	// publishers
	_pub_vehicle = this->create_publisher<msg::Vehicle>("vehicle/status", 1);
	_pub_act_toggle = this->create_publisher<msg::ActToggle>("actuator/toggle", 1);
	_pub_act_cmd = this->create_publisher<msg::ActCmd>("actuator/command", 1);
	// subscribers
	_sub_ad_toggle = this->create_subscription<msg::AdToggle>("vehicle/ad_toggle", 1,
															  std::bind(&VehicleComponent::on_ad_toggle,
															  			this, std::placeholders::_1));
	_sub_act_status = this->create_subscription<msg::ActStatus>("actuator/status", 1,
																std::bind(&VehicleComponent::on_act_status,
													 					  this, std::placeholders::_1));
	_sub_input = this->create_subscription<msg::ActCmd>("vehicle/input", 1,
														std::bind(&VehicleComponent::on_input,
														   		  this, std::placeholders::_1));
	_sub_gnss = this->create_subscription<msg::GNSS>("sensors/gnss", 1,
													 std::bind(&VehicleComponent::on_gnss,
													 		   this, std::placeholders::_1));
	_sub_pc = this->create_subscription<msg::PointCloud>("sensors/points", 1,
														 std::bind(&VehicleComponent::on_point_cloud,
																   this, std::placeholders::_1));
	_sub_image = this->create_subscription<msg::CompressedImage>("sensors/camera/compressed", 1,
													   			 std::bind(&VehicleComponent::on_image,
																 		   this, std::placeholders::_1));
	_sub_planning = this->create_subscription<msg::Planning>("planning/trajectory", 1,
															 std::bind(&VehicleComponent::on_planning,
															 		   this, std::placeholders::_1));
	_sub_control = this->create_subscription<msg::ActCmd>("vehicle/control", 1,
														  std::bind(&VehicleComponent::on_control,
															 		this, std::placeholders::_1));
}

void VehicleComponent::serve() {
	msg::ActCmd act_cmd;
	act_cmd.header.stamp = this->get_clock()->now();
	act_cmd.steering = 0.0;
	act_cmd.steering_speed = _steering_speed;
	act_cmd.throttle = 0.0;
	act_cmd.brake = 0.0;

	if (status_check()) {
		std::unique_lock<std::mutex> lock(_m_a);
		act_cmd = _act_control;

		// direct input override
		if (_act_input.steering != 0.0)
			act_cmd.steering = _act_input.steering;
		if (_act_input.throttle != 0.0)
			act_cmd.throttle = _act_input.throttle;
		if (_act_input.brake != 0.0) {
			act_cmd.throttle = 0.0;
			act_cmd.brake = _act_input.brake;
		}
		lock.unlock();

		// smooth actuation
		if (act_cmd.brake > 0.0) {
			_ma_brake->add(act_cmd.brake);
			act_cmd.brake = _ma_brake->EMA_avg();
			_ma_throttle->add(0.0);
			act_cmd.throttle = _ma_throttle->EMA_avg();
		}
		else {
			_ma_brake->add(0.0);
			act_cmd.brake = _ma_brake->EMA_avg();
			_ma_throttle->add(act_cmd.throttle);
			act_cmd.throttle = _ma_throttle->EMA_avg();
		}

		// steering
		_ma_steering->add(act_cmd.steering);
		act_cmd.steering = _ma_steering->EMA_avg();
		act_cmd.steering = act_cmd.steering * _steering_ratio;
		act_cmd.steering_speed = _steering_speed;
	}

	_pub_act_cmd->publish(act_cmd);
}

bool VehicleComponent::status_check() {
	std::vector<std::string> errors;

	msg::Vehicle vehicle;
	std::unique_lock<std::mutex> lock(_m_a);
	// init vehicle status from actuator status
	vehicle.header.stamp = this->get_clock()->now();
	vehicle.ad_engaged = _act_status.ad_engaged;
	vehicle.steering_status_stamp = _act_status.steering_status_stamp;
	vehicle.steering_status = _act_status.steering_status;
	vehicle.steering_stamp = _act_status.steering_stamp;
	vehicle.steering = _act_status.steering;
	vehicle.throttle_status_stamp = _act_status.throttle_status_stamp;
	vehicle.throttle_status = _act_status.throttle_status;
	vehicle.velocity_stamp = _act_status.velocity_stamp;
	vehicle.velocity = _act_status.velocity;
	vehicle.brake_status_stamp = _act_status.brake_status_stamp;
	vehicle.brake_status = _act_status.brake_status;
	vehicle.brake_stamp = _act_status.brake_stamp;
	vehicle.brake = _act_status.brake;
	vehicle.gnss = STATUS_OK;
	vehicle.lidar = STATUS_OK;
	vehicle.camera = STATUS_OK;
	vehicle.steering = -vehicle.steering / _steering_ratio;
	auto now = cycle::utils::ros_to_unix_ms_time(vehicle.header.stamp);
	// release actuator mutex
	lock.unlock();

	// check actuator status
	if (vehicle.steering_status == STATUS_ERROR) {
		errors.push_back("steering fault");
	}
	if (vehicle.throttle_status == STATUS_ERROR) {
		errors.push_back("throttle fault");
	}
	if (vehicle.brake_status == STATUS_ERROR) {
		errors.push_back("brake fault");
	}

	// check CAN frames stamp
	if ((_can_timeout > 0) && ((now - vehicle.steering_status_stamp) > _can_timeout)) {
		vehicle.steering_status = STATUS_TIMEOUT;
		errors.push_back("CAN steering status timeout");
	}
	if ((_can_timeout > 0) && ((now - vehicle.throttle_status_stamp) > _can_timeout)) {
		vehicle.throttle_status = STATUS_TIMEOUT;
		errors.push_back("CAN throttle status timeout");
	}
	if ((_can_timeout > 0) && ((now - vehicle.brake_status_stamp) > _can_timeout)) {
		vehicle.brake_status = STATUS_TIMEOUT;
		errors.push_back("CAN brake status timeout");
	}
	if ((_can_timeout > 0) && ((now - vehicle.steering_stamp) > _can_timeout)) {
		vehicle.steering_status = STATUS_TIMEOUT;
		errors.push_back("CAN steering timeout");
	}
	//if ((_can_timeout > 0) && ((now - vehicle.velocity_stamp) > _can_timeout)) {
		//vehicle.throttle_status = STATUS_TIMEOUT;
		//errors.push_back("CAN speed timeout");
	//}
	//if ((_can_timeout > 0) && ((now - vehicle.brake_stamp) > _can_timeout)) {
		//vehicle.brake_status = STATUS_TIMEOUT;
		//errors.push_back("CAN brake timeout");
	//}
	// check GNSS stamp
	if ((_gnss_timeout > 0) && ((now - _gnss_stamp) > _gnss_timeout)) {
		vehicle.gnss = STATUS_TIMEOUT;
		errors.push_back("GNSS timeout");
	}
	// check LiDAR point cloud stamp
	if ((_pc_timeout > 0) && ((now - _pc_stamp) > _pc_timeout)) {
		vehicle.lidar = STATUS_TIMEOUT;
		errors.push_back("LiDAR timeout");
	}
	// check camera stamp
	if ((_camera_timeout > 0) && ((now - _camera_stamp) > _camera_timeout)) {
		vehicle.camera = STATUS_TIMEOUT;
		errors.push_back("camera timeout");
	}

	// compute steering speed
	double steering_delta_t = (vehicle.steering_stamp - _prev_steering_stamp) / 1000.0;
	if (steering_delta_t > 0.0) {
		_steering_delta[_sd_index] = (vehicle.steering - _prev_steering) / steering_delta_t;
		_prev_steering_stamp = vehicle.steering_stamp;
		_prev_steering = vehicle.steering;
		_sd_index++;
		if (_sd_index > (_steering_delta.size() - 1))
			_sd_index = 0;
	}
	double steering_delta = 0.0;
	for (auto delta : _steering_delta)
		steering_delta += std::abs(delta);
	steering_delta = steering_delta / _steering_delta.size();
	// check steering speed out of limit
	double delta_limit = (-_steering_safety_coef * _velocity) + _max_steering_speed;
	if (delta_limit < _min_steering_speed)
		delta_limit = _min_steering_speed;
	if (steering_delta > delta_limit) {
		vehicle.steering_status = STATUS_ERROR;
		errors.push_back("steering speed out of limit");
	}

	// check GNSS accuracy
	if (_gnss_uncertainty > _max_gnss_uncertainty) {
		vehicle.gnss = STATUS_ERROR;
		errors.push_back("inaccurate GNSS position");
	}

	// check planning validity
	if ((_planning_timeout > 0) && ((now - _planning_stamp) > _planning_timeout)) {
		errors.push_back("planning validity timeout");
	}

	// check if disengage and print errors
	if (vehicle.ad_engaged && (!errors.empty())) {
		msg::ActToggle act_toggle;
		act_toggle.header.stamp = vehicle.header.stamp;
		act_toggle.toggle = false;
		_pub_act_toggle->publish(act_toggle);
		for (auto& msg : errors) {
			LOG_ERROR(msg);
		}
	}

	_pub_vehicle->publish(vehicle);
	return errors.empty();
}

void VehicleComponent::on_ad_toggle(const msg::AdToggle& ad_toggle) {
	msg::ActToggle act_toggle;
	act_toggle.header.stamp = this->get_clock()->now();
	act_toggle.toggle = ad_toggle.toggle;
	_pub_act_toggle->publish(act_toggle);
}

void VehicleComponent::on_act_status(const msg::ActStatus& act_status) {
	std::unique_lock<std::mutex> lock(_m_a);
	_act_status = act_status;
}

void VehicleComponent::on_input(const msg::ActCmd& act_cmd) {
	std::unique_lock<std::mutex> lock(_m_a);
	_act_input = act_cmd;
	//std::cout << _act_input.steering << " " << _act_input.throttle << " " << _act_input.brake << std::endl;
}

void VehicleComponent::on_gnss(const msg::GNSS& gnss) {
	_gnss_stamp = cycle::utils::ros_to_unix_ms_time(gnss.header.stamp);
	_gnss_uncertainty = std::max(gnss.sigma_x, gnss.sigma_y);
	_velocity = gnss.velocity;
}

void VehicleComponent::on_point_cloud(const msg::PointCloud::ConstSharedPtr& pc) {
	_pc_stamp = cycle::utils::ros_to_unix_ms_time(pc->header.stamp);
}

void VehicleComponent::on_image(const msg::CompressedImage::ConstSharedPtr& img) {
    _camera_stamp = cycle::utils::ros_to_unix_ms_time(img->header.stamp);
}

void VehicleComponent::on_planning(const msg::Planning& planning) {
	if (planning.trajectory.waypoints.size() > 0) {
		_planning_stamp = cycle::utils::ros_to_unix_ms_time(planning.header.stamp);
	}
}

void VehicleComponent::on_control(const msg::ActCmd& act_cmd) {
	std::unique_lock<std::mutex> lock(_m_a);
	_act_control = act_cmd;
}