/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/tod/tod_component.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>

#include <opencv2/opencv.hpp>

using namespace robocar::tod;

const char* VehicleData::to_bytes() {
    rapidjson::Document doc;

    // set values
    doc.SetObject();
	doc.AddMember("stamp", rapidjson::Value(stamp), doc.GetAllocator());
    doc.AddMember("tod_msg_stamp", rapidjson::Value(tod_msg_stamp), doc.GetAllocator());
	doc.AddMember("ad_engaged", rapidjson::Value(ad_engaged), doc.GetAllocator());
    doc.AddMember("steering", rapidjson::Value(steering), doc.GetAllocator());
    doc.AddMember("steering_n", rapidjson::Value(steering_n), doc.GetAllocator());
	doc.AddMember("lat", rapidjson::Value(lat), doc.GetAllocator());
	doc.AddMember("lon", rapidjson::Value(lon), doc.GetAllocator());
	doc.AddMember("alt", rapidjson::Value(alt), doc.GetAllocator());
	doc.AddMember("roll", rapidjson::Value(roll), doc.GetAllocator());
	doc.AddMember("pitch", rapidjson::Value(pitch), doc.GetAllocator());
	doc.AddMember("yaw", rapidjson::Value(yaw), doc.GetAllocator());
	doc.AddMember("vel", rapidjson::Value(vel), doc.GetAllocator());

    // convert to JSON string
    _str_buffer.Clear();
    auto writer = rapidjson::Writer<rapidjson::StringBuffer>(_str_buffer);
    doc.Accept(writer);
    return _str_buffer.GetString();
}

TODComponent::TODComponent(const robocar::Params &params) : robocar::Component(params)
{
    // params
    _target_velocity = params.get("target_speed").to_double() / 3.6;
    if (_target_velocity <= 0.0)
    {
        throw std::invalid_argument("'target_speed' must be > 0.0");
    }
    _max_steering = params.get("max_steering").to_double();
	if (_max_steering <= 0.0) {
        throw std::invalid_argument("'max_steering' must be > 0.0");
    }
    _max_throttle = params.get("max_throttle").to_double();
    if ((_max_throttle <= 0.0) || (_max_throttle > 1.0)) {
        throw std::invalid_argument("'max_throttle' must belong to ]0.0, 1.0]");
    }
	_max_brake = params.get("max_brake").to_double();
	if ((_max_brake <= 0.0) || (_max_brake > 1.0)) {
        throw std::invalid_argument("'max_brake' must belong to ]0.0, 1.0]");
    }

    // init ZMQ
    //TODO buffer memory limit
    _context = zmq::context_t(2);
    _sock_data = zmq::socket_t(_context, zmq::socket_type::push);
    _sock_data.bind("tcp://*:5000");
    _sock_cmd = zmq::socket_t(_context, zmq::socket_type::pull);
    _sock_cmd.bind("tcp://*:5001");

    // publishers
    _pub_ad_toggle = this->create_publisher<msg::AdToggle>("vehicle/ad_toggle", 1);
    _pub_input = this->create_publisher<msg::ActCmd>("vehicle/input", 1);

    // subscribers
    _sub_vehicle = this->create_subscription<msg::Vehicle>("vehicle/status", 1,
                                                           std::bind(&TODComponent::on_vehicle,
                                                                     this, std::placeholders::_1));
	_sub_gnss = this->create_subscription<msg::GNSS>("sensors/gnss", 1,
													 std::bind(&TODComponent::on_gnss,
															   this, std::placeholders::_1));
    _sub_loc = this->create_subscription<msg::Localization>("localization/position", 1,
                                                            std::bind(&TODComponent::on_loc,
                                                                      this, std::placeholders::_1));
    _sub_camera = this->create_subscription<msg::Image>("sensors/camera",
                                                        rclcpp::SensorDataQoS().keep_last(1),
                                                        std::bind(&TODComponent::on_camera,
                                                                  this, std::placeholders::_1));
}

void TODComponent::serve()
{
    auto lock = std::unique_lock<std::mutex>(_m_vd);

    //TODO optimize this by only applying latest tod_msg
    zmq::recv_result_t nb = 1;
    while (nb) {
        zmq::message_t z_msg;
        nb = _sock_cmd.recv(z_msg, zmq::recv_flags::dontwait);
        on_tod_msg(std::string(static_cast<char*>(z_msg.data()), z_msg.size()));
    }

    auto payload = _vehicle_data.to_bytes();
    auto payload_len = strlen(payload);

    lock.unlock();

    auto z_msg = zmq::message_t(payload_len);
    memcpy(z_msg.data(), payload, payload_len);
    //TODO zmq warning
    _sock_data.send(z_msg);
}

void TODComponent::on_vehicle(const msg::Vehicle &vehicle) {
    _m_vd.lock();
    _vehicle_data.ad_engaged = vehicle.ad_engaged;
    _vehicle_data.steering = vehicle.steering;
    _vehicle_data.steering_n = vehicle.steering / _max_steering;
    _m_vd.unlock();
    _ad_engaged = vehicle.ad_engaged;
}

void TODComponent::on_gnss(const msg::GNSS &gnss)
{
    auto lock = std::unique_lock<std::mutex>(_m_vd);

	_vehicle_data.stamp = robocar::utils::ros_to_unix_ms_time(gnss.header.stamp);
    _vehicle_data.lat = gnss.lat;
    _vehicle_data.lon = gnss.lon;
    _vehicle_data.alt = gnss.altitude;
    _vehicle_data.roll = gnss.roll;
    _vehicle_data.pitch = gnss.pitch;
    _vehicle_data.yaw = gnss.heading;
    _vehicle_data.vel = gnss.velocity;
}

void TODComponent::on_loc(const msg::Localization &loc)
{
    _m_vd.lock();
    _vehicle_data.vel = loc.vel;
    _m_vd.unlock();
}

void TODComponent::on_camera(const msg::Image::ConstSharedPtr& img) {
    //TODO publish img to RTSP server
}

void TODComponent::on_tod_msg(const std::string& tod_msg)
{
    rapidjson::Document doc;
    if (!doc.Parse(tod_msg.c_str()).HasParseError()) {
        // check message
        auto itr = doc.FindMember("stamp");
        if (itr == doc.MemberEnd() || !itr->value.IsUint64()) {
            return;
        }
        itr = doc.FindMember("ad_engage");
        if (itr == doc.MemberEnd() || !itr->value.IsBool()) {
            return;
        }
        itr = doc.FindMember("mode");
        if (itr == doc.MemberEnd() || !itr->value.IsInt()) {
            return;
        }
        itr = doc.FindMember("steering");
        if (itr == doc.MemberEnd() || !itr->value.IsDouble()) {
            return;
        }
        itr = doc.FindMember("throttle");
        if (itr == doc.MemberEnd() || !itr->value.IsDouble()) {
            return;
        }
        itr = doc.FindMember("brake");
        if (itr == doc.MemberEnd() || !itr->value.IsDouble()) {
            return;
        }

        _vehicle_data.tod_msg_stamp = doc["stamp"].GetUint64();

        bool ad_engage = doc["ad_engage"].GetBool();

        int mode = doc["mode"].GetInt();
        if ((mode != ACT_OVERRIDE_PARTIAL) && (mode != ACT_OVERRIDE_FULL)) {
            mode = ACT_OVERRIDE_NONE;
        }

        double steering = doc["steering"].GetDouble();
        steering = steering * _max_steering;
        if (steering > _max_steering) {
            steering = _max_steering;
        }
        if (steering < -_max_steering) {
            steering = -_max_steering;                
        }

        double throttle = doc["throttle"].GetDouble();
        if (throttle < 0.0) {
            throttle = 0.0;
        }
        if (throttle > _max_throttle) {
            throttle = _max_throttle;
        }
        // basic speed limiter
        //TODO improve
        if (_vehicle_data.vel > _target_velocity) {
            throttle = 0.0;
        }

        double brake = doc["brake"].GetDouble();
        if (brake < 0.0) {
            brake = 0.0;                
        }
        if (brake > _max_brake) {
            brake = _max_brake;
        }

        // msg::AdToggle ad_toggle;
        // ad_toggle.toggle = ad_engage;
        // if ((_ad_engaged != ad_toggle.toggle) && _pub_ad_toggle) {
		// 	_pub_ad_toggle->publish(ad_toggle);
        // }

        msg::ActCmd act_cmd;
        act_cmd.header.stamp = this->get_clock()->now();
        act_cmd.mode = mode;
        act_cmd.steering = steering;
        act_cmd.throttle = throttle;
        act_cmd.brake = brake;
        if (_pub_input) {
			_pub_input->publish(act_cmd);
		}
    }
}
