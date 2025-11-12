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

std::mutex _m_g;
static GstElement* _gst_appsrc;

static void rtsp_media_configure(GstRTSPMediaFactory* factory, GstRTSPMedia* media, gpointer user_data)
{
    if (_gst_appsrc != NULL) {
        gst_object_unref(_gst_appsrc);
    }

    std::unique_lock<std::mutex> lock(_m_g);

    GstElement* element = gst_rtsp_media_get_element(media);
    _gst_appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "tod");
    gst_object_unref(element);
}

TODComponent::TODComponent(const robocar::Params &params) : robocar::Component(params)
{
    // params
    _target_velocity = params.get("target_speed").to_double() / 3.6;
    if (_target_velocity <= 0.0)
    {
        throw std::invalid_argument("'target_speed' must be > 0.0");
    }
    _sl_threshold = params.get("speed_limiter_threshold").to_double();
    if ((_sl_threshold <= 0.0) || (_sl_threshold > 1.0)) {
        throw std::invalid_argument("'speed_limiter_threshold' must belong to ]0.0, 1.0]");
    }
    _sl_smoothing = params.get("speed_limiter_smoothing").to_double();
    if ((_sl_smoothing < 0.0) || (_sl_smoothing >= 1.0)) {
        throw std::invalid_argument("'speed_limiter_smoothing' must belong to [0.0, 1.0[");
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
    _stream_width = params.get("stream_width").to_int();
	if ((_stream_width < 128) || (_stream_width > 3840)) {
        throw std::invalid_argument("'stream_width' must belong to [128, 3840]");
    }
    _stream_height = params.get("stream_height").to_int();
	if ((_stream_height < 72) || (_stream_height > 2160)) {
        throw std::invalid_argument("'stream_height' must belong to [72, 2160]");
    }
    _stream_framerate = params.get("stream_framerate").to_int();
	if ((_stream_framerate < 1) || (_stream_framerate > 60)) {
        throw std::invalid_argument("'stream_framerate' must belong to [1, 60]");
    }
    _stream_bitrate = params.get("stream_bitrate").to_int();
	if ((_stream_bitrate < 100) || (_stream_bitrate > 100000)) {
        throw std::invalid_argument("'stream_bitrate' must belong to [100, 100000]");
    }

    // init ZMQ
    _context = zmq::context_t(2);
    _sock_data = zmq::socket_t(_context, zmq::socket_type::push);
    _sock_data.bind("tcp://*:5000");
    _sock_cmd = zmq::socket_t(_context, zmq::socket_type::pull);
    _sock_cmd.bind("tcp://*:5001");

    // init GStreamer
    _gst_argc = params.get_argc();
    _gst_argv = params.get_argv();
    gst_init(&_gst_argc, &_gst_argv);

    // specify GStreamer pipeline
    std::ostringstream gst_pipeline;
    gst_pipeline << "( appsrc name=tod is-live=true block=true do-timestamp=true format=TIME "
                 << "caps=video/x-raw,format=BGR"
                 << ",width=" << _stream_width << ",height=" << _stream_height
                 << ",framerate=" << _stream_framerate << "/1 ! "
                 << "videoconvert ! video/x-raw,format=I420 ! "
                 << "x264enc bitrate=" << _stream_bitrate << " speed-preset=superfast tune=zerolatency ! "
                 << "rtph264pay name=pay0 pt=96 )";

    // init RTSP server
    _gst_rtsp_server = gst_rtsp_server_new();
    _gst_rtsp_mounts = gst_rtsp_server_get_mount_points(_gst_rtsp_server);

    // init RTSP media factory
    _gst_rtsp_factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_shared(_gst_rtsp_factory, true);
    gst_rtsp_media_factory_set_launch(_gst_rtsp_factory, gst_pipeline.str().c_str());

    // add RTSP mount point
    g_signal_connect(_gst_rtsp_factory, "media-configure", (GCallback)rtsp_media_configure, NULL);
    gst_rtsp_mount_points_add_factory(_gst_rtsp_mounts, "/robocar", _gst_rtsp_factory);
    g_object_unref(_gst_rtsp_mounts);

    // attach RTSP server to main context
    gst_rtsp_server_attach(_gst_rtsp_server, NULL);

    // init GStreamer main loop
    _gst_loop = g_main_loop_new(NULL, FALSE);

    // publishers
    _pub_ad_toggle = this->create_publisher<msg::AdToggle>("vehicle/ad_toggle", 1);
    _pub_input = this->create_publisher<msg::ActCmd>("vehicle/input", 1);

    // subscribers
    _sub_act_cmd = this->create_subscription<msg::ActCmd>("actuator/command", 1,
                                                          std::bind(&TODComponent::on_act_cmd,
                                                                    this, std::placeholders::_1));
    _sub_vehicle = this->create_subscription<msg::Vehicle>("vehicle/status", 1,
                                                           std::bind(&TODComponent::on_vehicle,
                                                                     this, std::placeholders::_1));
	_sub_gnss = this->create_subscription<msg::GNSS>("sensors/gnss", 1,
													 std::bind(&TODComponent::on_gnss,
															   this, std::placeholders::_1));
    _sub_camera_0 = this->create_subscription<msg::Image>("sensors/camera_0",
                                                          rclcpp::SensorDataQoS().keep_last(1),
                                                          std::bind(&TODComponent::on_camera_0,
                                                                    this, std::placeholders::_1));
    _sub_camera_1 = this->create_subscription<msg::Image>("sensors/camera_1",
                                                          rclcpp::SensorDataQoS().keep_last(1),
                                                          std::bind(&TODComponent::on_camera_1,
                                                                    this, std::placeholders::_1));
}

void TODComponent::serve()
{
    g_main_context_iteration(g_main_loop_get_context(_gst_loop), FALSE);

    auto lock = std::unique_lock<std::mutex>(_m_av);

    // retrieve latest TOD message
    std::string l_msg;
    zmq::recv_result_t nb = 1;
    while (nb) {
        zmq::message_t z_msg;
        nb = _sock_cmd.recv(z_msg, zmq::recv_flags::dontwait);

        if (nb) {
            l_msg.assign(static_cast<char*>(z_msg.data()), z_msg.size());
        }
    }
    // process TOD message
    if (!l_msg.empty()) {
        on_tod_msg(l_msg);
    }

    auto payload = _vehicle_data.to_bytes();
    auto payload_len = strlen(payload);

    lock.unlock();

    auto z_msg = zmq::message_t(payload_len);
    memcpy(z_msg.data(), payload, payload_len);
    _sock_data.send(z_msg, zmq::send_flags::dontwait);
}

void TODComponent::on_act_cmd(const msg::ActCmd &act_cmd) {
    _m_av.lock();
    _act_mode = act_cmd.mode;
    _m_av.unlock();
}

void TODComponent::on_vehicle(const msg::Vehicle &vehicle) {
    _m_av.lock();
    _vehicle_data.ad_engaged = vehicle.ad_engaged;
    _vehicle_data.steering = vehicle.steering;
    _vehicle_data.steering_n = vehicle.steering / _max_steering;
    //_vehicle_data.vel = vehicle.velocity;
    _m_av.unlock();
}

void TODComponent::on_gnss(const msg::GNSS &gnss)
{
    auto lock = std::unique_lock<std::mutex>(_m_av);

	_vehicle_data.stamp = robocar::utils::ros_to_unix_ms_time(gnss.header.stamp);
    _vehicle_data.lat = gnss.lat;
    _vehicle_data.lon = gnss.lon;
    _vehicle_data.alt = gnss.altitude;
    _vehicle_data.roll = gnss.roll;
    _vehicle_data.pitch = gnss.pitch;
    _vehicle_data.yaw = gnss.heading;
    _vehicle_data.vel = gnss.velocity;
}

void TODComponent::on_camera_0(const msg::Image::ConstSharedPtr& img) {
    // check image
    if (img->encoding.empty() || img->data.empty())
    {
        return;
    }

    // convert front frame (cam_0) to cv image
    auto cv_cam_0 = cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::BGR8);

    // resize image if necessary
    if ((cv_cam_0->image.cols != _stream_width) || (cv_cam_0->image.rows != _stream_height)) {
        cv::resize(cv_cam_0->image, cv_cam_0->image, cv::Size(_stream_width, _stream_height));
    }

    _m_av.lock();
    int act_mode = _act_mode;
    bool ad_engaged = _vehicle_data.ad_engaged;
    //double vehicle_steering = _vehicle_data.steering;
    double vehicle_velocity = _vehicle_data.vel;
    _m_av.unlock();

    std::unique_lock<std::mutex> lock_c(_m_c);

    // overlay the vehicle speed on cam_0
    std::string speed_str = robocar::utils::round_str(vehicle_velocity * 3.6, 0) + " KM/H";
    cv::putText(cv_cam_0->image, speed_str, cv::Point((_stream_width / 2) - 30, _stream_height - 25),
                cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 0), 2);

    // overlay ADS status on cam_0
    if (ad_engaged)
    {
        cv::putText(cv_cam_0->image, "ADS ON", cv::Point(200, _stream_height - 25),
                    cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    }
    else
    {
        cv::putText(cv_cam_0->image, "ADS OFF", cv::Point(200, _stream_height - 25),
                    cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }

    // overlay actuation mode on cam_0
    auto tod_str = std::string("TOD OFF");
    auto tod_color = cv::Scalar(0, 0, 255);
    if ((act_mode == ACT_OVERRIDE_PARTIAL) || (act_mode == ACT_OVERRIDE_FULL))
    {
        tod_str = std::string("TOD ON");
        tod_color = cv::Scalar(0, 255, 0);
    }
    cv::putText(cv_cam_0->image, tod_str, cv::Point(30, _stream_height - 25),
                cv::FONT_HERSHEY_DUPLEX, 1.0, tod_color, 2);

    if (_cv_cam_1) {
        // resize the cabin frame (cam_1) to be smaller
        double scale = 0.3;
        cv::resize(_cv_cam_1->image, _cv_cam_1->image,
                   cv::Size(static_cast<int>(cv_cam_0->image.cols * scale),
                            static_cast<int>(cv_cam_0->image.rows * scale)));

        // put cabin frame in the upper right corner of the front frame
        int y1 = 0;
        int y2 = _cv_cam_1->image.rows;
        int x1 = cv_cam_0->image.cols - _cv_cam_1->image.cols;
        int x2 = cv_cam_0->image.cols;

        // overlay the cabin frame onto the front frame
        _cv_cam_1->image.copyTo(cv_cam_0->image(cv::Rect(x1, y1, _cv_cam_1->image.cols,
                                                         _cv_cam_1->image.rows)));
    }

    lock_c.unlock();

    // convert image to gst buffer
    GstBuffer *buffer;
    GstMapInfo map;
    int size = cv_cam_0->image.total() * cv_cam_0->image.elemSize();
    buffer = gst_buffer_new_allocate(NULL, size, NULL);
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, cv_cam_0->image.data, size);
    gst_buffer_unmap(buffer, &map);

    GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(gst_clock_get_time(gst_system_clock_obtain()), 1, GST_USECOND);
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, 30);

    std::unique_lock<std::mutex> lock_g(_m_g);

    if (_gst_appsrc == NULL) {
        gst_buffer_unref(buffer);
        return;
    }

    // push buffer
    GstFlowReturn ret;
    g_signal_emit_by_name(_gst_appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    //if (ret != GST_FLOW_OK) break;
    //g_signal_emit_by_name(_appsrc, "end-of-stream", &ret);
}

void TODComponent::on_camera_1(const msg::Image::ConstSharedPtr& img) {
    // check image
    if (img->encoding.empty() || img->data.empty())
    {
        return;
    }

    _m_c.lock();
    _cv_cam_1 = cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::BGR8);
    _m_c.unlock();
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
        if (itr == doc.MemberEnd() || !itr->value.IsInt()) {
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

        int ad_engage = doc["ad_engage"].GetInt();

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
        if (_vehicle_data.vel > (_sl_threshold * _target_velocity)) {
            throttle = _sl_smoothing * _prev_throttle;
        }
        else {
            throttle = (1.0 - _sl_smoothing) * throttle + _sl_smoothing * _prev_throttle;
        }
        _prev_throttle = throttle;

        double brake = doc["brake"].GetDouble();
        if (brake < 0.0) {
            brake = 0.0;                
        }
        if (brake > _max_brake) {
            brake = _max_brake;
        }

        if ((ad_engage == 0) || (ad_engage == 1)) {
            msg::AdToggle ad_toggle;
            ad_toggle.toggle = ad_engage;
            if (_pub_ad_toggle) {
                _pub_ad_toggle->publish(ad_toggle);
            }
        }

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
