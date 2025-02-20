/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/map/mapping_component.h"

namespace robocar
{
	namespace map
	{

		MappingComponent::MappingComponent(const robocar::Params &params) : robocar::Component(params)
		{
			// params
			_waypoints_interval = params.get("waypoints_interval").to_double();
			if (_waypoints_interval <= 0.0)
			{
				throw std::invalid_argument("'waypoints_interval' must be > 0.0");
			}

			// publishers
			_pub_toggle = this->create_publisher<msg::Mapping>("mapping/toggle", 1);
			_pub_status = this->create_publisher<msg::Mapping>("mapping/status", 1);
			// subscribers
			_sub_toggle = this->create_subscription<msg::Mapping>("mapping/toggle", 1,
																  std::bind(&MappingComponent::on_toggle,
																			this, std::placeholders::_1));
			_sub_vehicle = this->create_subscription<msg::Vehicle>("vehicle/status", 1,
																   std::bind(&MappingComponent::on_vehicle,
																			 this, std::placeholders::_1));
			_sub_gnss = this->create_subscription<msg::GNSS>("sensors/gnss", 1,
															 std::bind(&MappingComponent::on_gnss,
																	   this, std::placeholders::_1));

			_prev_gnss.header.stamp = rclcpp::Time(0);
			_prev_gnss.lat = 0.0;
			_prev_gnss.lon = 0.0;
			_prev_gnss.altitude = 0.0;
			_prev_gnss.roll = 0.0;
			_prev_gnss.pitch = 0.0;
			_prev_gnss.heading = 0.0;
			_prev_gnss.velocity_north = 0.0;
			_prev_gnss.velocity_east = 0.0;
			_prev_gnss.velocity_down = 0.0;
			_prev_gnss.velocity = 0.0;
			_prev_gnss.angular_rate_x = 0.0;
			_prev_gnss.angular_rate_y = 0.0;
			_prev_gnss.angular_rate_z = 0.0;
			_prev_gnss.accel_x = 0.0;
			_prev_gnss.accel_y = 0.0;
			_prev_gnss.accel_z = 0.0;
			_prev_gnss.sigma_x = 0.0;
			_prev_gnss.sigma_y = 0.0;
		}

		void MappingComponent::serve()
		{
			std::unique_lock<std::mutex> lock(_m);
			flush_queue();
		}

		void MappingComponent::flush_queue()
		{
			while (!_queue.empty())
			{
				auto gnss = std::move(_queue.front());
				_queue.pop_front();

				// feature
				rapidjson::Document feature;
				feature.SetObject();
				feature.AddMember("type", "Feature", feature.GetAllocator());

				// properties
				rapidjson::Value properties;
				properties.SetObject();
				properties.AddMember("name", _counter, feature.GetAllocator());
				properties.AddMember("latitude", gnss.lat, feature.GetAllocator());
				properties.AddMember("longitude", gnss.lon, feature.GetAllocator());
				properties.AddMember("heading", gnss.heading, feature.GetAllocator());
				properties.AddMember("speed", gnss.velocity, feature.GetAllocator());
				feature.AddMember("properties", properties, feature.GetAllocator());

				// geometry
				rapidjson::Value geometry;
				geometry.SetObject();
				geometry.AddMember("type", "Point", feature.GetAllocator());
				// coordinates
				auto coordinates = rapidjson::Value(rapidjson::kArrayType);
				coordinates.PushBack(gnss.lon, feature.GetAllocator());
				coordinates.PushBack(gnss.lat, feature.GetAllocator());
				geometry.AddMember("coordinates", coordinates, feature.GetAllocator());
				feature.AddMember("geometry", geometry, feature.GetAllocator());

				// write to file
				feature.Accept(*_writer);
				if (_file)
				{
					std::fwrite(_buffer->GetString(), sizeof(char), _buffer->GetSize(), _file);
					std::fwrite(",", sizeof(char), 1, _file);
				}
				_buffer->Clear();
				_counter++;
			}
		}

		double MappingComponent::compute_dist(const msg::GNSS &gnss_a, const msg::GNSS &gnss_b)
		{
			const double earth_radius = 6378137.0;
			double lat_a = gnss_a.lat * M_PI / 180.0;
			double lon_a = gnss_a.lon * M_PI / 180.0;
			double lat_b = gnss_b.lat * M_PI / 180.0;
			double lon_b = gnss_b.lon * M_PI / 180.0;

			// haversine formula
			double sqrt = std::sqrt(std::pow(sin((lat_b - lat_a) / 2.0), 2.0) + cos(lat_a) * cos(lat_b) * std::pow(sin((lon_b - lon_a) / 2.0), 2.0));
			return 2 * earth_radius * asin(sqrt);
		}

		void MappingComponent::on_toggle(const msg::Mapping &mapping)
		{
			if ((_mapping) && (!mapping.status))
			{
				// flush queue before closing file
				std::unique_lock<std::mutex> lock(_m);
				flush_queue();
				// close file
				auto str = std::string("]}");
				if (_counter > 0)
				{
					std::fseek(_file, -1 * sizeof(char), SEEK_END);
				}
				std::fwrite(str.c_str(), sizeof(char), str.size(), _file);
				std::fclose(_file);
				_mapping = false;
				lock.unlock();

				// publish status
				auto m_status = mapping;
				m_status.status = _mapping;
				_pub_status->publish(m_status);
				RCLCPP_INFO(this->get_logger(), "saved map file to '%s'", _filename.c_str());
			}

			if ((!_mapping) && (mapping.status))
			{
				std::unique_lock<std::mutex> lock(_m);
				// open file
				_filename = mapping.filename.data;
				_file = std::fopen(_filename.c_str(), "w");
				if (!_file)
				{
					throw std::runtime_error("unable to save map to '" + _filename + "'");
				}

				// init
				auto str = std::string("{\"type\":\"FeatureCollection\",\"features\":[");
				std::fwrite(str.c_str(), sizeof(char), str.size(), _file);
				_buffer.reset(new rapidjson::StringBuffer());
				_writer.reset(new rapidjson::Writer(*_buffer));
				_queue.clear();
				_counter = 0;
				_mapping = true;
				lock.unlock();

				// publish status
				auto m_status = mapping;
				m_status.status = _mapping;
				_pub_status->publish(m_status);
				RCLCPP_INFO(this->get_logger(), "starting map recording");
			}
		}

		void MappingComponent::on_vehicle(const msg::Vehicle &vehicle)
		{
			if ((vehicle.gnss != STATUS_OK) && _mapping)
			{
				RCLCPP_ERROR(this->get_logger(), "invalid GNSS status, stopping map recording");

				std::unique_lock<std::mutex> lock(_m);
				msg::Mapping mapping;
				mapping.header.stamp = vehicle.header.stamp;
				mapping.status = false;
				mapping.filename.data = _filename;
				_pub_toggle->publish(mapping);
			}
		}

		void MappingComponent::on_gnss(const msg::GNSS &gnss)
		{
			if (_mapping)
			{
				if (compute_dist(gnss, _prev_gnss) >= _waypoints_interval)
				{
					std::unique_lock<std::mutex> lock(_m);
					_queue.push_back(gnss);
					_prev_gnss = gnss;
				}
			}
		}

	} // namespace map
} // namespace robocar