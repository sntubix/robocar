/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "map/map_component.h"
#include "common/geodesy.h"

#include <rapidjson/document.h>

namespace robocar
{
	namespace map
	{

		MapComponent::MapComponent(const cycle::Params &params) : cycle::Service(params)
		{
			// params
			_trace = load_waypoints(params.get("map").to_string());
			_waypoints_length = params.get("waypoints_length").to_int();
			if (_waypoints_length <= 0)
			{
				throw std::invalid_argument("'waypoints_length' must be > 0");
			}
			_waypoints_interdistance = params.get("waypoints_interdistance").to_double();
			if (_waypoints_interdistance <= 0.0)
			{
				throw std::invalid_argument("'waypoints_interdistance' must be > 0.0");
			}
			_waypoints_interdistance_max = params.get("waypoints_interdistance_max").to_double();
			if (_waypoints_interdistance_max <= 0.0)
			{
				throw std::invalid_argument("'waypoints_interdistance_max' must be > 0.0");
			}
			_waypoints_delta = params.get("waypoints_delta").to_double();
			if (_waypoints_delta <= 0.0)
			{
				throw std::invalid_argument("'waypoints_delta' must be > 0.0");
			}
			_x_overlap_offset = params.get("x_overlap_offset").to_double();
			if (_x_overlap_offset <= 0.0)
			{
				throw std::invalid_argument("'x_overlap_offset' must be > 0.0");
			}

			// init loc
			_loc.x = 0.0;
			_loc.y = 0.0;
			_loc.pitch = 0.0;
			_loc.yaw = 0.0;
			_loc.vel = 0.0;
			_loc.accel = 0.0;
			_loc.sigma_x = 0.0;
			_loc.sigma_y = 0.0;

			// publishers
			_pub_path_trace = this->create_publisher<msg::Path>("map/trace", 1);
			_pub_path_waypoints = this->create_publisher<msg::Path>("map/waypoints", 1);
			// subscribers
			_sub_loc = this->create_subscription<msg::Localization>("localization/position", 1,
																	std::bind(&MapComponent::on_localization,
																			  this, std::placeholders::_1));
		}

		msg::Path MapComponent::load_waypoints(const std::string &filename)
		{
			msg::Path trace;

			if (!std::filesystem::exists(filename))
				throw std::runtime_error("map file does not exist '" + filename + "'");

			// open file
			std::ifstream fs(filename, std::ios::in);
			std::string json((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
			// parse file
			rapidjson::Document d;
			d.Parse(json.c_str());
			if (!d.IsObject())
				throw std::runtime_error("unable to parse map file '" + filename + "'");

			// get features
			rapidjson::Value &features = d["features"];
			if (features.IsNull())
				throw std::runtime_error("expected member 'features' in map: " + filename);
			trace.waypoints.resize(static_cast<int>(features.Size()));

			// get points
			geodesy::Position _prev_pos = {0.0, 0.0};
			for (int i = 0; i < static_cast<int>(features.Size()); ++i)
			{
				// get properties
				auto &properties = features[i]["properties"];
				if (properties.IsNull())
					throw std::runtime_error("expected member 'properties' for waypoint: " + std::to_string(i));

				// get waypoint index
				rapidjson::Value id;
				if (properties.HasMember("id"))
					id = properties["id"];
				else if (properties.HasMember("name"))
					id = properties["name"];
				else if (properties.HasMember("Name"))
					id = properties["Name"];
				else
					throw std::runtime_error("expected member 'id' or 'name' for waypoint: " + std::to_string(i));
				int index = 0;
				if (id.IsInt())
					index = id.GetInt();
				else if (id.IsString())
					index = -cycle::utils::string2number<int>(id.GetString());
				else
					throw std::runtime_error("invalid 'id' for waypoint: " + std::to_string(i));
				if (index < 0)
					index = std::abs(index);

				// get coordinates
				auto &coord = features[i]["geometry"]["coordinates"];
				if (coord.IsNull())
					throw std::runtime_error("invalid 'coordinates' for waypoint: " + std::to_string(i));
				double latitude = coord[1].GetDouble() * M_PI / 180.0;
				double longitude = coord[0].GetDouble() * M_PI / 180.0;
				if (i == 0)
					geodesy::Geodesy::set_pos_ref(latitude, longitude);
				auto pos = geodesy::Geodesy::get_pos(latitude, longitude);
				trace.waypoints[index].x = pos.x();
				trace.waypoints[index].y = pos.y();

				// get heading
				if (properties.HasMember("heading"))
				{
					double heading = (5 * M_PI / 2) - (properties["heading"].GetDouble() * M_PI / 180.0);
					if (heading >= (2 * M_PI))
						heading -= (2 * M_PI);
					trace.waypoints[index].yaw = heading;
				}
				else
				{
					// throw std::runtime_error("expected member 'heading' for waypoint: " + std::to_string(i));
					double dx = pos.x() - _prev_pos.x();
					double dy = pos.y() - _prev_pos.y();
					double heading = 0.0;
					if ((dx != 0.0) || (dy != 0.0))
					{
						heading = atan2(dy, dx);
					}
					trace.waypoints[index].yaw = heading;
					if (index > 0)
					{
						trace.waypoints[index - 1].yaw = heading;
					}
				}
				_prev_pos = pos;

				// get speed
				double velocity = 0.0;
				if (properties.HasMember("speed"))
					velocity = properties["speed"].GetDouble();
				else if (properties.HasMember("Field_4"))
				{
					auto str = properties["Field_4"].GetString();
					if (!str)
						throw std::runtime_error("unable to get velocity limit for waypoint: " + std::to_string(i));
					// remove text around value
					std::string speed_str(str);
					speed_str = speed_str.substr(7, speed_str.length());
					for (size_t i = 0; i < 4; i++)
						speed_str.pop_back();
					// convert from mph to m/s
					velocity = cycle::utils::string2number<double>(speed_str) * 0.44704;
				}
				else
					throw std::runtime_error("expected member 'speed' for waypoint: " + std::to_string(i));
				if (velocity < 0.0)
					throw std::runtime_error("velocity limit < 0 for waypoint: " + std::to_string(i));
				if (velocity == 0.0)
					LOG_WARN("velocity limit is 0 for waypoint: " + std::to_string(i));
				trace.waypoints[index].vel = velocity;

				// check traffic light
				if (properties.HasMember("traffic_light"))
				{
					trace.waypoints[index].tfl = properties["traffic_light"].GetBool();
					if (properties.HasMember("traffic_light_right"))
					{
						trace.waypoints[index].tfl_right = properties["traffic_light_right"].GetBool();
					}
					else
					{
						trace.waypoints[index].tfl_right = true;
					}
				}
				else
				{
					trace.waypoints[index].tfl = false;
					trace.waypoints[index].tfl_right = false;
				}
			}

			fs.close();
			LOG_INFO("waypoints loaded: " + std::to_string(trace.waypoints.size()));

			return trace;
		}

		void MapComponent::serve()
		{
			// publish trace
			if (_pub_trace)
			{
				_pub_path_trace->publish(_trace);
				_pub_trace = false;
			}
			// check trace
			if (_trace.waypoints.empty())
			{
				return;
			}

			_mtx.lock();
			auto loc = _loc;
			_mtx.unlock();

			static size_t frame_counter = 0;
			msg::Path waypoints;
			{
				frame_counter = get_closest_waypoint(frame_counter);
				const size_t starting_counter = frame_counter;
				size_t tmp_index = frame_counter;

				msg::MapWaypoint p;
				p.x = loc.x;
				p.y = loc.y;
				double Dx = 0.0, Dy = 0.0;
				do
				{
					auto c = _trace.waypoints[tmp_index];
					double dx = std::abs(c.x - p.x);
					double dy = std::abs(c.y - p.y);
					double edist = std::sqrt((dx * dx) + (dy * dy));

					if (((edist >= _waypoints_interdistance) || (waypoints.waypoints.size() == 0)) && (edist <= _waypoints_interdistance_max))
					{
						if ((Dx != 0.0 && dx < Dx - _x_overlap_offset) || (Dy != 0.0 && dy < Dy - _x_overlap_offset))
							break;
						waypoints.waypoints.push_back(c);

						p = c;
						Dx = dx;
						Dy = dy;
					}

					tmp_index = (tmp_index + 1) % _trace.waypoints.size();
				} while (waypoints.waypoints.size() < _waypoints_length && tmp_index != starting_counter);
			}

			msg::Path waypoints_lane;
			for (int i = 0; i < (int(waypoints.waypoints.size()) - 1); i++)
			{
				msg::MapWaypoint waypoint = waypoints.waypoints[i];

				// remove waypoints behind vehicle
				double x_v = std::cos(loc.yaw) * waypoint.x + std::sin(loc.yaw) * waypoint.y - std::cos(loc.yaw) * loc.x - std::sin(loc.yaw) * loc.y;
				if (x_v <= 0.0)
				{
					continue;
				}

				double x_0 = waypoint.x;
				double y_0 = waypoint.y;
				double x_1 = waypoints.waypoints[i + 1].x;
				double y_1 = waypoints.waypoints[i + 1].y;
				double dx = x_1 - x_0;
				double dy = y_1 - y_0;

				// distance between the two waypoints to interpolate
				double d = 0.0;
				double d_max = std::sqrt((dx * dx) + (dy * dy));
				if (d_max == 0.0)
					continue;
				// direction angle and velocity slope
				double theta = atan2(dy, dx);
				double v_slope = (waypoints.waypoints[i + 1].vel - waypoint.vel) / d_max;
				double velocity = waypoint.vel;

				while (d < d_max)
				{
					// create new waypoint
					msg::MapWaypoint waypoint_int;
					waypoint_int.x = x_0;
					waypoint_int.y = y_0;
					waypoint_int.yaw = waypoint.yaw;
					waypoint_int.vel = velocity;
					// traffic light only for the first waypoint
					if (d == 0.0)
					{
						waypoint_int.tfl = waypoint.tfl;
						waypoint_int.tfl_right = waypoint.tfl_right;
					}
					else
					{
						waypoint_int.tfl = false;
						waypoint_int.tfl_right = false;
					}
					// add waypoint
					waypoints_lane.waypoints.push_back(waypoint_int);

					// interpolation
					velocity += v_slope * _waypoints_delta;
					x_0 += _waypoints_delta * cos(theta);
					y_0 += _waypoints_delta * sin(theta);
					d += _waypoints_delta;
				}
			}

			_pub_path_waypoints->publish(waypoints_lane);
		}

		size_t MapComponent::get_closest_waypoint(size_t frame_counter)
		{
			_mtx.lock();
			auto loc = _loc;
			_mtx.unlock();

			// check trace
			if (_trace.waypoints.empty())
			{
				return 0;
			}

			size_t tmp_index = frame_counter;
			double abs_error_i, temp_error = 999999999999.9;
			uint counter = 0;
			while (counter < _trace.waypoints.size())
			{
				auto p = _trace.waypoints[frame_counter];

				double dx = p.x - loc.x;
				double dy = p.y - loc.y;
				abs_error_i = std::sqrt((dx * dx) + (dy * dy));
				if (abs_error_i <= temp_error)
				{
					tmp_index = frame_counter;
					temp_error = abs_error_i;
				}

				frame_counter = (frame_counter + 1) % _trace.waypoints.size();
				counter++;
			}
			return tmp_index;
		}

		void MapComponent::on_localization(const msg::Localization &loc)
		{
			std::unique_lock<std::mutex> lock(_mtx);
			_loc = loc;
		}

	} // namespace map
} // namespace robocar