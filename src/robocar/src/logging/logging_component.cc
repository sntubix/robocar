/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/logging/logging_component.h"

namespace robocar
{
	namespace logging
	{

		LoggingComponent::LoggingComponent(const robocar::Params &params) : robocar::Component(params)
		{
			// params
			auto names = params.get("value_names").to_string();
			std::string str = "";
			for (int i = 0; i < names.size(); i++)
			{
				if (names[i] == ',')
				{
					_names.push_back(str);
					_values.insert({str, 0.0});
					str = "";
				}
				else
				{
					str.push_back(names[i]);
					if (i == (names.size() - 1))
						_names.push_back(str);
					_values.insert({str, 0.0});
				}
			}
			int queue_size = params.get("queue_size").to_int();
			if (queue_size <= 0)
			{
				throw std::invalid_argument("'queue_size' must be > 0");
			}

			// create log file
			_file = std::fopen("robocar_log.csv", "w");
			if (_file == NULL)
				throw std::runtime_error("unable to create log file");
			names = "timestamp,ad_engaged";
			for (auto &name : _names)
			{
				names = names + "," + name;
			}
			names = names + "\n";
			std::fwrite(names.c_str(), sizeof(char), names.size(), _file);
			std::fflush(_file);

			// subscribers
			_sub_entry = this->create_subscription<msg::LogEntry>("logging/entry", queue_size,
																  std::bind(&LoggingComponent::on_entry,
																			this, std::placeholders::_1));
			_sub_vehicle = this->create_subscription<msg::Vehicle>("vehicle/status", 1,
																   std::bind(&LoggingComponent::on_vehicle,
																			 this, std::placeholders::_1));
		}

		LoggingComponent::~LoggingComponent()
		{
			std::fclose(_file);
		}

		void LoggingComponent::serve()
		{
			if (_ad_engaged)
			{
				write_entry(false);
			}
		}

		void LoggingComponent::write_entry(bool flush)
		{
			std::unique_lock<std::mutex> lock(_mtx);

			std::string str = std::to_string(robocar::Time::now().ms()) + "," + std::to_string(_ad_engaged);
			for (auto &name : _names)
			{
				str = str + "," + std::to_string(_values[name]);
			}
			str = str + "\n";

			std::fwrite(str.c_str(), sizeof(char), str.size(), _file);
			if (flush)
			{
				std::fflush(_file);
			}
		}

		void LoggingComponent::on_entry(const msg::LogEntry &entry)
		{
			std::unique_lock<std::mutex> lock(_mtx);

			if (entry.name.size() != entry.value.size())
			{
				throw std::runtime_error("invalid log entry");
			}

			for (int i = 0; i < entry.name.size(); i++)
			{
				_values[entry.name[i].data] = entry.value[i];
			}
		}

		void LoggingComponent::on_vehicle(const msg::Vehicle &vehicle)
		{
			if (_ad_engaged && !vehicle.ad_engaged)
			{
				_ad_engaged = vehicle.ad_engaged;
				write_entry(true);
			}
			else
			{
				_ad_engaged = vehicle.ad_engaged;
			}
		}

	} // namespace logging
} // namespace robocar