/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef CYCLE_UTILS_H
#define CYCLE_UTILS_H

#include <string>
#include <sstream>
#include <tuple>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

namespace cycle::utils
{

	template <class T>
	static T string2number(std::string const &s)
	{
		std::istringstream iss(s);
		iss.imbue(std::locale("C"));
		T t;
		iss >> t;
		return t;
	}

	static std::string round_str(double val, int precision)
	{
		if (precision < 0)
		{
			throw std::runtime_error("precision cannot be less than 0");
		}
		std::stringstream ss;
		ss.flags(std::ios::fixed);
		ss.precision(precision);
		ss << val;
		return ss.str();
	}

	inline rclcpp::Time unix_ms_to_ros_time(uint64_t timestamp)
	{
		return rclcpp::Time(timestamp * 1e6);
	}

	inline uint64_t ros_to_unix_ms_time(rclcpp::Time timestamp)
	{
		return std::llround(timestamp.nanoseconds() / 1000000.0);
	}

}
#endif // CYCLE_UTILS_H
