/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef CYCLE_TIME_H
#define CYCLE_TIME_H

#include <chrono>

using namespace std::chrono;

namespace cycle
{
	class Time
	{
		_V2::system_clock::duration _time;

	public:
		Time(_V2::system_clock::duration time) : _time(time) {}

		static Time now()
		{
			return Time(high_resolution_clock::now().time_since_epoch());
		}

		inline uint64_t s()
		{
			return static_cast<uint64_t>(duration_cast<seconds>(_time).count());
		}

		inline uint64_t ms()
		{
			return static_cast<uint64_t>(duration_cast<milliseconds>(_time).count());
		}

		inline uint64_t us()
		{
			return static_cast<uint64_t>(duration_cast<microseconds>(_time).count());
		}
	};
}

#endif // CYCLE_TIME_H
