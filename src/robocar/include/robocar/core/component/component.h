/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_CORE_COMPONENT_COMPONENT_H
#define ROBOCAR_CORE_COMPONENT_COMPONENT_H

#include <atomic>
#include <thread>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "robocar/core/component/config.h"
#include "robocar/core/utils/time.h"

namespace robocar
{
	class Component : public rclcpp::Node
	{
	public:
		Component(const Params &params, bool loop = true)
			: rclcpp::Node("robocar_" + params.name(), rclcpp::NodeOptions().use_intra_process_comms(true)),
			  _period(get_period(params, loop))
		{
			_pub_robocar = this->create_publisher<std_msgs::msg::String>("robocar/listener", 1);
		}

		virtual ~Component() = default;

		virtual void serve() {};

		void start()
		{
			if (!_loop)
				return;

			this->stop();
			auto lock = std::unique_lock<std::mutex>(_mtx);
			_serve_thread.reset(new std::thread(
				[&]()
				{
					_run = true;
					while (_run)
					{
						auto start = robocar::Time::now().ms();
						serve();
						auto serve_time = robocar::Time::now().ms() - start;
						if (_period > serve_time)
						{
							std::this_thread::sleep_for(std::chrono::milliseconds(_period - serve_time));
						}
					}
				}));
		}

		void stop()
		{
			auto lock = std::unique_lock<std::mutex>(_mtx);
			_run = false;
			if (_serve_thread)
			{
				if (_serve_thread->joinable())
				{
					_serve_thread->join();
				}
			}
		}

		bool is_running()
		{
			auto lock = std::unique_lock<std::mutex>(_mtx);
			if (_serve_thread)
			{
				return _serve_thread->joinable();
			}
			return false;
		}

		uint64_t period()
		{
			return _period;
		}

	protected:
		void msg_robocar(const std::string msg)
		{
			std_msgs::msg::String r_msg;
			r_msg.data = msg;
			_pub_robocar->publish(r_msg);
		}

	private:
		int get_period(const Params &params, bool loop)
		{
			if (loop)
				return params.get("period").to_int();
			return 0;
		}

		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_robocar;

		const bool _loop = true;
		const uint64_t _period = 0;
		std::mutex _mtx;
		std::unique_ptr<std::thread> _serve_thread;
		std::atomic<bool> _run = false;
	};
}

#endif // ROBOCAR_CORE_COMPONENT_COMPONENT_H