/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef CYCLE_MODULE_H
#define CYCLE_MODULE_H

#include <rclcpp/rclcpp.hpp>

#include "cycle/nodes/config.h"
#include "cycle/utils/logging.h"

namespace cycle
{
	class Module : public rclcpp::Node
	{
	public:
		Module(const Params &params)
			: rclcpp::Node(params.name(), rclcpp::NodeOptions().use_intra_process_comms(true))
		{
			Logger::register_prefix(this, params.name());
			_pub_cycle = this->create_publisher<std_msgs::msg::String>("cycle/listener", 1);
		}

		virtual ~Module() = default;

	protected:
		void msg_cycle(const std::string msg)
		{
			std_msgs::msg::String r_msg;
			r_msg.data = msg;
			_pub_cycle->publish(r_msg);
		}

	private:
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_cycle;
	};
}

#endif // CYCLE_MODULE_H