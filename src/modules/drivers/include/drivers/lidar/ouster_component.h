/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef DRIVERS_LIDAR_OUSTER_COMPONENT_H
#define DRIVERS_LIDAR_OUSTER_COMPONENT_H

#include "cycle/cycle.h"
#include "common/common.h"

#include <ouster/client.h>
#include <ouster/types.h>
#include <ouster/lidar_scan.h>

namespace robocar::drivers::lidar
{
	// UDP buffer size, must be at least lidar_packet_bytes + 1 bytes
	const size_t UDP_BUF_SIZE = 65536;

	class OusterComponent : public cycle::Service
	{
	public:
		OusterComponent(const cycle::Params &params);

		void serve() override;

	private:
		// params
		std::string _src_ip = "192.168.1.102";
		std::string _dest_ip = "192.168.1.100";
		int _lidar_port = 7502;
		int _imu_port = 7503;

		// transform
		Eigen::Matrix<double, 4, 4> _tf;
		// ouster
		bool _sensor_connected = false;
		int _nb_timeouts = 0;
		std::unique_ptr<uint8_t[]> _packet_buf{new uint8_t[UDP_BUF_SIZE]};
		std::shared_ptr<ouster::sensor::client> _os_client;
		ouster::sensor::sensor_info _os_info;

		// publisher
		rclcpp::Publisher<msg::PointCloud>::SharedPtr _pub_pc;

		bool connect_sensor();
	};
}

#endif // DRIVERS_LIDAR_OUSTER_COMPONENT_H