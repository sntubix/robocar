/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_DRIVERS_GNSS_GSOF_COMPONENT_H
#define ROBOCAR_DRIVERS_GNSS_GSOF_COMPONENT_H

#include "robocar/core/robocar.h"

#include <deque>
#include <boost/asio.hpp>

namespace robocar::drivers::gnss
{
	class GsofComponent : public robocar::Component
	{
	public:
		GsofComponent(const robocar::Params &params);

		void serve() override;

	private:
		// params
		std::string _host = "";
		int _port = 0;
		int _leap_seconds = 18;

		bool _connected = false;
		std::deque<uint8_t> _buffer;
		std::unique_ptr<boost::asio::io_service> _io_service;
		std::unique_ptr<boost::asio::ip::tcp::socket> _socket;

		int _tx_number = -1;
		int _page_index = -1;
		uint8_t _max_page_index = 0;
		std::vector<uint8_t> _r_buffer;
		msg::GNSS _gnss;

		rclcpp::Publisher<msg::GNSS>::SharedPtr _pub_gnss;

		size_t read_data(size_t n);
		bool read_packet();
		void read_record();

		int16_t get_int16(uint8_t *data);
		int32_t get_int32(uint8_t *data);
		int64_t get_int64(uint8_t *data);
		float get_float(uint8_t *data);
		double get_double(uint8_t *data);

		uint64_t gps_to_unix_time(uint16_t gps_week, uint32_t gps_time);
	};
}

#endif // ROBOCAR_DRIVERS_GNSS_GSOF_COMPONENT_H