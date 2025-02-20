/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/drivers/gnss/gsof_component.h"

#include <boost/array.hpp>

using namespace robocar::drivers::gnss;
using namespace boost::asio;

namespace robocar::drivers::gnss::gsof
{
    const uint8_t STX = 0x02;
    const uint8_t ETX = 0x03;
    const uint8_t MAX_LENGTH = 0xFA;
    // record types
    const uint8_t POSITION_TIME = 0x01;
    const uint8_t LLH = 0x02;
    const uint8_t VELOCITY = 0x08;
    const uint8_t POSITION_SIGMA = 0x0C;
    const uint8_t ATTITUDE = 0x1B;
    const uint8_t INS_NAV_INFO = 0x31;
}

GsofComponent::GsofComponent(const robocar::Params &params) : robocar::Component(params)
{
    // params
    _host = params.get("host").to_string();
    if (_host.empty())
    {
        throw std::invalid_argument("invalid 'host'");
    }
    _port = params.get("port").to_int();
    if (_port < 0)
    {
        throw std::invalid_argument("'port' must be >= 0");
    }
    _leap_seconds = params.get("leap_seconds").to_int();

    // init GNSS message
    _gnss.header.stamp = rclcpp::Time(0);
    _gnss.header.frame_id = "vehicle";
    _gnss.lat = 0.0;
    _gnss.lon = 0.0;
    _gnss.altitude = 0.0;
    _gnss.roll = 0.0;
    _gnss.pitch = 0.0;
    _gnss.heading = 0.0;
    _gnss.velocity_north = 0.0;
    _gnss.velocity_east = 0.0;
    _gnss.velocity_down = 0.0;
    _gnss.velocity = 0.0;
    _gnss.angular_rate_x = 0.0;
    _gnss.angular_rate_y = 0.0;
    _gnss.angular_rate_z = 0.0;
    _gnss.accel_x = 0.0;
    _gnss.accel_y = 0.0;
    _gnss.accel_z = 0.0;
    _gnss.sigma_x = 0.0;
    _gnss.sigma_y = 0.0;

    // connect to GNSS
    _io_service.reset(new io_service());
    _socket.reset(new ip::tcp::socket(*_io_service));
    try
    {
        _socket->connect(ip::tcp::endpoint(ip::address::from_string(_host), _port));
        _connected = true;
        RCLCPP_INFO(this->get_logger(), "GNSS connected");
    }
    catch (std::exception &e)
    {
        _connected = false;
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }

    // publisher
    _pub_gnss = this->create_publisher<msg::GNSS>("sensors/gnss", 1);
}

size_t GsofComponent::read_data(size_t n)
{
    // allocate socket buffer
    auto buffer = std::vector<char>(n);
    boost::array<boost::asio::mutable_buffer, 1> bufs = {boost::asio::buffer(buffer)};
    // read bytes from socket
    size_t n_bytes = 0;
    try
    {
        n_bytes = _socket->receive(bufs);
    }
    catch (boost::system::system_error &e)
    {
        _connected = false;
        if (e.code() == boost::asio::error::eof)
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "GNSS connection closed, trying to reconnect..");
        else
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", e.what());
    }
    // copy bytes to main buffer
    for (int i = 0; i < n_bytes; i++)
        _buffer.push_back(buffer[i]);
    return n_bytes;
}

bool GsofComponent::read_packet()
{
    while (true)
    {
        // look for STX
        if (_buffer.size() < 6)
        {
            if (read_data(gsof::MAX_LENGTH) == 0)
                break;
            continue;
        }
        if (_buffer.front() != gsof::STX)
        {
            _buffer.pop_front();
            continue;
        }

        // header
        if (_buffer[0] != gsof::STX)
            break;
        uint8_t status = _buffer[1];
        uint8_t type = _buffer[2];
        uint8_t length = _buffer[3];
        if (length > gsof::MAX_LENGTH)
            break;
        int cksum = status + type + length;

        // retrieve additional data if needed
        while (_buffer.size() < (length + 6))
        {
            if (read_data(gsof::MAX_LENGTH) == 0)
                break;
        }

        // tx number and page index
        uint8_t tx_number = 0;
        uint8_t page_index = 0;
        uint8_t max_page_index = 0;
        length += 4;
        uint8_t index = 4;
        while (index < length)
        {
            if (index == 4)
                tx_number = _buffer[index];
            if (index == 5)
                page_index = _buffer[index];
            if (index == 6)
                max_page_index = _buffer[index];
            cksum += _buffer[index];
            index++;
        }

        // check for new transmission
        if ((tx_number != _tx_number) || (page_index > _max_page_index) || (page_index <= _page_index))
        {
            if (page_index == 0)
            {
                _tx_number = tx_number;
                _max_page_index = max_page_index;
                _r_buffer.clear();
            }
            else
                break;
        }
        _page_index = page_index;

        // read records
        for (index = 7; index < length; index++)
        {
            _r_buffer.push_back(_buffer[index]);
            if (_r_buffer.size() > 1)
            {
                if (_r_buffer.size() == (_r_buffer[1] + 2))
                {
                    read_record();
                    _r_buffer.clear();
                }
            }
        }

        // trailer
        if (_buffer[length] != (cksum % 256))
            break;
        if (_buffer[length + 1] != gsof::ETX)
            break;

        // remove packet from buffer
        for (int i = 0; i < length + 2; i++)
            _buffer.pop_front();
        return true;
    }

    // drop packet
    if (!_buffer.empty())
        _buffer.pop_front();
    return false;
}

void GsofComponent::serve()
{
    if (_connected)
    {
        if (read_packet())
        {
            _pub_gnss->publish(_gnss);
        }
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        try
        {
            _socket->close();
            _socket->connect(ip::tcp::endpoint(ip::address::from_string(_host), _port));
            _connected = true;
        }
        catch (std::exception &e)
        {
            _connected = false;
        }
    }
}

void GsofComponent::read_record()
{
    if (!_r_buffer.empty())
    {
        switch (_r_buffer[0])
        {
        case gsof::POSITION_TIME:
            if (_r_buffer.size() == (0x0A + 2))
            {
                int gps_time = get_int32(_r_buffer.data() + 2);
                int gps_weeks = get_int16(_r_buffer.data() + 6);
                uint64_t stamp = gps_to_unix_time(gps_weeks, gps_time);
                _gnss.header.stamp = robocar::utils::unix_ms_to_ros_time(stamp);
            }
            break;

        case gsof::LLH:
            if (_r_buffer.size() == (0x18 + 2))
            {
                _gnss.lat = get_double(_r_buffer.data() + 2) * 180.0 / M_PI;
                _gnss.lon = get_double(_r_buffer.data() + 10) * 180.0 / M_PI;
                _gnss.altitude = get_double(_r_buffer.data() + 18);
            }
            break;

        case gsof::VELOCITY:
            if (_r_buffer.size() == (0x0D + 2))
            {
                if (_r_buffer[2] & 1)
                {
                    _gnss.velocity = get_float(_r_buffer.data() + 3);
                }
            }
            break;

        case gsof::POSITION_SIGMA:
            if (_r_buffer.size() == (0x26 + 2))
            {
                _gnss.sigma_x = get_float(_r_buffer.data() + 6);
                _gnss.sigma_y = get_float(_r_buffer.data() + 10);
            }
            break;

        case gsof::ATTITUDE:
            if (_r_buffer.size() == (0x46 + 2))
            {
                _gnss.heading = get_double(_r_buffer.data() + 18) * 180.0 / M_PI;
            }
            break;

        case gsof::INS_NAV_INFO:
            if (_r_buffer.size() == (0x68 + 2))
            {
                int gps_weeks = get_int16(_r_buffer.data() + 2);
                int gps_time = get_int32(_r_buffer.data() + 4);
                uint64_t stamp = gps_to_unix_time(gps_weeks, gps_time);
                _gnss.header.stamp = robocar::utils::unix_ms_to_ros_time(stamp);
                _gnss.lat = get_double(_r_buffer.data() + 10);
                _gnss.lon = get_double(_r_buffer.data() + 18);
                _gnss.altitude = get_double(_r_buffer.data() + 26);
                _gnss.roll = get_double(_r_buffer.data() + 50);
                _gnss.pitch = get_double(_r_buffer.data() + 58);
                _gnss.heading = get_double(_r_buffer.data() + 66);
                _gnss.velocity_north = get_float(_r_buffer.data() + 34);
                _gnss.velocity_east = get_float(_r_buffer.data() + 38);
                _gnss.velocity_down = get_float(_r_buffer.data() + 42);
                _gnss.velocity = get_float(_r_buffer.data() + 46);
                _gnss.angular_rate_x = get_float(_r_buffer.data() + 82);
                _gnss.angular_rate_y = get_float(_r_buffer.data() + 86);
                _gnss.angular_rate_z = get_float(_r_buffer.data() + 90);
                _gnss.accel_x = get_float(_r_buffer.data() + 94);
                _gnss.accel_y = get_float(_r_buffer.data() + 98);
                _gnss.accel_z = get_float(_r_buffer.data() + 102);
            }
            break;

        default:
            break;
        }
    }
}

int16_t GsofComponent::get_int16(uint8_t *data)
{
    union
    {
        uint8_t bytes[2];
        int16_t value;
    };
    bytes[1] = data[0];
    bytes[0] = data[1];
    return value;
}

int32_t GsofComponent::get_int32(uint8_t *data)
{
    union
    {
        uint8_t bytes[4];
        int32_t value;
    };
    bytes[3] = data[0];
    bytes[2] = data[1];
    bytes[1] = data[2];
    bytes[0] = data[3];
    return value;
}

int64_t GsofComponent::get_int64(uint8_t *data)
{
    union
    {
        uint8_t bytes[8];
        int64_t value;
    };
    bytes[7] = data[0];
    bytes[6] = data[1];
    bytes[5] = data[2];
    bytes[4] = data[3];
    bytes[3] = data[4];
    bytes[2] = data[5];
    bytes[1] = data[6];
    bytes[0] = data[7];
    return value;
}

float GsofComponent::get_float(uint8_t *data)
{
    union
    {
        uint8_t bytes[4];
        float value;
    };
    bytes[3] = data[0];
    bytes[2] = data[1];
    bytes[1] = data[2];
    bytes[0] = data[3];
    return value;
}

double GsofComponent::get_double(uint8_t *data)
{
    union
    {
        uint8_t bytes[8];
        double value;
    };
    bytes[7] = data[0];
    bytes[6] = data[1];
    bytes[5] = data[2];
    bytes[4] = data[3];
    bytes[3] = data[4];
    bytes[2] = data[5];
    bytes[1] = data[6];
    bytes[0] = data[7];
    return value;
}

uint64_t GsofComponent::gps_to_unix_time(uint16_t gps_week, uint32_t gps_time)
{
    uint64_t nb_seconds = (gps_week * 7 * 24 * 3600) + 315964800 - _leap_seconds;
    return (nb_seconds * 1000) + gps_time;
}
