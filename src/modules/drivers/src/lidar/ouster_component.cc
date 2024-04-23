/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#include "drivers/lidar/ouster_component.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace robocar::drivers::lidar;

bool OusterComponent::connect_sensor() {
    _os_client = ouster::sensor::init_client(_src_ip, _dest_ip,
                                             ouster::sensor::MODE_1024x20,
                                             ouster::sensor::TIME_FROM_INTERNAL_OSC,
                                             _lidar_port, _imu_port);
    if (!_os_client) {
        return false;
    }
    auto meta = ouster::sensor::get_metadata(*_os_client);
    if (meta == "") {
        return false;
    }
    _os_info = ouster::sensor::parse_metadata(meta);
    return true;
}

OusterComponent::OusterComponent(const cycle::Params& params) : cycle::Service(params) {
    // params
    _src_ip = params.get("src_ip").to_string();
    if (_src_ip.empty()) {
        throw std::invalid_argument("invalid 'src_ip'");
    }
    _dest_ip = params.get("dest_ip").to_string();
    if (_dest_ip.empty()) {
        throw std::invalid_argument("invalid 'dest_ip'");
    }
    _lidar_port = params.get("lidar_port").to_int();
    if (_lidar_port < 0) {
        throw std::invalid_argument("'lidar_port' must be >= 0");
    }
    _imu_port = params.get("imu_port").to_int();
    if (_imu_port < 0) {
        throw std::invalid_argument("'imu_port' must be >= 0");
    }
    double theta = params.get("tf_theta").to_double() * M_PI / 180.0;
    _tf << cos(theta), -sin(theta), 0.0, params.get("tf_x").to_double(),
           sin(theta), cos(theta),  0.0, params.get("tf_y").to_double(),
           0.0,        0.0,         1.0, params.get("tf_z").to_double(),
           0.0,        0.0,         0.0, 1.0;

    // publisher
    _pub_pc = this->create_publisher<msg::PointCloud>("sensors/points", 1);

    // connect to lidar
    LOG_INFO("connecting sensor at '" + _src_ip + "'..");
    _sensor_connected = connect_sensor();
    if (_sensor_connected) {
        LOG_INFO("sensor connected");
    }
    else {
        LOG_ERROR("unable to connect sensor at '" + _src_ip + "'");
    }
}

void OusterComponent::serve() {
    if (_sensor_connected) {
        ouster::sensor::packet_format pf = ouster::sensor::get_format(_os_info);
        ouster::ScanBatcher scan_batcher(_os_info.format.columns_per_frame, pf);
        // a LidarScan holds lidar data for an entire rotation of the device
        ouster::LidarScan scan(_os_info.format.columns_per_frame, 
                               _os_info.format.pixels_per_column, 
                               _os_info.format.udp_profile_lidar);
        auto xyz_lut = ouster::make_xyz_lut(_os_info);

        while (this->is_running()) {
            // wait until sensor data is available
            ouster::sensor::client_state st = ouster::sensor::poll_client(*_os_client);

            // check for timeout
            if (st == ouster::sensor::TIMEOUT) {
                _nb_timeouts++;
                if (_nb_timeouts > 5) {
                    LOG_ERROR("sensor timeout, trying to reconnect..");
                    _sensor_connected = false;
                }
                break;
            }

            // check for error status
            if (st & ouster::sensor::CLIENT_ERROR) {
                LOG_ERROR("sensor client returned error state, trying to reconnect..");
                _sensor_connected = false;
                break;
            }

            // check for lidar data, read a packet and add it to the current batch
            if (st & ouster::sensor::LIDAR_DATA) {
                _nb_timeouts = 0;
                if (!ouster::sensor::read_lidar_packet(*_os_client, _packet_buf.get(), pf)) {
                    LOG_WARN("failed to read a packet of the expected size, dropping current scan");
                    break;
                }

                // batcher will return "true" when the current scan is complete
                if (scan_batcher(_packet_buf.get(), scan)) {
                    // check timestamp
                    size_t index = -1;
                    for (size_t i=0; i < scan.timestamp().size(); i++) {
                        if (scan.timestamp()(i) != 0) {
                            index = i;
                            break;
                        }
                    }

                    if (index != -1) {
                        // retrieve points and intensity
                        const auto& points = ouster::cartesian(scan, xyz_lut);
                        const auto& signal = scan.field(ouster::sensor::SIGNAL);

                        // init cloud
                        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                        cloud->resize(scan.w * scan.h);
                        cloud->header.stamp = cycle::Time::now().us();

                        // add points
                        for (auto i=0; i < scan.h; i++) {
                            for (auto j=0; j < scan.w; j++) {
                                auto index = i * scan.w + j;
                                const auto& xyz = points.row(index);

                                cloud->at(index) = pcl::PointXYZI{
                                    {{static_cast<float>(xyz(0)),
                                      static_cast<float>(xyz(1)),
                                      static_cast<float>(xyz(2)),
                                      static_cast<float>(1.0f)}}
                                };
                                cloud->at(index).intensity = signal(i, j);
                            }
                        }

                        // convert to vehicle frame and publish
                        pcl::transformPointCloud(*cloud, *cloud, _tf);
                        cloud->header.frame_id = "vehicle";
                        if (cloud->size() > 0) {
                            msg::PointCloud pc_msg;
                            pcl::toROSMsg(*cloud, pc_msg);
                            _pub_pc->publish(pc_msg);
                        }
                        break;
                    }
                }
            }

            // clear IMU data flag
            if (st & ouster::sensor::IMU_DATA) {
                ouster::sensor::read_imu_packet(*_os_client, _packet_buf.get(), pf);
            }
        }
    }
    else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        _sensor_connected = connect_sensor();
    }
}