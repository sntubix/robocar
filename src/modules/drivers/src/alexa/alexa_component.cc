/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#include "drivers/alexa/alexa_component.h"

using namespace robocar::drivers::alexa;

AlexaComponent::AlexaComponent(const cycle::Params& params) : cycle::Service(params) {
    // params
    _serial_port = params.get("serial_port").to_string();
    if (_serial_port.empty()) {
        throw std::invalid_argument("invalid 'serial_port'");
    }
    _target_velocity = params.get("target_speed").to_double() / 3.6;
    if (_target_velocity <= 0.0) {
        throw std::invalid_argument("'target_speed' must be > 0.0");
    }

    // open serial port
    try {
        _serial.Open(_serial_port);
    }
   	catch (const LibSerial::OpenFailed& e) {
        LOG_ERROR("unable to open serial port");
		throw e;
    }
    _serial.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    _serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    _serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    _serial.SetParity(LibSerial::Parity::PARITY_NONE);
    _serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    // publishers
    _pub_ad_toggle = this->create_publisher<msg::AdToggle>("vehicle/ad_toggle", 1);
    _pub_target_speed = this->create_publisher<msg::TargetSpeed>("voice/target_speed", 1);
}

AlexaComponent::~AlexaComponent() {
    _serial.Close();
}

void AlexaComponent::serve() {
    try {
        // read line
        std::string str;
        _serial.ReadLine(str, (char)10, 250);

        // convert string to id and value
        auto id = std::atoi(std::string(str.substr(0, 1)).c_str());
        auto value = std::atoi(std::string(str.substr(1)).c_str());

        switch (id) {
            case 0: {
                msg::AdToggle ad_toggle;
                ad_toggle.toggle = value;
                _pub_ad_toggle->publish(ad_toggle);
                break;
            }
            case 2: {
                double ratio = value / 255.0;
                if (ratio < 0.0) {
                    ratio = 0.0;
                }
                if (ratio > 1.0) {
                    ratio = 1.0;
                }
                msg::TargetSpeed target_speed;
                target_speed.speed = ratio * _target_velocity;
                _pub_target_speed->publish(target_speed);
                break;
            }
            default: {
                LOG_ERROR("unsupported id: " + std::to_string(id));
                break;
            }
        }
    }
    catch (const LibSerial::ReadTimeout& e) {
        // do nothing
    }
}