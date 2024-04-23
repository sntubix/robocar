/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef DRIVERS_JOYSTICK_JOYSTICK_CONTROLLER_H
#define DRIVERS_JOYSTICK_JOYSTICK_CONTROLLER_H

#include "cycle/cycle.h"
#include "common/common.h"

namespace robocar::drivers::joystick {

class JoystickComponent : public cycle::Service {
public:
    JoystickComponent(const cycle::Params& params);
    ~JoystickComponent() override;

	void serve() override;

private:
    // params
    double _max_steering = 0.6108652;
    double _steering_pow = 1.0;

    msg::ActCmd _act;

    // publishers
    rclcpp::Publisher<msg::AdToggle>::SharedPtr _pub_ad_toggle;
	rclcpp::Publisher<msg::ActCmd>::SharedPtr _pub_input;

    int joystick_get_guid_at_index(unsigned long device_index);
    int joystick_get_num_devices();

    int joystick_open(unsigned long device_index);
    void joystick_close();

    int joystick_update();
    int check_trigger_positions();

    int joystick_get_axis(const unsigned long axis_index);
    int joystick_get_button(const unsigned long button_index);
    double get_normalized_position(unsigned long axis_index);
};

}

#endif // DRIVERS_JOYSTICK_JOYSTICK_CONTROLLER_H