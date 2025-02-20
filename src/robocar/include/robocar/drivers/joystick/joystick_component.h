/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_DRIVERS_JOYSTICK_JOYSTICK_CONTROLLER_H
#define ROBOCAR_DRIVERS_JOYSTICK_JOYSTICK_CONTROLLER_H

#include "robocar/core/robocar.h"

namespace robocar::drivers::joystick
{

    class JoystickComponent : public robocar::Component
    {
    public:
        JoystickComponent(const robocar::Params &params);
        ~JoystickComponent() override;

        void serve() override;

    private:
        // params
        double _max_steering = 0.6108652;
        double _steering_pow = 1.0;

        int _prev_act_mode = ACT_OVERRIDE_NONE;

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

#endif // ROBOCAR_DRIVERS_JOYSTICK_JOYSTICK_CONTROLLER_H