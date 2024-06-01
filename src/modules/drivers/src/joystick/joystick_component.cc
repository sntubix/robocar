/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <SDL2/SDL_gamecontroller.h>

#include "drivers/joystick/joystick_component.h"

namespace robocar::drivers::joystick
{

#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define BUTTON_PRESSED_DELAY 5000                 // Button press debounce delay. [microseconds]
#define JOYSTICK_DEVICE_CONTROLLER_INVALID (NULL) // Invalid \ref joystick_device_s.controller value
#define JOYSTICK_ID_DATA_SIZE 16                  // Joystick Identifier Data
#define JOYSTICK_ID_STRING_SIZE 64                // Joystick Description String
#define JOYSTICK_DELAY_INTERVAL 50000

#define JOYSTICK_BUTTON_STATE_NOT_PRESSED 0
#define JOYSTICK_BUTTON_STATE_PRESSED 1
#define JOYSTICK_BUTTON_ENABLE_CONTROLS 0
#define JOYSTICK_BUTTON_DISABLE_CONTROLS 1
#define JOYSTICK_AXIS_STEER 0
#define JOYSTICK_AXIS_THROTTLE 1
#define JOYSTICK_AXIS_BRAKE 2

    enum JoystickStatus
    {
        JOYSTICK_OK,
        JOYSTICK_WARNING,
        JOYSTICK_ERROR
    };

    typedef struct
    {
        unsigned char data[JOYSTICK_ID_DATA_SIZE];
        char ascii_string[JOYSTICK_ID_STRING_SIZE];
    } joystick_guid_s;

    typedef struct
    {
        SDL_GameController *controller;
        SDL_Haptic *haptic;
        joystick_guid_s *guid;
    } joystick_device_data_s;

    static joystick_guid_s joystick_guid;
    static joystick_device_data_s joystick_data = {
        .controller = NULL,
        .haptic = NULL,
        .guid = &joystick_guid};
    static joystick_device_data_s *joystick = NULL;

    int JoystickComponent::joystick_get_guid_at_index(unsigned long device_index)
    {
        int ret = JOYSTICK_ERROR;

        if (joystick != NULL)
        {
            ret = JOYSTICK_OK;

            const SDL_JoystickGUID m_guid =
                SDL_JoystickGetDeviceGUID((int)device_index);

            memcpy(joystick_guid.data, m_guid.data, sizeof(m_guid.data));

            memset(joystick_guid.ascii_string, 0,
                   sizeof(joystick_guid.ascii_string));

            SDL_JoystickGetGUIDString(m_guid,
                                      joystick_guid.ascii_string,
                                      sizeof(joystick_guid.ascii_string));
        }
        return ret;
    }

    int JoystickComponent::joystick_get_num_devices()
    {
        int num_joysticks = JOYSTICK_ERROR;

        if (joystick != NULL)
        {
            num_joysticks = SDL_NumJoysticks();

            if (num_joysticks < 0)
            {
                LOG_ERROR("SDL_NumJoysticks - " + std::string(SDL_GetError()));
                num_joysticks = JOYSTICK_ERROR;
            }
        }
        return (num_joysticks);
    }

    int JoystickComponent::check_trigger_positions()
    {
        int return_code = JOYSTICK_ERROR;

        return_code = joystick_update();

        double normalized_throttle_position = get_normalized_position(JOYSTICK_AXIS_THROTTLE);
        double normalized_brake_position = get_normalized_position(JOYSTICK_AXIS_BRAKE);

        if (return_code == JOYSTICK_OK)
        {
            if ((normalized_throttle_position > 0.0) || (normalized_brake_position > 0.0))
            {
                return_code = JOYSTICK_WARNING;
            }
        }

        return return_code;
    }

    JoystickComponent::JoystickComponent(const cycle::Params &params) : cycle::Service(params)
    {
        // params
        _max_steering = params.get("max_steering").to_double();
        if (_max_steering <= 0.0)
        {
            throw std::invalid_argument("'max_steering' must be > 0.0");
        }
        _steering_pow = params.get("steering_pow").to_double();
        if (_steering_pow <= 0.0)
        {
            throw std::invalid_argument("'steering_pow' must be > 0.0");
        }

        _act.steering = 0.0;
        _act.throttle = 0.0;
        _act.brake = 0.0;

        int ret = JOYSTICK_ERROR;

        if (joystick == NULL)
        {
            int init_result = SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC);

            ret = JOYSTICK_OK;

            if (init_result < 0)
            {
                LOG_ERROR("SDL_Init - " + std::string(SDL_GetError()));
                ret = JOYSTICK_ERROR;
            }
        }

        if (ret == JOYSTICK_ERROR)
        {
            throw std::runtime_error("init subsystem error\n");
        }
        else
        {
            joystick = &joystick_data;
            joystick->controller = JOYSTICK_DEVICE_CONTROLLER_INVALID;

            const int num_joysticks = joystick_get_num_devices();

            if (num_joysticks > 0)
            {
                unsigned long device_index = 0;

                ret = joystick_get_guid_at_index(device_index);

                if (ret == JOYSTICK_OK)
                {
                    LOG_INFO("Found " + std::to_string(num_joysticks) + " devices -- connecting to device at system index " + std::to_string(device_index) + " - GUID: " + std::string(joystick_guid.ascii_string));

                    ret = joystick_open(device_index);
                }
            }
            else
            {
                throw std::runtime_error("no joystick/devices available on the host\n");
            }
        }

        LOG_INFO("waiting for joystick controls to zero");

        while (ret != JOYSTICK_ERROR)
        {
            ret = check_trigger_positions();

            if (ret == JOYSTICK_WARNING)
            {
                (void)usleep(JOYSTICK_DELAY_INTERVAL);
            }
            else if (ret == JOYSTICK_ERROR)
            {
                throw std::runtime_error("failed to wait for joystick to zero the control values\n");
            }
            else
            {
                LOG_INFO("joystick controls successfully initialized");

                break;
            }
        }

        // publishers
        _pub_ad_toggle = this->create_publisher<msg::AdToggle>("vehicle/ad_toggle", 1);
        _pub_input = this->create_publisher<msg::ActCmd>("vehicle/input", 1);
    }

    JoystickComponent::~JoystickComponent()
    {
        joystick_close();
    }

    int JoystickComponent::joystick_open(unsigned long device_index)
    {
        if (joystick != NULL)
        {
            joystick->controller = SDL_GameControllerOpen((int)device_index);

            if (joystick->controller == JOYSTICK_DEVICE_CONTROLLER_INVALID)
            {
                throw std::runtime_error("JOYSTICK_ERROR: SDL_JoystickOpen - %s\n" + std::string(SDL_GetError()));
            }
            else
            {
                const SDL_JoystickGUID m_guid =
                    SDL_JoystickGetGUID(
                        SDL_GameControllerGetJoystick(joystick->controller));

                memcpy(joystick_guid.data, m_guid.data, sizeof(m_guid.data));

                memset(joystick_guid.ascii_string,
                       0,
                       sizeof(joystick_guid.ascii_string));

                SDL_JoystickGetGUIDString(m_guid,
                                          joystick_guid.ascii_string,
                                          sizeof(joystick_guid.ascii_string));

                joystick->haptic = SDL_HapticOpenFromJoystick(
                    SDL_GameControllerGetJoystick(joystick->controller));

                if (SDL_HapticRumbleInit(joystick->haptic) != 0)
                {
                    SDL_HapticClose(joystick->haptic);
                }
            }

            return JOYSTICK_OK;
        }

        return JOYSTICK_ERROR;
    }

    void JoystickComponent::joystick_close()
    {
        if (joystick != NULL)
        {
            if (joystick->controller != JOYSTICK_DEVICE_CONTROLLER_INVALID)
            {
                if (SDL_GameControllerGetAttached(joystick->controller) == SDL_TRUE)
                {
                    if (joystick->haptic)
                    {
                        SDL_HapticClose(joystick->haptic);
                    }
                    SDL_GameControllerClose(joystick->controller);
                }

                joystick->controller = JOYSTICK_DEVICE_CONTROLLER_INVALID;
            }
            joystick = NULL;
        }
        // Release the joystick subsystem
        SDL_Quit();
    }

    int JoystickComponent::joystick_update()
    {
        int ret = JOYSTICK_ERROR;

        if (joystick != NULL)
        {
            if (joystick->controller != JOYSTICK_DEVICE_CONTROLLER_INVALID)
            {
                SDL_GameControllerUpdate();

                if (SDL_GameControllerGetAttached(joystick->controller) == SDL_FALSE)
                {
                    LOG_ERROR("SDL_GameControllerGetAttached - device not attached");
                }
                else
                {
                    ret = JOYSTICK_OK;
                }
            }
        }
        return ret;
    }

    int JoystickComponent::joystick_get_axis(unsigned long axis_index)
    {
        SDL_GameControllerAxis axis = SDL_CONTROLLER_AXIS_INVALID;
        switch (axis_index)
        {
        case JOYSTICK_AXIS_STEER:
            axis = SDL_CONTROLLER_AXIS_LEFTX;
            break;

        case JOYSTICK_AXIS_THROTTLE:
            axis = SDL_CONTROLLER_AXIS_TRIGGERRIGHT;
            break;

        case JOYSTICK_AXIS_BRAKE:
            axis = SDL_CONTROLLER_AXIS_TRIGGERLEFT;
            break;

        default:
            throw std::runtime_error("invalid joystick axis index");
        }

        if (joystick != NULL)
        {
            // const Sint16 position = SDL_GameControllerGetAxis(joystick->controller, axis);
            return (int)SDL_GameControllerGetAxis(joystick->controller, axis); // position;
        }

        return 0;
    }

    int JoystickComponent::joystick_get_button(unsigned long button_index)
    {
        SDL_GameControllerButton button = SDL_CONTROLLER_BUTTON_INVALID;
        switch (button_index)
        {
        case JOYSTICK_BUTTON_ENABLE_CONTROLS:
            button = SDL_CONTROLLER_BUTTON_START;
            break;

        case JOYSTICK_BUTTON_DISABLE_CONTROLS:
            button = SDL_CONTROLLER_BUTTON_BACK;
            break;

        default:
            throw std::runtime_error("invalid joystick button index");
        }

        if (joystick != NULL)
        {
            const Uint8 m_state = SDL_GameControllerGetButton(joystick->controller,
                                                              button);

            if (m_state == 1)
            {
                if (joystick->haptic)
                {
                    SDL_HapticRumblePlay(joystick->haptic, 1.0f, 100);
                }

                (void)usleep(BUTTON_PRESSED_DELAY);

                return JOYSTICK_BUTTON_STATE_PRESSED;
            }
            else
                return JOYSTICK_BUTTON_STATE_NOT_PRESSED;
        }

        return JOYSTICK_BUTTON_STATE_NOT_PRESSED;
    }

    double JoystickComponent::get_normalized_position(unsigned long axis_index)
    {
        static const float deadzone = 0.3;
        int axis_position = joystick_get_axis(axis_index);
        // this is between -1.0 and 1.0
        double raw_normalized_position = ((double)axis_position / INT16_MAX);

        if (axis_index == JOYSTICK_AXIS_STEER)
        {
            // if axis is in deadzone, do nothing
            if (fabs(raw_normalized_position) < deadzone)
            {
                return 0.0;
            }
            else
            {
                // normalize over non deadzone range
                raw_normalized_position *= (fabs(raw_normalized_position) - deadzone) / (1.0 - deadzone);
                return CONSTRAIN(raw_normalized_position, -1.0, 1.0);
            }
        }
        else
        {
            return CONSTRAIN(raw_normalized_position, 0.0, 1.0);
        }

        return 0.0;
    }

    void JoystickComponent::serve()
    {
        if (joystick_update() == JOYSTICK_OK)
        {
            // ad toggle
            if (joystick_get_button(JOYSTICK_BUTTON_ENABLE_CONTROLS) == JOYSTICK_BUTTON_STATE_PRESSED)
            {
                msg::AdToggle ad_toggle;
                ad_toggle.toggle = true;
                _pub_ad_toggle->publish(ad_toggle);
            }
            if (joystick_get_button(JOYSTICK_BUTTON_DISABLE_CONTROLS) == JOYSTICK_BUTTON_STATE_PRESSED)
            {
                msg::AdToggle ad_toggle;
                ad_toggle.toggle = false;
                _pub_ad_toggle->publish(ad_toggle);
            }

            msg::ActCmd act;
            act.header.stamp = cycle::utils::unix_ms_to_ros_time(cycle::Time::now().ms());
            // steering
            double steering = get_normalized_position(JOYSTICK_AXIS_STEER);
            if (steering < 0.0)
                steering = std::pow(std::abs(steering), _steering_pow) * _max_steering;
            else
                steering = -std::pow(steering, _steering_pow) * _max_steering;
            act.steering = steering;
            // throttle and brake
            act.brake = get_normalized_position(JOYSTICK_AXIS_BRAKE);
            if (act.brake == 0.0)
                act.throttle = get_normalized_position(JOYSTICK_AXIS_THROTTLE);
            else
                act.throttle = 0.0;

            // publish actuation
            if ((act.steering != _act.steering) || (act.throttle != _act.throttle) || (act.brake != _act.brake))
            {
                _act = act;
                _pub_input->publish(_act);
            }
            // std::cout << "steering: " << _act.steering() << ", throttle: " << _act.throttle() << ", brake: " << _act.brake() << std::endl;
        }
    }

}