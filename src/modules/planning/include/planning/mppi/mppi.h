/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef PLANNING_MPPI_H
#define PLANNING_MPPI_H

#include <CL/cl.h>
#include <vector>
#include <random>

namespace mppi {

typedef struct Params {
    std::string cl_file = "";
    uint64_t platform_id = 0;
    uint64_t local_size = 128;
    uint64_t multiplier = 15;
    uint64_t N = 12;
    float delta = 0.25;
    float lambda = 1.0;
    float sigma_s = 0.05;
    float sigma_a = 0.85;
    float wheel_base = 2.57;
    float max_steering = 0.55;
    float max_steering_rate = 0.11;
    float max_accel = 1.0;
    float min_accel = -2.5;
    float diverge_threshold = 1.0;
    float diverge_left = 0.5;
    float diverge_right = 0.5;
    float rss_min_dist = 7.0;
	float rss_reaction_time = 0.3;
	float rss_max_accel = 2.5;
	float rss_min_brake = 2.0;
	float rss_max_brake = 9.0;
    float stop_velocity = 1.5;
} Params;

class Waypoint {
    public:
        Waypoint(float x, float y, float yaw, float yaw_rate, float vel, float accel)
            : _x(x), _y(y), _yaw(yaw), _yaw_rate(yaw_rate), _vel(vel), _accel(accel) {}

        float x() { return _x; }
        float y() { return _y; }
        float yaw() { return _yaw; }
        float yaw_rate() { return _yaw_rate; }
        float vel() { return _vel; }
        float accel() { return _accel; }

    private:
        float _x = 0.0;
        float _y = 0.0;
        float _yaw = 0.0;
        float _yaw_rate = 0.0;
        float _vel = 0.0;
        float _accel = 0.0;
};

class MPPI {
    public:
        MPPI(Params params);

        std::vector<Waypoint> run(const std::vector<float>& state,
                                  std::vector<float> waypoints,
                                  std::vector<float> objects);

    private:
        // params
        Params _p;
        uint64_t _K = 0;
        std::vector<float> _params;

        cl_platform_id* _platforms;
        cl_device_id _device;
        cl_context _context;
        cl_command_queue _queue;
        cl_program _program;
        cl_kernel _kernel;

        cl_mem __params;
        cl_mem __state;
        cl_mem __inputs;
        cl_mem __seed;
        cl_mem __noise;
        cl_mem __costs;

        std::random_device _rd;
        std::mt19937 _rg_seed;
        //std::mt19937 _rg_s;
        //std::mt19937 _rg_a;
        std::uniform_int_distribution<uint32_t> _dist_seed;
        //std::normal_distribution<float> _dist_s;
        //std::normal_distribution<float> _dist_a;

        //uint64_t _prev_seed_time = 0;
        float _dt = 0.0f;
        size_t _state_size = 6;
        float* _state;
        size_t _waypoint_dim = 4;
        size_t _object_dim = 4;
        size_t _input_dim = 2;
        size_t _inputs_size;
        float* _inputs;
        uint32_t* _seed;
        size_t _noise_size;
        float* _noise;
        float* _costs;
    };
}

#endif // PLANNING_MPPI_H