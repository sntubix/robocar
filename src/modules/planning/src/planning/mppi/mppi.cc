/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#include "planning/mppi/mppi.h"

#include <fstream>
#include <iostream>
#include <chrono>
#include <limits>

using namespace mppi;
namespace chrono = std::chrono;

MPPI::MPPI(Params params) {
    // params
    _p = params;
    _K = _p.local_size * _p.multiplier;

    // params buffer
    _params.push_back(_p.delta);
    _params.push_back(_p.sigma_s);
    _params.push_back(_p.sigma_a);
    _params.push_back(_p.wheel_base);
    _params.push_back(_p.max_steering);
    _params.push_back(_p.max_steering_rate);
    _params.push_back(_p.max_accel);
    _params.push_back(_p.min_accel);
    _params.push_back(_p.diverge_threshold);
    _params.push_back(_p.diverge_left);
    _params.push_back(_p.diverge_right);
    _params.push_back(_p.rss_min_dist);
    _params.push_back(_p.rss_reaction_time);
    _params.push_back(_p.rss_max_accel);
    _params.push_back(_p.rss_min_brake);
    _params.push_back(_p.rss_max_brake);
    _params.push_back(_p.stop_velocity);

    // state
    _state = new float[_state_size];
    for (int i=0; i < _state_size; i++) {
        _state[i] = 0.0;
    }

    // seed
    _rg_seed = std::mt19937(_rd());
    _dist_seed = std::uniform_int_distribution<uint32_t>(0, std::numeric_limits<uint32_t>::max());
    _seed = new uint32_t[_K];
    for (int i=0; i < _K; i++) {
        _seed[i] = _dist_seed(_rg_seed);
    }

    // inputs
    _inputs_size = _input_dim * _p.N;
    _inputs = new float[_inputs_size];
    for (int i=0; i < _inputs_size; i++) {
        _inputs[i] = 0.0;
    }

    // noise
    //_rg_s = std::mt19937(_rd());
    //_rg_a = std::mt19937(_rd());
    //_dist_s = std::normal_distribution<float>(0.0, _p.sigma_s);
    //_dist_a = std::normal_distribution<float>(0.0, _p.sigma_a);
    _noise_size = _K * _inputs_size;
    _noise = new float[_noise_size];
    for (int i=0; i < _noise_size; i++) {
        _noise[i] = 0.0;
    }

    // costs
    _costs = new float[_K];
    for (int i=0; i < _K; i++) {
        _costs[i] = 0.0;
    }

    // get a platform
    cl_uint num_platforms = 0;
    if (_p.platform_id > 10)
        throw std::runtime_error("invalid platform_id: " + std::to_string(_p.platform_id));
    _platforms = new cl_platform_id[_p.platform_id + 1];
    cl_int ret = clGetPlatformIDs(_p.platform_id + 1, _platforms, &num_platforms);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to get OpenCL platforms");
    if (num_platforms <= _p.platform_id)
        throw std::runtime_error("unable to get OpenCL platform id: " + std::to_string(_p.platform_id));

    // get a GPU device
    ret = clGetDeviceIDs(_platforms[_p.platform_id], CL_DEVICE_TYPE_GPU, 1, &_device, NULL);
    if (ret == CL_DEVICE_NOT_FOUND)
        throw std::runtime_error("unable to get an OpenCL device");

    // create a context and command queue
    _context = clCreateContext(NULL, 1, &_device, NULL, NULL, &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create an OpenCL context");
    _queue = clCreateCommandQueue(_context, _device, 0, &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create an OpenCL queue");

    // load kernel code
    std::ifstream ifs;
    ifs.open(_p.cl_file);
    auto str = std::string((std::istreambuf_iterator<char>(ifs)),
                           (std::istreambuf_iterator<char>()));
    ifs.close();
    if (str.empty())
        throw std::runtime_error("unable to load OpenCL program: '" + _p.cl_file + "'");
    size_t size = str.size();
    auto source = new char[size];
    for (int i=0; i < size; i++) {
        source[i] = str[i];
    }

    // build program and get kernel
    _program = clCreateProgramWithSource(_context, 1, (const char**) &source, &size, &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL program");
    ret = clBuildProgram(_program, 1, &_device, NULL, NULL, NULL);
    if (ret != CL_SUCCESS) {
        size_t log_size = 0;
        clGetProgramBuildInfo(_program, _device, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);
        std::string log;
        log.resize(log_size);
        clGetProgramBuildInfo(_program, _device, CL_PROGRAM_BUILD_LOG, log_size + 1, log.data(), NULL);
        std::cout << log << std::endl;
        throw std::runtime_error("unable to build OpenCL program");
    }
    _kernel = clCreateKernel(_program, "mppi", &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to build OpenCL kernel");

    // params buffer
    __params = clCreateBuffer(_context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                              _params.size() * sizeof(cl_float), _params.data(), &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL buffer");
    // state buffer
    __state = clCreateBuffer(_context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                             _state_size * sizeof(cl_float), _state, &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL buffer");
    // inputs buffer
    __inputs = clCreateBuffer(_context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                              _inputs_size * sizeof(cl_float), _inputs, &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL buffer");
    // seed buffer
    __seed = clCreateBuffer(_context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                            _K * sizeof(cl_uint), _seed, &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL buffer");
    // noise buffer
    __noise = clCreateBuffer(_context, CL_MEM_WRITE_ONLY | CL_MEM_COPY_HOST_PTR,
                             _noise_size * sizeof(cl_float), _noise, &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL buffer");
    // costs buffer
    __costs = clCreateBuffer(_context, CL_MEM_WRITE_ONLY | CL_MEM_COPY_HOST_PTR,
                               _K * sizeof(cl_float), _costs, &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL buffer");

    // set kernel arguments
    ret = clSetKernelArg(_kernel, 0, sizeof(cl_uint), (void*) &_p.N);
    ret |= clSetKernelArg(_kernel, 1, sizeof(cl_mem), (void*) &__params);
    ret |= clSetKernelArg(_kernel, 2, sizeof(cl_mem), (void*) &__state);
    ret |= clSetKernelArg(_kernel, 5, sizeof(cl_uint), (void*) &_waypoint_dim);
    ret |= clSetKernelArg(_kernel, 8, sizeof(cl_uint), (void*) &_object_dim);
    ret |= clSetKernelArg(_kernel, 9, sizeof(cl_mem), (void*) &__inputs);
    ret |= clSetKernelArg(_kernel, 10, sizeof(cl_uint), (void*) &_input_dim);
    ret |= clSetKernelArg(_kernel, 11, sizeof(cl_mem), (void*) &__seed);
    ret |= clSetKernelArg(_kernel, 12, sizeof(cl_mem), (void*) &__noise);
    ret |= clSetKernelArg(_kernel, 13, sizeof(cl_mem), (void*) &__costs);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to set OpenCL kernel arguments");
}

std::vector<Waypoint> MPPI::run(const std::vector<float>& state,
                                std::vector<float> waypoints,
                                std::vector<float> objects) {
    //auto time = chrono::high_resolution_clock::now().time_since_epoch();
    //uint64_t now = static_cast<uint64_t>(chrono::duration_cast<chrono::milliseconds>(time).count());
    cl_int ret;

    if (state.empty() || waypoints.empty())
        return std::vector<Waypoint>();

    // update state buffer
    _state[0] = state[0]; // x
    _state[1] = state[1]; // y
    _state[2] = state[2]; // yaw
    _state[3] = state[3]; // steering
    _state[4] = state[4]; // velocity
    ret = clEnqueueWriteBuffer(_queue, __state, CL_TRUE, 0,
                               _state_size * sizeof(cl_float), _state, 0, NULL, NULL);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to enqueue OpenCL write buffer");

    // update inputs buffer
    ret = clEnqueueWriteBuffer(_queue, __inputs, CL_TRUE, 0,
                               _inputs_size * sizeof(cl_float), _inputs, 0, NULL, NULL);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to enqueue OpenCL write buffer");

    // reset seed
    /*
    if ((now - _prev_seed_time) >= 5000) {
        _prev_seed_time = now;
        for (int i=0; i < _K; i++) {
            _seed[i] = _dist_seed(_rg_seed);
        }
        ret = clEnqueueWriteBuffer(_queue, __seed, CL_TRUE, 0,
                                   _K * sizeof(cl_uint), _seed, 0, NULL, NULL);
        if (ret != CL_SUCCESS)
            throw std::runtime_error("unable to enqueue OpenCL write buffer");
    }
    */

    // generate noise
    size_t offset_n = 0;
    /*
    for (int i=0; i < (_K * _p.N); i++) {
        _noise[offset_n] = _dist_s(_rg_s);
        _noise[offset_n+1] = _dist_a(_rg_a);
        offset_n += _input_dim;
    }
    ret = clEnqueueWriteBuffer(_queue, __noise, CL_TRUE, 0,
                               _noise_size * sizeof(cl_float), _noise, 0, NULL, NULL);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to enqueue OpenCL write buffer");
    */

    // waypoints buffer
    auto _waypoints = clCreateBuffer(_context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                     waypoints.size() * sizeof(cl_float), waypoints.data(), &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL buffer");
    uint32_t _waypoints_size = waypoints.size();

    // update kernel arguments
    ret |= clSetKernelArg(_kernel, 3, sizeof(cl_mem), (void*) &_waypoints);
    ret |= clSetKernelArg(_kernel, 4, sizeof(cl_uint), (void*) &_waypoints_size);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to set OpenCL kernel arguments");

    // objects buffer
    if (objects.empty()) {
        for (size_t i=0; i < _object_dim; i++) {
            objects.push_back(0.0);
        }
    }
    auto _objects = clCreateBuffer(_context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                   objects.size() * sizeof(cl_float), objects.data(), &ret);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to create OpenCL buffer");
    uint32_t _objects_size = objects.size();

    // update kernel arguments
    ret |= clSetKernelArg(_kernel, 6, sizeof(cl_mem), (void*) &_objects);
    ret |= clSetKernelArg(_kernel, 7, sizeof(cl_uint), (void*) &_objects_size);
    if (ret != CL_SUCCESS)
        throw std::runtime_error("unable to set OpenCL kernel arguments");

    // run kernel
    ret = clEnqueueNDRangeKernel(_queue, _kernel, 1, NULL, &_K, &_p.local_size, 0, NULL, NULL);
    if (ret != CL_SUCCESS) {
        throw std::runtime_error("unable to enqueue OpenCL kernel");
    }

    // get noise and costs
    ret = clEnqueueReadBuffer(_queue, __noise, CL_TRUE, 0,
                              _noise_size * sizeof(cl_float), _noise, 0, NULL, NULL);
    ret |= clEnqueueReadBuffer(_queue, __costs, CL_TRUE, 0,
                               _K * sizeof(cl_float), _costs, 0, NULL, NULL);
    if (ret != CL_SUCCESS) {
        throw std::runtime_error("unable to enqueue OpenCL read buffer");
    }

    // compute min cost
    double min_cost = std::numeric_limits<double>::max();
    for (int i=0; i < _K; i++) {
        if (_costs[i] < min_cost)
            min_cost = _costs[i];
    }
    // compute weights in double precision
    auto weights = std::vector<double>(_K);
    for (int i=0; i < _K; i++) {
        weights[i] = exp((-1.0 / _p.lambda) * (_costs[i] - min_cost));
    }

    // compute the sum of the weights
    double sum_weights = 0.0;
    for (int i=0; i < _K; i++) {
        sum_weights += weights[i];
    }
    // normalize the weights
    if (sum_weights != 0.0) {
        for (int i=0; i < _K; i++) {
            weights[i] = weights[i] / sum_weights;
        }
    }

    // update nominal input
    size_t offset_i = 0;
    offset_n = 0;
    for (int i=0; i < _p.N; i++) {
        for (int j=0; j < _K; j++) {
            _inputs[offset_i] += weights[j] * _noise[offset_n];
            _inputs[offset_i+1] += weights[j] * _noise[offset_n+1];
            offset_n += _input_dim;
        }
        offset_i += _input_dim;
    }

    // Savitzky–Golay filter on input
    std::vector<float> inputs(_inputs_size);
    offset_i = 0;
    for (int i=0; i < _p.N; i++) {
        for (int j=0; j < _input_dim; j++) {
            double y_m_2 = 0.0;
            if (i > 1) {
                y_m_2 = _inputs[offset_i - (2 * _input_dim) + j];
            }

            double y_m_1 = 0.0;
            if (i > 0) {
                y_m_1 = _inputs[offset_i - (1 * _input_dim) + j];
            }

            double y_p_1 = 0.0;
            if (i < (_p.N - 1)) {
                y_p_1 = _inputs[offset_i + (1 * _input_dim) + j];
            }

            double y_p_2 = 0.0;
            if (i < (_p.N - 2)) {
                y_p_2 = _inputs[offset_i + (2 * _input_dim) + j];
            }

            inputs[offset_i+j] = (1.0 / 35.0) * (-3.0 * y_m_2 + 12.0 * y_m_1
                                                 + 17.0 * _inputs[offset_i+j]
                                                 + 12.0 * y_p_1 - 3.0 * y_p_2);
        }

        offset_i += _input_dim;
    }
    for (int i=0; i < _inputs_size; i++) {
        _inputs[i] = inputs[i];
    }

    float x = state[0];
    float y = state[1];
    float yaw = state[2];
    float yaw_rate = 0.0f;
    float steering = state[3];
    float vel = state[4];
    std::vector<Waypoint> trajectory;
    offset_i = 0;
    // compute trajectory from input sequence
    for (int i=0; i < _p.N; i++) {
        steering += _inputs[offset_i] * _p.delta;
        vel = fmax(vel + (_inputs[offset_i+1] * _p.delta), 0.0);
        yaw_rate = (vel * tan(steering) / _p.wheel_base);
        yaw += yaw_rate * _p.delta;
        x += cos(yaw) * vel * _p.delta;
        y += sin(yaw) * vel * _p.delta;

        // empty trajectory if no input update performed
        if (sum_weights != 0.0) {
            trajectory.push_back(Waypoint(x, y, yaw, yaw_rate, vel, _inputs[offset_i+1]));
        }

        offset_i += _input_dim;
    }

    // shift nominal input
    _dt += state[5];
    if (_dt >= _p.delta) {
        _dt = 0.0f;
        offset_i = 0;
        for (int i=0; i < _p.N; i++) {
            if (i == (_p.N - 1)) {
                //_inputs[offset_i] = 0.0f;
                //_inputs[offset_i+1] = 0.0f;
            }
            else {
                _inputs[offset_i] = _inputs[offset_i+_input_dim];
                _inputs[offset_i+1] = _inputs[offset_i+_input_dim+1];
            }
            offset_i += _input_dim;
        }
    }

    clReleaseMemObject(_waypoints);
    clReleaseMemObject(_objects);
    return trajectory;
}