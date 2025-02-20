/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

float get_noise(uint *seed, float mean, float std)
{
    uint r = *seed;
    float s = 0.0f;
    float u = 0.0f;
    float v = 0.0f;

    while ((s == 0.0f) || (s >= 1.0f))
    {
        // LCG
        r = (1103515245 * r + 12345) & 0xffffffff;
        // set in [-1,1]
        u = (2.0f * (float)r / 0xffffffff) - 1.0f;

        // LCG
        r = (1103515245 * r + 12345) & 0xffffffff;
        // set in [-1,1]
        v = (2.0f * (float)r / 0xffffffff) - 1.0f;
        s = (u * u) + (v * v);
    }
    *seed = r;

    return mean + (u * sqrt((-2.0f * log(s)) / s)) * std;
}

float get_diff_angle(float a, float b)
{
    const float PI = 3.1415927f;
    float diff = a - b;
    if (diff > PI)
        diff -= (2 * PI);
    if (diff < -PI)
        diff += (2 * PI);
    return fabs(diff);
}

float dir_obj_dist(float x, float y, float yaw, float obj_x, float obj_y, float obj_r)
{
    // check object is in front
    float cos_theta = cos(yaw);
    float sin_theta = sin(yaw);
    float w_x = cos_theta * obj_x + sin_theta * obj_y - cos_theta * x - sin_theta * y;
    if (w_x < -obj_r)
    {
        return -1.0f;
    }

    // distance between direction line and object
    float x_0 = x;
    float y_0 = y;
    float x_1 = x + cos_theta;
    float y_1 = y + sin_theta;
    float a = y_0 - y_1;
    float b = x_1 - x_0;
    float c = x_0 * y_1 - x_1 * y_0;
    float dist = fabs(a * obj_x + b * obj_y + c) / sqrt(a * a + b * b);

    return dist;
}

float get_obstacle_dist(float reaction_time,
                        float stop_velocity,
                        float vehicle_x,
                        float vehicle_y,
                        float vehicle_vel,
                        float x, float y, float yaw, float vel, float accel,
                        __constant float *waypoints,
                        uint waypoints_size,
                        uint waypoint_dim,
                        uint offset_c_w,
                        __constant float *objects,
                        uint objects_size,
                        uint object_dim)
{
    const uint nb_waypoints = waypoints_size / waypoint_dim;
    const uint nb_objects = objects_size / object_dim;
    const float cos_theta = cos(yaw);
    const float sin_theta = sin(yaw);

    // check vehicle too close
    float v_dx = x - vehicle_x;
    float v_dy = y - vehicle_y;
    float v_dist = sqrt(v_dx * v_dx + v_dy * v_dy);
    if (v_dist < (vehicle_vel * reaction_time))
    {
        return -1.0f;
    }

    // check vehicle at stop velocity
    if (vel <= stop_velocity)
    {
        return -1.0f;
    }

    // obstacles
    float min_obj_d = -1.0f;
    uint offset_o = 0;
    for (uint i = 0; i < nb_objects; i++)
    {
        float obj_x = objects[offset_o];
        float obj_y = objects[offset_o + 1];
        float obj_r = objects[offset_o + 2];

        // check collision
        float dx = obj_x - x;
        float dy = obj_y - y;
        float obj_d = sqrt(dx * dx + dy * dy) - obj_r;
        if (obj_d <= 0.0f)
        {
            if ((obj_d < min_obj_d) || (min_obj_d == -1.0f))
            {
                min_obj_d = obj_d;
            }
        }

        // check collision using direction and velocity
        float dir_dist = dir_obj_dist(x, y, yaw, obj_x, obj_y, obj_r);
        if ((dir_dist != -1.0f) && (dir_dist <= obj_r))
        {
            if ((obj_d < min_obj_d) || (min_obj_d == -1.0f))
            {
                min_obj_d = obj_d;
            }
        }

        // check collision using waypoints
        uint offset_w = 0;
        for (uint j = 0; j < nb_waypoints; j++)
        {
            dx = obj_x - waypoints[offset_w];
            dy = obj_y - waypoints[offset_w + 1];
            if ((sqrt(dx * dx + dy * dy) - obj_r) <= 0.0f)
            {
                if ((obj_d < min_obj_d) || (min_obj_d == -1.0f))
                {
                    min_obj_d = obj_d;
                }
            }
            offset_w += waypoint_dim;
        }

        offset_o += object_dim;
    }

    return min_obj_d;
}

__kernel void mppi(uint N,
                   __constant float *params,
                   __constant float *state,
                   __constant float *waypoints,
                   uint waypoints_size,
                   uint waypoint_dim,
                   __constant float *objects,
                   uint objects_size,
                   uint object_dim,
                   __constant float *inputs,
                   uint input_dim,
                   __global uint *seed,
                   __global float *noise,
                   __global float *costs)
{
    // params
    const float delta = params[0];
    const float sigma_s = params[1];
    const float sigma_a = params[2];
    const float wheel_base = params[3];
    const float max_steering = params[4];
    const float max_steering_rate = params[5];
    const float max_accel = params[6];
    const float min_accel = params[7];
    const float diverge_threshold = params[8];
    const float diverge_left = params[9];
    const float diverge_right = params[10];
    const float rss_min_dist = params[11];
    const float rss_reaction_time = params[12];
    const float rss_max_accel = params[13];
    const float rss_min_brake = params[14];
    const float rss_max_brake = params[15];
    const float stop_velocity = params[16];

    // check args
    bool args_valid = true;
    args_valid &= (N > 0);
    args_valid &= (delta > 0.0f);
    args_valid &= (waypoints_size >= waypoint_dim);
    args_valid &= (waypoint_dim >= 4);
    args_valid &= ((waypoints_size % waypoint_dim) == 0);
    args_valid &= (objects_size >= 0);
    args_valid &= (object_dim >= 3);
    args_valid &= ((objects_size % object_dim) == 0);
    args_valid &= (input_dim >= 2);
    if (!args_valid)
    {
        return;
    }

    const size_t K = get_global_size(0);
    const size_t global_id = get_global_id(0);
    const uint nb_waypoints = waypoints_size / waypoint_dim;

    // state
    const float vehicle_x = state[0];
    const float vehicle_y = state[1];
    const float vehicle_yaw = state[2];
    const float vehicle_steering = state[3];
    const float vehicle_vel = state[4];

    uint _seed = seed[global_id];
    float x = vehicle_x;
    float y = vehicle_y;
    float yaw = vehicle_yaw;
    float steering = vehicle_steering;
    float vel = vehicle_vel;
    float prev_target_dist = 0.0f;
    float cost = 0.0f;

    // compute vehicle minimum distance
    uint offset_w = 0;
    uint offset_c_w = 0;
    float v_min_dist = FLT_MAX;
    for (uint i = 0; i < nb_waypoints; i++)
    {
        float dx = vehicle_x - waypoints[offset_w];
        float dy = vehicle_y - waypoints[offset_w + 1];
        float dist = sqrt(dx * dx + dy * dy);

        // vehicle minimun distance
        if (dist < v_min_dist)
        {
            offset_c_w = offset_w;
            v_min_dist = dist;
        }

        offset_w += waypoint_dim;
    }
    float cos_yaw = cos(waypoints[offset_c_w + 2]);
    float sin_yaw = sin(waypoints[offset_c_w + 2]);
    v_min_dist = fabs(-sin_yaw * vehicle_x + cos_yaw * vehicle_y + sin_yaw * waypoints[offset_c_w] - cos_yaw * waypoints[offset_c_w + 1]);

    uint offset_n = global_id * input_dim;
    uint offset_i = 0;
    for (uint i = 0; i < N; i++)
    {
        // get perturbations
        float n_steering_rate = get_noise(&_seed, 0.0f, sigma_s);
        float n_accel = get_noise(&_seed, 0.0f, sigma_a);
        // float n_steering_rate = noise[offset_n];
        // float n_accel = noise[offset_n+1];

        // compute steering
        float steering_rate = inputs[offset_i] + n_steering_rate;
        if ((steering + steering_rate * delta) > max_steering)
        {
            n_steering_rate = ((max_steering - steering) / delta) - inputs[offset_i];
            steering_rate = inputs[offset_i] + n_steering_rate;
        }
        if ((steering + steering_rate * delta) < -max_steering)
        {
            n_steering_rate = ((-max_steering - steering) / delta) - inputs[offset_i];
            steering_rate = inputs[offset_i] + n_steering_rate;
        }
        float _max_steering_rate = max_steering_rate;
        if (steering_rate > _max_steering_rate)
        {
            steering_rate = _max_steering_rate;
            n_steering_rate = _max_steering_rate - inputs[offset_i];
        }
        if (steering_rate < -_max_steering_rate)
        {
            steering_rate = -_max_steering_rate;
            n_steering_rate = -_max_steering_rate - inputs[offset_i];
        }
        steering += steering_rate * delta;

        // compute acceleration
        float accel = inputs[offset_i + 1] + n_accel;
        if ((vel + accel * delta) < 0.0f)
        {
            accel = -vel / delta;
            n_accel = accel - inputs[offset_i + 1];
        }
        if (accel > max_accel)
        {
            accel = max_accel;
            n_accel = accel - inputs[offset_i + 1];
        }
        if (accel < min_accel)
        {
            accel = min_accel;
            n_accel = accel - inputs[offset_i + 1];
        }

        // compute new position
        vel += accel * delta;
        yaw += (vel * tan(steering) / wheel_base) * delta;
        if (yaw > 2.0f * M_PI_F)
        {
            yaw -= 2.0f * M_PI_F;
        }
        if (yaw < -2.0f * M_PI_F)
        {
            yaw += 2.0f * M_PI_F;
        }
        x += cos(yaw) * vel * delta;
        y += sin(yaw) * vel * delta;

        // search closest point
        uint offset_w = 0;
        uint offset_c_w = 0;
        float min_dist = FLT_MAX;
        for (uint j = 0; j < nb_waypoints; j++)
        {
            float dx = x - waypoints[offset_w];
            float dy = y - waypoints[offset_w + 1];
            float dist = sqrt(dx * dx + dy * dy);

            if (dist < min_dist)
            {
                offset_c_w = offset_w;
                min_dist = dist;
            }
            offset_w += waypoint_dim;
        }
        float cos_yaw = cos(waypoints[offset_c_w + 2]);
        float sin_yaw = sin(waypoints[offset_c_w + 2]);
        min_dist = -sin_yaw * x + cos_yaw * y + sin_yaw * waypoints[offset_c_w] - cos_yaw * waypoints[offset_c_w + 1];

        // check velocity doesn't exceed limit
        float target_velocity = waypoints[offset_c_w + 3];
        if (vel > target_velocity)
        {
            // step back
            x -= cos(yaw) * vel * delta;
            y -= sin(yaw) * vel * delta;
            yaw -= (vel * tan(steering) / wheel_base) * delta;
            vel -= accel * delta;
            // recompute velocity and perturbation
            accel = fmax((target_velocity - vel) / delta, min_accel);
            n_accel = accel - inputs[offset_i + 1];
            // recompute new position
            vel += accel * delta;
            yaw += (vel * tan(steering) / wheel_base) * delta;
            x += cos(yaw) * vel * delta;
            y += sin(yaw) * vel * delta;
        }

        // waypoints alignment
        float diff_yaw = get_diff_angle(yaw, waypoints[offset_c_w + 2]);

        // target speed difference
        float diff_v = fabs(vel - target_velocity);

        // target distance
        offset_w = waypoint_dim * (nb_waypoints - 1);
        float dx = x - waypoints[offset_w];
        float dy = y - waypoints[offset_w + 1];
        float target_dist = sqrt(dx * dx + dy * dy);

        // collision checker
        float min_obj_d = get_obstacle_dist(rss_reaction_time, stop_velocity,
                                            vehicle_x, vehicle_y, vehicle_vel,
                                            x, y, yaw, vel, accel,
                                            waypoints, waypoints_size, waypoint_dim, offset_c_w,
                                            objects, objects_size, object_dim);

        // safe distance from obstacle
        float safe_dist = 0.0f;
        // if ((min_obj_d != -1.0f) && (accel != min_accel)) {
        //     float v_r = vel;
        //     float v_f = 0.0f;
        //     float target_safe_dist = rss_min_dist + (v_r * rss_reaction_time)
        // 	                         + (0.5f * rss_max_accel * rss_reaction_time * rss_reaction_time)
        // 			                 + (pown(v_r + rss_reaction_time * rss_max_accel, 2) / (2 * rss_min_brake))
        // 			                 - ((v_f * v_f) / (2 * rss_max_brake));
        //     safe_dist = fmax((target_safe_dist - min_obj_d), 0.0f);
        // }

        // check divergence
        float diverge = 0.0f;
        // if (min_dist >= 0.0f) {
        //     diverge = min_dist / (diverge_left + 0.01f);
        // }
        // if (min_dist < 0.0f) {
        //     diverge = -min_dist / (diverge_right + 0.01f);
        // }
        // diverge = (v_min_dist < diverge_threshold) * diverge;

        // compute cost
        float c_min_dist = 70.0f * min_dist * min_dist;
        float c_target = 7.0f * (target_dist > prev_target_dist);
        float c_diff_yaw = 120.0f * diff_yaw * diff_yaw;
        float c_diff_v = 5.0f * diff_v * diff_v;
        float c_safe_dist = 25.0f * safe_dist * safe_dist;
        cost += c_min_dist + c_target + c_diff_yaw + c_diff_v + c_safe_dist;

        // if ((get_global_id(0) % 64) == 0) {
        //     printf("%u %f %f %f %f %f\n", i, c_min_dist, c_target, c_diff_yaw, c_diff_v, c_safe_dist);
        // }

        // iteration updates
        noise[offset_n] = n_steering_rate;
        noise[offset_n + 1] = n_accel;
        offset_n += K * input_dim;
        offset_i += input_dim;
        prev_target_dist = target_dist;
    }

    // update cost and seed
    costs[global_id] = cost;
    seed[global_id] = _seed;
}