/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_SIMULATION_SIM_OBJECTS_COMPONENT_H
#define ROBOCAR_SIMULATION_SIM_OBJECTS_COMPONENT_H

#include "robocar/core/robocar.h"
#include "robocar/simulation/sim_objects_agent.h"

namespace robocar
{
    namespace simulation
    {

        class SimObjectsComponent : public robocar::Component
        {
        public:
            SimObjectsComponent(const robocar::Params &params);

            void serve() override;

        private:
            uint64_t _prev_time = 0;
            std::vector<std::shared_ptr<SimObjectAgent>> _agents;
            std::mutex _mtx;
            msg::Localization _loc;
            msg::Path _waypoints;

            // publisher
            rclcpp::Publisher<msg::Objects3d>::SharedPtr _pub_objects;
            // subscribers
            rclcpp::Subscription<msg::Localization>::SharedPtr _sub_loc;
            rclcpp::Subscription<msg::Path>::SharedPtr _sub_waypoints;

            void on_localization(const msg::Localization &loc);
            void on_waypoints(const msg::Path &waypoints);
        };

    } // namespace simulation
} // namespace robocar

#endif // ROBOCAR_SIMULATION_SIM_OBJECTS_COMPONENT_H