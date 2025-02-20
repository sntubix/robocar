/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_CORE_ROBOCAR_H
#define ROBOCAR_CORE_ROBOCAR_H

#include <mutex>
#include <condition_variable>

#include <std_msgs/msg/string.hpp>

#include "robocar/core/messages.h"
#include "robocar/core/component/component.h"
#include "robocar/core/utils/utils.h"
#include "robocar/core/utils/time.h"

namespace robocar
{
    // status
    const int STATUS_OK = 0;
    const int STATUS_WARNING = 1;
    const int STATUS_ERROR = 2;
    const int STATUS_TIMEOUT = 3;
    const int STATUS_ENABLED = 4;
    const int STATUS_DISABLED = 5;
    // planner state
    const int STATE_STANDBY = 0;
    const int STATE_DRIVE = 1;
    const int STATE_KEEP_DIST = 2;
    // object type
    const int OBJ_NONE = 0;
    const int OBJ_OBSTACLE = 1;
    const int OBJ_VIRTUAL = 2;
    const int OBJ_TFL_NONE = 3;
    const int OBJ_TFL_GREEN = 4;
    const int OBJ_TFL_YELLOW = 5;
    const int OBJ_TFL_RED = 6;
    // actuation override mode
    const int ACT_OVERRIDE_NONE = 0;
    const int ACT_OVERRIDE_PARTIAL = 1;
    const int ACT_OVERRIDE_FULL = 2;
}

namespace robocar
{
    class RoboCar
    {
    public:
        RoboCar() = default;

        template <typename T>
        void register_component(const std::string name)
        {
            auto c_cons = std::function(
                [](const Params &params)
                { return std::shared_ptr<Component>{new T(params)}; });
            _c_cons_map.insert({name, c_cons});
        }

        template <typename T, typename N>
        void register_component(const std::string name, N arg)
        {
            auto c_cons = std::function(
                [arg](const Params &params)
                { return std::shared_ptr<Component>{new T(params, arg)}; });
            _c_cons_map.insert({name, c_cons});
        }

        void init(int argc, char **argv)
        {
            if (argc < 2)
            {
                throw std::invalid_argument("must provide a config file");
            }
            // if (argc > 2) {
            //     throw std::invalid_argument("must provide one argument");
            // }

            this->stop();
            rclcpp::init(argc, argv);
            // init robocar listener
            _node_listener = std::make_shared<rclcpp::Node>("robocar_core");
            _listener = _node_listener->create_subscription<std_msgs::msg::String>(
                "robocar/listener", 10, std::bind(&RoboCar::on_listener, this, std::placeholders::_1));
            // init executor
            _executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
            _executor->add_node(_node_listener);

            auto cfg = Config(argc, argv, argv[1]);
            for (auto name : cfg.get_components())
            {
                if ((_c_cons_map.count(name) == 1))
                {
                    auto component = _c_cons_map.at(name)(cfg.get_params(name));
                    _components[name] = component;
                    _executor->add_node(component->get_node_base_interface());
                }

                else
                {
                    throw std::runtime_error("unregistered component '" + name + "'");
                }
            }
        }

        void spin()
        {
            this->stop();

            // start components
            for (auto &component : _components)
            {
                component.second->start();
            }

            // start executor
            _spin_thread.reset(new std::thread([&]()
                                               { _executor->spin(); }));
        }

        void wait_for_shutdown()
        {
            std::unique_lock<std::mutex> lock(_m_cv);
            _cv.wait(lock, [&]()
                     { return _shutdown; });

            this->stop();
            rclcpp::shutdown();
            _shutdown = false;
        }

    private:
        std::map<std::string, std::function<std::shared_ptr<Component>(const Params &params)>> _c_cons_map;
        std::map<std::string, std::shared_ptr<Component>> _components;

        std::shared_ptr<rclcpp::Node> _node_listener;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _listener;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
        std::shared_ptr<std::thread> _spin_thread;

        std::mutex _m_cv;
        std::condition_variable _cv;
        bool _shutdown = false;

        void stop()
        {
            // stop executor
            if (_executor)
            {
                _executor->cancel();
            }
            if (_spin_thread)
            {
                if (_spin_thread->joinable())
                {
                    _spin_thread->join();
                }
            }

            // stop components
            for (auto &component : _components)
            {
                component.second->stop();
            }
        }

        void on_listener(const std_msgs::msg::String &msg)
        {
            if (msg.data == "shutdown")
            {
                _m_cv.lock();
                _shutdown = true;
                _m_cv.unlock();
                _cv.notify_one();
            }
        }
    };
}

#endif // ROBOCAR_CORE_ROBOCAR_H