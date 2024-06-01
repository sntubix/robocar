/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef CYCLE_CYCLE_H
#define CYCLE_CYCLE_H

#include <mutex>
#include <condition_variable>

#include <std_msgs/msg/string.hpp>

#include "cycle/nodes/service.h"
#include "cycle/utils/utils.h"
#include "cycle/utils/time.h"
#include "cycle/utils/logging.h"

namespace cycle
{
    class Cycle
    {
    public:
        Cycle() = default;

        template <typename T>
        void register_module(const std::string name)
        {
            auto m_cons = std::function(
                [](const Params &params)
                { return std::shared_ptr<Module>{new T(params)}; });
            _m_cons_map.insert({name, m_cons});
        }

        template <typename T, typename N>
        void register_module(const std::string name, N arg)
        {
            auto m_cons = std::function(
                [arg](const Params &params)
                { return std::shared_ptr<Module>{new T(params, arg)}; });
            _m_cons_map.insert({name, m_cons});
        }

        template <typename T>
        void register_service(const std::string name)
        {
            auto s_cons = std::function(
                [](const Params &params)
                { return std::shared_ptr<Service>{new T(params)}; });
            _s_cons_map.insert({name, s_cons});
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
            // init cycle listener
            _node_listener = std::make_shared<rclcpp::Node>("cycle");
            _listener = _node_listener->create_subscription<std_msgs::msg::String>(
                "cycle/listener", 10, std::bind(&Cycle::on_listener, this, std::placeholders::_1));
            // init executor
            _executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
            _executor->add_node(_node_listener);

            auto cfg = Config(argc, argv, argv[1]);
            for (auto name : cfg.get_services())
            {
                if ((_m_cons_map.count(name) == 1))
                {
                    auto module = _m_cons_map.at(name)(cfg.get_params(name));
                    _modules[name] = module;
                    _executor->add_node(module);
                }

                else if ((_s_cons_map.count(name) == 1))
                {
                    auto service = _s_cons_map.at(name)(cfg.get_params(name));
                    _services[name] = service;
                    _executor->add_node(service);
                }

                else
                {
                    throw std::runtime_error("unregistered module/service '" + name + "'");
                }
            }
        }

        void spin()
        {
            this->stop();

            // start services
            for (auto &service : _services)
            {
                service.second->start();
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
        std::map<std::string, std::function<std::shared_ptr<Module>(const Params &params)>> _m_cons_map;
        std::map<std::string, std::function<std::shared_ptr<Service>(const Params &params)>> _s_cons_map;
        std::map<std::string, std::shared_ptr<Module>> _modules;
        std::map<std::string, std::shared_ptr<Service>> _services;

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

            // stop services
            for (auto &service : _services)
            {
                service.second->stop();
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

#endif // CYCLE_CYCLE_H