/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef LOGGING_H
#define LOGGING_H

#include <iostream>
#include <map>
#include <utility>
#include <mutex>

namespace cycle
{
	class Logger
	{
	public:
		Logger(Logger &other) = delete;
		void operator=(const Logger &) = delete;

		static Logger &instance()
		{
			static Logger logger;
			return logger;
		}

		static void register_prefix(void *context, std::string prefix)
		{
			Logger::instance()._register_prefix(context, prefix);
		}

		enum LogLevel
		{
			DEBUG,
			INFO,
			WARN,
			ERROR
		};

		void log(void *context, int level, std::string text)
		{
			std::unique_lock<std::mutex> lck(_mtx);
			auto prefix = _prefix_map.at(context);
			if (level == cycle::Logger::DEBUG)
				std::cout << "\x1b[34m" << "[DEBUG] " << "[" + prefix + "]: " << text << "\x1b[0m\n";
			else if (level == cycle::Logger::INFO)
				std::cout << "\x1b[37m" << "[INFO] " << "[" + prefix + "]: " << text << "\x1b[0m\n";
			else if (level == cycle::Logger::WARN)
				std::cout << "\x1b[33m" << "[WARN] " << "[" + prefix + "]: " << text << "\x1b[0m\n";
			else if (level == cycle::Logger::ERROR)
				std::cerr << "\x1b[31m" << "[ERROR] " << "[" + prefix + "]: " << text << "\x1b[0m\n";
			else
				throw std::runtime_error("invalid log level");
		}

	protected:
		Logger() = default;
		std::map<void *, std::string> _prefix_map;
		std::mutex _mtx;

		void _register_prefix(void *context, std::string prefix)
		{
			std::unique_lock<std::mutex> lck(_mtx);
			if (_prefix_map.count(context))
				throw std::runtime_error("context already registered");
			_prefix_map.insert(std::pair<void *, std::string>(context, prefix));
		}
	};

	inline void log(void *context, int level, std::string text)
	{
		Logger::instance().log(context, level, text);
	}

#define LOG_DEBUG(TEXT) cycle::log(this, cycle::Logger::DEBUG, TEXT)
#define LOG_INFO(TEXT) cycle::log(this, cycle::Logger::INFO, TEXT)
#define LOG_WARN(TEXT) cycle::log(this, cycle::Logger::WARN, TEXT)
#define LOG_ERROR(TEXT) cycle::log(this, cycle::Logger::ERROR, TEXT)
}

#endif // LOGGING_H