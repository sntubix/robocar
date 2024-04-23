/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef CYCLE_SERVICE_H
#define CYCLE_SERVICE_H

#include <atomic>
#include <thread>
#include <unistd.h>

#include "cycle/nodes/module.h"
#include "cycle/utils/time.h"

namespace cycle {
	class Service : public Module {
	public:
		Service(const Params& params)
			: Module(params), _period(params.get("period").to_int()) {}

		virtual ~Service() = default;

		virtual void serve() = 0;

		void start() {
			this->stop();
			auto lock = std::unique_lock<std::mutex>(_mtx);
			_serve_thread.reset(new std::thread(
				[&]() {
					_run = true;
					while (_run) {
						auto start = cycle::Time::now().ms();
						serve();
						auto serve_time = cycle::Time::now().ms() - start;
						if (_period > serve_time) {
							std::this_thread::sleep_for(std::chrono::milliseconds(_period - serve_time));
						}
					}
				})
			);
		}

		void stop() {
			auto lock = std::unique_lock<std::mutex>(_mtx);
			_run = false;
			if (_serve_thread) {
				if (_serve_thread->joinable()) {
					_serve_thread->join();
				}
			}
		}

		bool is_running() {
			auto lock = std::unique_lock<std::mutex>(_mtx);
			if (_serve_thread) {
				return _serve_thread->joinable();
			}
			return false;
		}

		uint64_t period() {
			return _period;
		}

	private:
		std::mutex _mtx;
		std::unique_ptr<std::thread> _serve_thread;
		std::atomic<bool> _run = false;
		const uint64_t _period = 0;
	};
}

#endif //CYCLE_SERVICE_H