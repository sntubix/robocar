/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef CYCLE_BROKER_H
#define CYCLE_BROKER_H

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <memory>

#include "cycle/messaging/message.h"
#include "cycle/messaging/thread_pool.hpp"

namespace cycle {
	class VSubscriber {
	public:
		VSubscriber(uint32_t q_size)
			: _q_size(q_size) {}

		virtual void callback() = 0;

		void add_msg(const vmsg_t& msg) {
			std::unique_lock<std::mutex> lock(_mtx_q);
			if (_queue.size() == _q_size)
				_queue.pop_front();
			_queue.emplace_back(msg);
		}

		void clear_queue() {
			std::unique_lock<std::mutex> lock(_mtx_q);
			_queue.clear();
		}

		inline bool probe() {
			return !_queue.empty();
		}

		bool get_flag() {
			std::unique_lock<std::mutex> lock(_mtx_f);
			if (_flag == 1) {
				_flag = 0;
				return true;
			}
			return false;
		}

	protected:
		const uint32_t _q_size;
		std::mutex _mtx_q;
		std::deque<vmsg_t> _queue;
		std::mutex _mtx_f;
		int _flag = 1;
	};
	using vsub_t = std::shared_ptr<VSubscriber>;

	class WorkerPool {
    public:
        WorkerPool(int n) 
            : _wflags(n, 0), _pool(n) {}

		bool submit(vsub_t sub) {
			for(int i=0; i < _pool.get_thread_count(); i++) {
				if(_wflags[i] == 0) {
					_wflags[i] = 1;
					_pool.push_task(WorkerPool::task, sub, _wflags.data(), i);
					return true;
				}
			}
			return false;
		}

		void stop() {
			_pool.paused = true;
			_pool.wait_for_tasks();
		}

    private:
		std::vector<int> _wflags;
        thread_pool _pool;

		static void task(vsub_t sub, int* flags, int id) {
			sub->callback();
			flags[id] = 0;
		}
    };

	class Broker {
	public:
		static Broker& get() {
			static Broker _b;
			return _b;
		}

		Broker(const Broker&) = delete;
        void operator=(const Broker&) = delete;

		void subscribe(const std::string& topic, const vsub_t& sub) {
			std::unique_lock<std::mutex> lock(_m_s);
			_subscriptions[topic].push_back(sub);
			// notify changes to subscriptions
			_subs_flag = true;
		}

		void unsubscribe(const std::string& topic, const vsub_t& sub) {
			// remove from subscriptions
			std::unique_lock<std::mutex> lock(_m_s);
			auto& subs = _subscriptions[topic];
			for (auto s = subs.cbegin(); s < subs.cend(); ++s) {
				if (s->get() == sub.get())
					subs.erase(s);
			}
			// get flag to prevent further callback execs
			while (!sub->get_flag());
			// clear queue
			sub->clear_queue();
			// notify changes to subscriptions
			_subs_flag = true;
		}

		void publish(const vmsg_t& msg) {
			// get subscribers of the topic
			std::unique_lock<std::mutex> lock(_m_s);
			auto subs = _subscriptions[msg->topic];
			lock.unlock();
			// enqueue messages and callbacks
			for(auto& sub : subs) {
				sub->add_msg(msg);
			}
			// notify the broker
			this->notify();
		}

		void run() {
			this->stop();

			// init worker pool
			auto hc = std::thread::hardware_concurrency();
			if(hc > 1) {
				hc -= 1;
			}
			_wpool.reset(new WorkerPool(hc));

			// run broker main thread
			_bthread.reset(new std::thread([&]() {
				std::unordered_map<std::string, std::vector<vsub_t>> subscriptions;
				_run = true;
				while (_run) {
					{
						std::unique_lock<std::mutex> lock(this->_m_cv);
						if (!_msg_av) {
							_cv.wait(lock, [&]() {
								return _msg_av || !_run;
							});
						}
						_msg_av = false;
					}
					while (_run) {
						if (_subs_flag) {
							_m_s.lock();
							subscriptions = _subscriptions;
							_subs_flag = false;
							_m_s.unlock();
						}

						bool proceed = LOW_LATENCY;
						for (const auto& topic : subscriptions) {
							for (const auto& sub : topic.second) {
								if (sub->probe()) {
									proceed = true;
									if (sub->get_flag()) {
										while(!_wpool->submit(sub));
									}
								}
							}
						}
						if (!proceed)
							break;
					}
				}
			}));
		}

		void stop() {
			_run = false;
			this->notify();
			if (_bthread) {
				if (_bthread->joinable()) {
					_bthread->join();
				}
			}
			if (_wpool) {
				_wpool->stop();
			}
		}

	private:
		Broker() = default;
		virtual ~Broker() = default;

		std::mutex _m_s;
		bool _subs_flag = false;
		std::unordered_map<std::string, std::vector<vsub_t>> _subscriptions;
		const bool LOW_LATENCY = false;
		std::mutex _m_cv;
		std::condition_variable _cv;
		bool _msg_av = false;
		std::atomic<bool> _run = false;
		std::unique_ptr<std::thread> _bthread;
		std::unique_ptr<WorkerPool> _wpool;

		void notify() {
			_m_cv.lock();
			_msg_av = true;
			_m_cv.unlock();
			_cv.notify_all();
		}
	};

	template<typename T>
	class Publisher {
	public:
		Publisher(std::string topic)
			: _topic(topic) {}

		inline void publish(const T& msg) {
			auto msg_ptr = std::make_shared<const T>(msg);
			Broker::get().publish(std::make_shared<Message<T>>(_topic, msg_ptr));
		}

	private:
		const std::string _topic;
	};

	template<typename T>
	using handler_t = std::function<void(const std::shared_ptr<const T>&)>;

	template<typename T>
	class Subscriber : public VSubscriber {
	public:
		Subscriber(handler_t<T> callback, uint32_t q_size)
			: VSubscriber(q_size), _callback(callback) {}

		void callback() override {
			std::unique_lock<std::mutex> lock(_mtx_q);
			if(!_queue.empty()) {
				std::shared_ptr<Message<T>> msg_ptr = std::static_pointer_cast<Message<T>>(_queue.front());
				_queue.pop_front();
				lock.unlock();

				if (msg_ptr) {
					if (typeid(T).name() == msg_ptr->get_type())
						_callback(msg_ptr->get());
					else
						std::cout << "message type mismatch on topic: " << msg_ptr->topic << std::endl;
				}
				else
					std::cout << "message pointer cast failed on topic: " << msg_ptr->topic << std::endl;
			}
			_flag = 1;
		}

	private:
		handler_t<T> _callback;
	};

	template<typename T>
	std::shared_ptr<Publisher<T>> create_publisher(const std::string& topic) {
		return std::make_shared<Publisher<T>>(topic);
	}

	template<typename T>
	std::shared_ptr<Subscriber<T>> create_subscriber(const std::string& topic, 
													 handler_t<T> callback,
													 uint32_t q_size) {
		auto sub = std::make_shared<Subscriber<T>>(callback, q_size);
		Broker::get().subscribe(topic, sub);
		return sub;
	}

	#define BIND(F) std::bind(F, this, std::placeholders::_1)
}

#endif //CYCLE_BROKER_H
