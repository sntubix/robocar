/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
*/

#ifndef CYCLE_MESSAGE_H
#define CYCLE_MESSAGE_H

#include <memory>
#include <string>

namespace cycle {
	class VMessage {
	public:
		explicit VMessage(std::string topic) 
			: topic(topic) {}

		const std::string topic;
	};
	using vmsg_t = std::shared_ptr<VMessage>;

	template<typename T>
	class Message : public VMessage {
	public:
		explicit Message(std::string topic, std::shared_ptr<const T> msg) 
			: VMessage(topic), _type(typeid(T).name()), _msg(msg) {}

		inline std::string const& get_type() const { return _type; }
		inline std::shared_ptr<const T> const& get() const { return _msg; }

	private:
		const std::string _type;
		const std::shared_ptr<const T> _msg;
	};

	template<typename T>
	using msg_t = const std::shared_ptr<const T>&;
}

#endif //CYCLE_MESSAGE_H
