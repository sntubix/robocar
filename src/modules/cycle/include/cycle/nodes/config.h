/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef CYCLE_CONFIG_H
#define CYCLE_CONFIG_H

#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

#include <rapidjson/document.h>

namespace cycle
{
	class Param
	{
	public:
		Param(std::string const &name, int value)
			: _name(name), _type(typeid(value).name()), _v_int(value) {}

		Param(std::string const &name, double value)
			: _name(name), _type(typeid(value).name()), _v_double(value) {}

		Param(std::string const &name, bool value)
			: _name(name), _type(typeid(value).name()), _v_bool(value) {}

		Param(std::string const &name, std::string value)
			: _name(name), _type(typeid(value).name()), _v_str(value) {}

		const std::string &name() const
		{
			return _name;
		}

		int to_int() const
		{
			if (_type != typeid(int).name())
			{
				throw std::runtime_error(_name + " is not of type 'int'");
			}
			return _v_int;
		}

		double to_double() const
		{
			if (_type != typeid(double).name())
			{
				throw std::runtime_error(_name + " is not of type 'double'");
			}
			return _v_double;
		}

		bool to_bool() const
		{
			if (_type != typeid(bool).name())
			{
				throw std::runtime_error(_name + " is not of type 'bool'");
			}
			return _v_bool;
		}

		std::string to_string() const
		{
			if (_type != typeid(std::string).name())
			{
				throw std::runtime_error(_name + " is not of type 'string'");
			}
			return _v_str;
		}

	private:
		std::string _name;
		std::string _type;
		int _v_int = 0;
		double _v_double = 0.0;
		bool _v_bool = false;
		std::string _v_str = "";
	};

	class Params
	{
		const std::string _name;
		const std::map<std::string, Param> _params;
		inline static int _argc = 0;
		inline static char **_argv = nullptr;
		inline static std::map<std::string, Param> _g_params = {};

	public:
		Params(std::string name, std::map<std::string, Param> params)
			: _name(name), _params(params) {}

		std::string const &name() const
		{
			return _name;
		}

		static void set_argcv(int argc, char **argv)
		{
			_argc = argc;
			_argv = argv;
		}

		static int get_argc()
		{
			return _argc;
		}

		static char **get_argv()
		{
			return _argv;
		}

		static void clear_global()
		{
			_g_params = {};
		}

		static void add_global(Param param)
		{
			if (_g_params.count(param.name()) > 0)
			{
				throw std::runtime_error("duplicate global parameter '" + param.name() + "'");
			}
			else
			{
				_g_params.insert({param.name(), param});
			}
		}

		Param get(const std::string &param) const
		{
			if (_params.count(param) > 0)
			{
				return _params.at(param);
			}
			if (_g_params.count(param) > 0)
			{
				return _g_params.at(param);
			}
			throw std::runtime_error("undefined parameter '" + param + "' in '" + _name + "'");
		}
	};

	class Config
	{
	public:
		Config(int argc, char **argv, const std::string &filename)
		{
			Params::set_argcv(argc, argv);
			Params::clear_global();
			parse(filename);
		};

		const std::vector<std::string> &get_services() const
		{
			return _services;
		}

		const Params &get_params(const std::string &s_name) const
		{
			if (_s_params.count(s_name) < 1)
			{
				throw std::runtime_error("no parameters provided for service '" + s_name + "'");
			}
			return _s_params.at(s_name);
		}

	private:
		std::vector<std::string> _services;
		std::map<std::string, Params> _s_params;

		void parse_global(const rapidjson::Value &g)
		{
			for (rapidjson::Value::ConstMemberIterator m = g.MemberBegin(); m != g.MemberEnd(); ++m)
			{
				std::string p_name = m->name.GetString();
				if (m->value.IsInt())
				{
					Params::add_global(Param(p_name, m->value.GetInt()));
				}
				else if (m->value.IsDouble())
				{
					Params::add_global(Param(p_name, m->value.GetDouble()));
				}
				else if (m->value.IsBool())
				{
					Params::add_global(Param(p_name, m->value.GetBool()));
				}
				else if (m->value.IsString())
				{
					Params::add_global(Param(p_name, std::string(m->value.GetString())));
				}
				else
				{
					throw std::runtime_error("invalid global parameter '" + p_name + "'");
				}
			}
		}

		void parse_section(std::string s_name, const rapidjson::Value &s)
		{
			// check duplicate service parameters
			if (_s_params.count(s_name) > 0)
			{
				throw std::runtime_error("duplicate parameters for '" + s_name + "'");
			}

			// check service enabled/disabled
			if (s.HasMember("enable"))
			{
				if (!s.FindMember("enable")->value.GetBool())
				{
					return;
				}
			}

			// parse parameters
			std::map<std::string, Param> params;
			for (rapidjson::Value::ConstMemberIterator m = s.MemberBegin(); m != s.MemberEnd(); ++m)
			{
				std::string p_name = m->name.GetString();
				if (p_name == "enable")
				{
					continue;
				}
				if (m->value.IsInt())
				{
					params.insert({p_name, Param(p_name, m->value.GetInt())});
				}
				else if (m->value.IsDouble())
				{
					params.insert({p_name, Param(p_name, m->value.GetDouble())});
				}
				else if (m->value.IsBool())
				{
					params.insert({p_name, Param(p_name, m->value.GetBool())});
				}
				else if (m->value.IsString())
				{
					params.insert({p_name, Param(p_name, std::string(m->value.GetString()))});
				}
				else
				{
					throw std::runtime_error("invalid parameter '" + p_name + "'");
				}
			}
			_services.push_back(s_name);
			_s_params.insert({s_name, Params(s_name, params)});
		}

		void parse(const std::string &filename)
		{
			// open file
			std::ifstream fs(filename, std::ios::in);
			std::string json((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
			// check for "cycle" section
			rapidjson::Document d;
			d.Parse(json.c_str());
			if (!d.IsObject())
			{
				throw std::runtime_error("unable to parse '" + filename + "', path might be incorrect!");
			}
			if (!d.HasMember("cycle"))
			{
				throw std::runtime_error("expecting member 'cycle' in '" + filename + "'");
			}
			rapidjson::Value &c = d["cycle"];

			// modules loop
			for (rapidjson::Value::ConstMemberIterator m = c.MemberBegin();
				 m != c.MemberEnd(); ++m)
			{
				std::string const &m_name = m->name.GetString();
				if (!m->value.IsObject())
				{
					throw std::runtime_error("invalid module '" + m_name + "'");
				}

				// check global parameters
				if (m_name == "global")
				{
					parse_global(m->value);
					continue;
				}

				// check module enabled/disabled
				if (m->value.HasMember("enable"))
				{
					if (!m->value.FindMember("enable")->value.GetBool())
					{
						continue;
					}
				}

				// services loop
				for (rapidjson::Value::ConstMemberIterator s = m->value.MemberBegin();
					 s != m->value.MemberEnd(); ++s)
				{
					std::string const &s_name = s->name.GetString();
					if (!s->value.IsObject())
					{
						if (s_name == "enable")
						{
							continue;
						}
						else
						{
							throw std::runtime_error("invalid service '" + s_name + "'");
						}
					}
					else
					{
						parse_section(s_name, s->value);
					}
				}
			}
		}
	};
}

#endif // CYCLE_CONFIG_H
