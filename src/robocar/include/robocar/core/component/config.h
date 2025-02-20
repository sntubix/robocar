/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef ROBOCAR_CORE_COMPONENT_CONFIG_H
#define ROBOCAR_CORE_COMPONENT_CONFIG_H

#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

#include <rapidjson/document.h>

namespace robocar
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

		static Param get_global(const std::string &param)
		{
			if (_g_params.count(param) > 0)
			{
				return _g_params.at(param);
			}
			throw std::runtime_error("undefined global parameter '" + param + "'");
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

		const std::vector<std::string> &get_components() const
		{
			return _components;
		}

		const Params &get_params(const std::string &c_name) const
		{
			if (_c_params.count(c_name) < 1)
			{
				throw std::runtime_error("no parameters provided for component '" + c_name + "'");
			}
			return _c_params.at(c_name);
		}

	private:
		std::vector<std::string> _components;
		std::map<std::string, Params> _c_params;

		void parse_global(const rapidjson::Value &g)
		{
			for (rapidjson::Value::ConstMemberIterator m = g.MemberBegin(); m != g.MemberEnd(); ++m)
			{
				std::string const &p_name = m->name.GetString();
				if (p_name.empty())
				{
					throw std::runtime_error("invalid global parameter '" + p_name + "'");
				}

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

		void parse_component(std::string c_name, const rapidjson::Value &c)
		{
			// check duplicate component parameters
			if (_c_params.count(c_name) > 0)
			{
				throw std::runtime_error("duplicate parameters section for '" + c_name + "'");
			}

			// parse groups
			std::vector<std::string> groups;
			if (c.HasMember("groups"))
			{
				auto m_g = c.FindMember("groups");
				if (!m_g->value.IsArray())
				{
					throw std::runtime_error("invalid 'groups' parameter in '" + c_name + "'");
				}

				auto _groups = m_g->value.GetArray();
				for (auto group = _groups.Begin(); group != _groups.End(); ++group)
				{
					if (group->IsString())
					{
						auto group_str = std::string(group->GetString());
						if (group_str.empty())
						{
							continue;
						}
						groups.push_back(group_str);
					}
					else
					{
						throw std::runtime_error("invalid 'groups' parameter in '" + c_name + "'");
					}
				}
			}
			else
			{
				throw std::runtime_error("expecting 'groups' parameter in '" + c_name + "'");
			}

			// parse parameters
			std::map<std::string, Param> params;
			for (rapidjson::Value::ConstMemberIterator m = c.MemberBegin(); m != c.MemberEnd(); ++m)
			{
				std::string const &p_name = m->name.GetString();
				if (p_name.empty())
				{
					throw std::runtime_error("invalid parameter '" + p_name + "' in '" + c_name + "'");
				}
				if (p_name == "groups")
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
					throw std::runtime_error("invalid parameter '" + p_name + "' in '" + c_name + "'");
				}
			}

			auto launch_group = Params::get_global("launch_group").to_string();
			for (auto &group : groups)
			{
				if (group == launch_group)
				{
					_components.push_back(c_name);
					_c_params.insert({c_name, Params(c_name, params)});
					break;
				}
			}
		}

		void parse(const std::string &filename)
		{
			// open file
			std::ifstream fs(filename, std::ios::in);
			std::string json((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
			// check for "robocar" section
			rapidjson::Document doc;
			doc.Parse(json.c_str());
			if (!doc.IsObject())
			{
				throw std::runtime_error("unable to parse '" + filename + "', path might be incorrect!");
			}
			if (!doc.HasMember("robocar"))
			{
				throw std::runtime_error("expecting root object 'robocar' in '" + filename + "'");
			}
			rapidjson::Value &robocar = doc["robocar"];

			// parse global parameters first
			if (robocar.HasMember("global"))
			{
				parse_global(robocar.FindMember("global")->value);
			}

			for (rapidjson::Value::ConstMemberIterator c = robocar.MemberBegin();
				 c != robocar.MemberEnd(); ++c)
			{
				std::string const &c_name = c->name.GetString();
				if (c_name.empty())
				{
					throw std::runtime_error("invalid component '" + c_name + "'");
				}
				if (c_name == "global")
				{
					continue;
				}

				// parse component
				if (!c->value.IsObject())
				{
					throw std::runtime_error("invalid component '" + c_name + "'");
				}
				parse_component(c_name, c->value);
			}
		}
	};
}

#endif // ROBOCAR_CORE_COMPONENT_CONFIG_H
