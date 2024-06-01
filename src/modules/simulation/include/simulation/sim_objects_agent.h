/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef SIMULATION_SIM_OBJECTS_AGENT_H
#define SIMULATION_SIM_OBJECTS_AGENT_H

#include "cycle/utils/time.h"
#include "common/common.h"

#include <Eigen/Core>

namespace robocar
{
	namespace simulation
	{

		class SimObjectAgent
		{
		public:
			SimObjectAgent(double dim_x, double dim_y, double dim_z,
						   double gc_x, double gc_y);

			virtual msg::Object3d update(msg::Localization loc, msg::Path waypoints, double dt) = 0;

		protected:
			std::vector<Eigen::Vector3d> _bbox;
			msg::Object3d _object;

			void update_bbox();
		};

		class SimObjectFollow : public SimObjectAgent
		{
		public:
			SimObjectFollow(double dim_x, double dim_y, double dim_z,
							double gc_x, double gc_y, double vel, double accel,
							double max_dist, uint64_t drive_period, uint64_t brake_period)
				: SimObjectAgent(dim_x, dim_y, dim_z, gc_x, gc_y),
				  _vel(vel), _accel(accel), _max_dist(max_dist),
				  _state_0_period(drive_period), _state_1_period(brake_period) {}

			msg::Object3d update(msg::Localization loc, msg::Path waypoints, double dt) override;

		private:
			int _state = 1;
			double _state_dt = 0.0;
			double _state_0_period = 10.0;
			double _state_1_period = 25.0;

			double _vel = 7.0;
			double _accel = 2.0;
			double _max_dist = 100.0;
		};

		class SimObjectClose : public SimObjectAgent
		{
		public:
			SimObjectClose(double dim_x, double dim_y, double dim_z,
						   double gc_x, double gc_y, double offset_x, double offset_y,
						   double vel, double accel, double max_dist,
						   uint64_t wait_period, uint64_t drive_period)
				: SimObjectAgent(dim_x, dim_y, dim_z, gc_x, gc_y),
				  _offset_x(offset_x), _offset_y(offset_y),
				  _vel(vel), _accel(accel), _max_dist(max_dist),
				  _state_0_period(wait_period), _state_1_period(drive_period) {}

			msg::Object3d update(msg::Localization loc, msg::Path waypoints, double dt) override;

		private:
			int _state = 0;
			double _state_dt = 0.0;
			double _state_0_period = 10.0;
			double _state_1_period = 10.0;

			double _offset_x = 10.0;
			double _offset_y = 4.0;
			double _vel = 7.0;
			double _accel = 2.0;
			double _max_dist = 100.0;
		};

		class SimObjectStatic : public SimObjectAgent
		{
		public:
			SimObjectStatic(double dim_x, double dim_y, double dim_z,
							double gc_x, double gc_y, double max_dist);

			msg::Object3d update(msg::Localization loc, msg::Path waypoints, double dt) override;

		private:
			double _offset_x = 0.0;
			double _offset_y = 0.0;
			double _max_dist = 100.0;
		};

	} // namespace simulation
} // namespace robocar

#endif // SIMULATION_SIM_OBJECTS_AGENT_H