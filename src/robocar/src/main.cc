/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "robocar/core/robocar.h"
#include "robocar/drivers/actuator/actuator_component.h"
#include "robocar/drivers/camera/camera_component.h"
#include "robocar/drivers/gnss/gsof_component.h"
#include "robocar/drivers/joystick/joystick_component.h"
#include "robocar/drivers/lidar/ouster_component.h"
#include "robocar/tod/tod_component.h"
#include "robocar/vehicle/vehicle_component.h"
#include "robocar/control/control_component.h"
#include "robocar/localization/localization_component.h"
#include "robocar/map/map_component.h"
#include "robocar/map/mapping_component.h"
#include "robocar/perception/lidar_perception_component.h"
#include "robocar/planning/planning_component.h"
#include "robocar/hmi/hmi_component.h"
#include "robocar/logging/logging_component.h"
#include "robocar/simulation/sim_vehicle_component.h"
#include "robocar/simulation/sim_objects_component.h"

// TODO health status for each component

int main(int argc, char **argv)
{
	auto qt_app = QApplication(argc, argv);

	auto robocar = robocar::RoboCar();
	robocar.register_component<robocar::drivers::actuator::ActuatorComponent>("actuator_component");
	robocar.register_component<robocar::drivers::camera::CameraComponent>("camera_component");
	robocar.register_component<robocar::drivers::gnss::GsofComponent>("gsof_component");
	robocar.register_component<robocar::drivers::joystick::JoystickComponent>("joystick_component");
	robocar.register_component<robocar::drivers::lidar::OusterComponent>("ouster_component");
	robocar.register_component<robocar::tod::TODComponent>("tod_component");
	robocar.register_component<robocar::vehicle::VehicleComponent>("vehicle_component");
	robocar.register_component<robocar::control::ControlComponent>("control_component");
	robocar.register_component<robocar::localization::LocalizationComponent>("localization_component");
	robocar.register_component<robocar::map::MapComponent>("map_component");
	robocar.register_component<robocar::map::MappingComponent>("mapping_component");
	robocar.register_component<robocar::perception::LidarPerceptionComponent>("lidar_perception_component");
	robocar.register_component<robocar::planning::PlanningComponent>("planning_component");
	robocar.register_component<robocar::hmi::HMIComponent>("hmi_component", &qt_app);
	robocar.register_component<robocar::logging::LoggingComponent>("logging_component");
	robocar.register_component<robocar::simulation::SimVehicleComponent>("sim_vehicle_component");
	robocar.register_component<robocar::simulation::SimObjectsComponent>("sim_objects_component");

	robocar.init(argc, argv);
	robocar.spin();
	auto qt_res = qt_app.exec();
	robocar.wait_for_shutdown();
	return 0;
}