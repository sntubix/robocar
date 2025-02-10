/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#include "cycle/cycle.h"

#include "drivers/actuator/actuator_component.h"
#include "drivers/alexa/alexa_component.h"
#include "drivers/camera/camera_component.h"
#include "drivers/gnss/gsof_component.h"
#include "drivers/joystick/joystick_component.h"
#include "drivers/lidar/ouster_component.h"
#include "drivers/tod/tod_component.h"
#include "vehicle/vehicle_component.h"
#include "control/control_component.h"
#include "localization/localization_component.h"
#include "map/map_component.h"
#include "map/mapping_component.h"
#include "perception/lidar_component.h"
#include "planning/planning_component.h"
#include "visualization/visualization_component.h"
#include "visualization/dashboard_component.h"
#include "logging/logging_component.h"
#include "simulation/sim_vehicle_component.h"
#include "simulation/sim_objects_component.h"

int main(int argc, char **argv)
{
	auto qt_app = QApplication(argc, argv);

	auto cycle = cycle::Cycle();
	cycle.register_service<robocar::drivers::actuator::ActuatorComponent>("actuator_component");
	cycle.register_service<robocar::drivers::alexa::AlexaComponent>("alexa_component");
	cycle.register_service<robocar::drivers::camera::CameraComponent>("camera_component");
	cycle.register_service<robocar::drivers::gnss::GsofComponent>("gsof_component");
	cycle.register_service<robocar::drivers::joystick::JoystickComponent>("joystick_component");
	cycle.register_service<robocar::drivers::lidar::OusterComponent>("ouster_component");
	cycle.register_service<robocar::drivers::tod::TODComponent>("tod_component");
	cycle.register_service<robocar::vehicle::VehicleComponent>("vehicle_component");
	cycle.register_service<robocar::control::ControlComponent>("control_component");
	cycle.register_module<robocar::localization::LocalizationComponent>("localization_component");
	cycle.register_service<robocar::map::MapComponent>("map_component");
	cycle.register_service<robocar::map::MappingComponent>("mapping_component");
	cycle.register_service<robocar::perception::lidar::LidarComponent>("lidar_component");
	cycle.register_service<robocar::planning::PlanningComponent>("planning_component");
	cycle.register_service<robocar::visualization::VisualizationComponent>("visualization_component");
	cycle.register_service<robocar::visualization::DashboardComponent>("dashboard_component", &qt_app);
	cycle.register_service<robocar::logging::LoggingComponent>("logging_component");
	cycle.register_service<robocar::simulation::SimVehicleComponent>("sim_vehicle_component");
	cycle.register_service<robocar::simulation::SimObjectsComponent>("sim_objects_component");

	cycle.init(argc, argv);
	cycle.spin();
	auto qt_res = qt_app.exec();
	cycle.wait_for_shutdown();
	return 0;
}