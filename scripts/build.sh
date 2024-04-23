#!/bin/bash

BUILD_TYPE=$1
source /opt/ros/humble/setup.bash

if [ "$BUILD_TYPE" = "debug" ]; then
	colcon build --cmake-args ' -Wno-dev' ' -DCMAKE_BUILD_TYPE=Debug'
elif [ "$BUILD_TYPE" = "release" ]; then
	colcon build --cmake-args ' -Wno-dev' ' -DCMAKE_BUILD_TYPE=Release'
else
	echo "invalid build type: '$BUILD_TYPE'"
fi
