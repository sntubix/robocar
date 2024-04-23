# RoboCar
RoboCar is a modular, low footprint and easy to deploy autonomous driving software based on ROS2.<br>

<img src="./robocar_viz.png" width="520"/>

### Software and Hardware Overview
<img src="./robocar_overview.svg" width="750"/>

## Get Started

### Prerequisites
The provided Dockerfile can be used to get a development environment easily.<br>
Use `docker build -f docker/Dockerfile -t ubix/robocar .` and then run with `bash docker/run.sh`.<br>
Please note that some specific adjustments might be needed for deployment in a vehicle.

### Build
* Debug : `bash scripts/build.sh debug`.<br>
* Release : `bash scripts/build.sh release`.

### Run
First, source using `source install/setup.sh`.<br>
RoboCar can be run using ROS2 launch : `ros2 launch startup robocar.launch`.

## License
See [NOTICE](./NOTICE.txt) and [LICENSE](./LICENSE.txt).