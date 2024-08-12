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

### Configuration
Internal RoboCar nodes (components) are configured using a custom [`robocar.json`](./src/modules/startup/config/robocar.json) file. Below is an example configuration:

```json
{
  "cycle": {
    "global": {
      "param_a": 50,
      "param_b": "b"
    },
    "module_a": {
      "enable": true,
      "component_a": {
        "enable": true,
        "param_a": 10,
        "param_c": 0.25,
        "other_params": "..."
      }
    },
    "module_b": {
      "enable": false,
      "component_b": {
        "other_params": "..."
      },
      "component_c": {
        "other_params": "..."
      }
    }
  }
}
```

The configuration file is structured around modules containing components (ROS2 nodes). A component section contains parameters and an optional "enable" option. The component section name must match a registered RoboCar component, see [`src/modules/startup/src/main.cc`](./src/modules/startup/src/main.cc) for how to register components. A disabled module will be skipped during initialization, thus disabling all contained components irrespective of their individual settings. Global parameters are defined in the `"global"` section and can be overriden in a component section, see `"param_a"` in the above example.

You should rebuild for changes in the configuration file to take effect or change `args="$(find-pkg-share startup)/config/robocar.json"` in [`robocar.launch`](./src/modules/startup/launch/robocar.launch) to a custom path.

External ROS2 nodes should be added and configured using [`robocar.launch`](./src/modules/startup/launch/robocar.launch).

### Build
* Debug : `bash scripts/build.sh debug`.<br>
* Release : `bash scripts/build.sh release`.

### Run
First, source using `source install/setup.sh`.<br>
RoboCar can be run using ROS2 launch : `ros2 launch startup robocar.launch`.

## Citation
If you find RoboCar useful or relevant for your research, please cite our paper:
```bibtex
@article{testouri2024robocar,
  title={RoboCar: A Rapidly Deployable Open-Source Platform for Autonomous Driving Research},
  author={Mehdi Testouri and Gamal Elghazaly and Raphael Frank},
  year={2024},
  eprint={2405.03572},
  archivePrefix={arXiv},
  primaryClass={cs.RO}
}
```

## License
See [NOTICE](./NOTICE.txt) and [LICENSE](./LICENSE.txt).
