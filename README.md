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
