# Synthetic Autonomous Driving Point-Cloud Data
This repo is a product of the Intelligent Robots Lab at the university of Kassel. Main focus is to work on [CARLA](https://carla.readthedocs.io/en/latest/) and [ROS2](https://docs.ros.org/en/foxy/index.html). Building the bridge between those two technologies and creating synthetic multi lidar sensor vehicle datasets. 
We introduce a pipeline that include three parts:

- Configuration and recording of your experiment
- Post-processing that includes coordinate translation from world and sensor points of view into the ego perspective
- 3D Vizualization of your point-cloud data

## Requierments
This repo uses [ros2bridge](https://github.com/ttgamage/carla-ros-bridge) for CARLA which comes with the following requirements
```
– Operating system: Ubuntu 22.04 LTS (native or via WSL 2),
– CARLA simulator: version 0.9.15,
– ScenarioRunner: version 0.9.15,
– ROS 2 distribution: Humble.
```
## Package Demo
A short video that shows you how to generate one batch of synthetic point cloud data, post-process that data and visualize it.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/viNa4aCaWtE/0.jpg)](https://www.youtube.com/watch?v=viNa4aCaWtE)
