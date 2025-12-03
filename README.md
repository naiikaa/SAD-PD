# Intelligent Robots Lab 2025 – Intelligent Embedded Systems
Intelligent Robots Lab at the university of Kassel. Main focus is to work on [CARLA](https://carla.readthedocs.io/en/latest/) and [ROS2](https://docs.ros.org/en/foxy/index.html). Building the bridge between those two technologies and creating a multi sensor vehicle dataset. 
This repository documents the practical work carried out in the Intelligent Robots Lab at the University of Kassel.  
The lab focuses on the integration of **CARLA** (open-source autonomous driving simulator) and **ROS 2** (Robot Operating System 2) to create a reproducible environment for multi-sensor vehicle simulation and dataset generation.  
The materials provided here summarize essential setup procedures, system interactions, and mapping resources used in this project.

---

## Documentation and Resources

- **[Installation Guide (LaTeX/PDF)](Installation_Guide_Carla_ROS.pdf)**  
  A detailed guide for installing and configuring CARLA 0.9.15 together with ROS 2 Humble on Ubuntu 22.04, including environment setup and troubleshooting.

- **[Clients and Agents in CARLA Simulator (PDF)](Client_and_Agents_in_CARLA_Simulator.pdf)**  
  Describes the Client–World and Agent architecture in CARLA, explaining how control logic, weather, and sensor data interact within the simulator.

- **[Open Street Map Integration for CARLA Simulation](kasselMap/readme.md)**  
  Notes and conversion workflow for importing OSM maps (e.g., Kassel region) into CARLA to enable scenario-specific experiments and synthetic dataset generation.

---

### Purpose
These documents collectively provide the foundation for building a **CARLA–ROS 2 bridge** and generating reproducible multi-sensor autonomous-driving datasets within the Intelligent Embedded Systems group.
*The whole workflow based on official documents which are mentioned before.

## Package Demo
A short video that shows you how to generate one batch of synthetic point cloud data, post-process that data and visualize it.
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/viNa4aCaWtE/0.jpg)](https://www.youtube.com/watch?v=viNa4aCaWtE)
