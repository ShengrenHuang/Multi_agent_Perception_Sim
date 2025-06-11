# Multi-Agent Perception Simulation

This project integrates **ROS 2 Jazzy** and the **Gazebo Harmonic** simulator to simulate and visualize multi-agent perception.

Based on the official template:  
👉 https://github.com/gazebosim/ros_gz_project_template

---

## 🖥️ Environment

- **Operating System**: Ubuntu 24.04
- **ROS 2**: Jazzy (C++ primary)
- **Gazebo**: Harmonic
---

## 📦 Installation

Clone the repository:

```bash
git clone https://github.com/ShengrenHuang/Multi_agent_Perception_Sim.git
cd Multi_agent_Perception_Sim
```
### ROS 2 Jazzy
Please refer to [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html).

### Gazebo Harmonic 
Please refer to [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/).

Recommend to use binary installation.

### Build
```bash
export GZ_VERSION=harmonic

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

## Quick start
```bash
ros2 launch fws_robot_sim fws_robot_spawn.launch.py 

## Ground robot control
ros2 topic pub -r 100.1 forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1000.0, 1000.0, 1000.0, 1000.0]}"

## Activate python virtual env
source ~/ros2_rl_env/bin/activate

```
