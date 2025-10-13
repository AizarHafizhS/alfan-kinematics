# **alfan-kinematics**
Kinematics for humanoid robot's biped walk.  

For now, only Webots simulation demo and simple MoveIt demo is working and still progressing... 🔨 
- 📌 **Personal reminder**: *You are currently working on inverse kinematics implementation using MoveIt API and your own robot config (`alfan_moveit_config`).*   

## Requirements
- Ubuntu 22.04
- [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)
- [Webots R2025a](https://cyberbotics.com/doc/guide/installing-webots)
- [MoveIt2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
- Git (of course lah ya)

## Setup this repo
1. Create a workspace
``` bash
    mkdir -p ~/alfan_ros2_ws/
    cd ~/alfan_ros2_ws
```
2. Clone this repo
``` bash
    git clone https://github.com/AizarHafizhS/alfan-kinematics.git
```
3. Build it using colcon (only several package)
``` bash
    colcon build --packages-select alfan_msgs alfan_walking alfan_webots_sim alfan_robot_description alfan_moveit_config
    source install/setup.bash
```
4.1. Launch Webots simulation (*only moving elbow in sync hhe..*)
``` bash
    ros2 launch alfan_webots_sim simulation.launch
```
4.2. Launch MoveIt demo
``` bash
    ros2 launch alfan_moveit_config demo.launch.py
```
