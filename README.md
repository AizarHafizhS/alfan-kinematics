# **alfan-kinematics**
Kinematics for humanoid robot's biped walk.  

For now, only `alfan_msgs`, `alfan_walking`, `alfan_webots_simulator` package is working fine. So, only simulator is working and still progressing...  

## Requirements
- Webots
- ROS2 Humble
- Ubuntu 22.04
- Git (of course lah ya)

## To run simulator
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
    colcon build --packages-select alfan_msgs alfan_walking alfan_webots_sim
    source install/setup.bash
```
4. Launch the simulation
``` bash
    ros2 launch alfan_webots_sim simulation.launch
```