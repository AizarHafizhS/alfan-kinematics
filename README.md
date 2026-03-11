# **alfan-kinematics**
Kinematics engine and locomotion framework for Alfan humanoid robot's bipedal walk.

## 🚀 Progress
- ✅ Simulation system active via Gazebo Fortress + ROS 2.
- ✅ IMU sensor physically simulated and integrated via `ros2_control`.
- 📌 **Personal reminder**: *You are currently working on integrating **any algorithm/method** for humanoid robot locomotion*

## 🛠️ Requirements
- Ubuntu 22.04
- [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)
- [Gazebo Fortress](https://gazebosim.org/docs/fortress/install/)
- [Git](https://git-scm.com/)  

## 🧠 Reasoning of the Stack
- **ROS 2**: ROS 1 is completely End-of-Life (EOL) and no longer maintained.
- **ROS 2 Humble**: Directly compatible with Ubuntu 22.04 and ensures compatibility with the latest DynamixelSDK (supported in late 2024/early 2025).
- **Gazebo Fortress**: The official recommended version from [Gazebo](https://gazebosim.org/docs/latest/getstarted/) to pair with ROS 2 Humble.  

## ⚠️ Migration Considerations
If you plan to migrate to another stack or physical hardware, consider these rules:
- Gazebo Fortress simulation is bridged via [`ign_ros2_control`](https://index.ros.org/p/ign_ros2_control/). Changing simulators (e.g., to physical hardware or to other Gazebo distro) means you **must** rewrite the hardware interface tags in the URDF/XACRO config.
- Validate hardware compatibility before physical deployment.
- Ensure hardware middleware/API availability (e.g. Dynamixel with DynamixelSDK).
- **PLEASE! FEEL FREE UNTUK MENCOBA BERBAGAI HAL BARU. JANGAN STUCK DI SINI YA!**  

## 🗑️ Deprecation List (Delete-Soon Packages)
- *NONE*
  
## ⚙️ Setup Instructions

1. **Create the workspace directory**
```bash
mkdir -p ~/alfan_ros2_ws/
cd ~/alfan_ros2_ws/
```

2. **Clone this repository**
``` bash
git clone -b devel https://github.com/AizarHafizhS/alfan-kinematics.git
```
3. **Build the packages**
``` bash
colcon build
source install/setup.bash
```
4. **Launch the simulation**  
    **Launch Gazebo Fortress + ROS2**
    ``` bash
    ros2 launch alfan_gazebo gazebo_sim.launch.py
    ```
    on other terminal:  
    ``` bash
    ros2 run alfan_walking walking_node
    ```
    *Now robot will start squatting*


