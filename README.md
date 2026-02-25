# **alfan-kinematics**
Kinematics engine and locomotion framework for Alfan humanoid robot's bipedal walk.

## 🚀 Progress
- ✅ Simulation system active via Gazebo Fortress + MoveIt 2 + ROS 2.
- ✅ IMU sensor physically simulated and integrated via `ros2_control`.
- 📌 **Personal reminder**: *You are currently working on integrating the `bitbots_quintic_walk` locomotion algorithm from the Bit-Bots team as the primary walking engine.*

## 🛠️ Requirements
- Ubuntu 22.04
- [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)
- [Gazebo Fortress](https://gazebosim.org/docs/fortress/install/)
- [MoveIt 2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
- [Git](https://git-scm.com/)
- *(Deprecated)* [Webots R2025a](https://cyberbotics.com/doc/guide/installing-webots) - if you still want to run the legacy simulation.  

## 🧠 Reasoning of the Stack
- **ROS 2**: ROS 1 is completely End-of-Life (EOL) and no longer maintained.
- **ROS 2 Humble**: Directly compatible with Ubuntu 22.04 and ensures compatibility with the latest DynamixelSDK (supported in late 2024/early 2025).
- **Gazebo Fortress**: The official recommended version from [Gazebo](https://gazebosim.org/docs/latest/getstarted/) to pair with ROS 2 Humble.
- **MoveIt 2**: The industry standard for kinematics and trajectory planning in ROS 2.  

## ⚠️ Migration Considerations
If you plan to migrate to another stack or physical hardware, consider these rules:
- Gazebo Fortress simulation is bridged via [`ign_ros2_control`](https://index.ros.org/p/ign_ros2_control/). Changing simulators (e.g., to physical hardware or to other Gazebo distro) means you **must** rewrite the hardware interface tags in the URDF/XACRO config.
- Validate hardware compatibility before physical deployment.
- Ensure hardware middleware/API availability (e.g. Dynamixel with DynamixelSDK).
- **PLEASE! FEEL FREE UNTUK MENCOBA BERBAGAI HAL BARU. JANGAN STUCK DI SINI YA!**  

## 🗑️ Deprecation List (Delete-Soon Packages)
- `alfan_kinematics` (Obsolete, completely replaced by MoveIt 2).
- `robot_controller` (Legacy trial-and-error package).
- `alfan_webots_sim` (Gazebo Fortress architecture is vastly superior hhe).  
  
## ⚙️ Setup Instructions

1. **Create the workspace directory**
```bash
mkdir -p ~/alfan_ros2_ws/
cd ~/alfan_ros2_ws/
```

2. **Clone this repository**
``` bash
git clone https://github.com/AizarHafizhS/alfan-kinematics.git
```
3. **Build the packages**
``` bash
colcon build --packages-select alfan_msgs alfan_walking alfan_webots_sim alfan_robot_description alfan_moveit_config
source install/setup.bash
```
4. **Launch the simulation**  
   Choose one of the environments below depending on your testing focus:  
   **A. MoveIt 2 Demo (No Physics)**
    ``` bash
    ros2 launch alfan_moveit_config demo.launch.py
    ```
    **B. Launch Gazebo Fortress + MoveIt2 (Primary)**
    ``` bash
    ros2 launch alfan_moveit_config gazebo_moveit.launch.py
    ```
    **C. Webots Simulation (OUTDATED) (only moving elbows in sync hhe..)**  
   ``` bash
    ros2 launch alfan_webots_sim simulation.launch
   ```


