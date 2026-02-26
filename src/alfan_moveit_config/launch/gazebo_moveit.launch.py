import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    moveit_config_dir = FindPackageShare('alfan_moveit_config').find('alfan_moveit_config')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    # Paksa penggunaan masa simulasi secara global
    global_sim_time = SetParameter(name='use_sim_time', value=True)

    # 1. Mulakan Gazebo Fortress (dunia kosong)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 2. Jambatan Masa (Kritikal untuk MoveIt dan ros2_control)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # 3. Masukkan (Spawn) robot dari topik robot_description
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'alfan_2025', '-allow_renaming', 'true'],
        output='screen'
    )

    # 4. Robot State Publisher & MoveIt
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'rsp.launch.py'))
    )
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_fake_hardware': 'false'}.items()
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'moveit_rviz.launch.py'))
    )

    # 5. Spawners
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    head_spawner = Node(package="controller_manager", executable="spawner", arguments=["head_controller"])
    l_arm_spawner = Node(package="controller_manager", executable="spawner", arguments=["left_arm_controller"])
    r_arm_spawner = Node(package="controller_manager", executable="spawner", arguments=["right_arm_controller"])
    legs_spawner = Node(package="controller_manager", executable="spawner", arguments=["legs_controller"])
    imu_spawner = Node(package="controller_manager", executable="spawner", arguments=["imu_broadcaster"])

    # Beri masa 4 saat untuk Gazebo sedia sebelum memasukkan controller
    delay_spawners = TimerAction(
        period=4.0,
        actions=[jsb_spawner, imu_spawner, head_spawner, l_arm_spawner, r_arm_spawner, legs_spawner]
    )

    return LaunchDescription([
        global_sim_time,
        gazebo,
        clock_bridge,
        rsp_launch,
        spawn_entity,
        move_group_launch,
        rviz_launch,
        delay_spawners
    ])