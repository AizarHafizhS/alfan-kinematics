import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Definisikan Lokasi Paket
    pkg_robot_desc = FindPackageShare('alfan_robot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # 2. Parsing file Xacro secara langsung menjadi URDF
    urdf_file = PathJoinSubstitution([pkg_robot_desc, 'urdf', 'alfan_2025.urdf.xacro'])
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file]
    )
    robot_description = {'robot_description': robot_description_content}

    # Paksa penggunaan waktu simulasi
    global_sim_time = SetParameter(name='use_sim_time', value=True)

    # 3. Mulai Gazebo Fortress (dunia kosong)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 4. Jembatan Topik (Ignition ke ROS2) - Sangat Krusial untuk sinkronisasi /clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # 5. Urat Nadi ROS2: Robot State Publisher
    # Menggantikan peran rsp.launch.py dari MoveIt
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description] # Memasukkan hasil parsing URDF
    )

    # 6. Masukkan (Spawn) robot ke dalam Gazebo
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'alfan_2025', 
            '-allow_renaming', 'true',
            '-z', '0.4' # Argumen Kritis: Menjatuhkan robot dari ketinggian 0.4m agar kaki tidak terjebak menembus tanah saat frame pertama
        ],
        output='screen'
    )

    # 7. Eksekutor Kontrol Level Bawah (Pengganti MoveIt)
    # Daftar ini wajib sama persis dengan nama di ros2_controllers.yaml Anda
    controllers = [
        "joint_state_broadcaster",
        "imu_broadcaster",
        "legs_controller",
        "head_controller",
        "left_arm_controller",
        "right_arm_controller"
    ]

    spawner_nodes = []
    for controller in controllers:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen"
            )
        )

    # Beri jeda 5 detik agar model 3D selesai dimuat di Gazebo sebelum controller aktif
    delay_spawners = TimerAction(
        period=5.0,
        actions=spawner_nodes
    )

    return LaunchDescription([
        global_sim_time,
        gazebo,
        clock_bridge,
        node_robot_state_publisher,
        spawn_entity,
        delay_spawners
    ])