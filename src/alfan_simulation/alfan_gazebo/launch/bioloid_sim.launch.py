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

    # 2. Parsing file URDF Bioloid
    # Mengarah langsung ke folder bioloid yang ada di dalam alfan_robot_description
    urdf_file = PathJoinSubstitution([pkg_robot_desc, 'bioloid', 'urdf', 'bioloid.urdf.xacro'])
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file]
    )
    robot_description = {'robot_description': robot_description_content}

    # Paksa penggunaan waktu simulasi agar sinkron dengan Gazebo
    global_sim_time = SetParameter(name='use_sim_time', value=True)

    # 3. Mulai Gazebo Fortress (menggunakan dunia kosong bawaan)
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
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description] 
    )

    # 6. Masukkan (Spawn) robot Bioloid ke dalam Gazebo
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'bioloid',  # Harus sama dengan tag <robot name="typea"> di URDF
            '-allow_renaming', 'true',
            '-z', '0.0' # Robot dijatuhkan dari ketinggian 25 cm agar tidak menembus tanah
        ],
        output='screen'
    )

    # 7. Eksekutor Kontrol Level Bawah (Memanggil nama persis dari ros2_controllers.yaml)
    controllers = [
        "joint_state_broadcaster",
        "imu_broadcaster",
        "legs_controller",
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

    # Beri jeda 5 detik agar model 3D Bioloid selesai dimuat sempurna di Gazebo 
    # sebelum controllernya diaktifkan (menghindari error tumpah tindih)
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