import math
import os
import time
from typing import Optional

from controller import Robot
from geometry_msgs.msg import PointStamped
from rclpy.node import Node as RclpyNode
from rclpy.time import Time
from sensor_msgs.msg import JointState
from alfan_msgs.msg import JointCommand
import traceback

class RobotDriver():
    def __init__(
        self,
        ros_node: Optional[RclpyNode] = None,
        robot="alfan",
        robot_node=None
    ):
        """
        The RobotDriver, a Webots controller that controls a single robot.

        :param robot: The name of the robot to use, currently one of wolfgang, darwin, nao, op3
        :param robot_node: The Webots node (!= ROS node) of the robot, can normally be left empty
        """

        # Setup ROS node
        if ros_node is None:
            ros_node = RclpyNode("robot_controller")
        self.ros_node = ros_node
        
        # Setup Webots robot node
        if robot_node is None:
            self.robot_node = Robot()
        else:
            self.robot_node = robot_node

        # self.walkready = [0] * 20
        self.time = 0.0

        self.motors = []
        self.sensors = []
        # for direct access
        self.motors_dict = {}
        self.sensors_dict = {}
        self.timestep = int(self.robot_node.getBasicTimeStep())
        
        # names of the joint devices defined in the proto file
        self.motor_names = [
            # HEAD
            "head_yaw",
            "head_roll",

            # RIGHT HAND
            "r_hand_21",
            "r_hand_22",
            "r_hand_23",
            "r_hand_24",
            "r_hand_25",
            "r_hand_26",

            # LEFT HAND
            "l_hand_31",
            "l_hand_32",
            "l_hand_33",
            "l_hand_34",
            "l_hand_35",
            "l_hand_36",

            # RIGHT LEG
            "r_hip_yaw",
            "r_hip_roll",
            "r_hip_pitch",
            "r_knee",
            "r_ankle_pitch",
            "r_ankle_roll",

            # LEFT LEG
            "l_hip_yaw",
            "l_hip_roll",
            "l_hip_pitch",
            "l_knee",
            "l_ankle_pitch",
            "l_ankle_roll",
        ]
        self.initial_joint_positions = {
            # RIGHT HAND
            "r_hand_21": 0,
            "r_hand_22": 0,
            "r_hand_23": 0,
            "r_hand_24": 0,
            "r_hand_25": 0,
            "r_hand_26": 0,

            # LEFT HAND
            "l_hand_31": 0,
            "l_hand_32": 0,
            "l_hand_33": 0,
            "l_hand_34": 0,
            "l_hand_35": 0,
            "l_hand_36": 0,

            # RIGHT LEG
            "r_hip_yaw"     : 0,
            "r_hip_roll"    : 0,
            "r_hip_pitch"   : 0,
            "r_knee_pitch"  : 0,
            "r_ankle_pitch" : 0,
            "r_ankle_roll"  : 0,

            # LEFT LEG
            "l_hip_yaw"     : 0,
            "l_hip_roll"    : 0,
            "l_hip_pitch"   : 0,
            "l_knee_pitch"  : 0,
            "l_ankle_pitch" : 0,
            "l_ankle_roll"  : 0,
        }
        self.sensor_suffix = "_sensor"

        self.current_positions = {}
        self.joint_limits = {}
        for motor_name in self.motor_names:
            motor = self.robot_node.getDevice(motor_name)
            motor.enableTorqueFeedback(self.timestep)
            self.motors.append(motor)
            self.motors_dict[motor_name] = motor

            sensor = self.robot_node.getDevice(motor_name + self.sensor_suffix)
            sensor.enable(self.timestep)
            self.sensors.append(sensor)
            self.sensors_dict[motor_name] = sensor
            
            # Get current pos
            self.current_positions[motor_name] = sensor.getValue()

            # min, max and middle position (precomputed since it will be used at each step)
            self.joint_limits[motor_name] = (
                motor.getMinPosition(),
                motor.getMaxPosition(),
                0.5 * (motor.getMinPosition() + motor.getMaxPosition()),
            )

        
        self.pub_js = self.ros_node.create_publisher(JointState, "joint_states", 1)
        self.ros_node.create_subscription(JointCommand, "DynamixelController/command", self.command_cb, 10)

        # needed to run this one time to initialize current position, otherwise velocity will be nan
        self.get_joint_values(self.motor_names)

    def step_sim(self):
        self.time += self.timestep / 1000
        self.robot_node.step(self.timestep)

    def step(self):
        self.step_sim()
        self.publish_ros()

    def publish_ros(self):
        # For now, only publishing joint states
        self.publish_joint_states()

        # TODO: add IMU publisher

    def convert_joint_radian_to_scaled(self, joint_name, pos) -> float: 
        # helper method to convert to scaled position between [-1,1] for this joint using min max scaling
        lower_limit, upper_limit, mid_position = self.joint_limits[joint_name]
        return 2.0 * (pos - mid_position) / (upper_limit - lower_limit)

    def convert_joint_scaled_to_radian(self, joint_name, position) -> float:
        # helper method to convert to scaled position for this joint using min max scaling
        lower_limit, upper_limit, mid_position = self.joint_limits[joint_name]
        return position * (upper_limit - lower_limit) / 2 + mid_position

    def set_joint_goal_position(self, joint_name: str, goal_position: float, goal_velocity=-1, scaled=False, relative=False):
        motor = self.motors_dict[joint_name]
        if motor is None:
            print(f"Joint {motor} not found. Can not set goal position.")
            return
        if scaled:
            goal_position = self.convert_joint_scaled_to_radian(joint_name, goal_position)
        if relative:
            goal_position = goal_position + self.get_joint_values([joint_name])[0]
        motor.setPosition(goal_position)
        if goal_velocity == -1:
            motor.setVelocity(motor.getMaxVelocity())
        else:
            motor.setVelocity(goal_velocity)

    def set_joint_goals_position(self, joint_names, goal_positions, goal_velocities):
        for i in range(len(joint_names)):
            try:
                if len(goal_velocities) != 0:
                    self.set_joint_goal_position(joint_names[i], goal_positions[i], goal_velocities[i])
                else:
                    self.set_joint_goal_position(joint_names[i], goal_positions[i])
            except ValueError:
                print(f"invalid motor specified ({joint_names[i]})")

    def command_cb(self, command: JointCommand):
        try:
            self.ros_node.get_logger().info(f"Moving joint in progress...")
            if len(command.positions) != 0:
                # position control
                # todo maybe needs to match external motor names to interal ones fist?
                self.set_joint_goals_position(command.joint_names, command.positions, command.velocities)
            else:
                # torque control
                for i, name in enumerate(command.joint_names):
                    try:
                        self.motors_dict[name].setTorque(command.accelerations[i])
                    except ValueError:
                        print(f"invalid motor specified ({name})")
        except Exception as e:
            self.ros_node.get_logger().error(f"Something occured: {traceback.format_exc()}")


    def set_head_tilt(self, pos):
        self.motors[-1].setPosition(pos)

    def set_arms_zero(self):
        positions = [
            -0.8399999308200574,
            0.7200000596634105,
            -0.3299999109923385,
            0.35999992683575216,
            0.5099999812500172,
            -0.5199999789619728,
        ]
        for i in range(0, 6):
            self.motors[i].setPosition(positions[i])

    def get_joint_values(self, used_joint_names, scaled=False):
        joint_positions = []
        joint_velocities = []
        joint_torques = []
        for joint_name in used_joint_names:
            value = self.sensors_dict[joint_name].getValue()
            if scaled:
                value = self.convert_joint_radiant_to_scaled(joint_name, value)
            joint_positions.append(value)
            joint_velocities.append(self.current_positions[joint_name] - value)
            joint_torques.append(self.motors_dict[joint_name].getTorqueFeedback())
            self.current_positions[joint_name] = value
        return joint_positions, joint_velocities, joint_torques

    def get_joint_state_msg(self):
        js = JointState()
        js.name = []
        js.header.stamp = Time(seconds=int(self.time), nanoseconds=int(self.time % 1 * 1e9)).to_msg()
        js.position = []
        js.effort = []
        for joint_name in self.motor_names:
            js.name.append(joint_name)
            value = self.sensors_dict[joint_name].getValue()
            js.position.append(value)
            js.velocity.append(self.current_positions[joint_name] - value)
            js.effort.append(self.motors_dict[joint_name].getTorqueFeedback())
            self.current_positions[joint_name] = value
        return js

    def publish_joint_states(self):
        self.pub_js.publish(self.get_joint_state_msg())

