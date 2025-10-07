#!/usr/bin/env python3
import argparse
import os
import threading
import rclpy
from alfan_webots_sim.alfan_webots_driver import RobotDriver
from controller import Robot
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class RobotNode:
    def __init__(self, simulator_port, robot_name, void_controller, robot_type):
        self.node = Node("robot_node")
        self.void_controller = void_controller
        os.environ["WEBOTS_CONTROLLER_URL"] = f"ipc://{simulator_port}/{robot_name}"
        
        if void_controller:
            self.node.get_logger().info("Starting void interface for " + robot_name)
            self.robot = Robot()
        else:
            self.node.get_logger().info("Starting ros interface for " + robot_name)
            self.robot = RobotDriver(
                ros_node=self.node,
                robot=robot_type,
            )

    def run(self):
        while rclpy.ok():
            if self.void_controller:
                self.robot.step(int(self.robot.getBasicTimeStep()))
            else:
                self.robot.step()


def spin_executor(executor):
    """Function to run the executor in a separate thread"""
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f"Executor error: {e}")
            break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_name", help="which robot should be started", default="chaca")
    parser.add_argument("--robot-type", help="which robot should be started", default="alfan")
    parser.add_argument("--sim-port", help="port of the simulation", default="1234")
    parser.add_argument(
        "--void-controller", action="store_true", help="if true, a controller that only steps and does nothing else"
    )
    
    args, unknown = parser.parse_known_args()
    
    rclpy.init()
    
    robot = RobotNode(
        args.sim_port, args.robot_name, args.void_controller, args.robot_type
    )
    
    # Use SingleThreadedExecutor instead of abstract Executor
    executor = SingleThreadedExecutor()
    executor.add_node(robot.node)
    
    # Start the executor in a separate thread with proper error handling
    thread = threading.Thread(target=spin_executor, args=(executor,), daemon=True)
    thread.start()
    
    try:
        robot.run()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        robot.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()