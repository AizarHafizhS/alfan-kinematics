#!/usr/bin/env python3
import argparse
import subprocess
import sys
import threading

import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node


class WebotsSim(Node):
    def __init__(self, sim_port, robot_type):
        super().__init__("webots_sim")
        pkg_path = get_package_share_directory("alfan_webots_sim")

        # construct arguments with which webots is started depending on this scripts args
        extra_args = set()
        mode = "realtime"

        if robot_type == "alfan":
            world_name = "alfan_arena.wbt"

        cmd = ["webots"]

        # actually start webots
        cmd_with_args = (
            cmd + list(extra_args) + [f"--mode={mode}", f"--port={sim_port}", f"{pkg_path}/worlds/{world_name}"]
        )
        print(f"running {' '.join(cmd_with_args)}")
        self.sim_proc = subprocess.Popen(cmd_with_args)

    def run_simulation(self):
        # join with child process
        try:
            sys.exit(self.sim_proc.wait())
        except KeyboardInterrupt:
            sys.exit(self.sim_proc.returncode)

def main():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--sim-port", help="port of the simulation", default="1234")
    parser.add_argument("--robot-type", help="which robot should be started", default="alfan")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = WebotsSim(args.sim_port, args.robot_type)

    node.run_simulation()
    
    node.destroy_node()



if __name__ == "__main__":
    main()