#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.node import Node
import sys
import os
import numpy as np
import argparse
import threading
from sksurgerynditracker.nditracker import NDITracker

class PolarisReader:
    def __init__(self, rom_paths=[], save_flag=True, save_path="."):
        self.rom_paths = rom_paths
        self.SETTINGS = []
        self.TRACKER = []
        self.port_handles = []
        self.timestamps = []
        self.framenumbers = []
        self.trackings = []
        self.qualities = []
        self.save_flag = save_flag
        self.save_path = save_path
        self.save_rom_paths = []
        self.load_polaris()

    def load_polaris(self):
        self.SETTINGS = {
            "tracker type": "polaris",
            "romfiles": self.rom_paths
        }
        self.TRACKER = NDITracker(self.SETTINGS)
        if self.save_flag:
            for i, rom_path in enumerate(self.rom_paths):
                save_rom_path = os.path.join(self.save_path, os.path.splitext(os.path.basename(rom_path))[0])
                self.save_rom_paths.append(save_rom_path)
                if not os.path.exists(save_rom_path):
                    os.makedirs(save_rom_path)

    def run_polaris(self):
        self.TRACKER.start_tracking()
        try:
            counter = 0
            while True:
                self.port_handles, self.timestamps, self.framenumbers, self.trackings, self.qualities = self.TRACKER.get_frame()
                for i, timestamp in enumerate(self.timestamps):
                    print(f"[{i}] {timestamp}")
                    print(self.trackings[i])
                if self.save_flag:
                    for i, tracking in enumerate(self.trackings):
                        if i < len(self.save_rom_paths):  # Check to avoid index errors
                            np.savetxt(
                                os.path.join(self.save_rom_paths[i], f"{counter}.txt"), 
                                tracking, 
                                fmt='%.10f', 
                                delimiter='\t'
                            )
                    counter += 1

        except KeyboardInterrupt:
            # Allow the user to interrupt the loop by pressing Ctrl+C
            pass
        finally:
            self.TRACKER.stop_tracking()
            self.TRACKER.close()


class ROS2PolarisReaderNode(Node):
    def __init__(self, rom_paths=[], save_flag=True, save_path="."):
        super().__init__('ros2_polaris_reader')
        
        # Initialize logging
        self.get_logger().info("Python version: " + sys.version)
        self.get_logger().info("Python installation path: " + sys.executable)
        
        # Create the PolarisReader instance (same as in your original code)
        self.polaris_reader = PolarisReader(rom_paths, save_flag, save_path)
        
        # Here you can add ROS2 publishers if needed, for example:
        # from std_msgs.msg import String
        # self.publisher = self.create_publisher(String, 'polaris_data', 10)
        
        # Log initialization complete
        self.get_logger().info("ROS2 Polaris Reader initialized")

    def run(self):
        # Start polaris reader in a separate thread (same as in your original code)
        self.get_logger().info("Starting Polaris tracking...")
        tracking_thread = threading.Thread(
            target=self.polaris_reader.run_polaris, 
            daemon=True
        )
        tracking_thread.start()
        
        # Log that tracking has started
        self.get_logger().info("Polaris tracking thread started")


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Parse command line arguments (same as in your original code)
    parser = argparse.ArgumentParser()
    parser.add_argument('--rom_path', help="rom file paths", action="append")
    parser.add_argument('--save_flag', help="save_flag", action="store_true")
    parser.add_argument('--save_path', help="save path", default=".")
    
    # Parse the arguments, but separate ROS2 args
    parsed_args, remaining = parser.parse_known_args()
    
    try:
        # Create and initialize the node with the parsed arguments
        node = ROS2PolarisReaderNode(
            rom_paths=parsed_args.rom_path if parsed_args.rom_path else [],
            save_flag=parsed_args.save_flag,
            save_path=parsed_args.save_path
        )
        
        # Start the tracking thread
        node.run()
        
        # Keep the node running
        rclpy.spin(node)
        
    except Exception as e:
        print(f"Error in ROS2 Polaris Reader: {str(e)}")
    finally:
        # Clean up
        rclpy.shutdown()


if __name__ == '__main__':
    main()