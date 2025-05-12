#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.node import Node
import sys
import os
import numpy as np
import argparse
import threading
import json
from std_msgs.msg import String  # Import String message type
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
            "romfiles": self.rom_paths,
            "serial port": "/dev/ttyUSB0"  # Serial port configuration
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
        
        # Create PolarisReader instance
        self.polaris_reader = PolarisReader(rom_paths, save_flag, save_path)
        
        # Add publisher to publish to ndi_transforms topic
        self.publisher = self.create_publisher(String, 'ndi_transforms', 10)
        
        # Log initialization complete
        self.get_logger().info("ROS2 Polaris Reader initialized")
        self.get_logger().info("Publishing tracking data to 'ndi_transforms' topic")

    def run(self):
        # Start a thread for updating data and publishing
        self.get_logger().info("Starting Polaris tracking...")
        tracking_thread = threading.Thread(
            target=self.tracking_loop, 
            daemon=True
        )
        tracking_thread.start()
        
        # Log that tracking has started
        self.get_logger().info("Polaris tracking thread started")
    
    def tracking_loop(self):
        """Tracking loop to read data and publish to topic"""
        try:
            self.polaris_reader.TRACKER.start_tracking()
            counter = 0
            
            while rclpy.ok():  # Check if ROS is still running
                # Get tracking data
                port_handles, timestamps, framenumbers, trackings, qualities = self.polaris_reader.TRACKER.get_frame()
                
                # Print data to console (same as original code)
                for i, timestamp in enumerate(timestamps):
                    print(f"[{i}] {timestamp}")
                    print(trackings[i])
                
                # Save data (if enabled)
                if self.polaris_reader.save_flag:
                    for i, tracking in enumerate(trackings):
                        if i < len(self.polaris_reader.save_rom_paths):
                            np.savetxt(
                                os.path.join(self.polaris_reader.save_rom_paths[i], f"{counter}.txt"), 
                                tracking, 
                                fmt='%.10f', 
                                delimiter='\t'
                            )
                    counter += 1
                
                # Publish tracking data to ROS topic
                self.publish_tracking_data(port_handles, timestamps, framenumbers, trackings, qualities)
                
        except Exception as e:
            self.get_logger().error(f"Error in tracking loop: {str(e)}")
        finally:
            # Ensure tracker is closed on error or exit
            try:
                self.polaris_reader.TRACKER.stop_tracking()
                self.polaris_reader.TRACKER.close()
                self.get_logger().info("Polaris tracking stopped")
            except Exception as e:
                self.get_logger().error(f"Error stopping tracker: {str(e)}")
    
    def publish_tracking_data(self, port_handles, timestamps, framenumbers, trackings, qualities):
        """Publish tracking data to ndi_transforms topic"""
        try:
            # Prepare transform data
            transforms_data = []
            
            for i, tracking in enumerate(trackings):
                if i < len(port_handles):
                    # Extract translation and rotation data from tracking matrix
                    matrix = tracking.tolist() if isinstance(tracking, np.ndarray) else tracking
                    
                    # Create transform info object
                    transform_info = {
                        "tool_id": int(port_handles[i]) if isinstance(port_handles[i], (int, np.integer)) else str(port_handles[i]),
                        "quality": float(qualities[i]) if i < len(qualities) else 0.0,
                        "matrix": matrix
                    }
                    
                    # If matrix is 4x4, we can also extract translation and rotation
                    if isinstance(tracking, np.ndarray) and tracking.shape == (4, 4):
                        # Extract translation vector (first 3 rows of last column)
                        transform_info["translation"] = tracking[:3, 3].tolist()
                        
                        # More info can be added, such as extracting quaternion from matrix
                        # Quaternion calculation code can be added here if needed
                    
                    transforms_data.append(transform_info)
            
            # Create the complete data object
            data = {
                "timestamp": str(timestamps[0]) if timestamps else "",
                "frame_number": int(framenumbers[0]) if framenumbers else 0,
                "transforms": transforms_data
            }
            
            # Create and publish message
            msg = String()
            msg.data = json.dumps(data)
            self.publisher.publish(msg)
            
            # Log publishing event (optional)
            # self.get_logger().debug(f"Published tracking data with {len(transforms_data)} transforms")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing tracking data: {str(e)}")


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Parse command line arguments
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
