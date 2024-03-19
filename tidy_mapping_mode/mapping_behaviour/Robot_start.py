#!/usr/bin/env python3
# An example of TurtleBot 3 drawing a 1 meter square.
# Written for humble

#ROS imports 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int8,String
from nav_msgs.msg import OccupancyGrid

import yaml
import numpy as np
#importing the custom functionalities
import Robot_move as rm
import Position_node as pn
import Laser_node as ln
from slam_toolbox.srv import SaveMap
import psutil
import sys
import os


#this will be a node housing several core components of the robot
class Robot(Node):

    kill_switch =0
    def __init__(self):
        super().__init__('Robot_core')
        self.moving = rm.Move(self)
        self.laser = ln.GetScan(self)
        self.odometry = pn.Positioning(self)
        self.kill_sub = self.create_subscription(Int8, '/Kill_switch', self.Killcallback,1)
        self.map_sub = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)

    def Killcallback(self,switch):
        
        self.kill_switch = switch.data
        


    def map_callback(self, msg):
        
        if self.kill_switch == 1:
            # Create a client to call the save_map service
            client = self.create_client(SaveMap, '/slam_toolbox/save_map')

            # Ensure the service is available
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')

            # Create a request object
            request = SaveMap.Request()
            request.name.data = 'World_map'

            # Call the service
            future = client.call_async(request)

            # Wait for the service response
            rclpy.spin_until_future_complete(self, future,timeout_sec=5.0)
            
            # Ensure the directory exists
            directory = '/workspaces/AMR-Assessment'
            filename = 'World_map.pgm'
            if not os.path.exists(directory) or not os.path.isdir(directory):
                print(f"Error: Directory '{directory}' does not exist or is not a directory.")
                

            # Walk through the directory and its subdirectories
            for root, dirs, files in os.walk(directory):
                if filename in files:
                    self.get_logger().info('Map saved successfully!')
                    for proc in psutil.process_iter(['pid', 'name']):
                        cmdline = proc.cmdline()
                        # for arg in cmdline:
                        #     self.get_logger().info(arg)
                        if any('/slam' in arg for arg in cmdline):
                            process = psutil.Process(proc.pid)
                            process.terminate()
                    self.get_logger().info('Mapping complete cleaning up nodes') 
                    sys.exit(0)  
                    
                    
        
        


    


def main(args=None):
    rclpy.init(args=args)

    try:
        print("Running custom nodes")
    
        node = Robot()
       
        try:
            rclpy.spin(node)
            node.destroy_node()
        finally:
           node.destroy_node()
           
           
           
            
    finally:
        rclpy.shutdown()
   
if __name__ == '__main__':
    main()  