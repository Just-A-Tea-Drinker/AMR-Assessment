#!/usr/bin/env python


# Written for humble

#ROS imports 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time 

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_msgs.msg import Float32MultiArray
#importing the custom nodes
import Position_node as Pn
import Laser_node as Ln
import IMGpro as IMGn
import BoxSort as BSn
import Robot_move as Rm
import Beh_Controller as Bc




def main(args=None):
    """launching different nodes

        launching the network of different nodes using the multi async spinner
    """
    rclpy.init(args=args)

    try:
        print("Running custom nodes")
        
        pos_node = Pn.Positioning() 
        laser_node= Ln.GetScan()
        Image_node = IMGn.ImageProcessing()
        Box_node = BSn.BoxSort()
        Move_node = Rm.Move()
        
        High_think = Bc.BehaviourController()

        executor = MultiThreadedExecutor()
        
        executor.add_node(pos_node)
        executor.add_node(laser_node)
        executor.add_node(Image_node)
        executor.add_node(Box_node)
        executor.add_node(Move_node)
        


        executor.add_node(High_think)


        try:
            executor.spin()
        finally:
            executor.shutdown()
            pos_node.destroy_node()
            laser_node.destroy_node()
            Image_node.destroy_node()
            Box_node.destroy_node()
            
    finally:
        rclpy.shutdown()
   
if __name__ == '__main__':
    main()  