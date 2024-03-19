#!/usr/bin/env python

# An example of TurtleBot 3 drawing a 1 meter square.
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
import Robot_pathplanning as Rpp


class Robot(Node):
    def __init__(self):
        super().__init__('Main_robot_controller')
        ##subscribing to relevant topics 
        self.sub = self.create_subscription(Float32MultiArray, '/Range_Pub', 1)
        self.sub = self.create_subscription(Odometry,"/odom",self.PosCallback,1)
        #need to change the image one as this will will actually be used to process image and identify targets
        #self.sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.Cam_callback, 10)



def main(args=None):
    rclpy.init(args=args)

    try:
        print("Running custom nodes")
        
        pos_node = Pn.Positioning() 
        laser_node= Ln.GetScan()
        Image_node = IMGn.ImageProcessing()
        Box_node = BSn.BoxSort()
        Move_node = Rm.Move()
        PathPlan =Rpp.PathPlanning()
        High_think = Bc.BehaviourController()

        executor = MultiThreadedExecutor()
        
        executor.add_node(pos_node)
        executor.add_node(laser_node)
        executor.add_node(Image_node)
        executor.add_node(Box_node)
        executor.add_node(Move_node)
        #executor.add_node(PathPlan)


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