#!/usr/bin/env python3
#ROS imports 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray,Int8

#python imports
import math

class Positioning:
    #0:X, 1:Y, 2:yaw, 3:starting yaw
    Robo_pos = [0,1,2,3]
    start_yaw = None
    yaw = 0.0

    def __init__(self,node):
        self.node = node
        #subscribing to the odometry topic
        self.sub = self.node.create_subscription(Odometry,"/odom",self.PosCallback,1)
        #making a publisher to share compressed/ more relevant odom data
        self.Odom_pub = self.node.create_publisher(Float32MultiArray,"/Compressed_Odom",60)
        self.Comp_odom = Float32MultiArray()
        

    
        
    def PosCallback(self,data):
        if(data.child_frame_id == "base_link"):
   
            #position
            self.Robo_pos[0] = data.pose.pose.position.x
            self.Robo_pos[1] = data.pose.pose.position.y

            #orientation converted into euler
            #storing the quaternion
            quat = (
            data.pose.pose.orientation.x, #0
            data.pose.pose.orientation.y, #1
            data.pose.pose.orientation.z, #2
            data.pose.pose.orientation.w  #3
            )
            ##calculating the roll pitch and yaw
            ysqr = quat[1] * quat[1]

            t0 = +2.0 * (quat[3] * quat[0] + quat[1] * quat[2])
            t1 = +1.0 - 2.0 * (quat[0] * quat[0] + ysqr)
            #roll = math.degrees(math.atan2(t0, t1))

            t2 = +2.0 * (quat[3] * quat[1] - quat[2] * quat[0])
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            #pitch = math.degrees(math.asin(t2))

            t3 = +2.0 * (quat[3] * quat[2] + quat[0] * quat[1])
            t4 = +1.0 - 2.0 * (ysqr + quat[2] * quat[2])
            self.yaw = math.degrees(math.atan2(t3, t4))
            if self.start_yaw == None:
                self.start_yaw = self.yaw
            self.Robo_pos[2] = self.yaw
            self.Robo_pos[3] = self.start_yaw
            #print(self.Robo_pos)
            #publishing new compressed and relative information to custom topic
            self.Comp_odom.data = self.Robo_pos
            #print(type(self.Comp_odom))
            self.Odom_pub.publish(self.Comp_odom)


