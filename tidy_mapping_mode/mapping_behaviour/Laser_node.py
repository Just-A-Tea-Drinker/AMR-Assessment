#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray,Int8
#this is a node dedicated for the the scanning of the lidar  data

class GetScan():
    ranges = []
    def __init__(self,node):
        self.node = node
        #subscribing to the laser scan topic
        self.sub = self.node.create_subscription(LaserScan,"/scan",self.Scancallback, 1)
        #creating a publisher for the the range data being received 
        self.ranges_pub = self.node.create_publisher(Float32MultiArray, '/Range_Pub', 60)
        self.lidar_range = Float32MultiArray()
       

    

    def Scancallback(self, data):
        ##obtaining the scan data
       
        #doing some basic filtering for the lidar data
        actual_ranges = []
        for range in data.ranges:
            if range != 0.0:
                actual_ranges.append(range)
        if len(actual_ranges)%2 !=0:
            actual_ranges =actual_ranges[1:]
        self.ranges = actual_ranges
        
        #taking this new set of values and publish them to a new topic
        self.lidar_range.data =self.ranges
        self.ranges_pub.publish(self.lidar_range)



