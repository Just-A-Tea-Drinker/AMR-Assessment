#!/usr/bin/env python3

# Written for humble

#ROS imports 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray,Float32MultiArray
from std_msgs.msg import Int8

#PYTHON imports
import numpy as np
from math import radians
import math
import time

class Move:
    lin_vel = 0.0
    ang_vel = 0.0

    headingSet = False
    target_heading = -45.0
   

    target = [0.0,0.0]
    waypoint=  [0.0,0.0]

    ranges =[]
    turn = True

    state = [0]
    failed =False
    response = [0]

    count = 0
    count1 = 0#

    X = 0.0
    Y = 0.0
    yaw = 0.0
    start_yaw =0.0

    wallsInfo = [0.0]*4
    Xs = []
    Ys = []

   
    
    count = 0
    #constructor
    def __init__(self,node):
        self.node = node
        #setting a publisher for the wheel velocities        
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 60)
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0  # m/s
        self.move_cmd.angular.z = 0.0
        self.SetMove()

        ##subscriber to read the custom scan data
        self.range_get = self.node.create_subscription(Float32MultiArray,'/Range_Pub',self.ScanCallback,1)

        #subscriber for the custom odometry
        self.pos_get = self.node.create_subscription(Float32MultiArray,'/Compressed_Odom',self.OdomCallback,1)
        
        #creating a time based callback that is not event driven
        timer_period = 0.01  # seconds
        self.updater = self.node.create_timer(timer_period, self.Broadcast)

        self.kill_pub = self.node.create_publisher(Int8, '/Kill_switch', 60)
        self.kill_msg = Int8()


    def Broadcast(self):
        #used to broadcast the state of the mapping function
        if self.state[0] ==0:
            self.boxScan()
        if self.state[0] == 1:
            #broadcast the kill switch
            self.kill_msg.data = 1
            self.kill_pub.publish(self.kill_msg)
            
              
    def OdomCallback(self,odom):
        self.start_yaw = odom.data[3]
        self.yaw = odom.data[2]
        self.X = odom.data[0]
        self.Y = odom.data[1]
    def ScanCallback(self,ran):
        self.ranges = ran.data


    #updates what values that should be published 
    def SetMove(self):
        #updates the message to be published
        self.move_cmd.linear.x = self.lin_vel 
        self.move_cmd.angular.z = self.ang_vel
        self.cmd_vel_pub.publish(self.move_cmd)

   


    def ZoneRange(self,zones):
        #gettting the average of the distances in the lidar zones
        zone_avg = []
        for zone in zones:
           avg = np.average(zone)
           zone_avg.append(avg)

        ##taking these averages and making them relative to left and right
        left = []
        right = []

        for i in range(len(zone_avg)):
            if i<1:
                left.append(zone_avg[i])
            else:
                right.append(zone_avg[i])
        #getting the average of the left and right zones
        left_avg = np.average(left)
        right_avg = np.average(right)

        return left_avg,right_avg


    #reading  the sensor data to detect to see if the move is viable
    def ObsAvoid(self):
        #NOTE TO SELF:
        #ranges[0] = most right
        #ranges[290] = most left
        #ranges[145] = middle point/ front
        
        
        DectZones = []
        for i in range(0, len(self.ranges), int(len(self.ranges)/2)):
            zone = self.ranges[i:i + int(len(self.ranges)/2)]
            DectZones.append(zone)
        
        #NOTE TO SELF
        #DectZones[0] will be on the right
        #DectZones[10] will be on the left
        # -vel = turning left
        # +vel = turning right

        #these zones can now be used to detect obstacles in them and then activate a set of avoidance rules
        #changing the ang_vel and lin_vel in the process
        if(self.ranges[int(len(self.ranges)/2)]>0.25):
            self.ang_vel = 0.0
            self.lin_vel = 0.2
        else:
            ##quickly decides whether or not the robot should go left or right when theres an obstacle
            
            ##using the detection zones to get an average length bigger = more room to go towards
            left,right = self.ZoneRange(DectZones)
            if left >right:
                self.ang_vel = -1.0
                self.lin_vel = 0.0
                
            else:
                self.ang_vel = 1.0
                self.lin_vel = 0.0
                
        ##the zone idea is not perfect as it will go towards a zone that biggest even in a tight space so a small statement used to detect and combat this
        if  self.ranges[int(len(self.ranges)/2)]<0.5 and self.ranges[int((len(self.ranges)/2))-65]<0.5 and self.ranges[int((len(self.ranges)/2))+65]<0.5:  
            self.ang_vel = -1.0
            self.lin_vel = 0.0
            self.SetMove
            return   
        ##generalistic avoidance for the robot
        if self.ranges[int((len(self.ranges)/2))-65]<0.25 and self.ranges[int((len(self.ranges)/2))+65]>0.25:
            self.ang_vel = 1.0
            self.lin_vel = 0.0
        
        elif  self.ranges[int((len(self.ranges)/2))+65]<0.25 and self.ranges[int((len(self.ranges)/2))-65]>0.25:
            self.ang_vel = -1.0
            self.lin_vel = 0.0

    def boxScan(self):
        #method first called to find any boxes
        for range in self.ranges:
            if(range <0.2):
                print("Vehicle cannot turn without collision\nExploration active to locate any boxes that are not tidy")
                self.ang_vel = 0.0
                self.lin_vel = 0.0
                self.turn = False
                return

            
        #checking if the control count and yaw has been filled
        if self.start_yaw != None and self.count==0:
            
            #using an odometer to detect a complete 360 scan, using a timestamp to offset the yaw checking otherwise it will end before even moving
            if self.count1 ==0:
                #starting the time flag
                self.start_time =time.time()
                
                self.count1+=1
                
            else:
                #announcing the change in time
                self.elapsed_time = time.time() - self.start_time
                
                #if the flag is greater than 5 seconds and then checks if the yaw is back to the origin
                if self.elapsed_time >= 5 and abs(self.yaw - self.start_yaw) <= 0.5:
                    #if the yaw and start yaw are equal the robot is stopped from "scanning"
                    self.node.get_logger().info("Generating map complete")
                    self.ang_vel = 0.0
                    self.lin_vel = 0.0
                    self.SetMove()
                    self.count+=1
                    self.state[0] = 1
                    self.turn = False
                   
                    ##returning confirmation that the action has been completed
                    self.response[0] = 1.0
                   
                    
        if self.turn == True:
            self.ang_vel = 0.3
            self.lin_vel = 0.0
        self.SetMove()


