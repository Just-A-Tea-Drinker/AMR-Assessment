#!/usr/bin/env python

# Written for humble

#ROS imports 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

#PYTHON imports
import numpy as np
from math import radians
import math
import time

#custom import

import global_pathplanning as gp

class Move(Node):
    """Secondary brain node used to control the movement of the robot

        Inputs:
        
        /Conductor, interprets the main command to control behaviour
        /Range_Pub customised lidar topic
        /Compressed_Odom, inputting the robot location with stamped yaw
        /BoxInfo' getting the information of all the boxes

        Output:
        /Box_Id, what boxes are required to be moved


        converts quaternion into eular whilst also maintaining a stamp of the initial yaw value
        conversion function code was sourced from: https://stackoverflow.com/questions/56207448/efficient-quaternions-to-euler-transformation
    """
    lin_vel=0.0
    ang_vel=0.0


    ranges =[]
    turn = True

    state = [0]
    failed =False
    response = [0]

    count = 0
    count2 =0
    count1 = 0#

    X = 0.0
    Y = 0.0
    yaw = 0.0
    start_yaw =0.0

    #boxes to move
    green_boxes = []
    red_boxes = []
    selecBox = [None]
    boxIndex = 0

    ##what walls to move the boxes at
    green_wall = []
    red_red = []


    #path finding
    path_made = False
   
    path = [None]
    bearings = [None]
    path2Start= []
    
    to_start = False
    box_move =False

    prev_box = []
    box_coll = []
    prev_2path = []
    prev_path = []
    path_coll = []
    globalpp=None
    inv_Box = []

    count3 = 0
    count4= 0
    BothList =[]
    Reset = False
    release = False
    #constructor
    def __init__(self):
        super().__init__('Robot_move')
        #setting a publisher for the wheel velocities        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_tidy_vel', 60)
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.0  # m/s
        self.move_cmd.angular.z = 0.0
        self.SetMove()
        ##setting up a response publisher that will talk to the brain to communicate back when something has been achieved
        self.responder = self.create_publisher(Float32MultiArray,'/Brain_Respond',60)
        self.brain_response  = Float32MultiArray()
       
        #publishing what box that is moving
        self.box_id = self.create_publisher(Float32MultiArray,'/Box_ID',60)
        self.box2send  = Float32MultiArray()
     
        ##setting up a subscriber to read brain commands
        self.beh_get = self.create_subscription(Float32MultiArray,'/Conductor',self.CommandCallback,1)

        ##subscriber to read the custom scan data
        self.range_get = self.create_subscription(Float32MultiArray,'/Range_Pub',self.ScanCallback,1)
       
        #subscriber for the custom odometry
        self.pos_get = self.create_subscription(Float32MultiArray,'/Compressed_Odom',self.OdomCallback,1)
        #subscribing to the box topic to obtain the boxes
        self.box_get = self.create_subscription(Float32MultiArray,'/BoxInfo',self.BoxCallback,1)

        timer_period = 0.01  # seconds
        self.updater = self.create_timer(timer_period, self.Broadcast)
        #navigator for nav2
        self.navigator = BasicNavigator()

    
    def Broadcast(self):
        if self.turn ==False:

            ##returning confirmation that the action has been completed
            self.brain_response.data = self.response
            self.responder.publish(self.brain_response)
       
    def BoxCallback(self,boxes):
        self.green_boxes = []
        self.red_boxes = []
        split_box = []
        for i in range(0,len(boxes.data), 4):
            for x in range(0,4):
                split_box.append(boxes.data[i+x])
            if split_box[1] == 1.0:
                self.green_boxes.append(split_box)
            else:
                #they must be red
                self.red_boxes.append(split_box)
            split_box = []
    def OdomCallback(self,odom):
        self.start_yaw = odom.data[3]
        self.yaw = odom.data[2]
        self.X = odom.data[0]
        self.Y = odom.data[1]
    def ScanCallback(self,ran):
        self.ranges = ran.data

    def CommandCallback(self,command):
        self.state = command.data[0]
        self.green_wall = command.data[1:3]
        self.red_wall = command.data[3:]
        #this will be control area of this module as well
        if self.state == 0.0:
            self.boxScan()
        elif self.state ==1.0:
            #activate the movement,heading and box pushing protocols
            
            self.BoxMove()
    def StartCompCreate(self):
        #used to feed to nav2 to get to the start collision free hopefully
        waypoints = []
        if len(self.path2Start)>1:
            for i in range(len(self.path2Start)):
                #time to convert the steps into quaternion with a goal orientation
                if i+1< len(self.path2Start):
                    changeY = self.path2Start[i+1][1] - self.path2Start[i][1]
                    changeX = self.path2Start[i+1][0] - self.path2Start[i][0]
                    yaw = math.atan2(changeY,changeX)
                    quaternion = self.eularToQuaternion(yaw)
                    goal_pose1 = PoseStamped()
                    goal_pose1.header.frame_id = 'map'
                    goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()##############
                    goal_pose1.pose.position.x = self.path2Start[i][0]+0.0
                    goal_pose1.pose.position.y = self.path2Start[i][1]+0.0
                    goal_pose1.pose.orientation.w = quaternion[3]
                    goal_pose1.pose.orientation.z = quaternion[2]
                    waypoints.append(goal_pose1)
                else:
                    print(self.path)
                    print(self.bearings)
                    changeY = self.path[1][1] - self.path[0][1]
                    changeX = self.path[1][0] - self.path[0][0]
                    yaw = math.atan2(changeY,changeX)
                    quaternion = self.eularToQuaternion(yaw)
                    goal_pose1 = PoseStamped()
                    goal_pose1.header.frame_id = 'map'
                    goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()##############
                    goal_pose1.pose.position.x = self.path[0][0]
                    goal_pose1.pose.position.y = self.path[0][1]
                    goal_pose1.pose.orientation.w = quaternion[3]
                    goal_pose1.pose.orientation.z = quaternion[2]
                    waypoints.append(goal_pose1)
            return waypoints
    
    def BoxPathCreate(self):
        box_path=[]
        for i in range(0,len(self.path)):
            #as we should be a the bearing point at the start then we start with the path point
            if i+1<len(self.path):
                #but path points are an aprox where the box should be so we need to calculate a relative displacement
                heading = math.atan2(self.path[i+1][1]-self.path[i][1],self.path[i+1][0]-self.path[i][0])
                px = self.path[i][0] + (0.17*math.cos(heading))
                py = self.path[i][1]+(0.17*math.sin(heading))

                changeY = self.path[i+1][1] - py
                changeX = self.path[i+1][0] - px
                yaw = math.atan2(changeY,changeX)
                quaternion = self.eularToQuaternion(yaw)    
                
                goal_pose3 = PoseStamped()
                goal_pose3.header.frame_id = 'map'
                goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()######################
                goal_pose3.pose.position.x = px
                goal_pose3.pose.position.y = py
                goal_pose3.pose.orientation.w = quaternion[3]
                goal_pose3.pose.orientation.z = quaternion[2]
                box_path.append(goal_pose3)

            else:
                #but path points are an aprox where the box should be so we need to calculate a relative displacement
                heading = math.atan2(self.bearings[-1][1]-self.path[-1][1],self.bearings[-1][0]-self.path[-1][0])
                px = self.path[-1][0] + (0.025*math.cos(heading))
                py = self.path[-1][1]+(0.025*math.sin(heading))
                changeY =  py-self.bearings[-1][1]
                changeX = px-self.bearings[-1][0]
                yaw = math.atan2(changeY,changeX)
                quaternion = self.eularToQuaternion(yaw)  
                
                goal_pose3 = PoseStamped()
                goal_pose3.header.frame_id = 'map'
                goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()#####################
                goal_pose3.pose.position.x = px
                goal_pose3.pose.position.y = py
                box_path.append(goal_pose3)

        return(box_path)  
    def eularToQuaternion(self,yaw):
        roll = 0.0
        pitch = 0.0
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return qx, qy, qz, qw

    def BoxVia(self):
        #testing the viability of the boxes found and making it known
        valid_boxes = []
        invalid_boxes = []
        new_green = []
        new_red = []
           
        for gboxes in self.green_boxes:
            if gboxes not in self.box_coll:
                new_green.append(gboxes)

        for rboxes in self.red_boxes:
            if rboxes not in self.box_coll:
                new_red.append(rboxes)

        self.globalpp =gp.PathPlanning(new_green,new_red)
        for gboxes in new_green:
            global_test = self.globalpp.PathMakingBackup(self.X,self.Y,gboxes)
            if global_test == True:
                local_test = self.globalpp.Nav2Start(self.X,self.Y,new_green,new_red)
                if local_test ==True:
                    valid_boxes.append(gboxes)
                else:
                    invalid_boxes.append(gboxes)
            else:
                invalid_boxes.append(gboxes)
        
        for rboxes in new_red:
            global_test = self.globalpp.PathMakingBackup(self.X,self.Y,rboxes)
            if global_test == True:
                local_test = self.globalpp.Nav2Start(self.X,self.Y,new_green,new_red)
                if local_test ==True:
                    valid_boxes.append(rboxes)
                else:
                    invalid_boxes.append(rboxes)

            else:
                invalid_boxes.append(rboxes)

        return valid_boxes

        pass
    def BoxMove(self):
        #method used to control and move the boxes in a category
        new_green = []
        new_red = []
        viable_box = self.BoxVia()
       
        for gboxes in self.green_boxes:
            if gboxes in viable_box:
                new_green.append(gboxes)

        for rboxes in self.red_boxes:
            if rboxes in viable_box:
                new_red.append(rboxes)
        
        if(len(new_green) !=0 or len(new_red) !=0)and self.release == True:
            self.count2 = 0
            self.selecBox[0] = None
            self.release = False
            self.Reset = False
            
        else:
            if len(new_green) ==0 and len(new_red) ==0 and self.release == True:
                print("The boxes that are moveable have been moved")
                return

        if self.release ==False:
            viable_box = self.BoxVia()

            if self.selecBox[0] ==None:
                #testing whether or not the boxes in are the the viable box category
                new_green = []
                new_red = []
            
                for gboxes in self.green_boxes:
                    if gboxes in viable_box:
                        if gboxes not in self.box_coll:
                            new_green.append(gboxes)

                for rboxes in self.red_boxes:
                    if rboxes in viable_box:
                        if rboxes not in self.box_coll:
                            new_red.append(rboxes)
                
                if len(new_green)>len(new_red) and len(new_green)>0:
                    
                    self.selecBox[0] = new_green[0]
                else:
                    if len(new_red)>0:
                        self.selecBox[0] = new_red[0]
                
                if len(new_green) ==0 and len(new_red) ==0:
                    self.selecBox[0] = [0.0,0.0,0.0,0.0]
                    self.box_move = False
                    self.Reset = True;  
            
                print(self.selecBox[0])
                    
                
            
            else:
                
                #setting up the pathfinding with the current target and wall
                
               
                #calling the pathfinding to contruct a path
                #calling the custom global planner
                if self.count2==0:
                    self.globalpp =gp.PathPlanning(self.green_boxes,self.red_boxes)
                    self.count2+=1

                if self.Reset == False:
                    
                    global_path = self.globalpp.PathMakingBackup(self.X,self.Y,self.selecBox[0])
                    self.path = self.globalpp.Pix2Cart(self.globalpp.path)
                    self.bearings = self.globalpp.Pix2Cart(self.globalpp.push)
                else:
                    global_path = True
                    self.path = [[0.0,0.0],[0.0,0.0]]
                    
                   

                if global_path == True:
            
                    #the path and bearing have been made properly
                    #locally path planning to the start if im not near it
                    #creating the pathfinding object to intialise

                    if self.Reset == True:
                        x =  int(((0.0 - -1.5)/3) * 600)
                        y = 600-int(((0.0  - - 1.5)/3) * 600)
                        self.globalpp.push = [[0]]
                        self.globalpp.push[0] = [x,y]
                        print(self.globalpp.push)
                    local_path = self.globalpp.Nav2Start(self.X,self.Y,self.green_boxes,self.red_boxes)
                    self.path2Start = self.globalpp.Pix2Cart(self.globalpp.start)
                    
                

                else:
                    

                    if self.selecBox[0] not in self.inv_Box:
                        self.inv_Box.append(self.selecBox[0])
                    self.selecBox[0] = None
                    local_path = False

                                        
                if local_path ==True and self.selecBox[0] !=self.prev_box:
                   
                    #getting and setting the initial pose
                    
                    if self.count1 <=1:
                        quaternion =self.eularToQuaternion(math.radians(self.yaw))
                        initial_pose = PoseStamped()
                        initial_pose.header.frame_id = 'map'
                        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                        initial_pose.pose.position.x = self.X
                        initial_pose.pose.position.y = self.Y
                        initial_pose.pose.orientation.z = quaternion[2]
                        initial_pose.pose.orientation.w = quaternion[3]

                        self.navigator.setInitialPose(initial_pose)
                        self.count1+=1
                            
                    # Wait for navigation to fully activate, since autostarting nav2
                    self.navigator.waitUntilNav2Active()
                        
                    #calcualting a comprehensive movement plan to push the boxes eg
                    goal_poses = self.StartCompCreate()

                    
                    # sanity check a valid path exists
                    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
                    if len(goal_poses)>=1 and goal_poses != self.prev_2path and goal_poses not in self.path_coll:
                        for goal in goal_poses:
                            self.navigator.goToPose(goal)

                            i=0
                            while not self.navigator.isTaskComplete():
                                i += 1
                                feedback = self.navigator.getFeedback()
                                if feedback and i % 5 == 0:
                                        

                                    # Some failure mode, must stop since the robot is clearly stuck
                                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                                        self.navigator.cancelTask()

                        # Do something depending on the return code
                        result = self.navigator.getResult()
                        if result == TaskResult.SUCCEEDED:
                            print('Starting location met')
                            self.to_start =True
                            self.prev_2path = goal_poses
                            
                            if self.Reset == True:
                                print("Resetting")
                                #publish memory reset and behaviour response
                               
                                self.box2send.data = [-1.0]
                                self.box_id.publish(self.box2send)
                                
                                self.selecBox[0] = None
                                
                                self.path_made = False
                                self.to_start =False
                                self.count = 0

                                self.release =True
                                self.box_coll = []
                                self.path_coll = []
                                
                                #self.Reset = False
                            if self.box_move ==False and self.selecBox[0]!=self.prev_box and self.Reset ==False:
                                self.path_coll.append(goal_poses)
                                print("Moving box")
                                            
                                # Wait for navigation to fully activate, since autostarting nav2
                                self.navigator.waitUntilNav2Active()
                                        
                                #calcualting a comprehensive movement plan to push the boxes eg
                                box_poses = self.BoxPathCreate()
                                    


                                # sanity check a valid path exists
                                # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
                                if self.prev_path != box_poses and box_poses not in self.path_coll:
                                    for box in box_poses:  
                                        self.navigator.goToPose(box)
                                        i=0
                                        while not self.navigator.isTaskComplete():
                                            i += 1
                                            feedback = self.navigator.getFeedback()
                                            if feedback and i % 5 == 0:
                                                        

                                                # Some failure mode, must stop since the robot is clearly stuck
                                                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
                                                    print('Navigation has exceeded timeout of 120s, canceling request.')
                                                    self.navigator.cancelTask()

                                                # Do something depending on the return code
                                        result = self.navigator.getResult()
                                        if result == TaskResult.SUCCEEDED:
                                            
                                                
                                            if box ==box_poses[-1] and self.Reset == False:
                                                print('Box moved!')

                                                # self.box2send.data = [self.selecBox[0][0]]
                                                # self.box_id.publish(self.box2send)

                                                self.prev_box = self.selecBox[0]
                                                self.prev_path= box_poses

                                                self.box_coll.append(self.selecBox[0])
                                                self.path_coll.append(box_poses)  

                                                self.selecBox[0] = None
                                                
                                                self.path_made = False
                                                self.to_start =False    
                                                        

                                        elif result == TaskResult.CANCELED:
                                            print('Goal was canceled!')
                                        elif result == TaskResult.FAILED:
                                            print('Goal failed!')
                                        else:
                                            print('Goal has an invalid return status!')
                                            
                                            print("Box moved to the wall!")
                                            

                        elif result == TaskResult.CANCELED:
                            print('Goal was canceled!')
                        elif result == TaskResult.FAILED:
                            print('Goal failed!')
                        else:
                            print('Goal has an invalid return status!')
                    
                else:
                    
                    if self.selecBox[0]!= None and self.selecBox[0] not in self.inv_Box:
                        self.inv_Box.append(self.selecBox[0])
                    self.selecBox[0]=None

            if self.release == True:
                self.turn=True
                self.response[0]=2.0
               
                self.brain_response.data = self.response
                self.responder.publish(self.brain_response)
                self.count = 0
                self.count = 0
                self.count1 =0

        
            
            

    #updates what values that should be published 
    def SetMove(self):
        #updates the message to be published
        self.move_cmd.linear.x = self.lin_vel 
        self.move_cmd.angular.z = self.ang_vel
        self.cmd_vel_pub.publish(self.move_cmd)

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
                self.wallsInfo =[0.0]*4
                self.count1+=1
                
            else:
                #announcing the change in time
                self.elapsed_time = time.time() - self.start_time
                
                #if the flag is greater than 5 seconds and then checks if the yaw is back to the origin
                if self.elapsed_time >= 5 and abs(self.yaw - self.start_yaw) <= 0.5:
                    #if the yaw and start yaw are equal the robot is stopped from "scanning"
                    print("Scan complete")
                    self.ang_vel = 0.0
                    self.lin_vel = 0.0
                    self.SetMove()
                    self.count+=1
                    self.turn = False
                   
                    ##returning confirmation that the action has been completed
                    self.response[0] = 1.0
                    self.brain_response.data = self.response
                    self.responder.publish(self.brain_response)
                    
                    
        if self.turn == True:
            self.ang_vel = 0.3
            self.lin_vel = 0.0
        self.SetMove()