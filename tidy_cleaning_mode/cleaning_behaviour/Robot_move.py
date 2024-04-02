#!/usr/bin/env python

# Written for humble

#ROS imports 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

#PYTHON imports
import numpy as np
from math import radians
import math
import time

#custom import
import local_pathfinding as lp
import global_pathplanning as gp

class Move(Node):
    #used for RawScancallback
    Xs=[]
    Ys=[]
    RawRanges = []
    lin_vel=0.0
    ang_vel=0.0


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

    #boxes to move
    green_boxes = []
    red_boxes = []
    selecBox = [None]
    boxIndex = 0

    isgreen = False
    selecMove =False

    ##what walls to move the boxes at
    green_wall = []
    red_red = []


    #path finding
    path_made = False
    offCourse = False
    path = [None]
    bearings = [None]
    path2Start= []
    path2index = 0
    pathIndex = 0
    
    count = 0
    path_count = 0
    
    to_start = False
    box_move =False

    prev_box = []
    box_coll = []
    prev_2path = []
    prev_path = []
    path_coll = []
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
        ##subscriber to read the custom scan data
        self.range_get = self.create_subscription(LaserScan,"/scan",self.RawScancallback, 1)
        #subscriber for the custom odometry
        self.pos_get = self.create_subscription(Float32MultiArray,'/Compressed_Odom',self.OdomCallback,1)
        #subscribing to the box topic to obtain the boxes
        self.box_get = self.create_subscription(Float32MultiArray,'/BoxInfo',self.BoxCallback,1)

        timer_period = 0.01  # seconds
        self.updater = self.create_timer(timer_period, self.Broadcast)
        #calling the custom global planner
        self.globalpp =gp.PathPlanning()
        #navigator for nav2
        self.navigator = BasicNavigator()

    def RawScancallback(self,data):
        ##obtaining the scan data
       
        #doing some basic filtering for the lidar data
        actual_ranges = []
        for range in data.ranges:
            if range != 0.0:
                actual_ranges.append(range)
        if len(actual_ranges)%2 !=0:
            actual_ranges =actual_ranges[1:]
        self.RawRanges = actual_ranges
        self.Xs = []
        self.Ys = []
        for i, length in enumerate(self.RawRanges):
            angle = data.angle_min + i * data.angle_increment
            
            ##plotting the obstacles
            x = self.X+ ((length+0.15) * math.cos(angle + math.radians(self.yaw)))
            y = self.Y+ ((length+0.15) * math.sin(angle + math.radians(self.yaw)))

            self.Xs.append(x)
            self.Ys.append(y)
        self.globalpp.Xs = self.Xs
        self.globalpp.Ys = self.Ys

    def Broadcast(self):
        if self.turn ==False:

            ##returning confirmation that the action has been completed
            #self.response[0] = 1.0
            self.brain_response.data = self.response
            self.responder.publish(self.brain_response)
        if self.state ==1.0 and self.selecBox[0] != None:
            if self.box_move ==True:
                #self.box2send.data = [self.selecBox[0][0]]
                #self.box_id.publish(self.box2send)
                self.selecBox[0] = None
                self.box_move = False
                self.path_made = False
                self.to_start =False

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
                #they must be red so
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
            self.turn = True
            self.BoxMove()
    def StartCompCreate(self):
        #used to feed to nav2 to get to the start collision free hopefully
        waypoints = []
        print("START PATH ",self.path2Start)
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
                    goal_pose1.pose.position.x = self.path2Start[i][0]
                    goal_pose1.pose.position.y = self.path2Start[i][1]
                    goal_pose1.pose.orientation.w = quaternion[3]
                    goal_pose1.pose.orientation.z = quaternion[2]
                    waypoints.append(goal_pose1)
                else:
                    changeY = self.path[0][1] - self.bearings[0][1]
                    changeX = self.path[0][0] - self.bearings[0][0]
                    yaw = math.atan2(changeY,changeX)
                    quaternion = self.eularToQuaternion(yaw)
                    goal_pose1 = PoseStamped()
                    goal_pose1.header.frame_id = 'map'
                    goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()##############
                    goal_pose1.pose.position.x = self.bearings[0][0]
                    goal_pose1.pose.position.y = self.bearings[0][1]
                    goal_pose1.pose.orientation.w = quaternion[3]
                    goal_pose1.pose.orientation.z = quaternion[2]
                    waypoints.append(goal_pose1)
            return waypoints
    
    def BoxPathCreate(self):
        box_path=[]
        for i in range(len(self.path)):
            #as we should be a the bearing point at the start then we start with the path point
            if i+1<len(self.path) and i+1<len(self.bearings):
                #but path points are an aprox where the box should be so we need to calculate a relative displacement
                heading = math.atan2(self.bearings[i][1]-self.path[i][1],self.bearings[i][0]-self.path[i][0])
                px = self.path[i][0] + (0.17*math.cos(heading))
                py = self.path[i][1]+(0.17*math.sin(heading))

                changeY = self.bearings[i+1][1] - py
                changeX = self.bearings[i+1][0] - px
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

                changeY =  self.path[i+1][1]- self.bearings[i+1][1]
                changeX =  self.path[i+1][0]- self.bearings[i+1][0]
                yaw = math.atan2(changeY,changeX)
                quaternion = self.eularToQuaternion(yaw)
                goal_pose1 = PoseStamped()
                goal_pose1.header.frame_id = 'map'
                goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()######################
                goal_pose1.pose.position.x = self.bearings[i+1][0]
                goal_pose1.pose.position.y = self.bearings[i+1][1]
                goal_pose1.pose.orientation.w = quaternion[3]
                goal_pose1.pose.orientation.z = quaternion[2]
                box_path.append(goal_pose1)
            else:
                #but path points are an aprox where the box should be so we need to calculate a relative displacement
                heading = math.atan2(self.bearings[-1][1]-self.path[-1][1],self.bearings[-1][0]-self.path[-1][0])
                px = self.path[-1][0] + (0.10*math.cos(heading))
                py = self.path[-1][1]+(0.10*math.sin(heading))
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

           


        # goal_pose2 = PoseStamped()
        # goal_pose2.header.frame_id = 'map'
        # goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
        # goal_pose2.pose.position.x = 0.0
        # goal_pose2.pose.position.y = 0.0
        # box_path.append(goal_pose2)

        return(box_path)  
    def eularToQuaternion(self,yaw):
        roll = 0.0
        pitch = 0.0
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  
        return qx, qy, qz, qw

    def BoxMove(self):
        #method used to control and move the boxes in a category
        #checking which box type has more of them
        # if len(self.box_coll) == (len(self.green_boxes)+len(self.red_boxes)):
        #     del self.box_coll[-1]
        #     del self.path_coll[len(self.path_coll-2):]

        for box in self.box_coll:
            for gbox in self.green_boxes:

                if box[0] == gbox[0]:
                    box_index = self.green_boxes.index(box)
                    del self.green_boxes[box_index]
            for rbox in self.red_boxes:

                if box[0] == rbox[0]:
                    box_index = self.red_boxes.index(box)
                    del self.red_boxes[box_index]
                    
        
        #checking whether or not a box has been chosen
        #print(len(self.red_boxes)," ",len(self.green_boxes)," ",self.boxIndex)
        if self.selecBox[0] == None:
            #print(len(self.green_boxes)," ",len(self.red_boxes))
            if len(self.green_boxes)>len(self.red_boxes):
                self.isgreen =True
            else:
                self.isgreen = False
            
            #now we will select the box in the order they were found
            if self.isgreen == True:
                if len(self.green_boxes)>self.boxIndex:
                    self.selecMove = True
                    self.selecBox[0] = self.green_boxes[self.boxIndex]
                
            else:
                if len(self.red_boxes)>self.boxIndex:
                    self.selecMove = True
                    self.selecBox[0] = self.red_boxes[self.boxIndex]
            if self.count1 != 0:
                #this is were we will update the path finding
                pass
            else:
                #this must be the first time the code was ran and thus a bo must be selected
                self.count1+=1
            
            
        else:
           
            #setting up the pathfinding with the current target and wall
            self.globalpp.X = self.X
            self.globalpp.Y=self.Y
            self.globalpp.yaw = self.yaw
            self.globalpp.ranges = self.RawRanges
            self.globalpp.green_wall = self.green_wall
            self.globalpp.red_wall = self.red_wall

            if self.selecBox[0][1] == 1.0:
                self.globalpp.target_wall = self.green_wall
                
            else:
                self.globalpp.target_wall = self.red_wall
            self.globalpp.target_box = self.selecBox[0]
            #calling the pathfinding to contruct a path
            self.globalpp.PathPlan()
            self.bearings = self.globalpp.bearing
            self.path = self.globalpp.path
            




            if self.failed ==False and self.path_made == False:
                if self.path[0] != -1.0 :
                    #the path and bearing have been made properly
                    #locally path planning to the start if im not near it
                    print("Path created, planning to get to start position")

                    #creating the pathfinding object to intialise
                    self.localP = lp.LocalPathFinding(self.green_boxes,self.red_boxes,self.bearings,self.selecBox[0],self.ranges,self.X,self.Y,self.yaw)
                    
                    child = [self.X,self.Y]
                   
                    goal =self.bearings[0]
                    self.path2Start = []
                    #print(goal)
                    #print(type(goal)," ",type(self.selecBox[0]))
                    self.count = 0
                    if goal!=None:
                        while(child!=goal):
                            #looping through the positions until we meet the goal if possible

                            #getting the positions to move to
                            best_move= self.localP.LocalPathPlanning(child,goal,0.15)
                            
                            if best_move == child:
                                print("cannot reach the start")
                            
                                #signifying failure to the move node to get a new box
                                self.path_made = False
                                self.failed = True
                                
                                break
                            else:
                            
                                ##adding the child to the path to move on
                                self.path2Start.append(best_move)
                                child = best_move
                                self.path_made = True
                                self.failed = False
                        if self.failed == False:
                            self.path2Start = self.localP.EstiProjec(self.path2Start)

                else:
                    
                    if self.path[0] == -1.0 and self.path[1] == self.selecBox[0][0]:
                        print("global pathplanning failed ",self.path[1]," ",self.selecBox[0][0])
                        if self.isgreen == True:
                            if self.boxIndex<len(self.green_boxes)-1:
                                self.boxIndex+=1
                                self.selecBox[0] = self.green_boxes[self.boxIndex]
                            else:
                                if len(self.red_boxes)>0:
                                    self.boxIndex = 0
                                    self.isgreen=False
                                    self.selecBox[0] = self.red_boxes[self.boxIndex]
                        else:
                            if self.boxIndex<len(self.red_boxes)-1:
                                self.boxIndex+=1
                                self.selecBox[0] = self.red_boxes[self.boxIndex]
                            else:
                                if len(self.green_boxes)>0:
                                    self.boxIndex = 0
                                    self.isgreen = True
                                    self.selecBox[0] = self.green_boxes[self.boxIndex]

            
            #print(self.path_made)
                                    
            if self.path_made ==True and self.selecBox[0] !=self.prev_box:
                
                #getting and setting the initial pose
                if self.to_start ==False:
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
                                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                                        print('Navigation has exceeded timeout of 180s, canceling request.')
                                        self.navigator.cancelTask()
                            # # Some navigation timeout to demo cancellation
                            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                            #     self.navigator.cancelTask()

                            # Do something depending on the return code
                            result = self.navigator.getResult()
                            if result == TaskResult.SUCCEEDED:
                                print('Goal succeeded!')
                                self.to_start =True
                                self.prev_2path = goal_poses
                                self.path_coll.append(goal_poses)
                                #self.navigator.cancelTask()

                            elif result == TaskResult.CANCELED:
                                print('Goal was canceled!')
                            elif result == TaskResult.FAILED:
                                print('Goal failed!')
                            else:
                                print('Goal has an invalid return status!')
                else:
                    if self.box_move ==False and len(self.bearings)>0 and self.selecBox[0]!=self.prev_box:
                        print("Build the code you lazy sod")
                            
                        # Wait for navigation to fully activate, since autostarting nav2
                        self.navigator.waitUntilNav2Active()
                        
                        #calcualting a comprehensive movement plan to push the boxes eg
                        box_poses = self.BoxPathCreate()
                       


                        # sanity check a valid path exists
                        # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
                        if self.prev_path != box_poses and box_poses not in self.path_coll:   
                            self.navigator.goThroughPoses(box_poses)
                            i=0
                            while not self.navigator.isTaskComplete():
                                i += 1
                                feedback = self.navigator.getFeedback()
                                if feedback and i % 5 == 0:
                                    

                                    # Some failure mode, must stop since the robot is clearly stuck
                                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                                        print('Navigation has exceeded timeout of 180s, canceling request.')
                                        self.navigator.cancelTask()
                            # # Some navigation timeout to demo cancellation
                            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                            #     self.navigator.cancelTask()

                            # Do something depending on the return code
                            result = self.navigator.getResult()
                            if result == TaskResult.SUCCEEDED:
                                print('Goal succeeded!')
                                self.prev_box = self.selecBox[0]
                                self.box_move =True
                                self.prev_path= box_poses
                                self.box_coll.append(self.selecBox[0])
                                self.path_coll.append(box_poses)    
                                #self.navigator.cancelTask()    
                                    

                            elif result == TaskResult.CANCELED:
                                print('Goal was canceled!')
                            elif result == TaskResult.FAILED:
                                print('Goal failed!')
                            else:
                                print('Goal has an invalid return status!')
                        
                                print("Box moved to the wall!")
                            # if self.box_move ==True:
                            #     self.box2send.data = [self.selecBox[0][0]]
                            #     self.box_id.publish(self.box2send)
                            #     self.selecBox[0] = None
                            #     self.box_move = False
                                
                            # # self.box2send.data = [self.selecBox[0][0]]
                            # # self.box_id.publish(self.box2send)
                            # # self.selecBox[0] = None
            
                        
                        

                            
                            
                    
                        
                
               
            else:
                
                if self.path[0] == -1.0 and self.path[1] == self.selecBox[0][0]:
                    #they havent been generated time to move the index and change the box
                    print("Local pathplanning failed ",self.path[1]," ",self.selecBox[0][0])
                    if self.path[0] == -1.0 and self.path[1] == self.selecBox[0][0]:
                        if self.isgreen == True:
                            if self.boxIndex<len(self.green_boxes)-1:
                                self.boxIndex+=1
                                self.selecBox[0] = self.green_boxes[self.boxIndex]
                            else:
                                if len(self.red_boxes)>0:
                                    self.boxIndex = 0
                                    self.isgreen=False
                                    self.selecBox[0] = self.red_boxes[self.boxIndex]
                        else:
                            if self.boxIndex<len(self.red_boxes)-1:
                                self.boxIndex+=1
                                self.selecBox[0] = self.red_boxes[self.boxIndex]
                            else:
                                if len(self.green_boxes)>0:
                                    self.boxIndex = 0
                                    self.isgreen = True
                                    self.selecBox[0] = self.green_boxes[self.boxIndex]
                else:
                    print("BOX HASNT BEEN SWITCHED")
                    if self.selecBox[0] ==self.prev_box and self.cur_box == self.goal_pos:
                        self.box_move =True
                  



   
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

   

            
