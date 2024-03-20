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

class Move(Node):
    lin_vel = 0.0
    ang_vel = 0.0

    headingSet = False
    target_heading = -45.0
    wp_heading = 0.0

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
        #publisher broadcasting the box that a path should be planned for
        self.target_box = self.create_publisher(Float32MultiArray,'/Box_Target',60)
        self.box2Target  = Float32MultiArray()

        

        ##setting up a subscriber to read brain commands
        self.beh_get = self.create_subscription(Float32MultiArray,'/Conductor',self.CommandCallback,1)

        ##subscriber to read the custom scan data
        self.range_get = self.create_subscription(Float32MultiArray,'/Range_Pub',self.ScanCallback,1)

        #subscriber for the custom odometry
        self.pos_get = self.create_subscription(Float32MultiArray,'/Compressed_Odom',self.OdomCallback,1)
        #subscribing to the box topic to obtain the boxes
        self.box_get = self.create_subscription(Float32MultiArray,'/BoxInfo',self.BoxCallback,1)

        ##subscribing to the path planning based topics
        self.get_bearings = self.create_subscription(Float32MultiArray,'/Box_bearings',self.RoboPos,1)
        self.get_path = self.create_subscription(Float32MultiArray,'/PathFinding',self.GetPlan,1)

        timer_period = 0.01  # seconds
        self.updater = self.create_timer(timer_period, self.Broadcast)

        #navigator for nav2
        self.navigator = BasicNavigator()


    def GetPlan(self, path):
        self.path = []
        if path.data[0] ==-1.0:
            self.path = path.data
        else:
            Node = []
            for i in range(0,len(path.data), 2):
                for x in range(0,2):
                    Node.append(path.data[i+x])
                self.path.append(Node)
                Node = []
    def RoboPos(self,bearings):
        self.bearings = []
        if bearings.data[0] ==-1.0:
            self.bearings=  bearings.data
        else:
            bearing = []
            for i in range(0,len(bearings.data), 2):
                for x in range(0,2):
                    bearing.append(bearings.data[i+x])
                self.bearings.append(bearing)
                bearing = []

    def Broadcast(self):
        if self.turn ==False:

            ##returning confirmation that the action has been completed
            #self.response[0] = 1.0
            self.brain_response.data = self.response
            self.responder.publish(self.brain_response)
        if self.state ==1.0 and self.selecBox[0] != None:
            self.box2Target.data =self.selecBox[0]
            self.target_box.publish(self.box2Target)

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
        elif self.state == 2.0:
            #activating patrol mode to validate and check boxes
            self.ObsAvoid()
            self.SetMove()

    def LocalPathPlanning(self,cur,goal,gran):
        #this method will be used to allow to plan its path to begin the predetermined path

        #finding the walls
        
        self.Xs = []
        self.Ys = []
        
        
        angles =[-3.14,-1.57,0.0,1.57,3.14]
        children = []
        for i, length in enumerate(self.ranges):
            angle = -2.0 + i * 0.013745704665780067
            
            ##plotting the obstacles
            x = self.X+ (length * math.cos(angle + math.radians(self.yaw)))
            y = self.Y+ (length * math.sin(angle + math.radians(self.yaw)))

            self.Xs.append(x)
            self.Ys.append(y)
        
        
        #producing the locations the child could move to and checking their validity
        self.count+=1
        for a in range(0,360):
            cx = cur[0]+ (gran * math.cos(a))
            cy = cur[1]+ (gran * math.sin(a))
            #calculating where the robt must push from to reach each point

            #checking if the move is valid of not
            #checking near current lidar data
            valid = True
            
            if [cx,cy] == goal or math.dist(goal,[cx,cy])<=0.1:
                return goal
            
            for b in range(len(self.Xs)):
                if math.dist([self.Xs[b],self.Ys[b]],[cx,cy])<=0.25:
                    valid = False
                    break
                else:
                    valid = True
            
            #checking near other boxes
            if valid ==True:
                for green_box in self.green_boxes:
                    
                    if(green_box !=self.selecBox[0]):
                        
                        if abs(green_box[2] - cx) + abs(green_box[3] - cy)<=0.25:
                            valid = False
                            break
                        else:
                            valid = True
                   
            if valid ==True:
                for red_box in self.red_boxes:
                    if(red_box !=self.selecBox[0]):
                        if abs(red_box[2] - cx) + abs(red_box[3] - cy)<=0.25:
                            valid = False
                            break
                        else:
                            valid = True
                   
            if valid == True:
                #calculating where the robot will have to push from to achieve the move
                children.append([cx,cy])
        #now we have all the possible moves its time to decide which one is best
        child_cost = []
        if len(children) ==0:
            #print("There arent any valid moves")
            return cur
        for child in children:
            child_cost.append(math.dist(goal,child))
        
        #self.bearing[child_cost.index(min(child_cost))]
        return children[child_cost.index(min(child_cost))]
    
    def Displacement(self):
        #as the path planning is based on the cube being moved and not the robot it self we can calculate a approximate displacement to move by
        if self.pathIndex != 0:
            
            return abs(self.waypoint[0] - self.X) + abs(self.waypoint[1] - self.Y)#math.dist(self.waypoint,self.path[self.pathIndex-1])
    def BoxPathCreate(self):
        #used to to create the complete path taking into account displacement of the boxes to feed into nav2
        box_path = []
        

    def StartCompCreate(self):
        #used to feed to nav2 to get to the start collision free hopefully
        waypoints = []
        for i in range(len(self.path2Start)):
            #time to convert the steps into quaternion with a goal orientation
            if i+1< len(self.path2Start):
                changeY = self.path2Start[i+1][1] - self.path2Start[i][1]
                changeX = self.path2Start[i+1][0] - self.path2Start[i][0]
                yaw = math.atan2(changeY,changeX)
                quaternion = self.eularToQuaternion(yaw)
                goal_pose1 = PoseStamped()
                goal_pose1.header.frame_id = 'map'
                goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose1.pose.position.x = self.path2Start[i][0]
                goal_pose1.pose.position.y = self.path2Start[i][1]
                #goal_pose1.pose.orientation.w = quaternion[3]
                #goal_pose1.pose.orientation.z = quaternion[2]
                waypoints.append(goal_pose1)
            else:
                changeY = self.path[0][1] - self.bearings[0][1]
                changeX = self.path[0][0] - self.bearings[0][0]
                yaw = math.atan2(changeY,changeX)
                quaternion = self.eularToQuaternion(yaw)
                goal_pose1 = PoseStamped()
                goal_pose1.header.frame_id = 'map'
                goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose1.pose.position.x = self.bearings[0][0]
                goal_pose1.pose.position.y = self.bearings[0][1]
                goal_pose1.pose.orientation.w = quaternion[3]
                goal_pose1.pose.orientation.z = quaternion[2]
                waypoints.append(goal_pose1)
        return waypoints




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
                if len(self.red_boxes)-1>self.boxIndex:
                    self.selecMove = True
                    self.selecBox[0] = self.red_boxes[self.boxIndex]
            
            
        else:
            #print(self.isgreen," ",self.selecBox)
            #checking whether or not the path and bearing have been created properly
            
            if len(self.path)>0 and len(self.bearings)>0 and len(self.path2Start)==0:
                if self.path[0] != -1.0 :
                    #the path and bearing have been made properly
                    #locally path planning to the start if im not near it
                    print("Path created, planning to get to start position")
                    
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
                            best_move= self.LocalPathPlanning(child,goal,0.1)
                            
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

            
            #print(path_made)
                                    
            if self.path_made ==False:
                
                #getting and setting the initial pose
                if self.to_start ==False:
                    quaternion =self.eularToQuaternion(math.radians(self.yaw))
                    initial_pose = PoseStamped()
                    initial_pose.header.frame_id = 'map'
                    initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    initial_pose.pose.position.x = self.X
                    initial_pose.pose.position.y = self.Y
                    initial_pose.pose.orientation.z = quaternion[2]
                    initial_pose.pose.orientation.w = quaternion[3]

                    self.navigator.setInitialPose(initial_pose)
                        
                    # Wait for navigation to fully activate, since autostarting nav2
                    self.navigator.waitUntilNav2Active()
                    
                    #calcualting a comprehensive movement plan to push the boxes eg
                    goal_poses = self.StartCompCreate()

                
                    # sanity check a valid path exists
                    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

                    self.navigator.goThroughPoses(goal_poses)

                    i = 0
                    while not self.navigator.isTaskComplete():
                       
                        i = i + 1
                        feedback = self.navigator.getFeedback()
                        if feedback and i % 5 == 0:
                            print(
                                'Estimated time of arrival: '
                                + '{0:.0f}'.format(
                                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                                    / 1e9
                                )
                                + ' seconds.'
                            )

                            # Some navigation timeout to demo cancellation
                            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                                self.navigator.cancelTask()

                    # Do something depending on the return code
                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        print('Goal succeeded!')
                        self.to_start =True

                    elif result == TaskResult.CANCELED:
                        print('Goal was canceled!')
                    elif result == TaskResult.FAILED:
                        print('Goal failed!')
                    else:
                        print('Goal has an invalid return status!')
                else:
                    if self.box_move ==False:
                        quaternion =self.eularToQuaternion(math.radians(self.yaw))
                        initial_pose = PoseStamped()
                        initial_pose.header.frame_id = 'map'
                        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                        initial_pose.pose.position.x = self.X
                        initial_pose.pose.position.y = self.Y
                        initial_pose.pose.orientation.z = quaternion[2]
                        initial_pose.pose.orientation.w = quaternion[3]

                        self.navigator.setInitialPose(initial_pose)
                            
                        # Wait for navigation to fully activate, since autostarting nav2
                        self.navigator.waitUntilNav2Active()
                        
                        #calcualting a comprehensive movement plan to push the boxes eg
                        box_poses = self.BoxPathCreate()

                    
                        # sanity check a valid path exists
                        # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

                        self.navigator.goThroughPoses(box_poses)

                        i = 0
                        while not self.navigator.isTaskComplete():
                        
                            i = i + 1
                            feedback = self.navigator.getFeedback()
                            if feedback and i % 5 == 0:
                                print(
                                    'Estimated time of arrival: '
                                    + '{0:.0f}'.format(
                                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                                        / 1e9
                                    )
                                    + ' seconds.'
                                )

                                # Some navigation timeout to demo cancellation
                                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                                    self.navigator.cancelTask()

                        # Do something depending on the return code
                        result = self.navigator.getResult()
                        if result == TaskResult.SUCCEEDED:
                            print('Goal succeeded!')
                            self.box_move =True

                        elif result == TaskResult.CANCELED:
                            print('Goal was canceled!')
                        elif result == TaskResult.FAILED:
                            print('Goal failed!')
                        else:
                            print('Goal has an invalid return status!')
                
                            
                            
                    
                        
                
               
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





            





    def BoxCheck(self):
        if self.ranges[int(len(self.ranges)/2)]<=0.25:
            print("Box has been moved")
            self.box2send.data = [self.selecBox[0][0]]
            self.box_id.publish(self.box2send)
            self.lin_vel =0.0
            self.ang_vel=0.0
            if self.isgreen ==True and self.boxIndex< len(self.green_boxes)-1:
                #self.boxIndex+=1
                self.selecBox[0] = self.green_boxes[self.boxIndex]
                self.headingSet = False
           
            elif self.isgreen ==False and self.boxIndex<len(self.red_boxes)-1:
                #self.boxIndex+=1
                self.selecBox[0] = self.red_boxes[self.boxIndex]
                self.headingSet = False
            else:
                ##stop the program or use a protocol to check for more boxes
                print("No more boxes have been detected")
                ##returning confirmation that the action has been completed
                self.response[0] = 2.0
                self.selecBox[0] = None
                self.brain_response.data = self.response
                self.responder.publish(self.brain_response)
                    

    #updates what values that should be published 
    def SetMove(self):
        #updates the message to be published
        self.move_cmd.linear.x = self.lin_vel 
        self.move_cmd.angular.z = self.ang_vel
        self.cmd_vel_pub.publish(self.move_cmd)

    def SetHeading(self,bearing):
        #angle = atan2(y_change,x_change)
       
        if bearing == True:

            heading = (self.target_heading)-radians(self.yaw)
            x= np.array([heading])
            xwrap = np.remainder(x,2*np.pi)
            mask = np.abs(xwrap)>np.pi
            xwrap[mask] -=2*np.pi *np.sign(xwrap[mask])
        else:
            heading = (self.wp_heading)-radians(self.yaw)
            x= np.array([heading])
            xwrap = np.remainder(x,2*np.pi)
            mask = np.abs(xwrap)>np.pi
            xwrap[mask] -=2*np.pi *np.sign(xwrap[mask])

        self.ang_vel = 0.5*xwrap[0]


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

   

            
