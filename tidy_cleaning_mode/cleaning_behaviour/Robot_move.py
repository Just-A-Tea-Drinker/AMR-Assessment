#!/usr/bin/env python

# Written for humble

#ROS imports 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray,Float32MultiArray


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

    #bool flags to control the movement in general
    wp_reach =False
    goal_reach = False

    #path finding
    path_made = False
    offCourse = False
    path = [None]
    bearings = [None]
    path2Start= []
    path2index = 0
    pathIndex = 0
    
    count = 0
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
                    print(goal)
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
                                    
            if self.path_made ==True:
                ##CONTROLLING THE MOVEMENT HERE!!!!
                
               
                self.headingSet = False
                #print(self.waypoint," ",self.wp_heading," ",self.target_heading)
               
                    #this indicates that a path must be followed to reach the box
                    #setting the target and index
                self.waypoint = self.path2Start[self.path2index]
                    #self.target = self.path2Start[-1]

                    #lets calculate the heading
                self.wp_heading = math.atan2((self.waypoint[1])-self.Y,(self.waypoint[0])-self.X)
               
                if self.wp_reach == False:
                    print(self.path2Start)
                    heading = (self.wp_heading)-radians(self.yaw)
                    x= np.array([heading])
                    xwrap = np.remainder(x,2*np.pi)
                    mask = np.abs(xwrap)>np.pi
                    xwrap[mask] -=2*np.pi *np.sign(xwrap[mask])
                    #checking whether or not im at the check point
                        
                    if xwrap[0]<0.1 and xwrap[0]>-0.1:
                        self.headingSet = True
                    else:
                        self.headingSet = False

                    if math.dist(self.waypoint,[self.X,self.Y])<0.1:
                        #checkpoint has been reached
                        print("Waypoint reached!")
                        self.offCourse = False
                        self.ang_vel = 0.0
                        self.lin_vel= 0.0
                        self.SetMove()
                        if self.path2Start[-1]!=self.path2Start[self.path2index]:
                            self.path2index+=1
                            self.wp_reach = False
                        else:

                            self.wp_reach = True
                            self.path2index= 0
                        self.headingSet = False
                    if math.dist(self.waypoint,[self.X,self.Y])>0.4:
                                
                        self.offCourse = True
                        self.ang_vel = 0.0
                        self.lin_vel= 0.0
                        self.SetMove()
                    else:
                        self.offCourse = False
                        
                        
                else:
                    #the robot has reached the checkpoint and now i need to get to the goal
                    if len(self.path)>self.pathIndex:
                       
                        #
                        self.headingSet = False
                        if self.goal_reach == False and self.path[0] !=-1.0:
                            self.waypoint = self.bearings[self.pathIndex]
                            #print(self.path)
                            #lets calculate the heading
                            if self.offCourse == True:
                                if len(self.path)>self.pathIndex+1:
                                    self.target_heading = math.atan2((self.waypoint[1]*10)-(self.Y),(self.waypoint[0]*10)-(self.X))
                                else:
                                    self.target_heading = math.atan2((self.waypoint[1])-(self.bearings[self.pathIndex+1][0]),(self.waypoint[0])-(self.bearings[self.pathIndex+1][1]))
                            else:
                                self.target_heading = math.atan2((self.waypoint[1])-(self.Y),(self.waypoint[0])-(self.X))
                            heading2 = (self.target_heading)-radians(self.yaw)
                            x2= np.array([heading2])
                            xwrap2 = np.remainder(x2,2*np.pi)
                            mask2 = np.abs(xwrap2)>np.pi
                            xwrap2[mask2] -=2*np.pi *np.sign(xwrap2[mask2])
                            if xwrap2[0]<0.1 and xwrap2[0]>-0.1 :
                                print("moving to target")
                                #if math.dist(self.waypoint,[self.X,self.Y])>0.05:
                                self.headingSet = True
                            else:
                                self.headingSet = False

                            
                            
                                
                           
                                
                               
                            #calculating the potential displacement as the path planning is based on the box not the robot it self
                            dist = self.Displacement()
                            if math.dist(self.waypoint,[self.X,self.Y])>0.4:
                                
                                self.offCourse = True
                                self.ang_vel = 0.0
                                self.lin_vel= 0.0
                                self.SetMove()
                            else:
                                self.offCourse = False

                            
                            if math.dist(self.bearings[self.pathIndex],[self.X,self.Y])==0.1:
                                self.ang_vel = 0.0
                                self.lin_vel= 0.0
                                self.SetMove()
                                if self.bearings[-1]!=self.bearings[self.pathIndex]:
                                    self.pathIndex+=1
                                    self.target_reach = False
                                # else:

                                #     self.goal_reach_reach = True
                                #     self.path2index= 0
                                    
                                
                            if self.ranges[int(len(self.ranges)/2)]<=0.25 and self.bearings[-1]==self.bearings[self.pathIndex] :
                                print("Box moved to the wall!")
                                self.ang_vel = 0.0
                                self.lin_vel= 0.0
                                self.goal_reach = True
                                self.box2send.data = [self.selecBox[0][0]]
                                self.box_id.publish(self.box2send)
                            else:
                                if self.ranges[int(len(self.ranges)/2)]<=0.25 and math.dist(self.bearings[self.pathIndex-1],[self.X,self.Y])>0.15:
                                    print("obstacle encountered")
                                    self.headingSet = False

                            
                            
                    
                        
                
                ##Actually moving or deciphering a move
                if self.headingSet ==True and (self.goal_reach == False ):
                    self.lin_vel = 0.1
                    self.ang_vel = 0.0
                else:
                    self.lin_vel = 0.0
                    if self.wp_reach ==False:
                        self.SetHeading(False)
                    else:
                        if self.goal_reach==False:
                            self.SetHeading(True)
                self.SetMove()

                    # if self.goal_reach ==True:
                    #     self.selecBox[0] = None
                    #     self.goal_reach = False
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

   

            
