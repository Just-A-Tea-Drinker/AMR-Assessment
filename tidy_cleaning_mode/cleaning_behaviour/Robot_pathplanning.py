#!/usr/bin/env python
#ROS imports 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray,Float32MultiArray
from sensor_msgs.msg import LaserScan

import math

import matplotlib.pylab as plt
import numpy as np


#class will be dedicated to for trying to deduce a path/set of instructions for the robot to move a target box to the correct wall avoiding the obstacles and other boxes
class PathPlanning(Node):
    #odom data
    yaw = 0.0
    X = 0.0
    Y = 0.0

    #rangee data
    ranges = []
    largestMid =0.0
    smallestMid = 1000.0

    #wall data
    red_wall = [-1.5,0.0]
    green_wall=  [1.5,0.0]
    target_wall = []
    Xs = []
    Ys = []

    #boxes
    red_boxes = []
    green_boxes =[]
    boxes = []


    #box to move is with colour
    target_box = [None]
    

    #path planning
    path = [None]
    bearing = [None]
    

    #command based
    state = 0.0
    failed = False

    count = 0
    def __init__(self):
        super().__init__('PathPlanning')

        ##subscriber to read the custom scan data
        self.range_get = self.create_subscription(LaserScan,"/scan",self.Scancallback, 1)
        #subscribing to the box topic to obtain the boxes
        self.box_get = self.create_subscription(Float32MultiArray,'/BoxInfo',self.BoxCallback,1)
        ##setting up a subscriber to read brain commands
        self.beh_get = self.create_subscription(Float32MultiArray,'/Conductor',self.CommandCallback,1)
        ##setting up a subscriber to read the box to plan for
        self.Tar_get = self.create_subscription(Float32MultiArray,'/Box_Target',self.TargetCallback,1)
         #subscriber for the custom odometry
        self.pos_get = self.create_subscription(Float32MultiArray,'/Compressed_Odom',self.OdomCallback,1)

        #creating publishers dedicated for the pathfinding and moving of the box
        self.pathFinding =self.create_publisher(Float32MultiArray,'/PathFinding',60)
        self.push_loc =self.create_publisher(Float32MultiArray,'/Box_bearings',60)
        self.path_msg = Float32MultiArray()
        self.bearing_msg = Float32MultiArray()
        #creating a time controlled loop for general processes that arent event driven
        timer_period = 0.01  # seconds
        self.updater = self.create_timer(timer_period, self.Broadcast)
        
        # self.target_box=[1.0, 1.0, 0.7973528289794922, -0.5]
        # self.target_wall = [1.4828461408615112, -0.0376550517976284]
        # print(type(self.target_box))
      

        # self.PathPlan()
        
        


    def Broadcast(self):
        #method used to broad cast the current path or the state of the movement
        if len(self.bearing_msg.data)>0 and len(self.path_msg.data)>0:
            #publishing the messages to read by move node
            self.pathFinding.publish(self.path_msg)
            self.push_loc.publish(self.bearing_msg)
       
        
    def OdomCallback(self,odom):
        self.start_yaw = odom.data[3]
        self.yaw = odom.data[2]
        self.X = odom.data[0]
        self.Y = odom.data[1]

    def Scancallback(self,data):
        ##obtaining the scan data
       
        #doing some basic filtering for the lidar data
        actual_ranges = []
        for range in data.ranges:
            if range != 0.0:
                actual_ranges.append(range)
        if len(actual_ranges)%2 !=0:
            actual_ranges =actual_ranges[1:]
        self.ranges = actual_ranges
        self.Xs = []
        self.Ys = []
        for i, length in enumerate(self.ranges):
            angle = data.angle_min + i * data.angle_increment
            
            ##plotting the obstacles
            x = self.X+ ((length+0.15) * math.cos(angle + math.radians(self.yaw)))
            y = self.Y+ ((length+0.15) * math.sin(angle + math.radians(self.yaw)))

            self.Xs.append(x)
            self.Ys.append(y)

            plt.scatter(x, y, c ="blue")
            if (len(self.red_wall)>1 and len(self.green_wall)>1):
                plt.scatter(self.red_wall[0],self.red_wall[1], c ="darkred")
                plt.scatter(self.green_wall[0],self.green_wall[1], c ="darkgreen")
            
        # plotting the boxes if there are any
        if len(self.green_boxes)>0:
            for green_box in self.green_boxes:
                if green_box ==self.target_box:
                    plt.scatter(green_box[2], green_box[3], c ="orange")
                else:
                    plt.scatter(green_box[2], green_box[3], c ="lime")
        if len(self.red_boxes)>0:
            for red_box in self.red_boxes:
                if red_box ==self.target_box:
                    plt.scatter(red_box[2], red_box[3], c ="pink")
                else:
                    plt.scatter(red_box[2], red_box[3], c ="red")  
        #plotting the current location of the robot
        plt.scatter(self.X, self.Y, c ="black")
        if self.path[0]!=None and self.path[0]!=-1.0:
            for node in self.path:
                plt.scatter(node[0], node[1], c ="purple")
            for bear in self.bearing:
                plt.scatter(bear[0], bear[1], c ="yellow")
           
        plt.draw()
        plt.pause(0.1)
        plt.clf()
        
        
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

    def CommandCallback(self,command):
        self.state = command.data[0]
        self.green_wall = command.data[1:3]
        self.red_wall = command.data[3:]
        # if self.state == 1.0 and self.target_box!=None:
        self.PathPlan()
            

    def TargetCallback(self,target):
        if self.state== 1.0:
            box = []
            for comp in target.data:
                box.append(comp)
            
            self.target_box = box
            if self.target_box[1] == 1.0:
                self.target_wall = self.green_wall
            else:
                self.target_wall= self.red_wall
            print(self.target_box," ",self.target_wall)
            self.PathPlan()
            
        


    def PosCalc(self,cur,goal):
        #used to mathematically calculate a new cooridnate based on position
        gran= 0.15
        children = []
        pushing = []
        angles =[-3.14,-1.57,0.0,1.57,3.14]
        
        #producing the locations the child could move to and checking their validity
        for ang in angles:
            x = cur[0]+ (gran * math.cos(ang))
            y = cur[1]+ (gran * math.sin(ang))
            #calculating where the robt must push from to reach each point
            push_bearing = math.atan2(cur[1]-y,cur[0]-x)
            px = x+(0.4*math.cos(push_bearing))
            py = y+(0.4*math.sin(push_bearing))
            
            #checking if the move is valid of not
            #checking near current lidar data
            valid = True
            
            if [x,y] == goal or abs(goal[0] - x)<=0.25 or math.dist(goal,[x,y])<=0.25:
                return goal
            
            for i in range(len(self.Xs)):
                if abs(self.Xs[i] - x) + abs(self.Ys[i] - y)<=0.2:
                    valid = False
                    break
                else:
                    valid = True

                if abs(self.Xs[i] - px) + abs(self.Ys[i] - py)<=0.2:
                    valid = False
                    break
                else:
                    valid = True
            #checking near other boxes
            if valid ==True:
                for green_box in self.green_boxes:
                    if(green_box !=self.target_box):
                        if abs(green_box[2] - x) + abs(green_box[3] - y)<=0.2:
                            valid = False
                            break
                        else:
                            valid = True
                        if abs(green_box[2] - px) + abs(green_box[3] - py)<=0.2:
                            valid = False
                            break
                        else:
                         valid = True
                    
            if valid ==True:
                for red_box in self.red_boxes:
                    if(red_box !=self.target_box):
                        if abs(red_box[2] - x) + abs(red_box[3] - y)<=0.2:
                            valid = False
                            break
                        else:
                            valid = True
                        if abs(red_box[2] - px) + abs(red_box[3] - py)<=0.2:
                            valid = False
                            break
                        else:
                            valid = True
            if valid == True:
                #calculating where the robot will have to push from to achieve the move

                pushing.append([px,py]) 
                children.append([x,y])
        #now we have all the possible moves its time to decide which one is best
        child_cost = []
        
        if len(children) ==0:
            #print("There arent any valid moves")
            return cur
        for child in children:
            child_cost.append(abs(goal[0] - child[0]) + abs(goal[1] - child[1]))
        if cur == [self.target_box[2],self.target_box[3]]:
            #adding the push bearing to get from the start to the first move
            best = children[child_cost.index(min(child_cost))]
            start2child = math.atan2(cur[1]-best[1],cur[0]-best[0])
            px = cur[0]+(0.4*math.cos(start2child))
            py = cur[1]+(0.4*math.sin(start2child))
            self.bearing.append([px,py])
            #self.path.append(cur)
        self.bearing.append(pushing[child_cost.index(min(child_cost))])
        return children[child_cost.index(min(child_cost))]
    
    

                    




    def PathPlan(self):
        ##Main function that will provide the path planning
       
        if self.target_box[0] !=None:
            print("Planning a Path")
            start = [self.target_box[2],self.target_box[3]]
            goal = [self.target_wall[0],self.target_wall[1]]
            
            child = start
            self.path = []
            self.bearing = []
            self.bearing_msg.data = []
            self.path_msg.data = []
            self.count+=1
            while(child !=goal):
                #looping through the positions until we meet the goal if possible

                #getting the positions to move to
                best_move= self.PosCalc(child,goal)
                
                if best_move == child:
                    print("There arent any valid moves")
                    #signifying failure to the move node to get a new box
                    self.path = []
                    self.bearing = []
                    self.bearing.append(-1.0)
                    self.path.append(-1.0)
                    for comp in self.target_box:
                        self.bearing.append(comp)
                        self.path.append(comp)
                    
                    self.failed = True
                    break
                    
                else:
                   
                    ##adding the child to the path to move on
                    self.path.append(best_move)
                    child = best_move
                    self.failed = False

            

            if self.failed ==False:
                msg_data = []
                self.CompMoves2()
                for bearing in self.bearing:
                    
                    for xy in bearing:
                        msg_data.append(float(xy))
                self.bearing_msg.data = msg_data
                msg_data = []
                for node in self.path:
                    for xy in node:
                        msg_data.append(float(xy))
                self.path_msg.data = msg_data
            else:
                msg_data = []
                for bearing in self.bearing:
                    msg_data.append(float(bearing))
                self.bearing_msg.data = msg_data
                msg_data = []
                for node in self.path:
                    msg_data.append(float(node))
                self.path_msg.data = msg_data
            print("Path completed")
           
            #print(self.path_msg)
            #print(self.bearing_msg)
        #print(self.path)
        #print(self.bearing)
        #self.CompMoves()
       



    def douglas_peucker(self,points, epsilon):
        if len(points) <= 2:
            return points
        
        # Find the point with the maximum distance
        max_distance = 0
        max_index = 0
        start, end = points[0], points[-1]
        
        for i in range(1, len(points) - 1):
            distance = self.perpendicular_distance(points[i], start, end)
            if distance > max_distance:
                max_distance = distance
                max_index = i
                
        if max_distance > epsilon:
            # Recursive call for subpaths
            left_subpath = self.douglas_peucker(points[:max_index + 1], epsilon)
            right_subpath = self.douglas_peucker(points[max_index:], epsilon)
            return left_subpath[:-1] + right_subpath
        else:
            return [start, end]

    def perpendicular_distance(self,point, start, end):
        # Calculate perpendicular distance of 'point' from line connecting 'start' and 'end'
        return abs((end[1] - start[1]) * point[0] - (end[0] - start[0]) * point[1] + end[0] * start[1] - end[1] * start[0]) / ((end[1] - start[1]) ** 2 + (end[0] - start[0]) ** 2) ** 0.5   
    
    def bearingCalc(self):
        #taking the newly calculated path generated and calculating the push bearings for them
        for i in range(len(self.path)):
            if i+1<len(self.path):
                bearing =  math.atan2(  self.path[i][1]-self.path[i+1][1]  ,self.path[i][0]-self.path[i+1][0] )
                px = self.path[i][0]+(0.4*math.cos(bearing))
                py = self.path[i][1]+(0.4*math.sin(bearing))
                self.bearing.append([px,py])
            
    def CompMoves2(self):
        epsilon = 0.1

        self.bearing = []
        temp_path = []
        simplified_path = self.douglas_peucker(self.path, epsilon)
        self.path = simplified_path
        for i in self.path:
            if math.dist(i,self.target_box[2:])<0.2:
                temp_path.append(self.target_box[2:])
            else:
                temp_path.append(i)
        self.path = temp_path
        self.bearingCalc()
        temp_path = []
        for i in self.path:
            if i != [self.target_box[2],self.target_box[3]]:
                temp_path.append(i)
        self.path = temp_path
        # print(self.path)
        # print(self.bearing)
        




    def CompMoves(self):
        #function dedicated for compounding like moves like equal headings
        print("ORIGINAL")
        print(self.path)
        print(self.bearing)
        comp_path = []
        for i in range(len(self.path)):
            
            #calcualting if there are like vectors
            comp_path.append(math.atan2( self.path[i][1]-self.bearing[i][1],   self.path[i][0]-self.bearing[i][0]))
        print(comp_path)       
        #now we have a compilled direction we can get the index of the bearings and path and delete the duplicate instructions
        temp_bear = []
        temp_path = []
        
        
        bear = []
        node =[]
        
        for j in range(len(comp_path)):
            first_index = None
            last_index = None

            for i, value in enumerate(comp_path):
               
                if value == comp_path[j]:
                    if first_index is None:
                        first_index = i
                    last_index = i
            temp_bear.append(self.bearing[first_index])
            temp_path.append(self.path[last_index])

        
        
        ##trying to see if any of the bearings and te movements have any correlation

       

        self.path = []
        [self.path.append(x) for x in temp_path if x not in self.path]
        self.bearing = []
        [self.bearing.append(x) for x in temp_bear if x not in self.bearing]
        print("NEW")
        print(self.path)
        print(self.bearing)
        #self.bearing = []
               
                    
                      

                
         


def main():
    print('Starting path planning')
    
    #plt.ion()
    plt.show(block=False)
    rclpy.init()

    path= PathPlanning()

    rclpy.spin(path)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()