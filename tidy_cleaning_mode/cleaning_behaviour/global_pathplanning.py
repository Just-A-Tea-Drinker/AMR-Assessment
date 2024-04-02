#!/usr/bin/env python
#ROS imports 

from std_msgs.msg import Int32MultiArray,Float32MultiArray


import math
import numpy as np
import copy


#class will be dedicated to for trying to deduce a path/set of instructions for the robot to move a target box to the correct wall avoiding the obstacles and other boxes
class PathPlanning():
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
    #boxes = []

    #box to move is with colour
    target_box = [None]
    
    #path planning
    path = [None]
    bearing = [None]
        
    failed = False
    
    count = 0
    posGoals = []
   

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
            
            for points in self.posGoals:
                if abs(points[0] - x) + abs(points[1] - y)<=0.25:
                    print("returning point!")
                    return points
           
            
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
        #     child_cost.append(abs(goal[0] - child[0]) + abs(goal[1] - child[1]))
            child2goals = []
            for point in self.posGoals:
                child2goals.append(abs(point[0] - child[0]) + abs(point[1] - child[1]))
            child_cost.append(child2goals)
        smallest_cost = 100
        small_index = 0
        for i in range(len(child_cost)):
            for j in range(len(child_cost[i])):
                if child_cost[i][j]< smallest_cost:
                    small_index = i
                    smallest_cost = child_cost[i][j]
        if cur == [self.target_box[2],self.target_box[3]]:
            #adding the push bearing to get from the start to the first move
            best = children[small_index]
            start2child = math.atan2(cur[1]-best[1],cur[0]-best[0])
            px = cur[0]+(0.4*math.cos(start2child))
            py = cur[1]+(0.4*math.sin(start2child))
            self.bearing.append([px,py])
            #self.path.append(cur)
        self.bearing.append(pushing[small_index])
        return children[small_index]
    
    def PathPlan(self):
        ##Main function that will provide the path planning
        
        if self.target_box[0] !=None:
            print("Planning a Path")
            self.PointExtrap()
           
            start = [self.target_box[2],self.target_box[3]]
            goal = [self.target_wall[0],self.target_wall[1]]
            
            child = start
            self.path = []
            self.bearing = []
            
            self.count+=1
            while(child  not in self.posGoals):
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
                    for point in self.posGoals:
                        if point == child:
                            
                            break

            

            if self.failed ==False:
               
               
                self.CompMoves2()
                print("Global path has been created")
            else:
               print("Global path has failed")
           
       



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
            print(len(self.path))
            if i+1<len(self.path):
                bearing =  math.atan2(  self.path[i][1]-self.path[i+1][1]  ,self.path[i][0]-self.path[i+1][0] )
                px = self.path[i][0]+(0.5*math.cos(bearing))
                py = self.path[i][1]+(0.5*math.sin(bearing))
                self.bearing.append([px,py])
            else:
                if i == 0 and len(self.path)<=1:
                    start = [self.target_box[2],self.target_box[3]]
                    bearing =  math.atan2(  start[1]-self.path[-1][1]  ,start[0]-self.path[-1][0] )
                    px = start[0]+(0.45*math.cos(bearing))
                    py = start[1]+(0.45*math.sin(bearing))
                    self.bearing.append([px,py])
            
    def CompMoves2(self):
        epsilon = 0.1

        self.bearing = []
        temp_path = []
        simplified_path = self.douglas_peucker(self.path, epsilon)
        self.path = simplified_path
        for i in self.path:
            if math.dist(i,self.target_box[2:])<0.1:
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
                      
    def PointExtrap(self):
        #calculating extra points along the plane/ wall the box could be pushed to

        wall_mut = copy.deepcopy(self.target_wall)
       
        for i in range(len(wall_mut)):
            wall_mut[i] = math.sqrt(wall_mut[i]**2)
        
        if wall_mut[0]>wall_mut[1]:
            
            #wall should be on the x plane
            #as wall is on x modify the y
            wall_len = round(wall_mut[0]*2)-0.10
            
            iterate_num = wall_len/0.05
            self.posGoals = []
            for i in range(1,int(iterate_num/2)+1):

                if self.green_wall == self.target_wall:

                    self.posGoals.append([self.target_wall[0]-0.20,(self.target_wall[1]-(i*0.05))])
                
                    self.posGoals.append([self.target_wall[0]-0.20,(self.target_wall[1]+(i*0.05))])
                else:
                    self.posGoals.append([self.target_wall[0]+0.20,(self.target_wall[1]-(i*0.05))])
                
                    self.posGoals.append([self.target_wall[0]+0.20,(self.target_wall[1]+(i*0.05))])
            

        else:
            #wall is probably on the y plane
            pass
       
        if self.green_wall == self.target_wall:
             self.posGoals.append([self.target_wall[0]-0.20,self.target_wall[1]])
        else:
             self.posGoals.append([self.target_wall[0]+0.20,self.target_wall[1]])

        
         