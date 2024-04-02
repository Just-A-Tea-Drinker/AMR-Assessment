#!/usr/bin/env python

#ROS imports
from geometry_msgs.msg import PoseStamped


#python imports
import math
import numpy as np


class LocalPathFinding():
    #path finding
    path2Start = []
    bearings = []
    path =[]


    #box and wall information
    green_boxes = []
    red_boxes = []
    selecBox = []

    #sensor info
    ranges = []
    X =0
    Y= 0
    yaw = []

    def __init__(self,gboxes,rboxes,bear,sbox,ranges,x,y,Yaw):
        self.ranges = ranges
        self.bearings = bear
        self.green_boxes = gboxes
        self.red_boxes = rboxes
        self.selecBox = sbox
        self.X = x
        self.Y=y
        self.yaw = Yaw


    
    
    
    
    
    #basic form of pathfinding for the local path to the start
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
        
        for a in range(0,360):
            cx = cur[0]+ (gran * math.cos(a))
            cy = cur[1]+ (gran * math.sin(a))
            #calculating where the robt must push from to reach each point

            #checking if the move is valid of not
            #checking near current lidar data
            valid = True
            
            if [cx,cy] == goal or abs(goal[0] - cx) + abs(goal[1] - cy)<=0.2:
                return goal
            
            for b in range(len(self.Xs)):
                if math.dist([self.Xs[b],self.Ys[b]],[cx,cy])<=0.2:
                    valid = False
                    break
                else:
                    valid = True
            
            #checking near other boxes
            if valid ==True:
                for green_box in self.green_boxes:
                    
                    if(green_box !=self.selecBox[0]):
                        
                        if abs(green_box[2] - cx) + abs(green_box[3] - cy)<=0.35:
                            valid = False
                            break
                        else:
                            valid = True
                   
            if valid ==True:
                for red_box in self.red_boxes:
                    #if(red_box !=self.selecBox[0]):
                        if abs(red_box[2] - cx) + abs(red_box[3] - cy)<=0.35:
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
    

    def EstiProjec(self,Path):
        #this will take in a path and test whether or not the path must intercept a exclusion zone
        excl_radi = 0.15 #<- cane be changed based on the accuracy of the of the robots movement
        valid_path= True
        start = []
        end = Path[-1]
        #iterating over the path, interpolating locations and seeing if the path is too close
        cords = self.selecBox[2:]
        print(cords)
        print(self.selecBox)
        Rectpath = []
        for i in range(0,len(Path)-2):
            #getting the heading between points
            
            heading = math.atan2(Path[i+1][1]-Path[i][1],Path[i+1][0]-Path[i][0])
            dist = math.dist(Path[i],Path[i+1])
            
            interval = dist/10
            for a in range(0,10):
                x = Path[i][0]+ ((a*interval) * math.cos(heading))
                y = Path[i][1]+ ((a*interval) * math.sin(heading))
                if (x - cords[0])**2 + (y - cords[1])**2 - excl_radi**2 <=0:
                    valid_path = False
                    if i>0:
                        start = Path[i]
                    else:
                        start =Path[0]
                    break
            if valid_path ==False:
                break
        #has been shown that the path is either valid or invalid
        if valid_path == False:
            #this will be a new path is made to go around the exclusion zone
            
            start_vector = np.array(start) - np.array(cords)
            
            # Calculate the vector from the exclusion center to the desired location
            desired_vector = np.array(end) - np.array(cords)
            
            # Calculate the angle between the start point, exclusion center, and desired location
            start_angle = math.atan2(start_vector[1], start_vector[0])
            desired_angle = math.atan2(desired_vector[1], desired_vector[0])
            angle_diff = desired_angle - start_angle
            
            # Ensure the angle difference is positive
            if angle_diff < 0:
                angle_diff += 2 * np.pi
            
            # Generate points around the perimeter
            points = []
            for b in range(10):
                angle = start_angle + b * angle_diff / (10 - 1)
                x = cords[0] + np.cos(angle)
                y = cords[1] + np.sin(angle)
                points.append((x, y))
            ##making a new rectified path
            for point in Path:
                if point ==start:
                    for rectpoint in points:
                        Rectpath.append(rectpoint)
                else:
                    Rectpath.append(point)
            return Rectpath
        else:
            return Path

        
        
            

            
        

