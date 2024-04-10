#!/usr/bin/env python


import cv2
import numpy as np
import math
from pathfinding.core.grid import Grid
from pathfinding.finder.best_first import BestFirst
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.heuristic import euclidean

#class will be dedicated to for trying to deduce a path/set of instructions for the robot to move a target box to the correct wall avoiding the obstacles and other boxes
class PathPlanning():
   
    #colour presets
    red =(0,0,255)
    green = (0,255,0) 
    blue = (255,0,0)

    #common images
    img_open  = None
    binary_map = None
    dialated_image = None

    #image params
    width = 0
    height = 0

    #box information]
    green_boxes = None
    red_boxes = None
    target_box =None

    valid_path = True
    #pathing information 
    #main path
    path = []
    push = []
    #path to the start
    start = []



    def __init__(self,gboxs,rboxs):
        self.green_boxes = gboxs
        self.red_boxes = rboxs

        #basic config such as opening the image and rescaling
        img = cv2.imread('/workspaces/AMR-Assessment/World_map.pgm',cv2.COLOR_BGR2GRAY)
        height, width = img.shape[:2]

        #resolution makes 1 pixel = 0.5cm
        x_factor = 600/width
        y_factor = 600/height
        
        self.img_open = cv2.resize(img, None, fx=x_factor, fy=y_factor, interpolation=cv2.INTER_NEAREST)
        self.height, self.width = self.img_open.shape[:2]
        print("current image size ",self.width,"x",self.height) 

    def ObsBufferInc(self):
       
        x = 20  # Starting x-coordinate of the region
        y = 20  # Starting y-coordinate of the region
        width = 580  # Width of the region
        height = 580  # Height of the region
       
        # Create a binary mask specifying the region
        mask = np.zeros_like(self.binary_map[:, :])  # Create a mask with the same dimensions as the image
        mask[x:width,y:height] = 255  # Set the region to 255 (white)

        # Apply erosion to the mask
        kernel = np.ones((0, 0), np.uint8)
        eroded_mask = cv2.erode(mask, kernel, iterations=1)
        wall_mask = cv2.bitwise_and(cv2.bitwise_not(self.binary_map), cv2.bitwise_not(self.binary_map), mask=eroded_mask)
        oppi_wal  = cv2.bitwise_not(wall_mask)

        kernel = np.ones((70, 70), np.uint8)
        expanded_black_pixels_mask = cv2.erode(oppi_wal, kernel, iterations=1)
        self.dialated_image = cv2.bitwise_and(expanded_black_pixels_mask, eroded_mask)

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

    def GetBearing(self,path):
        bearings= []
        if self.target_box[1] == 1.0:
            x = 570
            
        else:
            x=50
        if len(path)==1:
            tx =  int(((self.target_box[2] - -1.5)/3) * self.width)
            ty= 600-int(((self.target_box[3]  - - 1.5)/3) * self.height)
            push_bearing = math.atan2(path[0][1]-ty,path[0][0]-tx)
            
            px = tx-int(60*math.cos(push_bearing))
            py = ty-int(60*math.sin(push_bearing))
            bearings.append([px,py])
        else:
            for i in range(0,len(path)-1):
                push_bearing = math.atan2(path[i][1]-path[i+1][1],path[i][0]-path[i+1][0])
                px = path[i][0]+int(60*math.cos(push_bearing))
                py = path[i][1]+int(60*math.sin(push_bearing))
                bearings.append([px,py])
                #cv2.circle(opencv_image, (px,py), 5, (255, 0, 0), -1)
                #cv2.circle(dilated_image, (px,py), 5, (255, 0, 0), -1)
        
        return bearings

    def InterBear(self,bearing):
        #drawing the temporary box to pathfind to each bearing
        inter_paths = []
        temp_map = self.dialated_image
        self.path = self.path[1:]
        
        for i in range(0,len(self.path)):
            inter = []
            costs = []
            if (self.path[i][0])+50<self.width and (self.path[i][1])+50<self.height:
                if temp_map[(self.path[i][0])+50][(self.path[i][1])+50] == 255:
                    inter.append([self.path[i][0]+50,self.path[i][1]+50])

            if (self.path[i][0])-50>0 and (self.path[i][1])-50>0:
                if temp_map[(self.path[i][0])-50][(self.path[i][1])-50] ==255:
                    inter.append([(self.path[i][0])-40,(self.path[i][1]-40)])

            if (self.path[i][0])+50<self.width and  (self.path[i][1])-50>0:      
                if temp_map[(self.path[i][0])+50][(self.path[i][1])-50] ==255:
                    inter.append([(self.path[i][0])+50,(self.path[i][1]-50)])
                    
            if (self.path[i][0])-50>0 and  (self.path[i][1])+50<self.height:
                if temp_map[(self.path[i][0])-50][(self.path[i][1])+50] == 255:
                    inter.append([(self.path[i][0])-50,(self.path[i][1])+50])
            for point in inter:
                if len(bearing)>i:
                    costs.append(math.dist(point,bearing[i]))
            if len(costs)>1:
                inter_paths.append(inter[costs.index(min(costs))])
            
        comp_path=[]
        #inter_paths = inter_paths[1:]
        inter_paths = inter_paths[:-1]
        temp_map = cv2.cvtColor(self.dialated_image, cv2.COLOR_GRAY2BGR)
        bruh = cv2.cvtColor(self.dialated_image, cv2.COLOR_GRAY2BGR)
        e = cv2.cvtColor(self.dialated_image, cv2.COLOR_GRAY2BGR)
        for j in range(0,len(self.path)):
            if j < len(bearing):
                if j<len(inter_paths):
                    cv2.circle(temp_map, bearing[j], 5, (0, 0, 255), -1)
                    comp_path.append(bearing[j])

                    cv2.circle(temp_map, self.path[j], 5, (255, 0, 0), -1)
                    comp_path.append(self.path[j])

                    cv2.circle(temp_map, inter_paths[j], 5, (0, 255, 0), -1)
                    comp_path.append(inter_paths[j])

                    
                else:
                    cv2.circle(temp_map, bearing[j], 5, (0, 0, 255), -1)
                    comp_path.append(bearing[j])

                    cv2.circle(temp_map, self.path[j], 5, (255, 0, 0), -1)
                    comp_path.append(self.path[j])
            else:
                cv2.circle(temp_map, self.path[j], 5, (255, 0, 0), -1)
                comp_path.append(self.path[j])
        self.path = comp_path

                    
        
    def GetPixel(self):
        if self.target_box[1] == 1.0:
            x =570
            distances = []
            for i in range(100,500):
                tx =  int(((self.target_box[2] - -1.5)/3) * self.width)
                ty= 600-int(((self.target_box[3]  - - 1.5)/3) * self.height)
                distances.append(abs(tx - x) + abs(ty - i))
                

            #iterate through the points and check none of them have obstalces nearby
            
            mindex = distances.index(min(distances))
            while(True):
                mindex = distances.index(min(distances))
                obs =False
                for i in range(450,500):
                    if self.dialated_image[i][mindex] ==0 and mindex<len(distances):
                        
                        distances.pop(mindex)
                        obs = True
                    else:
                        mindex = distances.index(min(distances))
                        obs = False
                
                if obs ==False:
                    break
            
        else:
            x=50
            distances = []
            for i in range(100,500):
                tx =  int(((self.target_box[2] - -1.5)/3) * self.width)
                ty= 600-int(((self.target_box[3]  - - 1.5)/3) * self.height)
                distances.append(abs(tx - x) + abs(ty - i))
                

            #iterate through the points and check none of them have obstalces nearby
            
            
            while(True):
                obs =False
                mindex = distances.index(min(distances))
                for i in range(100,150):
                    if self.dialated_image[i][mindex] ==0 and mindex<len(distances):
                       
                        distances.pop(mindex)
                        obs = True
                    else:
                        mindex = distances.index(min(distances))
                        obs = False
                
                
                if obs ==False:
                    break
       
       
        return[x,mindex]

        
        
        
        #return closest_pixel

    def MapOverlay(self):
        
        coordinate_min = -1.47
        coordinate_max = 1.47
        for boxes in self.green_boxes:
            box_x =  int(((boxes[2] - -1.5)/3) * self.width)
            box_y = 600-int(((boxes[3]  - - 1.5)/3) * self.height)
            cv2.rectangle(self.binary_map, (box_x-6, box_y-6), (box_x + 6, box_y + 6), (0,0,0), thickness=-1)

        for box in self.red_boxes:
            box_x =  int(((box[2] - -1.5)/3) * self.width)
            box_y = 600-int(((box[3]  - - 1.5)/3) * self.height)
            cv2.rectangle(self.binary_map, (box_x-6, box_y-6), (box_x + 6, box_y + 6), (0,0,0), thickness=-1)

   
    def Nav2Start(self,bot_x,bot_y,gboxes,rboxes):
        #method used to reach the first push bearing from the current location
        self.red_boxes = rboxes
        self.green_boxes = gboxes

        self.X =  int(((bot_x - -1.5)/3) * self.width)
        self.Y = 600-int(((bot_y  - - 1.5)/3) * self.height)

        a, self.binary_map = cv2.threshold(self.img_open, 127, 255, cv2.THRESH_BINARY)
        #adding the boxes that arent the except from the moving box to the obstacles
        self.MapOverlay()
        #creating a dialated image for obstacle free navigation
        self.ObsBufferInc()

        grid = Grid(matrix=np.array(self.dialated_image))
       
        matrix=np.array(self.binary_map)

        #start and goal point for the path finding
        start_point = [self.X,self.Y]

        
        goal_point = self.push[0]

        # Create a pathfinder object using best first
        finder = BestFirst(diagonal_movement=DiagonalMovement.always)
        

        # Find the path
        start = grid.node(start_point[0], start_point[1])
        end = grid.node(goal_point[0], goal_point[1])
        path, _ = finder.find_path(start, end, grid)
        
        points = []
        if len(path) != 0:
                
            print("Path to the start is available")
                
                
            for i in path:
                points.append([i.x,i.y])
            reduced = self.douglas_peucker(points,3)
            self.start = reduced

            for j in self.start:
                if j ==self.start[-1]:
                    cv2.circle(self.dialated_image, j, 5, (0, 0,0 ), -1)
                else:

                    cv2.circle(self.dialated_image, j, 5, (0, 0, 0), -1)
            # cv2.imshow('image',self.opencv_image)
            # cv2.imshow('dialated image',self.dialated_image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            return True
            
        else:
            print("Path to the start wasnt available")
            # cv2.imshow('image',self.opencv_image)
            # cv2.imshow('dialated image',self.dialated_image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            return False
        
                

    def Pix2Cart(self,path):
        #this will path in pixels coordinates back to gazebo cartesian
        converted = []
            
        for i in path:
                    
            x = (i[0]/self.width)*3-1.5
            y = ((i[1]/self.height)*3-1.5)*-1.0
                
            converted.append([x,y])
                    
        return converted
    
    def PathMakingBackup(self,botx,boty,target):
            ##Main function that will provide the path planning
            self.target_box=target
            if self.target_box[0] !=None:
                pot_goals = []
                if self.target_box[1] == 1.0:
                    x = 570
                    
                else:
                    x=50
                
                
                for i in range(100,500):
                    pot_goals.append([x,i])

                print("Planning a Path")
                sx =  int(((self.target_box[2] - -1.5)/3) * self.width)
                sy= 600-int(((self.target_box[3]  - - 1.5)/3) * self.height)
                self.X =  int(((botx - -1.5)/3) * self.width)
                self.Y = 600-int(((boty  - - 1.5)/3) * self.height)
                start = [sx,sy]
                
                
                child = start
                self.path = []
                new_green = []
                new_red = []
                for gboxes in self.green_boxes:
                    if gboxes != target:
                        new_green.append(gboxes) 
                for rboxes in self.red_boxes:
                    if rboxes != target:
                        new_red.append(rboxes)

                self.green_boxes = new_green
                self.red_boxes = new_red
                

                tx =  int(((target[2] - -1.5)/3) * self.width)
                ty= 600-int(((target[3]  - - 1.5)/3) * self.height)
                self.target_box = target
                
                #[tx,ty]
                a, self.binary_map = cv2.threshold(self.img_open, 127, 255, cv2.THRESH_BINARY)
                #adding the boxes that arent the except from the moving box to the obstacles
                self.MapOverlay()
                #creating a dialated image for obstacle free navigation
                self.ObsBufferInc()
                matrix=np.array(self.binary_map)
                finder = BestFirst(diagonal_movement=DiagonalMovement.always)
                self.path.append(start)
                
                while(child  not in pot_goals):
                    #looping through the positions until we meet the goal if possible

                    #getting the positions to move to
                    best_move= self.PosCalc(child,pot_goals)
                    
                    if best_move == child:
                        print("There arent any valid moves")
                        #signifying failure to the move node to get a new box
                       
                        #self.path = []
                        self.failed = True
                        break
                        
                    else:
                    
                        ##adding the child to the path to move on
                        self.path.append(best_move)
                        child = best_move
                        self.failed = False
                        for point in pot_goals:
                            if point == child:
                                print("ednning loop")
                                break
                


                if self.failed ==False:
                    print("reducing path")
                    
                    points=[]
                    for i in self.path:
                        points.append((i[0],i[1]))
                    if len(self.path)>1:
                        self.path = self.douglas_peucker(points,0.1)
                        


                    self.opencv_image = cv2.cvtColor(self.dialated_image, cv2.COLOR_GRAY2BGR)

                    bearings = self.GetBearing(self.path)
                    self.valid_path = True
                    self.InterBear(bearings)
                    # #drawing the path onto the map
                    for point in self.path:
                                
                        cv2.circle(self.opencv_image, (220,45), 5, (0, 255,0 ), -1)
                        cv2.circle(self.opencv_image, point, 5, (0, 0, 255), -1)  # Draw a filled circle at each point   
                            #cv2.circle(dilated_image, point, 1, (0, 255, 0), -1) 
                    #adding the ush bearing locations
                    for bear in bearings:
                        cv2.circle(self.opencv_image, bear, 5, (255, 0, 0), -1)  # Draw a filled circle at each point
                    #adding the coloured boxes to  the original image
                    for boxes in self.green_boxes:
                        box_x =  int(((boxes[2] - -1.5)/3) * self.width)
                        box_y = 600-int(((boxes[3]  - - 1.5)/3) * self.height)
                        cv2.rectangle(self.opencv_image, (box_x-6, box_y-6), (box_x + 6, box_y + 6), self.green, thickness=-1)

                    for box in self.red_boxes:
                        box_x =  int(((box[2] - -1.5)/3) * self.width)
                        box_y = 600-int(((box[3]  - - 1.5)/3) * self.height)
                        cv2.rectangle(self.opencv_image, (box_x-6, box_y-6), (box_x + 6, box_y + 6), self.red, thickness=-1)
                    if self.valid_path == False:
                        self.valid_path = True
                        # cv2.imshow('image',self.opencv_image)
                        # cv2.imshow('dialated image',self.dialated_image)
                        # cv2.waitKey(0)
                        # cv2.destroyAllWindows()
                        return False
                    else:
                        
                        self.push = bearings
                        
                        # cv2.imshow('image',self.opencv_image)
                        # cv2.imshow('dialated image',self.dialated_image)
                        # cv2.waitKey(0)
                        # cv2.destroyAllWindows()
                        return True 
                        
                        
                else:
                    print("no path was found")
                    bearings = self.GetBearing(self.path)
                    for point in self.path:
                                
                        cv2.circle(self.dialated_image, (220,45), 5, (0, 0,0 ), -1)
                        cv2.circle(self.dialated_image,point, 5, (0, 0, 0), -1)  # Draw a filled circle at each point   
                            #cv2.circle(dilated_image, point, 1, (0, 255, 0), -1) 
                    #adding the ush bearing locations
                    for bear in bearings:
                        cv2.circle(self.dialated_image, bear, 5, (0, 0, 0), -1)  # Draw a filled circle at each point
                    # cv2.imshow('image',self.dialated_image)
                    # cv2.imshow('dialated image',self.dialated_image)
                    # cv2.waitKey(0)
                    # cv2.destroyAllWindows()
                    return False
                

    def PosCalc(self,cur,pot_goals):
        #used to mathematically calculate a new cooridnate based on position
        gran= 50
        children = []
        pushing = []
        angles =[-3.14,-1.57,0.0,1.57,3.14]
        
        #producing the locations the child could move to and checking their validity
        for ang in angles:
            x = cur[0]+ int(gran * math.cos(ang))
            y = cur[1]+ int(gran * math.sin(ang))

            push_bearing = math.atan2(cur[1]-y,cur[0]-x)
            px = x+int(60*math.cos(push_bearing))
            py = y+int(60*math.sin(push_bearing))
            #print([[x,y]])
            
        
            #checking if the move is valid of not
            #checking near current lidar data
            valid = True
            
            
            radius = 0
            for points in pot_goals:
                    if abs(points[0] - x)<=25 and abs(points[1] - y)<=0:
                        print("returning point!")
                        return points
           
            if self.dialated_image[y][x] ==0:
                valid = False
            else:
                valid = True
            if valid ==True:
               
                if self.dialated_image[py][px] ==0:
                    valid = False
                else:
                    valid = True

          
            if valid == True:
                #calculating where the robot will have to push from to achieve the move

                
                children.append([x,y])
        #now we have all the possible moves its time to decide which one is best
        child_cost = []
        
        if len(children) ==0:
            #print("There arent any valid moves")
           
            return cur
        for child in children:
        #     child_cost.append(abs(goal[0] - child[0]) + abs(goal[1] - child[1]))
            child2goals = []
            for point in pot_goals:
                child2goals.append(abs(point[0] - child[0]) + abs(point[1] - child[1]))
            child_cost.append(child2goals)
        smallest_cost = 1000000000000
        small_index = 0

        for i in range(len(child_cost)):
            for j in range(len(child_cost[i])):
                if child_cost[i][j]< smallest_cost:
                    small_index = i
                    smallest_cost = child_cost[i][j]

        sx =  int(((self.target_box[2] - -1.5)/3) * self.width)
        sy= 600-int(((self.target_box[3]  - - 1.5)/3) * self.height) 

        if children[small_index] in self.path:
            return cur
        else:
            return children[small_index]