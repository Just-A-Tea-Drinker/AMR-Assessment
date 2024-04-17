#!/usr/bin/env python
import rclpy
from rclpy.node import Node


import cv2


import numpy as np
import math
import copy



from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray,Float32MultiArray
#Node dedicated for remembering and processing information about the boxes basically will make sure the robot wont try and move and already sorted box


class BoxSort(Node):
    """Node for maintaining a recollection of the boxes info

        Inputs:
        /Box_Id, what boxes are required to be moved
        /Cont_Origin takes an [x,y] of an image
        /Range_Pub customised lidar topic
        /Compressed_Odom, inputting the robot location with stamped yaw
        /WallInfo gets the wall information

        Output:
        
        /BoxInfo' getting the information of all the boxes
        /WallProcessed taking the pixel coordinate and return a cartesian transformation
        
    """
    Pos = [0,0,0]
    obs_cords = [None]*1
    lidar_range = []

    boxes = []
    need_move=[]
    boxes_moved = []

    boxbool = False
    walls = [0.0,0.0,0.0,0.0]
    green_wall = []
    red_wall =[]

    posGoals = []

    def __init__(self):
        super().__init__('BoxProcessing')
        #subscribing to the point cloud topic
        self.depth_sub = self.create_subscription(Image,"/limo/depth_camera_link/depth/image_raw",self.DepthCallback, 1)
        self.br = CvBridge()
        ##subscribing to the box/contour origin data to use this to find the depth
        self.ImgProsub = self.create_subscription(Int32MultiArray, "/Cont_Origin",self.OriginCallback,1)
        #subscribing to the custom odometry to get the co ordinates in order to store the boxes mathematically
        self.OdomSub = self.create_subscription(Float32MultiArray, "/Compressed_Odom",self.OdomCallback,1)
        #subscribing to the edited laser scan topic
        self.LaserSub = self.create_subscription(Float32MultiArray, "/Range_Pub",self.ScanCall,1)
        #subscribing to the box id topic
        self.LaserSub = self.create_subscription(Float32MultiArray, "/Box_ID",self.BoxId,1)
        #creating a subscription for the un processed wall data
        self.wall_get = self.create_subscription(Int32MultiArray,"/WallInfo",self.wallCallback,1)
        #creating a topic to publish the box data
        self.BoxinfoPub = self.create_publisher(Float32MultiArray, "/BoxInfo",60)
        self.box_msg = Float32MultiArray()
        #creating a topic to publish the wall data
        self.WallinfoPub = self.create_publisher(Float32MultiArray, "/WallProcessed",60)
        self.wall_msg = Float32MultiArray()
        #creating a time based callback
        timer_period = 0.01  # seconds
        self.updater = self.create_timer(timer_period, self.Broadcast)

    def Broadcast(self):
        if len(self.green_wall)>0 and len(self.red_wall)>0:
            #checking the distances between the boxes and the wall
            self.need_move = []
            for box in self.boxes:
                if box[1]==0.0:
                    self.PointExtrap(self.red_wall)
                    distances = []
                    for pot_goal in self.posGoals:
                        distances.append(math.dist(box[2:],pot_goal))
                    dist = distances[distances.index(min(distances))]
                    if dist>0.25:
                        self.need_move.append(box)
                else:
                    self.PointExtrap(self.green_wall)
                    distances = []
                    for pot_goal in self.posGoals:
                        distances.append(math.dist(box[2:],pot_goal))
                    dist = distances[distances.index(min(distances))]
                    if dist>0.25:
                        self.need_move.append(box)
                   
            msg_data = []
            for boxes in self.need_move:
                for info in boxes:
                    msg_data.append(float(info))
            self.box_msg.data = msg_data
            self.BoxinfoPub.publish(self.box_msg)

    def wallCallback(self,walldata):
        for a in range(0,len(walldata.data), 3):
             #print(i," ",origin.data[i]," ",origin.data[i+1]," ", origin.data[i+2])
            if walldata.data[a] < 300:
                pass
            # else if center of object is to the right of image center move right
            elif walldata.data[a] >= 400:
                pass
            else: # center of object is in a 100 px range in the center of the image so dont turn
        
                self.wall_dist = math.sqrt((self.cv_image[walldata.data[a+1]][walldata.data[a]]**2)-(0.175**2))+0.1
                #using trig to calculate the cartesian co ordinates to store/memorize them
                wallX = self.Pos[0] + (self.wall_dist *math.cos(math.radians(self.Pos[2])))
                wallY = self.Pos[1] + (self.wall_dist *math.sin(math.radians(self.Pos[2])))
                if walldata.data[a+2] ==0:
                    #adding to the red wall,
                    self.walls[2] = wallX
                    self.walls[3] = wallY
                    self.red_wall = self.walls[2:]
                else:
                    self.walls[0] = wallX
                    self.walls[1] = wallY
                    self.green_wall = self.walls[:2]
                
        self.wall_msg.data =self.walls
        self.WallinfoPub.publish(self.wall_msg)

    def BoxId(self, Id):
        #this call back will be used to push "moved" box into some kind of storage to make sure the same boxes arent flagged again and moved
        if Id.data[0] == -1.0:
            self.boxes = []
            self.need_move=[]
        for box in self.boxes:
            if box[0] == Id.data[0]:
                
                box_index = self.boxes.index(box)
                temp_box = self.boxes[box_index]
                del self.boxes[box_index]
                #checkig whether or not the box is already in the "moved" list
                moved_check = False 
                for moved in self.boxes_moved:
                    if moved[0] == temp_box[0]:
                        moved_check = True
                        break
                if moved_check !=True:
                    self.boxes_moved.append(temp_box)

                    


    
    def DepthCallback(self,depth):
        self.cv_image = self.br.imgmsg_to_cv2(depth, 'passthrough')
        #for showing the depth image created 
        ##cv2.imshow("Live", self.cv_image)
        ##cv2.waitKey(1)
    def OdomCallback(self,odom):
        #print(odom.data)
        self.Pos[0] = odom.data[0]
        self.Pos[1] = odom.data[1]
        self.Pos[2] = odom.data[2]

    def ScanCall(self,ranges):
        #making a coordinate based on the robot's rotation as well as lidar scan step
        # from scan topic echo found that the step increment 0.013745704665780067
        #using ranges to map what the robot can see as a potential wall
        self.lidar_range = ranges.data
        
        self.obs_cords = [None]*len(self.lidar_range)
        for i in range(len(self.lidar_range)):
            if i< int(len(self.lidar_range)/2):
                x = int(-len(self.lidar_range)/2) + i
            else:
                if i>int(len(self.lidar_range)/2):
                    x = i - int(len(self.lidar_range)/2)

            if(i == int(len(self.lidar_range)/2)):
                if self.Pos[2]<-45 and self.Pos[2]>-135:
                    estiX = self.Pos[0] + (self.lidar_range[i]* math.cos(math.radians(self.Pos[2])))
                    estiY = self.Pos[1] + (self.lidar_range[i]* math.sin(math.radians(self.Pos[2])))-0.1
                    self.obs_cords[i]= {estiX},{estiY}
                else:

                    estiX = self.Pos[0] + (self.lidar_range[i]* math.cos(math.radians(self.Pos[2])))-0.1
                    estiY = self.Pos[1] + (self.lidar_range[i]* math.sin(math.radians(self.Pos[2])))+0.1
                    self.obs_cords[i]= {estiX},{estiY}

            else:
                estiX = self.Pos[0] + (self.lidar_range[i]* math.cos((x*0.013745704665780067) + math.radians(self.Pos[2])))
                estiY = self.Pos[1] + (self.lidar_range[i]* math.sin((x*0.013745704665780067 + math.radians(self.Pos[2]))))
                self.obs_cords[i]= {estiX},{estiY}  
  
            
            


    def OriginCallback(self,origin):
        ##As we have just received the origins of the contours that the camera can see we have to see how
        #This can give me the depth from the camera to the box however using trigonometry i can estimate the actual distance in x
        #NOTE camera height is around 17.5 to 18 cm
        # we already have c^2 and a^2 so: b^2 = c^2-a^2
        #this equation calculates the relative distance from the center of the robot to 
        # if center of object is to the left of image center move left
        #print(origin.data)
        #for i in range(len(origin.data)-3):
        for a in range(0,len(origin.data), 3):
             #print(i," ",origin.data[i]," ",origin.data[i+1]," ", origin.data[i+2])
            if origin.data[a] < 325:
                pass
            # else if center of object is to the right of image center move right
            elif origin.data[a] >= 375:
                pass
            else: # center of object is in a 100 px range in the center of the image so dont turn
        
                self.flat_dist = math.sqrt((self.cv_image[origin.data[a+1]][origin.data[a]]**2)-(0.175**2))+0.1
                #using trig to calculate the cartesian co ordinates to store/memorize them
                boxX = self.Pos[0] + (self.flat_dist*math.cos(math.radians(self.Pos[2])))
                boxY = self.Pos[1] + (self.flat_dist*math.sin(math.radians(self.Pos[2])))
                #boxbool will be false by default if the box is not close to a wall
                boxbool = False
                #code a condition whether or not the box is close to a wall but not an obstacle
                #print(self.flat_dist)
                #print(boxX," ",boxY," ",self.Pos[2])
                self.boxbool = False
                 
                #storing the box info inside an array
                #boxinfo =  box_id,colour, x,y, x&y to push from,x&y to closest wall,baring for robot to push, bearing to get to coordinate
                boxinfo =[int(len(self.boxes))+1,origin.data[a+2],boxX,boxY]#,robX,robY,smallest_point[0],smallest_point[1],heading2box,heading2orbit]
                #checking whether or not the box is already in the list and thus dont add it again
                ##if there aren't any boxes add the first one seen
                boxfound = False
                boxfound2 = False
                if len(self.boxes) ==0:
                    self.boxes.append(boxinfo)


                #testing whether or not its in the list needing to be moved
                for box in self.boxes:
                    #using using euclidean distance to calculate the closeness to the coordinates
                    boxloc = [box[2],box[3]]
                    testloc= [boxX,boxY]
                    box_dist = math.dist(testloc,boxloc)
                    if box_dist>0.2:
                        boxfound = False
                    else:
                        boxfound = True
                        break

                #checking whether or not the box is in the already moved list
                
                for box in self.boxes_moved:
                    #using using euclidean distance to calculate the closeness to the coordinates
                    boxloc = [box[2],box[3]]
                    testloc= [boxX,boxY]
                    box_dist = math.dist(testloc,boxloc)
                    if box_dist>0.2:
                        boxfound2 = False
                    else:
                        boxfound2 = True
                        break

                #checking the distance to the wall that the box needs to go to
                
                if boxfound2 ==False and boxfound ==False:
                            
                    #add the box to the storage mechanism
                    self.boxes.append(boxinfo)

    def PointExtrap(self,wall):
        #calculating extra points along the plane/ wall the box could be pushed to

        wall_mut = copy.deepcopy(wall)

        for i in range(len(wall_mut)):
            wall_mut[i] = math.sqrt(wall_mut[i]**2)
        
        if wall_mut[0]>wall_mut[1]:
            
            #wall should be on the x plane
            #as wall is on x modify the y
            wall_len = round(wall_mut[0]*2)
            
            iterate_num = wall_len/0.05
            self.posGoals = []
            for i in range(1,int(iterate_num/2)+1):

                if self.green_wall == wall:

                    self.posGoals.append([wall[0],(wall[1]-(i*0.05))])
                
                    self.posGoals.append([wall[0],(wall[1]+(i*0.05))])
                else:
                    self.posGoals.append([wall[0],(wall[1]-(i*0.05))])
                
                    self.posGoals.append([wall[0],(wall[1]+(i*0.05))])
    

        else:
            #wall is probably on the y plane
            pass
       
        if self.green_wall == wall:
             self.posGoals.append([wall[0],wall[1]])
        else:
             self.posGoals.append([wall[0],wall[1]])

        
            


            
            
                    

def main():

    print('Start box memory')

    rclpy.init()

    box= BoxSort()

    rclpy.spin(box)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    box.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

   