#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray,Float32MultiArray

#Python imports
import math


class BehaviourController(Node):
    """Node for maintaining a recollection of the boxes info

        Inputs:
       
        /Brain_Respond takes a response from a topic to change the behaviour
        /Compressed_Odom, inputting the robot location with stamped yaw
        /WallProcessed gets the wall information

        Output:
        
        /conductor takes commands from the main brain
        
    """
    #state will be used to control the robot in essence becoming a high level conductor
    #0 = start scan, rotate 360 degrees find any boxes in local area
    #1 = moving boxes other behaviours for doing this will controlled on the lower level
    state = [0.0,0.0,0.0]

    #box information
    boxes2move =[]
    boxesMoved = []
    green_boxes= [0.0]
    red_boxes = [0.0]
    box_index = 0

    #wall information
    known_walls = [[-1.5,0.0],[0.0,-1.5],[1.5,0.0],[0.0,1.5]]
    walls= []
    green_wall = [None]
    red_wall = [None]
    
    def __init__(self):
        super().__init__('Thinking')
        ##creating a publisher for to "broadcast" the behavior
        self.behaviour_pub = self.create_publisher(Float32MultiArray,"/Conductor",60)
        # subscribing to the responses of the second "brain"
        self.response_sub = self.create_subscription(Float32MultiArray,"/Brain_Respond",self.responseCallback,1)
        #subscribing to the bpx processing node
        self.box_infosun = self.create_subscription(Float32MultiArray,"/BoxInfo",self.BoxCallback,1)
        #subscribing the the wall topic for the robot to give out a guess where the walls are
        self.wall_get = self.create_subscription(Float32MultiArray,"/WallProcessed",self.wallCallback,1)

        #creating a time controlled loop for general processes that arent event driven
        timer_period = 0.01  # seconds
        self.updater = self.create_timer(timer_period, self.Thinking)
        self.state_message = Float32MultiArray()

    def wallCallback(self,msg):
        #checking that the values have been filled in properly
        count = 0
        for i in msg.data:
            if i == 0.0:
                count+=1
        if count<2:
            if self.red_wall[0]==None and self.green_wall[0]==None:
                self.red_wall =msg.data[2:]
                self.green_wall = msg.data[:2]

    def Thinking(self):
        #checking if certain criteria have been met to perform certain tasks

        #default state will be the first state which will make the robot turn 360 degrees to assess the environment for box to move
        self.state_message.data = self.state
        self.behaviour_pub.publish(self.state_message)
    
    def responseCallback(self,resp):
        if 1.0 == resp.data[0]:
            self.state = [1.0,self.green_wall[0],self.green_wall[1],self.red_wall[0],self.red_wall[1]]
        if 2.0 == resp.data[0]:
            self.state = [0.0,0.0,0.0]

    ##this method will be used in order to get the box information and make decisions such as what walls to push the boxes towards, which box to push etc
    def BoxCallback(self,boxes):
        #firstly the data is all compiled together so first the data has to be split back up
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
       
def main():
    print('IM THINKING')

    rclpy.init()

    think= BehaviourController()

    rclpy.spin(think)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    think.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
