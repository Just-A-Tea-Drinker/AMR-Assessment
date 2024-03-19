import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Int32MultiArray,Float32MultiArray

##OTHER library imports
import cv2
import numpy as np

class ImageProcessing(Node):
    origins = []
    wallsInfo = []
    state = 0.0
    def __init__(self):
        super().__init__('ImagePRO')
        #subscribing to get the image to process
        self.sub = self.create_subscription(CompressedImage, '/limo/depth_camera_link/image_raw/compressed', self.Cam_callback, 10)
        #making an instance of open cv
        self.br = CvBridge()
        ##making a publisher for recording the walls
        self.wall_pub  =self.create_publisher(Int32MultiArray,'/WallInfo',60)
        self.walls = Int32MultiArray()
        #creating a publisher to return publish the origin of the largest contour
        self.Cont_Origin = self.create_publisher(Int32MultiArray, '/Cont_Origin',60)
        ##setting up a subscriber to read brain commands
        self.beh_get = self.create_subscription(Float32MultiArray,'/Conductor',self.CommandCallback,1)
        self.Origin = Int32MultiArray()
        
    def CommandCallback(self,command):
        self.state = command.data[0]
        self.green_wall = command.data[1:3]
        self.red_wall = command.data[3:]
        # if self.state == 1.0 and self.target_box!=None:
        #     self.PathPlan()
    def Cam_callback(self,data):
        #setting up the windows to show the camera and the cv processing
        
            cv2.namedWindow("Live CAM",1)
            #cv2.namedWindow("canned feed")
            #cv2.namedWindow("Ori prediction")

            #converting the ROS image into the openCV
            cv_image = self.br.compressed_imgmsg_to_cv2(data,desired_encoding='bgr8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            canny_img = cv2.Canny(gray_img, 100, 200)
            
            

            #finding the color green
            green_frame_mask = cv2.inRange(cv_image,(40, 150, 50), (80, 255, 255))
            green_contours, hierarchy = cv2.findContours(green_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            green_frame_contours = cv2.drawContours(cv_image, green_contours, -1, (0, 255, 0), 1)

            red1 = cv2.inRange(cv_image,(0, 150, 50), (15, 255, 255))
            red2 = cv2.inRange(cv_image,(170, 150, 50), (180, 255, 255))
            red_frame_mask = red1+red2
            red_contours, hierarchy = cv2.findContours(red_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            red_frame_contours = cv2.drawContours(cv_image, red_contours, -1, (0, 255, 0), 1)
            # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
            #current_frame_contours = cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 1)
            self.origins = []
            self.wallsInfo = []
            if len(green_contours) > 0:
                    
                    for i in range(len(green_contours)):
                        # finding the center of the largest of contours 
                        M = cv2.moments(green_contours[i]) # only select the largest 
                        # find the centroid of the contour
                        if M['m00'] > 0:
                            
                            # find the centroid of the contour
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            
                            # Draw a circle centered at centroid coordinates
                            # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                            cv2.circle(cv_image, (round(cx), round(cy)), 1, (0, 0, 0), -1)
                            cv2.line(cv_image, (320, 0), (320, 480), (0, 0, 0), thickness=1)
                            cv2.line(cv_image, (0, 240), (640, 240), (0, 0, 0), thickness=1)

                            
                            #width: 640
                            #height 480
                            #this topic will make a suggestion to the course of action based on what it sees
                            #PUBLISH THAT A BOX HAS BEEN FOUND WHICH COULD ACTIVATE ANOTHER NODE
                            # if origins[2] = 1 its green,0 its red
                            if self.state == 0.0:
                                if cy<240:
                                    ##this is a green wall not a box
                                    self.wallsInfo.append(cx)
                                    self.wallsInfo.append(cy)
                                    self.wallsInfo.append(1)
                                    self.walls.data = self.wallsInfo
                                    self.wall_pub.publish(self.walls)
                                else:
                                    self.origins.append(cx)
                                    self.origins.append(cy)
                                    self.origins.append(1)
                                    self.Origin.data = self.origins
                                    self.Cont_Origin.publish(self.Origin)
            if len(red_contours) > 0:
                    
                    for i in range(len(red_contours)):
                        # finding the center of the largest of contours 
                        M = cv2.moments(red_contours[i]) # only select the largest 
                        # find the centroid of the contour
                        if M['m00'] > 0:
                            
                            # find the centroid of the contour
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            
                            # Draw a circle centered at centroid coordinates
                            # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                            cv2.circle(cv_image, (round(cx), round(cy)), 1, (0, 0, 0), -1)
                            cv2.line(cv_image, (320, 0), (320, 480), (0, 0, 0), thickness=1)
                            cv2.line(cv_image, (0, 240), (640, 240), (0, 0, 0), thickness=1)

                            
                            #width: 640
                            #this topic will make a suggestion to the course of action based on what it sees
                            #PUBLISH THAT A BOX HAS BEEN FOUND WHICH COULD ACTIVATE ANOTHER NODE
                            # if origins[2] = 1 its green,0 its red
                            if self.state == 0.0:
                                if cy<240:
                                    ##this is a red wall not a box
                                    self.wallsInfo.append(cx)
                                    self.wallsInfo.append(cy)
                                    self.wallsInfo.append(0)
                                    self.walls.data = self.wallsInfo
                                    self.wall_pub.publish(self.walls)
                                else:
                                    self.origins.append(cx)
                                    self.origins.append(cy)
                                    self.origins.append(0)
                                    self.Origin.data = self.origins
                                    self.Cont_Origin.publish(self.Origin)
                

            
            #DISPLAY ONLY
            
            
        
            #cv2.imshow("Ori prediction", canny_img)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_HSV2BGR)
            cv2.imshow("Live CAM", cv_image)
            cv2.waitKey(1)


def main():
    print('Starting image processing')

    rclpy.init()

    IMGpro= ImageProcessing()

    rclpy.spin(IMGpro)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    IMGpro.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()