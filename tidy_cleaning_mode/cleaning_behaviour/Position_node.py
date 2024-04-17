#ROS imports 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

#python imports
import math

class Positioning(Node):
    """Node for processing the odom data to a custom topic

        reads from /odom outputs to /Compressed_Odom

        converts quaternion into eular whilst also maintaining a stamp of the initial yaw value
        conversion function code was sourced from: https://stackoverflow.com/questions/56207448/efficient-quaternions-to-euler-transformation
    """
    #0:X, 1:Y, 2:yaw, 3:starting yaw
    Robo_pos = [0,1,2,3]
    start_yaw = None
    yaw = 0.0

    def __init__(self):
        super().__init__('GetPos')
        #subscribing to the odometry topic
        self.sub = self.create_subscription(Odometry,"/odom",self.PosCallback,1)
        #making a publisher to share compressed/ more relevant odom data
        self.Odom_pub = self.create_publisher(Float32MultiArray,"/Compressed_Odom",60)
        self.Comp_odom = Float32MultiArray()

    def PosCallback(self,data):
        if(data.child_frame_id == "base_link"):
   
            #position
            self.Robo_pos[0] = data.pose.pose.position.x
            self.Robo_pos[1] = data.pose.pose.position.y

            #orientation converted into euler
            #storing the quaternion
            quat = (
            data.pose.pose.orientation.x, #0
            data.pose.pose.orientation.y, #1
            data.pose.pose.orientation.z, #2
            data.pose.pose.orientation.w  #3
            )
            ##calculating the roll pitch and yaw
            ysqr = quat[1] * quat[1]

            t0 = +2.0 * (quat[3] * quat[0] + quat[1] * quat[2])
            t1 = +1.0 - 2.0 * (quat[0] * quat[0] + ysqr)
            #roll = math.degrees(math.atan2(t0, t1))

            t2 = +2.0 * (quat[3] * quat[1] - quat[2] * quat[0])
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            #pitch = math.degrees(math.asin(t2))

            t3 = +2.0 * (quat[3] * quat[2] + quat[0] * quat[1])
            t4 = +1.0 - 2.0 * (ysqr + quat[2] * quat[2])
            self.yaw = math.degrees(math.atan2(t3, t4))
            if self.start_yaw == None:
                self.start_yaw = self.yaw
            self.Robo_pos[2] = self.yaw
            self.Robo_pos[3] = self.start_yaw
            #publishing new compressed and relative information to custom topic
            self.Comp_odom.data = self.Robo_pos
            self.Odom_pub.publish(self.Comp_odom)

def main():
    print('Starting odom collecting')

    rclpy.init()

    odom= Positioning()

    rclpy.spin(odom)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()