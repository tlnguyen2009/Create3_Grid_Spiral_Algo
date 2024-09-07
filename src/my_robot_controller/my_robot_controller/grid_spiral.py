#!/usr/bin/env python3
import math 
import rclpy #python package for ROS2
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import RotateAngle
from geometry_msgs.msg import Pose2D

class GridSpiral(Node):
    
    def __init__(self):
        super().__init__("grid_spiral")
        self.count = 1.0

        self.cmd_vel_pub_ = self.create_publisher(Twist,"/cmd_vel",10) # /msg type /topic name /queue size
        self.rotate_angle_pub = self.create_publisher(Pose2D, 'topic', 10)

        self.timer_ = self.create_timer(1.0, self.send_velocity_command)
        
        self.get_logger().info("Pathing in a grid spiral")

    def send_velocity_command(self):
        msg = Twist() #create a object of class Twist to send message 
        rTate = Pose2D()
        rTate.theta = 90.0
        self.rotate_angle_pub.publish(rTate)
        
        

        # msg.linear.x = self.count
        # msg.linear.x = 2.0s
        
        # msg.linear.x = 0.0
        # msg.angular.z = 0.5
        msg.linear.x = 0.1


        # msg.linear.y = 1.0

        self.cmd_vel_pub_.publish(msg)
    


    
        


def main(args = None):
    rclpy.init(args=args)
    node = GridSpiral()
    rclpy.spin(node)
    rclpy.shutdown()  