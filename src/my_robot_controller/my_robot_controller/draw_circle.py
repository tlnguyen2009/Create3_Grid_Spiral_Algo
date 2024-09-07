#!/usr/bin/env python3
import rclpy #python package for ROS2
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("Draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist,"/cmd_vel",10) # /msg type /topic name /queue size
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        msg = Twist() #create a object of class Twist to send message 
        msg.linear.x = 0.5
        msg.angular.z = 0.25
        self.cmd_vel_pub_.publish(msg)



def main(args = None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()  