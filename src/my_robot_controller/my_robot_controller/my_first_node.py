#!/usr/bin/env python3
import rclpy #python package for ROS2
from rclpy.node import Node

# create a derived class "MyNode" of base class "Node"
class MyNode(Node):

    # constructor calling constructor "super" from upper class which would allow us to use ROS2 functionalitiy 
    def __init__(self):
        super().__init__("first_node") # < name of the node when we run it in the graph and ros fuctionality 
        self.get_logger().info("I hate you") # the prefix self will get the functionalities of class "Node"
    def firstrun(self):
        print("Hello from ROS2")


def main(args=None):
    rclpy.init(args=args) # open ros2 communication 
    node = MyNode() # this is how we create a object of node "MyNode"
    node.firstrun()
    
    rclpy.spin(node) # this command keeps the object "node" alive
    rclpy.shutdown() # shutdown the ros2 communicaton or current node

# this function is for to call main from the terminal
if __name__ == '__main__':
    main()



