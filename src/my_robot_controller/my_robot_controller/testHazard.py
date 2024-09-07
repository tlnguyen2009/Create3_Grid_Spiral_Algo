#!/qcc/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import ReliabilityPolicy, QoSProfile




def is_front_hazard_active(hazards): #Returns T/F if a hazard is detected
   for detection in hazards.detections:
       if (detection.type != 0):
           return True
           # get_logger().info("returned true")
       return False




class MoveRandom(Node):




   def __init__(self):
       super().__init__("move_random")
       self.cmd_vel_pub_ = self.create_publisher(
           Twist, "/cmd_vel", 10)
       self.hazard_subscriber_ = self.create_subscription(
           HazardDetectionVector, "/hazard_detection", self.hazard_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
       self._start_time = 0
       self.get_logger().info("Move Random has been started")
      
   def hazard_callback(self, hazards: HazardDetectionVector):
       if (is_front_hazard_active(hazards)):
        #    self._start_time = time.perf_counter()
           self.move(True)
       else:
           self.move(False)




   def move(self, hazard_detected):
       cmd = Twist()
       if (hazard_detected or self._start_time + 1.5 > time.perf_counter()):
           print("hazard")
           cmd.linear.x = 0.0
           cmd.angular.z = math.pi/2 #choose next grid around it randomly
       else:
           print("works")
           cmd.linear.x = 0.25
           cmd.angular.z = 0.0
       self.cmd_vel_pub_.publish(cmd)




def main(args=None):
   rclpy.init(args=args)
   node = MoveRandom()
   rclpy.spin(node)
   rclpy.shutdown()




if __name__ == '__main__':
   main()