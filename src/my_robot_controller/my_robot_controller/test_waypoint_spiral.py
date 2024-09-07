#Authors: Daniel Adrujo & Tien Luc Nguyen
#2024

#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import tf_transformations
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from math import pi




def is_front_hazard_active(hazards): #Returns T/F if a hazard is detected
   for detection in hazards.detections:
       if (detection.type != 0):
           return True
       return False


class MoveToPoints(Node):


   def __init__(self, waypoints: list[PoseStamped]):
           super().__init__("move_to_points")


           self.nav = BasicNavigator() # --start up navigator + setup waypoints--
           self.nav.waitUntilNav2Active # ---------------------------------------
           self.waypoints = waypoints #  ----------------------------------------
           self.count = 0
        #    self.move_to_waypoint(waypoints[0])


           self.publisher = self.create_publisher(Twist, "/cmd_vel", 10) # --GRID SPIRAL IMPLEMENTATION --
           self.speed_forward = 0.2 #speed for moving forward
           self.desire_distance = 0.4 #Distance for moving forward, it increases every 2 rotation
           self.check_rotation = 0 #check how many rotation
           self.state = "move_forward"
           self.next_publish_duration = 8.0 #this is the minimum time for robot to finish 90 degrees rotation
           self.callback = True
           self.tcallback = MutuallyExclusiveCallbackGroup()
           self.scallback = MutuallyExclusiveCallbackGroup()
           self.timer = self.create_timer(self.next_publish_duration, self.move_spiral_pattern, callback_group=self.tcallback) # ----------
           self.timer.cancel()
          
           self.hazard_detected = False
           self.hazard_subscriber_ = self.create_subscription(
               HazardDetectionVector, "/hazard_detection", self.hazard_callback , QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.scallback)
          
           self.start_time = 0
           self.get_logger().info("MoveToPoints has been started")
           # self.move(False)


   # Rotate 90 degrees with this fuction
   def rotate(self):
        twist = Twist()




        angular_speed = 15*2*pi/360 #rotate x = 15 degrees/sec (rad/s) -> will finish around 6 - 8 seconds
        relative_angle = 90*2*pi/360 #90 degrees (rad)




        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed




        t0 = self.get_clock().now().seconds_nanoseconds()[0]
        current_angle = 0.0




        while (current_angle <= relative_angle):
            self.publisher.publish(twist) #publish to continue to send angular.z = 15 degree for the next loop
            t1 = self.get_clock().now().seconds_nanoseconds()[0]
            duration_sec = t1 - t0
            current_angle = angular_speed * duration_sec
            self.get_logger().info(f'Rotate @ duration: {duration_sec} seconds out of {self.next_publish_duration}')




        #force robot to stop
        twist.angular.z = 0.0
        self.publisher.publish(twist) #publish the new angular.z=0 to stop robot




   # Move forward
   def move_forward(self):
       twist = Twist()




       twist.linear.x = self.speed_forward # 0.2m/s forward
       twist.linear.y = 0.0
       twist.linear.z = 0.0
       twist.angular.x = 0.0
       twist.angular.y = 0.0
       twist.angular.z = 0.0
      
       t0 = self.get_clock().now().seconds_nanoseconds()[0]
       current_distance = 0.0
       distance = self.desire_distance #distance robot needs to finish




       while (current_distance < distance):
           self.publisher.publish(twist) # publish to continue to send linear.x = 0.2 for the next loop
           t1 = self.get_clock().now().seconds_nanoseconds()[0]
           duration_sec = t1 - t0
           current_distance = self.speed_forward * duration_sec
           self.get_logger().info(f'Move -> duration: within {duration_sec} seconds out of {self.next_publish_duration}')




       #force robot to stop
       twist.linear.x = 0.0
       self.publisher.publish(twist) #publish the new linear.x = 0 to stop robot


       # Spiral pattern
   def move_spiral_pattern(self):   
       if self.state == "move_forward":
           self.check_rotation = self.check_rotation + 1
           self.move_forward()




           if self.check_rotation == 2:
               self.desire_distance = self.desire_distance + 0.4 # add +0.4m every 2 rotations, 0.4m is the width of the robot
               self.check_rotation = 0




           self.state = "rotate" #change state to rotate after every moving forward
           self.next_publish_duration = 8.0 #8.0 seconds is enough for robot to finish a rotation




       else: #when self.state == "rotate"
           self.rotate()
           self.state = "move_forward" #change back to move forward after every rotation




           #calculate next_publish_duration for robot moving forward
           publish_duration = (self.desire_distance / self.speed_forward) + 2.0 # time = (distance/speed) + 2.0 seconds for errors
          
           #  if(publish_duration > 8.0)
           self.next_publish_duration = publish_duration    
   


  
  
   def hazard_callback(self, hazards: HazardDetectionVector):
          
       if (is_front_hazard_active(hazards) and self.count < 3):
           self.timer.cancel()
           self.move_to_waypoint(self.waypoints[self.count])
           self.count = self.count + 1
           self.callback = True
        #    self.start_time = time.perf_counter()
       elif  is_front_hazard_active(hazards):
           twist = Twist()
           twist.linear.x = 0.0
           twist.angular.z = 0.0
           self.publisher.publish(twist)
       elif self.callback :   
           self.speed_forward = 0.2 #speed for moving forward
           self.desire_distance = 0.4 #Distance for moving forward, it increases every 2 rotation
           self.check_rotation = 0 #check how many rotation
           self.state = "move_forward"
           self.next_publish_duration = 8.0 #this is the minimum time for robot to finish 90 degrees rotation
           self.timer.reset()
           self.callback = False
           # self.move(False)
  
  
   def move_to_waypoint(self, waypoint: PoseStamped):
       self.nav.goToPose(waypoint)
       while not self.nav.isTaskComplete():
           feedback = self.nav.getFeedback()
           print(feedback)






def create_pose_stamped(navigator, position_x, position_y, rotation_z):
   q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
   goal_pose = PoseStamped()
   goal_pose.header.frame_id = 'map'
   goal_pose.header.stamp = navigator.get_clock().now().to_msg()
   goal_pose.pose.position.x = position_x
   goal_pose.pose.position.y = position_y
   goal_pose.pose.position.z = 0.0
   goal_pose.pose.orientation.x = q_x
   goal_pose.pose.orientation.y = q_y
   goal_pose.pose.orientation.z = q_z
   goal_pose.pose.orientation.w = q_w
   return goal_pose


def main(args = None):
   # --- Init ROS2 communications and Simple Commander API ---
   rclpy.init(args=args)
   nav = BasicNavigator()


   # --- Set initial pose ---
   # !!! Comment if the initial pose is already set !!!
   # initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
   #nav.setInitialPose(initial_pose)


   # --- Wait for Nav2 ---
   nav.waitUntilNav2Active()


   # --- Create some Nav2 goal poses ---
   goal_pose1 = create_pose_stamped(nav, -0.459, -1.49, 0.005)
   goal_pose2 = create_pose_stamped(nav, 0.643, -2.12, 0.005)
   goal_pose3 = create_pose_stamped(nav, 1.06, -3.11, 0.005)


   # --- Going to one pose ---
   # nav.goToPose(goal_pose1)
   # while not nav.isTaskComplete():
   #         feedback = nav.getFeedback()
   #         # print(feedback)


   # --- Follow Waypoints ---
   waypoints = [goal_pose1, goal_pose2, goal_pose3]
       # for i in range(3):
       #     nav.followWaypoints(waypoints)
       #     while not nav.isTaskComplete():
       #         feedback = nav.getFeedback()
       #     print(feedback)
   node = MoveToPoints(waypoints)
   executor = MultiThreadedExecutor()
   executor.add_node(node)
   executor.spin()
   # --- Get the result ---
   # print(nav.getResult())

   # --- Shutdown ROS2 communications ---
   rclpy.shutdown()


if __name__ == '__main__':
   main()