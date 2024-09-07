
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi  
from rclpy.time import Time


class SquareController(Node):

    def __init__(self):
       super().__init__('square_controller')
       self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
       self.get_logger().info("Pathing in a grid spiral")
       self.speed_forward = 0.2 #speed for moving forward
       self.desire_distance = 0.4 #Distance for moving forward, it increases every 2 rotation
       self.check_rotation = 0 #check how many rotation
       self.state = "move_forward"
       self.next_publish_duration = 8.0 #this is the minimum time for robot to finish 90 degrees rotation
       self.timer = self.create_timer(self.next_publish_duration, self.move_spiral_pattern)

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
           self.publisher_.publish(twist) #publish to continue to send angular.z = 15 degree for the next loop
           t1 = self.get_clock().now().seconds_nanoseconds()[0]
           duration_sec = t1 - t0 
           current_angle = angular_speed * duration_sec
           self.get_logger().info(f'Rotate @ duration: {duration_sec} seconds out of {self.next_publish_duration}')

       #force robot to stop
        twist.angular.z = 0.0
        self.publisher_.publish(twist) #publish the new angular.z=0 to stop robot

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
         self.publisher_.publish(twist) # publish to continue to send linear.x = 0.2 for the next loop
         t1 = self.get_clock().now().seconds_nanoseconds()[0]
         duration_sec = t1 - t0
         current_distance = self.speed_forward * duration_sec
         self.get_logger().info(f'Move -> duration: within {duration_sec} seconds out of {self.next_publish_duration}')

      #force robot to stop
       twist.linear.x = 0.0
       self.publisher_.publish(twist) #publish the new linear.x = 0 to stop robot

   
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
       


def main(args=None):
   rclpy.init(args=args)
   square_controller = SquareController()
   rclpy.spin(square_controller)
   square_controller.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

