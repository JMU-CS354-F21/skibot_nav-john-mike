#!/usr/bin/env python
  
"""Python rocket control node using PID controller.

Author: Nathan Sprague & John Curley & Michael Leek
Version:

"""
import rclpy
import rclpy.node
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Wrench
from jmu_ros2_util import pid
from skibot_interfaces.msg import Pose



class GuidanceNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('skibot_nav_node')

        print("I init")
        #self.create_subscription(Point, 'location', self.location_callback, 10)
        self.create_subscription(Pose, 'pose', self.location_callback, 10)
        self.create_subscription(Point, 'target_event', self.target_callback,
                                 10)

        self.wrench_pub = self.create_publisher(Wrench, 'thrust', 10)

        # Set an arbitrary initial target
        self.target = Point()
        self.target.x = 3.0  
        self.target.y = 3.0
        self.target.z = 0.0
 # THESE SHOULD BE READ FROM PARAMETERS!
        p_gain_x = 0.3
        i_gain_x = 0.2
        d_gain_x = 0.2
        i_max_x = 5.0
        i_min_x = 0.0

        p_gain_z = 0.3
        i_gain_z = 0.2
        d_gain_z = 0.2
        i_max_z = 5.0
        i_min_z = 0.0



        self.pid_x = pid.PID(p_gain_x, i_gain_x, d_gain_x, i_min_x, i_max_x)
        self.pid_z = pid.PID(p_gain_z, i_gain_z, d_gain_z, i_min_z, i_max_z)
       # self.pid = pid.PID(p_gain, i_gain, d_gain, i_min, i_max)

        #log_str = "P gain: {}, I gain: {}, D gain: {}"
       # self.get_logger().info(log_str.format(p_gain, i_gain, d_gain))

    def location_callback(self, pose):
#        thrust = Vector3()
          print("location callback")
        #  print(self.get_logger().info())
         # need to initialize the wrench msg  
          thrust = Wrench()
         # thrust.force.x = 1.0
         # thrust.torque.z = 0.1

          print(thrust.force.x)
          print(thrust.torque.z)
         # need to create triangle and determine if 
          x_triangle = self.target.x - pose.x
          y_triangle = self.target.y - pose.y
# check if the arctan is the right function
          desired_angle = np.arctan2(y_triangle, x_triangle)
          if (desired_angle == pose.theta):
            thrust.torque.z = 0
 
          elif (desired_angle > pose.theta):
            thrust.torque.z = self.pid_z.update_PID(desired_angle - pose.theta)

          
# need different behavior based on which way to turn
          elif (desired_angle < pose.theta):
            thrust.torque.z = self.pid_z.update_PID(desired_angle - pose.theta)

          distance =np.sqrt (np.square(x_triangle) + np.square(y_triangle))
          if distance > 0.1:
            thrust.force.x = self.pid_x.update_PID(distance)
          else:
            thrust.force.x = 0.0
         # thrust.force.x = 1.0
          print(thrust.force.x)
          print(thrust.torque.z)
          print(thrust)
          self.wrench_pub.publish(thrust) 
          

 #       if location.y < self.target.y:
  #          thrust.y = self.pid.update_PID(self.target.y - location.y)

#        self.thrust_pub.publish(thrust)

    def target_callback(self, target_msg):
        self.target = target_msg
        self.pid_x.reset()
        self.pid_z.reset()

def main():
    rclpy.init()
    thruster_node = GuidanceNode()
    rclpy.spin(thruster_node)

    thruster_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

