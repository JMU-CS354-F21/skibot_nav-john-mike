#!/usr/bin/env python
  
"""Python skibot control node using PID controller.
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
from skibot_nav_interfaces.srv import SkibotToPoint


class GuidanceNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('skibot_nav_node')

        self.create_subscription(Pose, 'pose', self.location_callback, 10)
        self.create_subscription(Point, 'target_event', self.target_callback, 10)

        self.wrench_pub = self.create_publisher(Wrench, 'thrust', 10)

        # Set an arbitrary initial target
        self.target = Point()
        self.target.x = 3.0  
        self.target.y = 3.0
        self.target.z = 0.0

 # THESE SHOULD BE READ FROM PARAMETERS!
        #p_gain_x = 0.3
        #i_gain_x = 0.0
        #d_gain_x = 0.2
        #i_max_x = 5.0
        #i_min_x = 0.0

        #p_gain_z = 0.3
        #i_gain_z = 0.0
        #d_gain_z = 0.2
        #i_max_z = 5.0
        #i_min_z = 0.0

        self.declare_parameter('p_gain_x')
        self.declare_parameter('i_gain_x')
        self.declare_parameter('d_gain_x')
        self.declare_parameter('i_max_x')
        self.declare_parameter('i_min_x')

        self.declare_parameter('p_gain_z')
        self.declare_parameter('i_gain_z')
        self.declare_parameter('d_gain_z')
        self.declare_parameter('i_max_z')
        self.declare_parameter('i_min_z')

        p_gain_x = float(self.get_parameter('p_gain_x').value)
        i_gain_x = float(self.get_parameter('i_gain_x').value)
        d_gain_x = float(self.get_parameter('d_gain_x').value)
        i_max_x = float(self.get_parameter('i_max_x').value)
        i_min_x = float(self.get_parameter('i_min_x').value)

        p_gain_z = float(self.get_parameter('p_gain_z').value)
        i_gain_z = float(self.get_parameter('i_gain_z').value)
        d_gain_z = float(self.get_parameter('d_gain_z').value)
        i_max_z = float(self.get_parameter('i_max_z').value)
        i_min_z = float(self.get_parameter('i_min_z').value)

        self.pid_x = pid.PID(p_gain_x, i_gain_x, d_gain_x, i_min_x, i_max_x)
        self.pid_z = pid.PID(p_gain_z, i_gain_z, d_gain_z, i_min_z, i_max_z)

         # creating the SkibotToPoint service
        self.srv = self.create_service(SkibotToPoint, 'move_skibot_to_point', self.handle_toPoint_service)

    def location_callback(self, pose):

          thrust = Wrench()

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

          self.wrench_pub.publish(thrust) 

    def target_callback(self, target_msg):
        self.target = target_msg
        self.pid_x.reset()
        self.pid_z.reset()

    def handle_toPoint_service(self, SkibotToPoint_srv, response):

        if (SkibotToPoint_srv.goal_x < 0.0 or SkibotToPoint_srv.goal_x > 4.0 or SkibotToPoint_srv.goal_y < 0.0 or SkibotToPoint_srv.goal_y > 4.0):
            response.success = False
            return response

        self.target.x = float(SkibotToPoint_srv.goal_x)
        self.target.y = float(SkibotToPoint_srv.goal_y)
        self.target.z = 0.0
        response.success = True
        self.pid_x.reset()
        self.pid_z.reset()
        return response

def main():
    rclpy.init()
    nav_node = GuidanceNode()
    rclpy.spin(nav_node)

    nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
