#!/usr/bin/env python

"""Python rocket control node using PID controller.

Author: Nathan Sprague, John Curley, & Mike Leek
Version:

"""
import rclpy
import rclpy.node
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Wrench
<<<<<<< HEAD

=======
>>>>>>> a978185bc1863d8171d01f77f0984541ff07109c
from jmu_ros2_util import pid
from skibot_interfaces.msg import Pose



class NavigationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('skibot_nav_node')

        #self.create_subscription(Point, 'location', self.location_callback, 10)
        self.create_subscription(Pose, 'pose', self.location_callback, 10)
        self.create_subscription(Point, 'target_event', self.target_callback,
                                 10)

<<<<<<< HEAD
        self.thrust_pub = self.create_publisher(Wrench, 'thrust', 10)

        # Set an arbitrary initial target
        self.target = Point()
        self.target.x = 50.0
        self.target.y = 0.0
        self.target.z = 0.0

        # THESE SHOULD BE READ FROM PARAMETERS!
        p_gain = 0.3
        i_gain = 0.2
        d_gain = 0.2
        i_max = 5.0
        i_min = 0.0

        self.pid = pid.PID(p_gain, i_gain, d_gain, i_min, i_max)
=======
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
>>>>>>> a978185bc1863d8171d01f77f0984541ff07109c

        #log_str = "P gain: {}, I gain: {}, D gain: {}"
       # self.get_logger().info(log_str.format(p_gain, i_gain, d_gain))

<<<<<<< HEAD
    def location_callback(self, location):
        thrust = Wrench()
        

        thrust.force.x = self.pid.update_PID(self.target.y - location.y)
=======
    def location_callback(self, pose):
#        thrust = Vector3()
         # need to initialize the wrench msg  
          wrench = Wrench()
         # need to create triangle and determine if 
          x_triangle = self.target.x - pose.x
          y_triangle = self.target.y - pose.y
# check if the arctan is the right function
          desired_angle = np.arctan2(y_triangle, x_triangle)
          
          if (desired_angle == pose.theta):
            wrench.torque.z = 0
 
            pass
          elif (desired_angle > pose.theta):
            wrench.torque.z = self.pid_z.update_PID(desired_angle - pose.theta)

            pass
          
# need different behavior based on which way to turn
          elif (desired_angle < self.pos.theta):
            wrench.torque.z = self.pid_z.update_PID(desired_angle - self.pos.theta)

            pass
            distance =np.sqrt (np.square(x_triangle) + np.square(y_triangle))
            if distance > 0.2:
              wrench.force.x = self.pid_x.update_PID(distance)
            else:
              wrench.force.x = 0
            self.wrench_pub.publish(wrench) 
          
>>>>>>> a978185bc1863d8171d01f77f0984541ff07109c


        self.thrust_pub.publish(thrust)

    def target_callback(self, target_msg):
<<<<<<< HEAD
        self.pid.reset()
        self.target = target_msg

=======
        self.target = target_msg
        self.pid_x.reset()
        self.pid_z.reset()
>>>>>>> a978185bc1863d8171d01f77f0984541ff07109c

def main():
    rclpy.init()
    nav_node = NavigationNode()
    rclpy.spin(nav_node)

    nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
