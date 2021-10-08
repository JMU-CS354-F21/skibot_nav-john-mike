#!/usr/bin/env python

"""Python rocket control node using PID controller.

Author: Nathan Sprague, John Curley, & Mike Leek
Version:

"""
import rclpy
import rclpy.node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Wrench

from jmu_ros2_util import pid


class NavigationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('skibot_nav_node')

        self.create_subscription(Point, 'location', self.location_callback, 10)
        self.create_subscription(Point, 'target_event', self.target_callback,
                                 10)

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

        log_str = "P gain: {}, I gain: {}, D gain: {}"
        self.get_logger().info(log_str.format(p_gain, i_gain, d_gain))

    def location_callback(self, location):
        thrust = Wrench()
        

        thrust.force.x = self.pid.update_PID(self.target.y - location.y)


        self.thrust_pub.publish(thrust)

    def target_callback(self, target_msg):
        self.pid.reset()
        self.target = target_msg


def main():
    rclpy.init()
    nav_node = NavigationNode()
    rclpy.spin(nav_node)

    nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
