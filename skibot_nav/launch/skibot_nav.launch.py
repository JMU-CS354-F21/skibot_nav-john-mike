from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # By default (in Dashing) log messages are buffered and do not
    # immediately appear on the screen.  This is a hack to change that
    # behavior.
    lc = LaunchContext()
    SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED',
                           '1').visit(lc)

    return LaunchDescription([
        Node(
            package="skibot_nav",
            node_executable="skibot_nav_node",
            node_name="skibot_nav_node",
            output="screen",
            parameters=[
                {"p_gain_x":  0.3,
                 "i_gain_x":  0.0,
                 "d_gain_x":  0.2,
                 "i_max_x" :  5.0,
                 "i_min_x" :  0.0,
                 "p_gain_z":  0.3,
                 "i_gain_z":  0.0,
                 "d_gain_z":  0.2,
                 "i_max_z" :  5.0,
                 "i_min_z" :  0.0,}
            ]
        ),
        Node(
            package="skibot",
            node_executable="skibot_node",
            node_name="skibot_node",
            output="screen")

    ])
