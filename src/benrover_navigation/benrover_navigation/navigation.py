import math
from functools import partial

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from osr_interfaces.msg import CommandDrive, CommandCorner


class RoverNavigation(Node):
     def __init__(self):
          super().__init__('rover_naviguation') 
          self.log = self.get_logger()
          self.log.info('Initialized Rover.')

          self.declare_parameters(
            namespace='',
            parameters=[
                ('rover_dimensions.d1', Parameter.Type.DOUBLE),
                ('rover_dimensions.d2', Parameter.Type.DOUBLE),
                ('rover_dimensions.d3', Parameter.Type.DOUBLE),
                ('rover_dimensions.d4', Parameter.Type.DOUBLE),
                ('rover_dimensions.wheel_radius', Parameter.Type.DOUBLE),
                ('drive_no_load_rpm', Parameter.Type.DOUBLE),
                ('enable_odometry', Parameter.Type.BOOL)
            ]
          )
          self.cmd_vel_int_sub = self.create_subscription(Twist, "/cmd_vel_intuitive", 
                                                        self.command_callback, 1)

          self.turning_radius_pub = self.create_publisher(Float64, "/turning_radius", 1)
          if self.should_calculate_odom:
            self.odometry_pub = self.create_publisher(Odometry, "/odom", 2)
            self.tf_pub = tf2_ros.TransformBroadcaster(self)

          self.corner_cmd_pub = self.create_publisher(CommandCorner, "/cmd_corner", 1)
          self.drive_cmd_pub = self.create_publisher(CommandDrive, "/cmd_drive", 1)

          
          
    ## def command_callback(self, twist_msg):


     async def joy_callback(self, msg):
          self.log.info('Just received a command.')
          button = msg.buttons

          

def main(args=none):
    rclpy.init(args=args)

    rover_naviguation = RoverNavigation()

    rclpy.spin(rover_naviguation)
    rover_naviguation.destroy_node()
    rclpy.shutdown()

if __name__ == '__name__':
     main()
