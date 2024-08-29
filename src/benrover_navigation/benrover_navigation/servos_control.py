import math
##from functools import partial

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf2_ros
import serial

##from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistWithCovariance, TransformStamped
##from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
##from benrover_interfaces.msg import CommandDrive, CommandCorner


class ServoControl(Node):
     def __init__(self):
          super().__init__('servos_control') 
          corner_motors = ['right_front', 'right_back', 'left_front', 'left_back'] 
          self.log = self.get_logger()
          self.log.info('Initialized Rover Servo controller Node.')

def main(args=None):
    rclpy.init(args=args)

    servosController = ServoControl()

    rclpy.spin(servosController)
    servosController.destroy_node()
    rclpy.shutdown()

if __name__ == '__name__':
     main()
