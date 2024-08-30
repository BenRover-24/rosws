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
import serial
from benrover_interfaces.msg import ManageCorner
from sensor_msgs.msg import JointState


class DriverControl(Node):
     def __init__(self):
          super().__init__('servos_control') 
          driver_motors = ['right_front', 'right_back', 'right_middle', 'left_middle', 
                           'left_front', 'left_back'] 
          self.log = self.get_logger()
          self.log.info('Initialized Rover Driver Motor controller Node.')

          self.create_publisher(JointState, "corner_state", 1)
          self.subscription = self.create_subscription(
            ManageCorner,
            '/cmd_corner',
            self.cmd_corner_cb,
            10)
          self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        
     def cmd_driver_cb(self, cmd):
        self.get_logger().info(f'Received driver control commands.')
        ##angle = msg.data
        rows, cols = (4, 4)
        arr = [[0]*cols]*rows
        angles = []


        for ind, corner_name in zip(range(4), self.corner_motors):
            # store goal so we can estimate current angle
            angle = getattr(cmd, corner_name+"_pos") * RAD_TO_DEG
            # TODO make readable, cleaner
            self.corner_state_goal[ind] = (self.corner_state_goal[ind][0], angle)
            # offset to coordinate frame where x points to the middle of the rover, z down
            # and apply middle of actuation range offset, taking into account if servo is positive ccw or cw
            angle = self.centered_pulse_widths[ind] + self.servo_direction * angle
            self.log.debug(f"motor {corner_name} commanded to {angle}")
            # limit to operating range of servo
            angle = max(min(angle, self.servo_actuation_range), 0)
            # send to motor
            ##self.kit.servo[ind].angle = angle
        self.get_logger().info(f'Sent driver command to Arduino for movement')
        self.serial_port.write(f'{angles}\n'.encode('utf-8'))




def main(args=None):
    rclpy.init(args=args)

    driverController = DriverControl()

    rclpy.spin(driverController)
    driverController.destroy_node()
    rclpy.shutdown()

if __name__ == '__name__':
     main()
