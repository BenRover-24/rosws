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


class RoverNavigation(Node):
     def __init__(self):
          super().__init__('benrover_navigation') 
          self.log = self.get_logger()
          self.log.info('Initialized Rover Navigation Module.')

          self.declare_parameters(
            namespace='',
            parameters=[
                ('rover_dimensions.d1', Parameter.Type.DOUBLE),
                ('rover_dimensions.d2', Parameter.Type.DOUBLE),
                ('rover_dimensions.d3', Parameter.Type.DOUBLE),
                ('rover_dimensions.d4', Parameter.Type.DOUBLE),
                ('rover_dimensions.wheel_radius', Parameter.Type.DOUBLE)
                ##('drive_no_load_rpm', Parameter.Type.DOUBLE)
            ]
          )
          
          self.min_radius = 0.45  # [m]
          self.max_radius = 6.4  # [m]

          ##self.d1 = 0.177
          ##self.d2 = 0.310
          ##self.d3 = 0.234
          ##self.d4 = 0.355
          ##self.wheel_radius = 0.4
          self.d1 = self.get_parameter('rover_dimensions.d1').get_parameter_value().double_value
          self.d2 = self.get_parameter('rover_dimensions.d2').get_parameter_value().double_value
          self.d3 = self.get_parameter('rover_dimensions.d3').get_parameter_value().double_value
          self.d4 = self.get_parameter('rover_dimensions.d4').get_parameter_value().double_value

          self.wheel_radius = self.get_parameter(
            "rover_dimensions.wheel_radius").get_parameter_value().double_value  # [m]
          self.max_vel =  250 ##self.wheel_radius * drive_no_load_rpm / 60 * 2 * math.pi  # [m/s]
      
          self.curr_positions = {}
          self.curr_velocities = {}
          self.curr_twist = TwistWithCovariance()
          self.curr_turning_radius = self.max_radius
          
          self.cmd_vel_int_sub = self.create_subscription(Twist, "/cmd_vel_intuitive", 
                                                        self.command_callback, 1)

          self.turning_radius_pub = self.create_publisher(Float64, "/turning_radius", 1)

                    
     def command_callback(self, twist_msg):
        if twist_msg.angular.y and not twist_msg.linear.x:
            # command corners to point to center
            corner_cmd_msg, drive_cmd_msg = self.calculate_rotate_in_place_cmd(twist_msg)
        
        else:
            desired_turning_radius = self.twist_to_turning_radius(twist_msg)
            self.get_logger().debug("desired turning radius: " + "{}".format(desired_turning_radius), throttle_duration_sec=1)
            corner_cmd_msg = self.calculate_corner_positions(desired_turning_radius)

            # if we're turning, calculate the max velocity the middle of the rover can go
            max_vel = abs(desired_turning_radius) / (abs(desired_turning_radius) + self.d1) * self.max_vel
            if math.isnan(max_vel):  # turning radius infinite, going straight
                max_vel = self.max_vel
            velocity = min(max_vel, twist_msg.linear.x)
            self.get_logger().debug("velocity drive cmd: {} m/s".format(velocity), throttle_duration_sec=1)

            drive_cmd_msg = self.calculate_drive_velocities(velocity, desired_turning_radius)

        # if self.corner_cmd_threshold(corner_cmd_msg):
        self.get_logger().debug("drive cmd:\n{}".format(drive_cmd_msg), throttle_duration_sec=1)
        self.get_logger().debug("corner cmd:\n{}".format(corner_cmd_msg), throttle_duration_sec=1)

        self.corner_cmd_pub.publish(corner_cmd_msg)
        self.drive_cmd_pub.publish(drive_cmd_msg)



     def twist_to_turning_radius(self, twist, clip=True):
        """
        Convert a commanded twist into an actual turning radius

        ackermann steering: if l is distance travelled, rho the turning radius, and theta the heading of the middle of the robot,
        then: dl = rho * dtheta. With dt -> 0, dl/dt = rho * dtheta/dt
        dl/dt = twist.linear.x, dtheta/dt = twist.angular.z

        :param twist: geometry_msgs/Twist. Only linear.x and angular.z are used
        :param clip: whether the values should be clipped from min_radius to max_radius
        :param intuitive_mode: whether the turning radius should be mathematically correct (see cmd_cb()) or intuitive
        :return: physical turning radius in meter, clipped to the rover's limits
        """
        try:
            if twist.linear.x < 0:
                radius = twist.linear.x / -twist.angular.z
            else:
                radius = twist.linear.x / twist.angular.z
        except ZeroDivisionError:
                return float("Inf")

        # clip values so they lie in (-max_radius, -min_radius) or (min_radius, max_radius)
        if not clip:
            return radius
        if radius == 0:
                if twist.angular.z == 0:
                    return self.max_radius
                else:
                    radius = self.min_radius * self.max_vel / twist.angular.z  # proxy  
        if radius > 0:
            radius = max(self.min_radius, min(self.max_radius, radius))
        else:
            radius = max(-self.max_radius, min(-self.min_radius, radius))

        return radius

def main(args=None):
    rclpy.init(args=args)

    rover_navigation = RoverNavigation()

    rclpy.spin(rover_navigation)
    rover_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__name__':
     main()
