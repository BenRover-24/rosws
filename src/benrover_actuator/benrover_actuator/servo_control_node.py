import rclpy
from rclpy.node import Node
import smbus
from geometry_msgs.msg import Twist

class ServoControlNode(Node):

    def __init__(self):
        super().__init__('servo_control_node')
        self.declare_parameter('servo_channel', 0) 
        self.declare_parameter('i2c_address', 0x40) 

        self.servo_channel = self.get_parameter('servo_channel').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.i2c_bus = smbus.SMBus(1) 

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        # (Exemple : conversion de la vitesse linéaire en angle de servo)
        angle = int(msg.linear.x * 90) 

        # Conversion de l'angle en valeur pour le PCA9685 (à adapter)
        pulse = self.angle_to_pulse(angle)

        try:
            self.i2c_bus.write_byte_data(self.i2c_address, self.servo_channel, pulse) 
            self.get_logger().info(f"Angle envoyé au servo: {angle} (pulse: {pulse})")
        except IOError as e:
            self.get_logger().error(f"Erreur I2C: {e}")

    def angle_to_pulse(self, angle):
        return int(angle / 180 * (4096 - 1)) 

def main(args=None):
    rclpy.init(args=args)
    servo_control_node = ServoControlNode()
    rclpy.spin(servo_control_node)
    servo_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()