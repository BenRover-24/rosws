import rclpy
from rclpy.node import Node
import smbus
from test.msg import ServoCmd 

class ServoControlNode(Node):

    def __init__(self):
        super().__init__('servo_control_node')
        self.declare_parameter('servo_channel', 0)  # Canal du servo sur le PCA9685
        self.declare_parameter('i2c_address', 0x40)  # Adresse I2C du PCA9685

        self.servo_channel = self.get_parameter('servo_channel').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.i2c_bus = smbus.SMBus(1)  # Bus I2C 1 sur RPi

        self.subscription = self.create_subscription(
            ServoCmd,
            'servo_cmd',  # Topic pour recevoir les commandes
            self.servo_cmd_callback,
            10
        )

    def servo_cmd_callback(self, msg):
        angle = msg.angle
        # Conversion de l'angle en valeur pour le PCA9685 (à adapter)
        pulse = self.angle_to_pulse(angle)

        try:
            self.i2c_bus.write_byte_data(self.i2c_address, self.servo_channel, pulse) 
            self.get_logger().info(f"Angle envoyé au servo: {angle} (pulse: {pulse})")
        except IOError as e:
            self.get_logger().error(f"Erreur I2C: {e}")

    def angle_to_pulse(self, angle):
        # Conversion d'angle à pulse (à calibrer)
        return int(angle / 180 * (4096 - 1))  # Exemple pour servo 0-180 degrés

def main(args=None):
    rclpy.init(args=args)
    servo_control_node = ServoControlNode()
    rclpy.spin(servo_control_node)
    servo_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()