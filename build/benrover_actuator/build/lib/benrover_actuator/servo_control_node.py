import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray 
import serial

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')

        # Configuration de la liaison série
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Paramètre pour le port série
        self.declare_parameter('serial_baudrate', 115200) # Paramètre pour le baudrate

        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baudrate = self.get_parameter('serial_baudrate').value
        self.serial = serial.Serial(self.serial_port, self.serial_baudrate)

        # Abonnement à un topic pour recevoir les commandes 
        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/servo_commands',  
            self.servo_cmd_callback,
            10
        )
        self.get_logger().info("Noeud servo_control_node démarré, en attente de commandes sur /servo_commands")

    def servo_cmd_callback(self, msg):
        angles = msg.data  

        # Vérifier que le message contient 8 angles
        if len(angles) != 8:  
            self.get_logger().error("Message incorrect : 8 angles de servo attendus.")
            return

        #  Envoyer les angles à l'Arduino via la liaison série (format CSV)
        command = "S," + ",".join(str(angle) for angle in angles) + "\n" 
        try:
            self.serial.write(command.encode())
            self.get_logger().info(f"Angles des servos envoyés : {command.rstrip()}")  
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur de communication série : {e}")

def main(args=None):
    rclpy.init(args=args)
    servo_control_node = ServoControlNode()
    rclpy.spin(servo_control_node)
    servo_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()