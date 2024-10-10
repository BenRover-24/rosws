import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Initialisation de la communication série
        self.serial_port = '/dev/ttyUSB0'  
        self.baud_rate = 9600  # Taux de transmission, adapter selon ton matériel
        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connexion série ouverte sur {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur lors de l'ouverture du port série: {e}")
            rclpy.shutdown()
            return
        
        # Souscription au sujet pour les commandes des servos
        self.subscription = self.create_subscription(
            Int32MultiArray, 
            'servo_commands',  
            self.servo_cmd_callback,  # Fonction de rappel
            10  # Taille de la queue du buffer
        )
        
    def servo_cmd_callback(self, msg):
        angles = msg.data 
        if len(angles) != 8: 
            self.get_logger().warn(f"Nombre incorrect d'angles reçus: {len(angles)}")
            return
        
        # Envoyer les angles comme octets individuels
        for angle in angles:
            if not 0 <= angle <= 255: 
                self.get_logger().error(f"Angle invalide: {angle}, doit être entre 0 et 255")
                continue
            try:
                self.serial.write(bytes([angle]))  
                self.get_logger().info(f"Angle envoyé: {angle}") 
            except serial.SerialException as e:
                self.get_logger().error(f"Erreur lors de l'envoi de l'angle {angle}: {e}")
                break  

    def destroy(self):
        self.serial.close()  # Fermer le port série proprement
        self.get_logger().info("Port série fermé")
        super().destroy()


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
