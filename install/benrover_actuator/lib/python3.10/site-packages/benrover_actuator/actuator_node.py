import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Adafruit_PCA9685 import PCA9685

class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuator_node')

        # Configuration I2C et PCA9685
        self.pwm = PCA9685(0x40)  # Adresse I2C par défaut, à modifier si nécessaire
        self.pwm.set_pwm_freq(60)  # Fréquence PWM pour servos (60Hz)

        # Canaux PCA9685 pour les servos (à adapter)
        self.servo_front_left_channel = 0
        self.servo_front_right_channel = 1
        self.servo_rear_left_channel = 2
        self.servo_rear_right_channel = 3

        # Positions neutres des servos (à déterminer expérimentalement)
        self.servo_front_left_neutral = 300
        self.servo_front_right_neutral = 300
        self.servo_rear_left_neutral = 300
        self.servo_rear_right_neutral = 300

        # Abonnement au topic /cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Noeud actuator_node démarré.")

    def cmd_vel_callback(self, msg):
        # Vitesse linéaire et angulaire du message Twist
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Conversion en angle pour chaque servo (à adapter)
        front_left_servo_angle = self.servo_front_left_neutral + int(linear_speed * 100) - int(angular_speed * 50)
        front_right_servo_angle = self.servo_front_right_neutral + int(linear_speed * 100) + int(angular_speed * 50)
        rear_left_servo_angle = self.servo_rear_left_neutral + int(linear_speed * 100) - int(angular_speed * 50)
        rear_right_servo_angle = self.servo_rear_right_neutral + int(linear_speed * 100) + int(angular_speed * 50)

        # Limiter les angles (ex: 150 à 450)
        front_left_servo_angle = max(150, min(front_left_servo_angle, 450))
        front_right_servo_angle = max(150, min(front_right_servo_angle, 450))
        rear_left_servo_angle = max(150, min(rear_left_servo_angle, 450))
        rear_right_servo_angle = max(150, min(rear_right_servo_angle, 450))

        # Appliquer les angles aux servos
        self.pwm.set_pwm(self.servo_front_left_channel, 0, front_left_servo_angle)
        self.pwm.set_pwm(self.servo_front_right_channel, 0, front_right_servo_angle)
        self.pwm.set_pwm(self.servo_rear_left_channel, 0, rear_left_servo_angle)
        self.pwm.set_pwm(self.servo_rear_right_channel, 0, rear_right_servo_angle)

def main(args=None):
    rclpy.init(args=args)

    actuator_node = ActuatorNode()

    rclpy.spin(actuator_node)

    # Désactivez le noeud et arrêtez ROS 2
    actuator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()