from sys import argv
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
#import serial



class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_lidar_data = self.create_publisher(Float32, 'sensor_data', 10)
        self.publisher_temp_data = self.create_publisher(Float32, 'temperature_data', 10)
        self.publisher_distance_data = self.create_publisher(Float32, 'distance_data', 10)
        self.create_timer(1.0,self.send_data)
        #self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)

    # def get_data(self):
    #     if self.serial_port.in_waiting > 0:
    #         data = self.serial_port.readline().decode('utf-8').strip()
    #         try:
    #             # Séparer les données reçues en utilisant la virgule
    #             values = data.split(',')
    #             print('Les données ont bien été splittés')
    #             # Convertir les valeurs en floats
    #             float_values = [float(value) for value in values]
    #             return float_values
    #         except ValueError as e:
    #             print('Erreur')


    def send_data(self):
        # if self.serial_port.in_waiting > 0:
        #     data = self.serial_port.readline().decode('utf-8').strip()
        #     try:
        #         # Séparer les données reçues en utilisant la virgule
        #         values = data.split(',')
        #         # Convertir les valeurs en floats
        #         float_values = [float(value) for value in values]

        #         msg_temp = Float32()
        #         msg_temp.data = self.get_data()[0]
        #         self.publisher_lidar_data.publish(msg_temp)
                
        msg_distance = Float32()
        msg_distance.data = 5.0
        self.publisher_lidar_data.publish(msg_distance)
                
                
        #         # self.publisher_temp_data.publish(msg)

        #         # self.publisher_distance_data(msg)
        #         # self.get_logger().info(f'Publishing: "{msg.data}"')
        #     except ValueError as e:
        #         self.get_logger().error(f'Error parsing data: {e}')

    


def main():
    rclpy.init(args=argv)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()