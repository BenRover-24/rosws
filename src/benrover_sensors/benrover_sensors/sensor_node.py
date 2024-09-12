from sys import argv
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
# import RPi.GPIO as GPIO
import time



class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # self.publisher_lidar_data = self.create_publisher(Float32, 'sensor_data', 10)
        self.publisher_temp_data = self.create_publisher(Float32, 'temperature_data', 10)
        self.publisher_distance_data = self.create_publisher(Float32, 'sensor_data', 10)
        self.publisher_battery_status_data = self.create_publisher(Float32, 'battery_status', 10)
        self.create_timer(1.0,self.distance)
        
        self.Tr = 11
        self.Ec = 8

    def setup(self, Tr, Ec):
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
        # GPIO.setup(Ec, GPIO.IN)
        pass
    
    def distance(self):
        #Simulation de la récupération de la valeur du capteur de distance 
        
        # self.setup(self.Tr, self.Ec)
        samples = 5
        distances = []
        for _ in range(samples):
            # GPIO.output(self.Tr, GPIO.LOW) 
            # time.sleep(0.000002)
            # GPIO.output(self.Tr, GPIO.HIGH)
            # time.sleep(0.00001)
            # GPIO.output(self.Tr, GPIO.LOW)
            pulse_start = 5
            pulse_end = 7
            # while GPIO.input(self.Ec) == 0:
            #     # pulse_start = time.time()


            # while GPIO.input(self.Ec) == 1:
            #     # pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distances.append(distance)

        # Filtre simple : moyenne des distances
        filtered_distance = sum(distances) / len(distances)
        msg_distance = Float32()
        msg_distance.data = filtered_distance
        self.publisher_distance_data.publish(msg_distance)
        print(msg_distance.data)



   
    


def main():
    rclpy.init(args=argv)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()