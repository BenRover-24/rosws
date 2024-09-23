from sys import argv
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState, Imu
import time



class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # self.publisher_lidar_data = self.create_publisher(Float32, 'sensor_data', 10)
        self.publisher_temp_data = self.create_publisher(Float32, 'temperature_data', 10)
        self.publisher_distance_data = self.create_publisher(Float32, 'sensor_data', 10)
        self.publisher_battery_status_data = self.create_publisher(BatteryState, 'battery_status', 10)
        self.publisher_accelerometer_data = self.create_publisher(Imu, 'accelerometer_data', 10)
        self.publisher_gyroscope_data = self.create_publisher(Imu, 'gyroscope_data', 10)
        self.create_timer(1.0,self.temperature)
        self.create_timer(1.0,self.battery)
        self.create_timer(1.0,self.accelerometre)
        self.create_timer(1.0,self.gyroscope)

    
    def temperature(self):
        temp = Float32()
        temp.data = 0.3
        self.publisher_temp_data.publish(temp)
        pass

    def accelerometre(self):
        acc = Imu()
        acc.linear_acceleration.x = 0.7
        acc.linear_acceleration.y = 0.7
        acc.linear_acceleration.z = 0.7
        self.publisher_accelerometer_data.publish(acc)

    def gyroscope(self):
        gyro = Imu()
        gyro.angular_velocity.x = 0.9
        gyro.angular_velocity.y = 0.9
        gyro.angular_velocity.z = 0.9
        self.publisher_gyroscope_data.publish(gyro)

    def battery(self):
        battery_msg = BatteryState()
        battery_msg.temperature = 0.6
        battery_msg.percentage = 0.2
        self.publisher_battery_status_data.publish(battery_msg)
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
        # print(msg_distance.data)



   
    


def main():
    rclpy.init(args=argv)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()