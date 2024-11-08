import os
os.environ["KIVY_NO_ARGS"]="1"
import sys
import argparse
from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.properties import StringProperty, NumericProperty, ObjectProperty
from kivy_garden.graph import Graph, MeshLinePlot
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.core.image import Image as CoreImage
import cv2
from kivy.graphics.texture import Texture
import firebase_admin
from firebase_admin import db, credentials
from controller.ps5_controller import PS5Controller
from network.rover_client import RoverClient
import requests

KV = '''
#:import get_color_from_hex kivy.utils.get_color_from_hex

<SensorGraph>:
    Graph:
        id: graph
        xlabel: 'Temps'
        ylabel: 'Valeur'
        x_ticks_minor: 5
        x_ticks_major: 25
        y_ticks_major: 1
        y_grid_label: True
        x_grid_label: True
        padding: 5
        x_grid: True
        y_grid: True
        xmin: -0
        xmax: 100
        ymin: -1
        ymax: 1

<SensorData>:
    orientation: 'vertical'
    spacing: '5dp'
    MDLabel:
        text: f"X: {root.x_value:.2f}"
        theme_text_color: "Secondary"
        font_style: "Caption"
    MDLabel:
        text: f"Y: {root.y_value:.2f}"
        theme_text_color: "Secondary"
        font_style: "Caption"
    MDLabel:
        text: f"Z: {root.z_value:.2f}"
        theme_text_color: "Secondary"
        font_style: "Caption"

MDScreen:
    MDBoxLayout:
        orientation: "vertical"
        md_bg_color: get_color_from_hex("#0A192F")
        
        MDTopAppBar:
            title: "BenRover Dashboard"
            right_action_items: [["refresh", lambda x: app.update_data()]]
            md_bg_color: get_color_from_hex("#172A45")
        
        MDBoxLayout:
            padding: "16dp"
            spacing: "16dp"
            
            MDBoxLayout:
                orientation: "vertical"
                size_hint_x: 0.7
                spacing: "16dp"
                
                MDCard:
                    md_bg_color: get_color_from_hex("#172A45")
                    radius: [15, 15, 15, 15]
                    Image:
                        id: video_feed
                        source: "img/placeholder.jpeg"
                
                MDGridLayout:
                    cols: 2
                    spacing: "16dp"
                    
                    MDCard:
                        orientation: "vertical"
                        padding: "8dp"
                        md_bg_color: get_color_from_hex("#172A45")
                        radius: [15, 15, 15, 15]
                        
                        MDLabel:
                            text: "Accelerometer"
                            font_style: "H6"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: get_color_from_hex("#64FFDA")
                        
                        SensorGraph:
                            id: accel_graph
                        
                        SensorData:
                            id: accel_data
                    
                    MDCard:
                        orientation: "vertical"
                        padding: "8dp"
                        md_bg_color: get_color_from_hex("#172A45")
                        radius: [15, 15, 15, 15]
                        
                        MDLabel:
                            text: "Gyroscope"
                            font_style: "H6"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: get_color_from_hex("#64FFDA")
                        
                        SensorGraph:
                            id: gyro_graph
                        
                        SensorData:
                            id: gyro_data
            
            MDBoxLayout:
                orientation: "vertical"
                size_hint_x: 0.3
                spacing: "16dp"
                
                MDCard:
                    orientation: "vertical"
                    padding: "16dp"
                    md_bg_color: get_color_from_hex("#172A45")
                    radius: [15, 15, 15, 15]
                    
                    Image:
                        source: "img/logo.png"
                        size_hint_y: 0.3
                
                MDCard:
                    orientation: "vertical"
                    padding: "16dp"
                    md_bg_color: get_color_from_hex("#172A45")
                    radius: [15, 15, 15, 15]
                    
                    MDLabel:
                        text: "System Status"
                        font_style: "H6"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                    
                    MDLabel:
                        text: f"Name: {app.system_name}"
                        theme_text_color: "Secondary"
                    
                    MDLabel:
                        text: f"Status: {app.system_status}"
                        theme_text_color: "Secondary"

                    # Nouveau: Status de la manette
                    MDLabel:
                        text: "Controller Status"
                        font_style: "H6"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                        padding_top: "10dp"

                    MDLabel:
                        text: f"Status: {app.controller_status}"
                        theme_text_color: "Secondary"
                        
                    MDLabel:
                        text: f"Controller: {app.controller_name}"
                        theme_text_color: "Secondary"

                    # Nouveau: Status de la connexion
                    MDLabel:
                        text: "Network Status"
                        font_style: "H6"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                        padding_top: "10dp"

                    MDLabel:
                        text: f"Rover Connection: {app.rover_status}"
                        theme_text_color: "Secondary"

                    MDLabel:
                        text: f"WiFi Network: {app.wifi_name}"
                        theme_text_color: "Secondary"

                    MDLabel:
                        text: "Connection Quality"
                        halign: "center"
                        theme_text_color: "Secondary"
                        
                    MDProgressBar:
                        id: connection_quality
                        value: app.connection_quality
                        color: get_color_from_hex("#64FFDA")
                        
                    MDLabel:
                        text: f"Download: {app.download_speed:.1f} Mbps"
                        theme_text_color: "Secondary"
                        halign: "center"
                        
                    MDLabel:
                        text: f"Upload: {app.upload_speed:.1f} Mbps"
                        theme_text_color: "Secondary"
                        halign: "center"
                        
                    MDLabel:
                        text: f"Ping: {app.ping:.0f} ms"
                        theme_text_color: "Secondary"
                        halign: "center"
                    
                    MDLabel:
                        text: "Battery Status"
                        font_style: "H6"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                        padding_top: "10dp"
                    
                    MDProgressBar:
                        id: battery_bar
                        value: app.battery_level
                        color: get_color_from_hex("#64FFDA")
                    
                    MDLabel:
                        text: f"Level: {app.battery_level:.1f}%"
                        halign: "center"
                    
                    MDLabel:
                        text: f"Temperature: {app.battery_temp:.1f}°C"
                        halign: "center"
                    
                    MDLabel:
                        text: "Ambient Temperature"
                        font_style: "H6"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                    
                    MDLabel:
                        text: f"{app.temperature:.1f}°C"
                        font_style: "H4"
                        halign: "center"
'''

class SensorData(BoxLayout):
    x_value = NumericProperty(0)
    y_value = NumericProperty(0)
    z_value = NumericProperty(0)

class SensorGraph(BoxLayout):
    pass

class RoverDashboard(MDApp):
    # Propriétés existantes
    battery_level = NumericProperty(0)
    battery_temp = NumericProperty(0)
    temperature = NumericProperty(0)
    system_name = StringProperty("Raspberry Pi")
    system_status = StringProperty("Connected")
    
    # Nouvelles propriétés
    controller_status = StringProperty("Disconnected")
    controller_name = StringProperty("No Controller")
    rover_status = StringProperty("Disconnected")
    wifi_name = StringProperty("Not Connected")
    connection_quality = NumericProperty(0)
    download_speed = NumericProperty(0)
    upload_speed = NumericProperty(0)
    ping = NumericProperty(0)
    
    def __init__(self, camera_ip, ngrok_url, **kwargs):
        super().__init__(**kwargs)
        self.camera_ip = camera_ip
        
        # Initialize controller and network modules
        self.controller = PS5Controller(self.on_controller_state_change)
        self.rover_client = RoverClient(ngrok_url, self.on_network_state_change)
        
        # Firebase initialization
        cred = credentials.Certificate("gui/credentials.json")
        firebase_admin.initialize_app(cred, {
            "databaseURL": "https://fir-e3207-default-rtdb.firebaseio.com/"
        })
        self.root = db.reference("/")
    
    def build(self):
        self.icon = 'img/favicon.png'
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Teal"
        self.video_capture = None
        self.placeholder_texture = None
        return Builder.load_string(KV)
    
    def on_start(self):
        # Initialisation existante
        self.accel_plot = MeshLinePlot(color=[1, 0, 0, 1])
        self.gyro_plot = MeshLinePlot(color=[0, 1, 0, 1])
        self.root.ids.accel_graph.ids.graph.add_plot(self.accel_plot)
        self.root.ids.gyro_graph.ids.graph.add_plot(self.gyro_plot)
        self.accel_data = []
        self.gyro_data = []
        
        # Démarrage des mises à jour
        Clock.schedule_interval(self.update_data, 1)
        Clock.schedule_interval(self.update_video, 1/30)
        
        # Chargement de l'image placeholder
        placeholder_image = cv2.imread('img/placeholder.jpeg')
        if placeholder_image is not None:
            placeholder_image = cv2.cvtColor(placeholder_image, cv2.COLOR_BGR2RGB)
            self.placeholder_texture = Texture.create(size=(placeholder_image.shape[1], placeholder_image.shape[0]), colorfmt='rgb')
            self.placeholder_texture.blit_buffer(placeholder_image.tostring(), colorfmt='rgb', bufferfmt='ubyte')
        
        # Démarrage des modules de contrôle
        self.controller.start()
        self.rover_client.start()
        
        # Démarrage de la transmission du flux vidéo via ngrok
        Clock.schedule_interval(self.transmit_video, 1/30)  # 30 FPS
        
    
    def transmit_video(self, dt):
        if not self.video_capture:
            try:
                self.video_capture = cv2.VideoCapture(self.camera_ip)
            except Exception as e:
                print(f"Failed to connect to video stream: {e}")
                self.use_placeholder()
                return
        
        ret, frame = self.video_capture.read()
        if ret:
            buf1 = cv2.flip(frame, 0)
            buf = buf1.tobytes()
            image_texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
            image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            self.root.ids.video_feed.texture = image_texture
            
            # Transmettre le flux vidéo via ngrok
            try:
                response = requests.post(f"{self.ngrok_url}/video", files={'frame': buf})
                if response.status_code != 200:
                    print(f"Error transmitting video: {response.text}")
            except Exception as e:
                print(f"Error transmitting video: {e}")
        else:
            self.use_placeholder()
    
    def on_controller_state_change(self, state):
        self.controller_status = "Connected" if state.connected else "Disconnected"
        self.controller_name = state.controller_name or "No Controller"
        if state.connected:
            self.rover_client.send_command(state)
            
    def on_network_state_change(self, state):
        self.rover_status = "Connected" if state.connected else "Disconnected"
        self.wifi_name = state.wifi_name or "Not Connected"
        self.download_speed = state.download_speed
        self.upload_speed = state.upload_speed
        self.ping = state.ping
        
        # Calcul de la qualité de connexion (0-100)
        if state.connected:
            ping_score = max(0, 100 - (state.ping / 2))  # Lower ping is better
            speed_score = min(100, (state.download_speed + state.upload_speed) / 2)
            self.connection_quality = (ping_score + speed_score) / 2
        else:
            self.connection_quality = 0

    def on_stop(self):
        if self.video_capture:
            self.video_capture.release()
        self.controller.running = False
        self.rover_client.running = False
    
    def update_video(self, dt):
        if not self.video_capture:
            try:
                self.video_capture = cv2.VideoCapture(self.camera_ip)
            except Exception as e:
                print(f"Failed to connect to video stream: {e}")
                self.use_placeholder()
                return
        
        ret, frame = self.video_capture.read()
        if ret:
            buf1 = cv2.flip(frame, 0)
            buf = buf1.tobytes()
            image_texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
            image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            self.root.ids.video_feed.texture = image_texture
        else:
            self.use_placeholder()

    def use_placeholder(self):
        if self.placeholder_texture:
            self.root.ids.video_feed.texture = self.placeholder_texture
    
    def update_data(self, *args):
        try:
            data = self.root.get()
            if data:
                self.system_name = data.get('name', 'Raspberry Pi')
                self.system_status = data.get('status', 'Connected')

                battery_data = data.get('data', {}).get('battery', {})
                self.battery_level = battery_data.get('level', 0)
                self.battery_temp = battery_data.get('temperature', 0)

                sensor_data = data.get('data', {}).get('sensors', {})
                self.temperature = sensor_data.get('temperature', 0)

                # Update accelerometer data
                accel_data = sensor_data.get('accelerometer', {})
                accel_x = accel_data.get('x', 0)
                accel_y = accel_data.get('y', 0)
                accel_z = accel_data.get('z', 0)
                self.root.ids.accel_data.x_value = accel_x
                self.root.ids.accel_data.y_value = accel_y
                self.root.ids.accel_data.z_value = accel_z
                self.accel_data.append(accel_x)
                if len(self.accel_data) > 100:
                    self.accel_data.pop(0)
                self.accel_plot.points = [(i, y) for i, y in enumerate(self.accel_data)]

                # Update gyroscope data
                gyro_data = sensor_data.get('gyroscope', {})
                gyro_x = gyro_data.get('x', 0)
                gyro_y = gyro_data.get('y', 0)
                gyro_z = gyro_data.get('z', 0)
                self.root.ids.gyro_data.x_value = gyro_x
                self.root.ids.gyro_data.y_value = gyro_y
                self.root.ids.gyro_data.z_value = gyro_z
                self.gyro_data.append(gyro_x)
                if len(self.gyro_data) > 100:
                    self.gyro_data.pop(0)
                self.gyro_plot.points = [(i, y) for i, y in enumerate(self.gyro_data)]
        except Exception as e:
            print(f"Error updating data: {e}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="BenRover Dashboard")
    parser.add_argument("--camera-url", type=str, required=False, help="https://url-ngrok.com/video")
    parser.add_argument("--ngrok-url", type=str, required=True, help="URL of the ngrok tunnel")
    args = parser.parse_args()

    RoverDashboard(camera_ip=args.camera_ip, ngrok_url=args.ngrok_url).run()