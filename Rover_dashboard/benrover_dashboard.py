from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.properties import StringProperty, NumericProperty, ObjectProperty
from kivy.garden.graph import Graph, MeshLinePlot
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.core.image import Image as CoreImage
import cv2
import io
import numpy as np
from kivy.graphics.texture import Texture
import firebase_admin
from firebase_admin import db, credentials

# Initialisation de Firebase
cred = credentials.Certificate("credentials.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://benrover-8ebf6-default-rtdb.firebaseio.com/"
})
root = db.reference("/")

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
                        source: "img/pp.png"  # Votre image de remplacement
                
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
                        source: "img/logo.png"  # Votre logo
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
                    
                    MDLabel:
                        text: "Battery Status"
                        font_style: "H6"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                    
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
    battery_level = NumericProperty(0)
    battery_temp = NumericProperty(0)
    temperature = NumericProperty(0)
    system_name = StringProperty("Raspberry Pi")
    system_status = StringProperty("Connected")
    
    def build(self):
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Teal"
        self.video_capture = None
        self.placeholder_texture = None
        return Builder.load_string(KV)
    
    def on_start(self):
        self.accel_plot = MeshLinePlot(color=[1, 0, 0, 1])
        self.gyro_plot = MeshLinePlot(color=[0, 1, 0, 1])
        self.root.ids.accel_graph.ids.graph.add_plot(self.accel_plot)
        self.root.ids.gyro_graph.ids.graph.add_plot(self.gyro_plot)
        self.accel_data = []
        self.gyro_data = []
        Clock.schedule_interval(self.update_data, 1)
        Clock.schedule_interval(self.update_video, 1/30)  # 30 FPS
        
        # Charger l'image de remplacement
        placeholder_image = cv2.imread('img\placeholder.jpeg')
        if placeholder_image is not None:
            placeholder_image = cv2.cvtColor(placeholder_image, cv2.COLOR_BGR2RGB)
            self.placeholder_texture = Texture.create(size=(placeholder_image.shape[1], placeholder_image.shape[0]), colorfmt='rgb')
            self.placeholder_texture.blit_buffer(placeholder_image.tostring(), colorfmt='rgb', bufferfmt='ubyte')
        else:
            print("Failed to load placeholder image")
        
    def on_stop(self):
        if self.video_capture:
            self.video_capture.release()
    
    def update_video(self, dt):
        if not self.video_capture:
            try:
                self.video_capture = cv2.VideoCapture('http://votre_adresse_ip:port/video_feed')
            except Exception as e:
                print(f"Failed to connect to video stream: {e}")
                self.use_placeholder()
                return
        
        ret, frame = self.video_capture.read()
        if ret:
            # Convertir en texture
            buf1 = cv2.flip(frame, 0)
            buf = buf1.tobytes()
            image_texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
            image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            # Afficher l'image à partir de la texture
            self.root.ids.video_feed.texture = image_texture
        else:
            print("Failed to get frame, using placeholder")
            self.use_placeholder()

    def use_placeholder(self):
        if self.placeholder_texture:
            self.root.ids.video_feed.texture = self.placeholder_texture
        else:
            print("Placeholder texture not available")
    
    def update_data(self, *args):
        try:
            # Récupérer les données de Firebase
            data = root.get()
            if data:
                # Mettre à jour les informations du système
                self.system_name = data.get('name', 'Raspberry Pi')
                self.system_status = data.get('status', 'Connected')

                # Mettre à jour les informations de la batterie
                battery_data = data.get('data', {}).get('battery', {})
                self.battery_level = battery_data.get('level', 0)
                self.battery_temp = battery_data.get('temperature', 0)

                # Mettre à jour la température ambiante
                sensor_data = data.get('data', {}).get('sensors', {})
                self.temperature = sensor_data.get('temperature', 0)

                # Mettre à jour les données de l'accéléromètre
                accel_data = sensor_data.get('accelerometer', {})
                accel_x = accel_data.get('x', 0)
                accel_y = accel_data.get('y', 0)
                accel_z = accel_data.get('z', 0)
                self.root.ids.accel_data.x_value = accel_x
                self.root.ids.accel_data.y_value = accel_y
                self.root.ids.accel_data.z_value = accel_z
                self.accel_data.append(accel_x)  # Utiliser l'axe X pour le graphique
                if len(self.accel_data) > 100:
                    self.accel_data.pop(0)
                self.accel_plot.points = [(i, y) for i, y in enumerate(self.accel_data)]

                # Mettre à jour les données du gyroscope
                gyro_data = sensor_data.get('gyroscope', {})
                gyro_x = gyro_data.get('x', 0)
                gyro_y = gyro_data.get('y', 0)
                gyro_z = gyro_data.get('z', 0)
                self.root.ids.gyro_data.x_value = gyro_x
                self.root.ids.gyro_data.y_value = gyro_y
                self.root.ids.gyro_data.z_value = gyro_z
                self.gyro_data.append(gyro_x)  # Utiliser l'axe X pour le graphique
                if len(self.gyro_data) > 100:
                    self.gyro_data.pop(0)
                self.gyro_plot.points = [(i, y) for i, y in enumerate(self.gyro_data)]
        except Exception as e:
            print(f"Erreur lors de la mise à jour des données: {e}")

if __name__ == '__main__':
    RoverDashboard().run()