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
import io
import numpy as np
from kivy.graphics.texture import Texture
import firebase_admin
from firebase_admin import db, credentials
import pafy

from ultralytics import YOLO, SAM
import supervision as sv
from ultralytics.utils.plotting import Annotator, colors

#sam_model = SAM('sam2.1_b.pt')
yolo_model = YOLO('yolo11m-seg.pt')

names = yolo_model.model.names



# Initialisation de Firebase
cred = credentials.Certificate("credentials.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://fir-e3207-default-rtdb.firebaseio.com/"
})
root = db.reference("/")

class StatusIndicator(BoxLayout):
    status = StringProperty("Disconnected")
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_interval(self.update_animation, 1)
        self.alpha = 1
        
    def update_animation(self, dt):
        if self.status == "Connected":
            self.alpha = 1 if self.alpha == 0.3 else 0.3
            print("system connected")
        else:
            self.alpha = 0.7

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
        
        #MDTopAppBar:
          #  title: "BenRover Dashboard"
            #right_action_items: [["refresh", lambda x: app.update_data()]]
            #md_bg_color: get_color_from_hex("#172A45")

        # Top Section    
        BoxLayout:
            size_hint_y: 0.07
            padding: "10dp"
            spacing: "3dp"
            
            MDBoxLayout:
                size_hint_x: 0.3
                # md_bg_color: 1, 0, 0, 1  # Example color (Red)
                # orientation: "vertical"
                # padding: "16dp"
                # md_bg_color: get_color_from_hex("#172A45")
                #radius: [15, 15, 15, 15]
                
                Image:
                    source: "img/BENROVER_MARRON_LONG.png"  # Votre logo
                    size_hint_y: 0.7
                    #size: 100, 100
            
            MDBoxLayout:
                size_hint_x: 0.7
                spacing: "8dp"
                # md_bg_color: 0, 1, 0, 1  # Example color (Green)
                
                # IMPLEMENT THE BLINKING STATUS HERE
                MDBoxLayout:
                    size_hint_x: 0.3
                    #spacing: "10dp"
                    
                    MDLabel:
                        size_hint_x: 0.2
                        text: "System"
                        font_style: "H4"
                        halign: "left"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                        
                    StatusIndicator:
                        id: status_indicator
                        size_hint_x: 0.1
                        #width: "10dp"
                        
                        canvas:
                            Color:
                                rgba: (0, 1, 0, self.alpha) if self.status == "Connected" else (0.7, 0.7, 0.7, self.alpha)
                            Ellipse:
                                size: self.height * 0.5, self.height * 0.5
                                pos: self.x + (self.width - self.height * 0.5) / 2, self.y + (self.height - self.height * 0.5) / 2
                    
                    
                        
                    
                        
                MDBoxLayout:
                    size_hint_x: 0.3
                    MDLabel:
                        text: "Controller"
                        font_style: "H4"
                        halign: "left"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                
                MDBoxLayout:
                    size_hint_x: 0.3
                    MDLabel:
                        text: "Battery"
                        font_style: "H4"
                        halign: "left"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")

        # Middle section (60% height)
        MDBoxLayout:
            size_hint_y: 0.6
            padding: "10dp"
            spacing: "8dp"

            MDBoxLayout:
                size_hint_x: 0.5
                # md_bg_color: 0, 0, 1, 1  # Example color (Blue)
                # md_bg_color: get_color_from_hex("#172A45")
                radius: [15, 15, 15, 15]
                Image:
                    id: video_feed
                    source: "img/placeholder.jpeg"  # Votre image de remplacement
            
            MDBoxLayout:
                size_hint_x: 0.5
                # md_bg_color: 1, 1, 0, 1  # Example color (Yellow)
                #md_bg_color: get_color_from_hex("#172A45")
                radius: [15, 15, 15, 15]
                Image:
                    id: video_feed_2
                    source: "img/placeholder.jpeg"  # Votre image de remplacement

        # Bottom section (30% height)
        MDBoxLayout:
            size_hint_y: 0.33
            padding: "10dp"
            spacing: "8dp"
            
            MDBoxLayout:
                size_hint_x: 0.5
                #md_bg_color: 1, 0, 1, 1  # Example color (Magenta)
                MDCard:
                    orientation: "vertical"
                    padding: "8dp"
                    md_bg_color: get_color_from_hex("#172A45")
                    radius: [15, 15, 15, 15]
                    
                    MDLabel:
                        text: "Accelerometer"
                        font_style: "H3"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                    
                    SensorGraph:
                        id: accel_graph
                    
                    SensorData:
                        id: accel_data
            
            MDBoxLayout:
                size_hint_x: 0.5
                #md_bg_color: 0, 1, 1, 1  # Example color (Cyan)
                MDCard:
                    orientation: "vertical"
                    padding: "8dp"
                    md_bg_color: get_color_from_hex("#172A45")
                    radius: [15, 15, 15, 15]
                    
                    MDLabel:
                        text: "Gyroscope"
                        font_style: "H3"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: get_color_from_hex("#64FFDA")
                    
                    SensorGraph:
                        id: gyro_graph
                    
                    SensorData:
                        id: gyro_data
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
    
    def __init__(self, camera_ip, **kwargs):
        super().__init__(**kwargs)
        self.camera_ip = camera_ip

    def build(self):
        self.icon = 'img/favicon.png'
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
        
        # Load placeholder image
        placeholder_image = cv2.imread('img/placeholder.jpeg')
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
                print("Camera IP is", self.camera_ip)
                self.video_capture = cv2.VideoCapture(self.camera_ip)
            except Exception as e:
                print(f"Failed to connect to video stream: {e}")
                self.use_placeholder()
                return
        
        ret, frame = self.video_capture.read()
        if ret:
            # Convert original frame to texture
            buf1 = cv2.flip(frame, 0)
            buf = buf1.tobytes()
            image_texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
            image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            
            # Create annotated frame
            frame2 = frame.copy()
            results = yolo_model.predict(frame2)
            mask_annotator = sv.MaskAnnotator()

            annotated_image = frame2.copy()
            detections = sv.Detections.from_ultralytics(results[0])

            annotated_image = mask_annotator.annotate(annotated_image, detections=detections)

            annotator = Annotator(annotated_image, line_width=1)

            if results[0].masks is not None:
                clss = results[0].boxes.cls.cpu().tolist()
                masks = results[0].masks.xy
                for mask, cls in zip(masks, clss):
                    color = colors(int(cls), True)
                    txt_color = annotator.get_txt_color(color)
                    annotator.seg_bbox(mask=mask, mask_color=color, label=names[int(cls)], txt_color=txt_color)
            
            # Convert annotated frame to texture
            annotated_frame = annotator.result()  # Get the annotated frame
            buf2 = cv2.flip(annotated_frame, 0)  # Flip for Kivy
            buf2_bytes = buf2.tobytes()
            annotated_texture = Texture.create(size=(annotated_frame.shape[1], annotated_frame.shape[0]), colorfmt='bgr')
            annotated_texture.blit_buffer(buf2_bytes, colorfmt='bgr', bufferfmt='ubyte')
                    
            # Display images from textures
            self.root.ids.video_feed.texture = image_texture
            self.root.ids.video_feed_2.texture = annotated_texture
        else:
            print("Failed to get frame, using placeholder")
            self.use_placeholder(1)
            self.use_placeholder(2)

    def use_placeholder(self, feed_id=2):
        if self.placeholder_texture:
            if feed_id == 1:
                self.root.ids.video_feed.texture = self.placeholder_texture
            elif feed_id == 2:
                self.root.ids.video_feed_2.texture = self.placeholder_texture
            print("Placeholder texture not available")
    
    def update_data(self, *args):
        try:
            # Retrieve data from Firebase
            data = root.get()
            if data:
                # Update system information
                self.system_name = data.get('name', 'Raspberry Pi')
                self.system_status = data.get('status', 'Connected')

                # Update battery information
                battery_data = data.get('data', {}).get('battery', {})
                self.battery_level = battery_data.get('level', 0)
                self.battery_temp = battery_data.get('temperature', 0)

                # Update ambient temperature
                sensor_data = data.get('data', {}).get('sensors', {})
                self.temperature = sensor_data.get('temperature', 0)
                
                status = data.get('status', 'Disconnected')
                self.root.ids.status_indicator.status = status

                # Update accelerometer data
                accel_data = sensor_data.get('accelerometer', {})
                accel_x = accel_data.get('x', 0)
                accel_y = accel_data.get('y', 0)
                accel_z = accel_data.get('z', 0)
                self.root.ids.accel_data.x_value = accel_x
                self.root.ids.accel_data.y_value = accel_y
                self.root.ids.accel_data.z_value = accel_z
                self.accel_data.append(accel_x)  # Use X-axis for the graph
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
                self.gyro_data.append(gyro_x)  # Use X-axis for the graph
                if len(self.gyro_data) > 100:
                    self.gyro_data.pop(0)
                self.gyro_plot.points = [(i, y) for i, y in enumerate(self.gyro_data)]
        except Exception as e:
            print(f"Error updating data: {e}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="BenRover Dashboard")
    parser.add_argument("--camera-ip", type=str, default="0", help="IP address of the camera (use '0' for local webcam)")
    args = parser.parse_args()

    RoverDashboard(camera_ip=args.camera_ip).run()
