from kivy.lang import Builder
from kivymd.app import MDApp
from kivy.uix.image import Image
from kivy.clock import Clock
import cv2
from kivy.graphics.texture import Texture
import firebase_admin 
from firebase_admin import db, credentials

#cred = credentials.Certificate(" ") #credentials.json 
#firebase_admin.initialize_app(cred, {"databaseURL":"https://benrover-8ebf6-default-rtdb.firebaseio.com/"})
#root = db.reference("/")

KV = '''
Screen:
    MDBoxLayout:
        orientation: "vertical"
        md_bg_color: 20/255, 32/255, 57/255, 1

        MDBoxLayout:
            size_hint_y: None
            height: "100dp"
            md_bg_color: 20/255, 32/255, 57/255, 1

            Image:
                source: 'asset/img/logo.png'
                allow_stretch: False

        MDGridLayout:
            cols: 2
            spacing: 25
            padding: 10
            md_bg_color: 20/255, 32/255, 57/255, 1
            size_hint_y: 1

            MDBoxLayout:
                orientation: 'vertical'
                md_bg_color: 20/255, 32/255, 57/255, 1
                size_hint_x: 0.6

                Image:
                    id: camera_feed
                    allow_stretch: True
                    radius: [20, 20, 20, 20]

                MDLabel:
                    id: camera_status
                    text: "Camera   |   Signal: Disabled"
                    halign: "center"
                    theme_text_color: "Custom"
                    text_color: 1, 1, 1, 1
                    bold: True
                    font_style: "H6"
                    size_hint_y: 0.2

            MDBoxLayout:
                orientation: 'vertical'
                spacing: 10
                size_hint_x: 0.4

                MDBoxLayout:
                    orientation: 'horizontal'
                    size_hint_y: 0.5
                    spacing: 10

                    MDBoxLayout:
                        orientation: 'vertical'
                        md_bg_color: 40/255, 32/255, 57/255, 1
                        radius: [20, 20, 20, 20]

                        MDLabel:
                            text: "GLOBAL STATUS"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"

                        MDLabel:
                            id: battery_label
                            text: "Battery: "
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                
                    MDBoxLayout:
                        orientation: 'vertical'
                        md_bg_color: 40/255, 32/255, 57/255, 1
                        radius: [20, 20, 20, 20]

                        MDLabel:
                            text: "SENSORS"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"

                        MDLabel:
                            id: temp_label
                            text: "Temp: "
                            halign: "center"                         
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                
                MDBoxLayout:
                    orientation: 'vertical'
                    md_bg_color: 40/255, 32/255, 57/255, 1
                    radius: [20, 20, 20, 20]
                    MDLabel:
                        text: "SENSORS"
                        halign: "center"
                        theme_text_color: "Custom"
                        text_color: 1, 1, 1, 1
                        bold: True
                        font_style: "H6"

                    MDLabel:
                        id: temp_label
                        text: "Temp: "
                        halign: "center"                         
                        theme_text_color: "Custom"
                        text_color: 1, 1, 1, 1                    
                MDBoxLayout:
                    orientation: 'horizontal'
                    md_bg_color: 40/255, 32/255, 57/255, 1
                    radius: [20, 20, 20, 20]
                    
                    MDBoxLayout:
                        orientation: 'vertical'
                        md_bg_color: 40/255, 32/255, 57/255, 1
                        radius: [20, 20, 20, 20]
                        MDLabel:
                            text: "WHEEL"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                        MDLabel:
                            text: "W1"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                        MDLabel:
                            text: "W2"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                            text: "W3"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                        MDLabel:
                            text: "W4"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                            text: "W5"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                        MDLabel:
                            text: "W6"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                    MDBoxLayout:
                        orientation: 'vertical'
                        md_bg_color: 40/255, 32/255, 57/255, 1
                        radius: [20, 20, 20, 20]
                        MDLabel:
                            id: temp_label
                            text: "SERVO: "
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"       
                        MDLabel:
                            text: "S1"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                        MDLabel:
                            text: "S2"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                        MDLabel:
                            text: "S3"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                        MDLabel:
                            text: "S4"
                            halign: "center"
                            theme_text_color: "Custom"
                            text_color: 1, 1, 1, 1
                            bold: True
                            font_style: "H6"
                            


                               

                        
'''

class BenRoverInterface(MDApp):
    def build(self):
        return Builder.load_string(KV)
    # Il faut décommenter la suite pour travailler pour le backend et retirer le " def build(self): "
    
    # def __init__(self, **kwargs):
    #     super().__init__(**kwargs)
#         self.camera_connected = False

#     def build(self):
#         #self.capture = cv2.VideoCapture("http://10.61.16.199:8081")
#         Clock.schedule_interval(self.update_camera_feed, 1.0 / 30.0)
#         Clock.schedule_interval(self.update_data, 1.0)  # Update data every second
#         return Builder.load_string(KV)

#    # def update_camera_feed(self, dt):
#    #     ret, frame = self.capture.read()
#         if ret:
#             buffer = cv2.flip(frame, 0).tobytes()
#             texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
#             texture.blit_buffer(buffer, colorfmt='bgr', bufferfmt='ubyte')
#             self.root.ids.camera_feed.texture = texture
#             if not self.camera_connected:
#                 self.camera_connected = True
#                 self.root.ids.camera_status.text = "Camera   |   Signal: Enabled"
#         else:
#             if self.camera_connected:
#                 self.camera_connected = False
#                 self.root.ids.camera_status.text = "Camera   |   Signal: Disabled"

#    # def update_data(self, dt):
#       #  data = root.get()
#       #  if data:
#         #    self.root.ids.battery_label.text = f"Battery: {data.get('Battery', '79')}%"
#          #   self.root.ids.joystick_label.text = f"Joystick: {'Connected' if data.get('Joystick', False) else 'Disconnected'}"
#           #  self.root.ids.temp_label.text = f"Temp: {data.get('temperature', 'N/A')}° Celsius"
#            # self.root.ids.distance_label.text = f"Distance: {data.get('Distance', 'N/A')} Meters"
#             #self.root.ids.rock_data_label.text = f"Rock Data: {data.get('Rockdata', '...')}"
#             # Removed the camera status update from here as it's now handled in update_camera_feed

BenRoverInterface().run()