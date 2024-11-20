import pygame
from dataclasses import dataclass
import threading
import time

@dataclass
class ControllerState:
    connected: bool = False
    battery_level: int = 0
    controller_name: str = ""
    buttons: dict = None
    axes: dict = None

class PS5Controller:
    def __init__(self, on_state_change=None):
        pygame.init()
        pygame.joystick.init()
        self.controller = None
        self.state = ControllerState()
        self.on_state_change = on_state_change
        self.running = False

    def scan_and_connect(self):
        while not self.controller and self.running:
            try:
                for i in range(pygame.joystick.get_count()):
                    joy = pygame.joystick.Joystick(i)
                    if "DualSense" in joy.get_name():
                        joy.init()
                        self.controller = joy
                        self.state.connected = True
                        self.state.controller_name = joy.get_name()
                        if self.on_state_change:
                            self.on_state_change(self.state)
                        break
            except Exception as e:
                print(f"Controller scan error: {e}")
            time.sleep(1)

    def start(self):
        self.running = True
        threading.Thread(target=self.scan_and_connect).start()
        threading.Thread(target=self.update_loop).start()

    def update_loop(self):
        while self.running:
            if self.controller:
                try:
                    pygame.event.pump()
                    self.state.axes = {
                        'left_x': int((self.controller.get_axis(0))*100),
                        'left_y': int((self.controller.get_axis(1))*100),
                        'right_x': int((self.controller.get_axis(3))*100),
                        'right_y': int((self.controller.get_axis(4))*100)
                    }
                    self.state.buttons = {
                        'cross': self.controller.get_button(0),
                        'circle': self.controller.get_button(1),
                        'triangle': self.controller.get_button(3),
                        'square': self.controller.get_button(2)
                    }
                    if self.on_state_change:
                        self.on_state_change(self.state)
                except Exception as e:
                    print(f"Controller update error: {e}")
                    self.controller = None
                    self.state.connected = False
                    if self.on_state_change:
                        self.on_state_change(self.state)
            time.sleep(0.016)  # ~60Hz updates
