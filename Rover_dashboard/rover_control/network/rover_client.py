import requests
import threading
import time
from dataclasses import dataclass
import psutil
#import speedtest

@dataclass
class NetworkState:
    connected: bool = False
    wifi_name: str = ""
    ping: float = 0
    download_speed: float = 0
    upload_speed: float = 0

class RoverClient:
    def __init__(self, ngrok_url, on_state_change=None):
        self.ngrok_url = ngrok_url
        self.on_state_change = on_state_change
        self.network_state = NetworkState()
        self.controller_state = None
        self.running = False

    def start(self):
        self.running = True
        threading.Thread(target=self.update_network_stats).start()
        threading.Thread(target=self.check_connection).start()
        threading.Thread(target=self.send_command_loop).start()  # Ajouter une boucle d'envoi de commandes
    
    '''
    def update_network_stats(self):
        while self.running:
            try:
                st = speedtest.Speedtest()
                #self.network_state.download_speed = st.download() / 1_000_000
                self.network_state.upload_speed = st.upload() / 1_000_000
                if self.on_state_change:
                    self.on_state_change(self.network_state)
                    
                time.sleep(300)
                
            except Exception as e:
                print(f"Network stats error: {e}")
                time.sleep(10)
    '''
    
    def update_network_stats(self):
        while self.running:
            try:
                # Use Speedtest.net API
                url = 'https://www.speedtest.net/api/api.php?action=getTestServers'
                response = requests.get(url)
                data = response.json()
                
                # Extract upload speed
                self.network_state.upload_speed = data['upload_speed'] / 1_000_000
                
                if self.on_state_change:
                    self.on_state_change(self.network_state)
                
                time.sleep(300)
            
            except Exception as e:
                print(f"Network stats error: {e}")
                time.sleep(10)
    '''        
    def check_connection(self):
        while self.running:
            try:
                response = requests.get(f"{self.ngrok_url}/ping")
                self.network_state.connected = response.status_code == 200
                self.network_state.ping = response.elapsed.total_seconds() * 1000
                if self.on_state_change:
                    self.on_state_change(self.network_state)
            except:
                self.network_state.connected = False
                if self.on_state_change:
                    self.on_state_change(self.network_state)
            time.sleep(1)
    '''  
        
    def send_command_loop(self):
        while self.running:
            if self.network_state.connected and self.controller_state:
                self.send_command(self.controller_state)
            time.sleep(0.1)  # Envoie toutes les 100 ms

    def send_command(self, controller_state):
        try:
            requests.post(f"{self.ngrok_url}/command", json={
                'axes': controller_state.axes,
                'buttons': controller_state.buttons
            })
            print(controller_state.axes)
        except Exception as e:
            print(f"Send command error: {e}")

    def update_controller_state(self, state):
        self.controller_state = state
