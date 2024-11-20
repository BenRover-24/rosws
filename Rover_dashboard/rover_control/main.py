from gui.dashboard import RoverDashboard

app = RoverDashboard(
    camera_ip="https://8006-41-86-250-38.ngrok-free.app/video_feed",
    ngrok_url="https://8006-41-86-250-38.ngrok-free.app/"
)
app.run()