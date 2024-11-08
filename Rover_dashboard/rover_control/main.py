from gui.dashboard import RoverDashboard

app = RoverDashboard(
    camera_ip="https://e693-41-86-250-38.ngrok-free.app/video",
    ngrok_url="https://e693-41-86-250-38.ngrok-free.app"
)
app.run()