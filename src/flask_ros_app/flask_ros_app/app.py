from flask import Flask, render_template, request, redirect, url_for, flash
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from flask_interfaces.srv import HuntState, HomeIn
from datetime import datetime
import threading
import base64

app = Flask(__name__)
app.secret_key = "your_secret_key"

# Global variables for video feed and logging
video_frame = None
log_entries = []

# Initialize ROS 2
rclpy.init()
node = rclpy.create_node('flask_ros_node')

# ROS 2 Subscriber for video feed
def video_callback(msg):
    global video_frame
    # Convert CompressedImage data to base64-encoded string
    video_frame = base64.b64encode(msg.data).decode('utf-8')

node.create_subscription(CompressedImage, 'compressed_image', video_callback, 10)

# ROS 2 Service Clients
hunt_state_client = node.create_client(HuntState, '/hunt_state')
home_in_client = node.create_client(HomeIn, '/home_in')

# ROS 2 Spin Thread
def ros_spin():
    rclpy.spin(node)

ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

@app.route("/", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        username = request.form["username"]
        password = request.form["password"]
        if username == "admin" and password == "password":  # Replace with your validation
            return redirect(url_for("main"))
        else:
            flash("Invalid credentials, please try again.")
    return render_template("login.html")

@app.route("/main", methods=["GET", "POST"])
def main():
    global log_entries
    if request.method == "POST":
        action = request.form["action"]
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if action in ["놓침", "포획"]:
            request_msg = HuntState.Request()
            request_msg.state = action
            future = hunt_state_client.call_async(request_msg)
            log_entries.append(f"{current_time} - {action} 요청")
        elif action == "복귀":
            request_msg = HomeIn.Request()
            future = home_in_client.call_async(request_msg)
            log_entries.append(f"{current_time} - 복귀 요청")
    return render_template("main.html", video=video_frame, logs=log_entries)

if __name__ == "__main__":
    app.run(debug=True, use_reloader=False)
