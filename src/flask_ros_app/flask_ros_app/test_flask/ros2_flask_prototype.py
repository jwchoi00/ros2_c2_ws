import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template, Response, redirect, url_for, flash, request, jsonify
from flask_login import LoginManager, UserMixin, login_user, login_required, logout_user, current_user
import cv2
from ultralytics import YOLO
import psutil
from sensor_msgs.msg import CompressedImage
import base64
import numpy as np
import threading

app = Flask(__name__)
app.secret_key = "your_secret_key"
login_manager = LoginManager()
login_manager.init_app(app)

# User class for login
class User(UserMixin):
    def __init__(self, id):
        self.id = id

# Simulate user authentication (example: user credentials are hardcoded)
users = {"admin": {"password": "adminpassword"}}

# Login manager to load user from session
@login_manager.user_loader
def load_user(user_id):
    return User(user_id)

@app.route("/", methods=["GET", "POST"])
def index():
    global current_image_base64
    if not current_user.is_authenticated:
        return redirect(url_for('login'))  # Redirect to login page if not authenticated
    # Pass the current_image_base64 to the template
    return render_template("index.html", image_data=current_image_base64)

@app.route("/login", methods=["GET", "POST"])
def login():
    if current_user.is_authenticated:
        return redirect(url_for('index'))

    if request.method == "POST":
        username = request.form['username']
        password = request.form['password']
        
        if username in users and users[username]['password'] == password:
            user = User(username)
            login_user(user)
            return redirect(url_for('index'))
        else:
            flash("Invalid credentials")
            return redirect(url_for('login'))
    return render_template("login.html")

@app.route("/logout")
@login_required
def logout():
    logout_user()
    return redirect(url_for('login'))

def get_system_stats():
    cpu_usage = psutil.cpu_percent(interval=1)
    memory = psutil.virtual_memory()
    memory_usage = memory.percent
    total_memory = memory.total / (1024 * 1024 * 1024)  # GB
    disk = psutil.disk_usage('/')
    disk_usage = disk.percent
    total_disk = disk.total / (1024 * 1024 * 1024)  # GB
    
    stats = {
        "cpu_usage": cpu_usage,
        "memory_usage": memory_usage,
        "total_memory": total_memory,
        "disk_usage": disk_usage,
        "total_disk": total_disk
    }
    
    return stats

@app.route("/sysmon", methods=["GET"])
@login_required
def sysmon():
    stats = get_system_stats()
    return render_template("sysmon.html", stats=stats)

@app.route("/sysmon_stats", methods=["GET"])
@login_required
def sysmon_stats():
    stats = get_system_stats()
    return stats  # Return the stats as a JSON response

# ROS 2 Node to Publish Actions
class ActionPublisherNode(Node):
    def __init__(self):
        super().__init__('action_publisher_node')
        self.publisher = self.create_publisher(String, 'robot_actions', 10)
        self.get_logger().info("Flask Node initialized.")
        self.subscription = self.create_subscription(
            CompressedImage,
            'compressed_image',
            self.image_callback,
            10
        )
        global current_image_base64
        current_image_base64 = ""

    def publish_action(self, action):
        msg = String()
        msg.data = action
        self.publisher.publish(msg)
        self.get_logger().info(f"Published action: {action}")

    def image_callback(self, msg):
        
        # Convert the byte array to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        
        # Decode the image
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imwrite("/tmp/debug_image.jpg", frame)  # Save image to file for inspection
            global current_image_base64
            # Convert image to base64
            _, encoded_image = cv2.imencode('.jpg', frame)
            byte_image = encoded_image.tobytes()
            current_image_base64 = base64.b64encode(byte_image).decode('utf-8')
            self.get_logger().info("Image updated for Flask")
        else:
            self.get_logger().error('Failed to decode the image')

@app.route("/action", methods=["POST"])
@login_required
def action():
    global node  # Use the global node instance
    action = request.json.get('action')
    
    if action:
        node.publish_action(action)  # Send the action to the ROS 2 node
        return jsonify({"status": "success", "action": action}), 200
    else:
        return jsonify({"status": "error", "message": "No action provided"}), 400

def start_flask():
    app.run(host='0.0.0.0', port=5000)


node = None

def main(args=None):
    global node  # Declare node as global
    rclpy.init(args=args)
    node = ActionPublisherNode()
    
    # Flask server in a separate thread
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.start()
    
    rclpy.spin(node)
    
    flask_thread.join()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
