import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template, Response, redirect, url_for, flash, request, jsonify
from flask_login import LoginManager, UserMixin, login_user, login_required, logout_user, current_user
import cv2
from ultralytics import YOLO
import psutil

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
    if not current_user.is_authenticated:
        return redirect(url_for('login'))  # Redirect to login page if not authenticated
    return render_template("index.html")  # Keep the user on the index page after login

def generate_frames():
    cap = cv2.VideoCapture(0)
    
    # Initialize YOLOv8 model
    model = YOLO("yolov8n.pt")  # Replace with the correct model path
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        
        results = model(frame)
        
        result_img = results[0].plot()
        
        _, buffer = cv2.imencode('.jpg', result_img)
        frame = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
    
    cap.release()

@app.route("/video_feed")
@login_required
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

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

    def publish_action(self, action):
        msg = String()
        msg.data = action
        self.publisher.publish(msg)
        self.get_logger().info(f"Published action: {action}")

# Start ROS 2 node in Flask route
def start_ros_node():
    rclpy.init()
    node = ActionPublisherNode()
    return node

@app.route("/action", methods=["POST"])
@login_required
def action():
    action = request.json.get('action')
    
    if action:
        # Start the ROS 2 node if not started yet
        node = start_ros_node()
        node.publish_action(action)
        rclpy.shutdown()
        
        return jsonify({"status": "success", "action": action}), 200
    else:
        return jsonify({"status": "error", "message": "No action provided"}), 400

if __name__ == "__main__":
    app.run(debug=True)
