from flask import Flask, request, render_template, jsonify, redirect, url_for, flash
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import base64
from io import BytesIO

app = Flask(__name__)
app.secret_key = 'your_secret_key_here'  # For flash messages

# Dummy credentials (use secure authentication for production)
USERNAME = 'admin'
PASSWORD = 'password'

# Global variable to store the current image in base64 format
current_image_base64 = None

# Route for login page
@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form.get('username')
        password = request.form.get('password')
        
        # Check the credentials (modify for real authentication)
        if username == USERNAME and password == PASSWORD:
            return redirect(url_for('home'))  # Redirect to home after successful login
        else:
            flash('Invalid credentials. Please try again.', 'danger')
            return redirect(url_for('login'))
    
    return render_template('login.html')  # Make sure this template exists

# Home page after login
@app.route('/')
def home():
    return render_template('index.html')  # Ensure this template exists

# Route to get the image in base64 format
@app.route('/image', methods=['GET'])
def get_image():
    if current_image_base64:
        return jsonify({'image': current_image_base64})
    else:
        return "No image available", 404

# Function to start Flask server
def start_flask():
    app.run(host='0.0.0.0', port=5000)

# ROS 2 Node for image handling
class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self.get_logger().info("Flask Node initialized.")
        self.subscription = self.create_subscription(
            CompressedImage,
            'compressed_image',
            self.image_callback,
            10
        )
        
    def image_callback(self, msg):
        # Convert the byte array to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        
        # Decode the image
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            frame = cv2.resize(frame, (640, 480))
            # Convert image to base64
            _, encoded_image = cv2.imencode('.jpg', frame)
            byte_image = encoded_image.tobytes()
            global current_image_base64
            current_image_base64 = base64.b64encode(byte_image).decode('utf-8')
            self.get_logger().info("Image updated for Flask")
        else:
            self.get_logger().error('Failed to decode the image')

# Main function to run the ROS 2 node and Flask server
def main(args=None):
    rclpy.init(args=args)
    node = FlaskNode()
    
    # Flask server in a separate thread
    import threading
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.start()
    
    rclpy.spin(node)
    
    flask_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
