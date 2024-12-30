import sqlite3
import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
from std_msgs.msg import String

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.conn = sqlite3.connect('/home/g1/ros2_c2_ws/src/flask_ros_app/resource/captured_images.db', check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS CapturedImages (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                status TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                image BLOB NOT NULL
            )
        ''')
        self.conn.commit()

        self.status = "idle"  # 초기 상태 설정

        self.create_subscription(
            CompressedImage,
            'compressed_image',
            self.image_callback,
            10
        )

        self.create_subscription(
            String,
            'robot_actions',
            self.status_callback,
            10
        )

    def insert_captured_image(self, status, image):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        _, buffer = cv2.imencode('.jpg', image)
        image_blob = buffer.tobytes()
        self.cursor.execute('INSERT INTO CapturedImages (status, timestamp, image) VALUES (?, ?, ?)', (status, timestamp, image_blob))
        self.conn.commit()

    def status_callback(self, msg):
        action = msg.data
        if action in ["miss", "capture", "return"]:
            self.status = action
            self.get_logger().info(f"Status updated to: {self.status}")
        else:
            self.get_logger().warning(f"Unrecognized action: {action}")

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if not self.status or self.status not in ["miss", "capture", "return"]:
            self.get_logger().info("No valid status set. Ignoring image...")
            return

        self.get_logger().info(f"Handling {self.status} action...")
        self.insert_captured_image(self.status, frame)

    def destroy_node(self):
        self.conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    compressed_image_subscriber = CompressedImageSubscriber()
    rclpy.spin(compressed_image_subscriber)
    compressed_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
