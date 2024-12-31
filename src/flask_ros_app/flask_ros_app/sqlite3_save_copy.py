import sqlite3
import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
from std_msgs.msg import String
from interface.msg import CenterPoint

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.conn = sqlite3.connect('/home/g1/ros2_c2_ws/src/flask_ros_app/resource/copy_car_captured_images.db', check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS CapturedImages (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                status TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                image BLOB NOT NULL
            )''')
        self.conn.commit()
        self.status = "idle"  # 상태 초기와
        self.recording_active = False  # 트랙킹 중 이라면 기록
        self.center_point = None
        self.start_time = None  # 첫 번째 감지 시간 저장
        self.time_diff = None  # time_diff 초기화
        
        self.publisher = self.create_publisher(String, 'robot_actions', 10)
        self.create_subscription(
            CompressedImage,
            'compressed_image',
            self.image_callback,
            10
        )
        self.create_subscription(
            String,
            '/robot_action_to_db',
            self.status_callback,
            10
        )
        self.create_subscription(
            CenterPoint,
            '/center_point',
            self.detection_callback,
            10
        )

    def publish_to_robot(self, publish_to_robot):
        msg = String()
        msg.data = publish_to_robot
        self.publisher.publish(msg)
        self.get_logger().info(f"Published action: {publish_to_robot}")

    #db에 저장
    def insert_captured_image(self, status, image):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        _, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        image_blob = buffer.tobytes()
        self.cursor.execute('INSERT INTO CapturedImages (status, timestamp, image) VALUES (?, ?, ?)', (status, timestamp, image_blob))
        self.conn.commit()

    #flask로 부터 버튼 명령 수령
    def status_callback(self, msg):
        action = msg.data
        if action in ["miss", "capture", "return", "homeout", "patorol", "stop"]:
            self.status = action
            publish_to_robot_msg = ''
            self.get_logger().info(f"Recording stopped with {action}.")
            if self.status == 'miss' or self.status == 'capture' or self.status == 'patorol':
                publish_to_robot_msg = '1'
                self.publish_to_robot(publish_to_robot_msg) #로봇으로 publish
            elif self.status == 'homeout':
                publish_to_robot_msg = '2'
                self.publish_to_robot(publish_to_robot_msg) #로봇으로 publish
            elif self.status == 'return':
                publish_to_robot_msg = '3'
                self.publish_to_robot(publish_to_robot_msg) #로봇으로 publish
            elif self.status == 'stop':
                publish_to_robot_msg = '4'
                self.publish_to_robot(publish_to_robot_msg) #로봇으로 publish
        else:
            self.get_logger().warning(f"Unrecognized or inactive action: {action}")

    #이미지 subscribe 받고 db저장하는 함수 호출
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if not self.recording_active and self.status == 'miss':
            self.insert_captured_image(self.status, frame)
            self.status = ''
            self.get_logger().info("Lost target. Save and ignoring image...")
            return
        if not self.recording_active:
            self.get_logger().info("Recordin is inactive. Ignoring image...")
            return
        if self.status == "start_tracking":
            self.get_logger().info(f"Saving image with status: {self.status}")
            self.insert_captured_image(self.status, frame)
            self.status = "tracking"  # tracking이라는 로그로 이미지 저장

        elif self.status == "tracking":
            self.get_logger().info(f"Saving image with status: {self.status}")
            self.insert_captured_image(self.status, frame)

        elif self.status in ["miss", "capture", "return", "patorol", "stop"]:
            self.get_logger().info(f"Saving final image with status: {self.status}")
            self.insert_captured_image(self.status, frame)
            self.recording_active = False  # 저장 끄기
        time.sleep(1.0)
    #물체가 인식되었는지 판단하여 start_tracking
    def detection_callback(self, msg):
        self.center_point = [msg.centerx, msg.centery]
        if self.center_point and self.center_point[0] != 0.0 and self.center_point[1] != 0.0:
            # Reset the timer if a valid center point is detected
            self.start_time = None
            self.time_diff = None
            if not self.recording_active:
                self.status = "start_tracking"
                self.recording_active = True
        elif self.status == 'tracking':
            # Start or update the timer when no valid center point is detected
            if self.start_time is None:
                self.start_time = time.time()  # First detection time
            else:
                end_time = time.time()
                self.time_diff = end_time - self.start_time  # Time difference in seconds
                self.get_logger().info(f"Time without valid detection: {self.time_diff:.2f} seconds")

            # If the time without valid center_point is more than 5 seconds, stop recording
            if self.time_diff and self.time_diff > 5.0:
                self.status = 'miss'
                publish_to_robot_msg = '1'
                self.publish_to_robot(publish_to_robot_msg) #로봇으로 publish
                self.recording_active = False  # Stop recording
                self.get_logger().info(f"Recording stopped after {self.time_diff:.2f} seconds of no detection.")
                self.start_time = None  # Reset start time
                self.time_diff = None  # Reset time_diff

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
