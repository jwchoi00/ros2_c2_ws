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
        self.conn = sqlite3.connect('/home/g1/ros2_c2_ws/src/flask_ros_app/resource/test_car_captured_images.db', check_same_thread=False)
        self.cursor = self.conn.cursor()

        # TrackingSessions 테이블 생성
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS TrackingSessions (
                tracking_session_id INTEGER PRIMARY KEY AUTOINCREMENT,
                start_timestamp TEXT NOT NULL,
                end_timestamp TEXT,
                status TEXT NOT NULL
            )
        ''')

        # CapturedImages 테이블 생성
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS CapturedImages (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                tracking_session_id INTEGER,
                status TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                image BLOB NOT NULL,
                FOREIGN KEY (tracking_session_id) REFERENCES TrackingSessions (tracking_session_id)
            )
        ''')

        self.conn.commit()

        self.status = "idle"  # 상태 초기화
        self.recording_active = False  # 트래킹 중이라면 기록
        self.center_point = None
        self.tracking_session_id = None  # 세션 ID 초기화

        # Subscriptions 설정
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
        self.create_subscription(
            CenterPoint,
            '/center_point',
            self.detection_callback,
            10
        )

    # 트래킹 세션 시작
    def start_tracking_session(self):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        self.cursor.execute('INSERT INTO TrackingSessions (start_timestamp, status) VALUES (?, ?)', (timestamp, "start_tracking"))
        self.conn.commit()
        self.tracking_session_id = self.cursor.lastrowid  # 마지막 삽입된 tracking_session_id 저장
        self.get_logger().info(f"Tracking session started with ID: {self.tracking_session_id}")

    # 트래킹 세션 종료
    def end_tracking_session(self, status):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        self.cursor.execute('UPDATE TrackingSessions SET end_timestamp = ?, status = ? WHERE tracking_session_id = ?',
                            (timestamp, status, self.tracking_session_id))
        self.conn.commit()
        self.get_logger().info(f"Tracking session with ID {self.tracking_session_id} ended with status: {status}")

    # DB에 이미지 저장
    def insert_captured_image(self, status, image):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        _, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        image_blob = buffer.tobytes()
        self.cursor.execute('INSERT INTO CapturedImages (tracking_session_id, status, timestamp, image) VALUES (?, ?, ?, ?)',
                            (self.tracking_session_id, status, timestamp, image_blob))
        self.conn.commit()

    # Flask로부터 버튼 명령 수령
    def status_callback(self, msg):
        action = msg.data
        if action in ["miss", "capture", "3"]:
            self.status = action
            if self.tracking_session_id is not None:
                self.end_tracking_session(action)  # 세션 종료
            self.get_logger().info(f"Recording stopped with {action}.")
        else:
            self.get_logger().warning(f"Unrecognized or inactive action: {action}")

    # 이미지 subscribe 받고 db에 저장하는 함수 호출
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if not self.recording_active:
            self.get_logger().info("Recording is inactive. Ignoring image...")
            return

        if self.status == "start_tracking" and self.tracking_session_id is None:
            self.start_tracking_session()  # 세션 시작
            self.get_logger().info(f"Saving image with status: {self.status}")
            self.insert_captured_image(self.status, frame)
            self.status = "tracking"  # tracking 상태로 변경

        elif self.status == "tracking":
            self.get_logger().info(f"Saving image with status: {self.status}")
            self.insert_captured_image(self.status, frame)

        elif self.status in ["miss", "capture", "3"]:
            self.get_logger().info(f"Saving final image with status: {self.status}")
            if self.status == 3:
                self.status = 'return'
            self.insert_captured_image(self.status, frame)
            self.recording_active = False  # 저장 끄기

    # 물체가 인식되었는지 판단하여 start_tracking
    def detection_callback(self, msg):
        self.center_point = [msg.centerx, msg.centery]
        if self.center_point is not None and self.center_point[0] != 0.0 and self.center_point[1] != 0.0:
            # self.get_logger().info(f"Center point detected: {self.center_point}")
            if not self.recording_active:
                self.status = "start_tracking"
                self.recording_active = True
        else:
            self.get_logger().warning("No center_point detected.")

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
