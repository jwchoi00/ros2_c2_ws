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
        self.conn = sqlite3.connect('/home/g1/ros2_c2_ws/src/flask_ros_app/resource/auto_test_car_captured_images.db', check_same_thread=False)
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

    def publish_action(self, action):
        msg = String()
        msg.data = action
        self.publisher.publish(msg)
        self.get_logger().info(f"Published action: {action}")

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
        #action_data_to_robot = action[0]
        #action_data_to_db = action[1]
        #print(action_data_to_robot, action_data_to_db)
        if action in ["miss", "capture", "return", "homeout", "start"]: #1놓침, 2 잡음, 3 복귀 4 로그인 5일시 정지
            self.status = action
            #self.get_logger().info(f"Recording stopped with {actionaction_data_to_db}.")
        else:
            self.get_logger().warning(f"Unrecognized or inactive action: {action}")

    # detection_callback에서 time_diff 계산을 계속 갱신하도록 수정
    def detection_callback(self, msg):
        self.center_point = [msg.centerx, msg.centery]
        if self.center_point and self.center_point[0] != 0.0 and self.center_point[1] != 0.0:
            # Reset the timer if a valid center point is detected
            self.start_time = None
            self.time_diff = None
            if not self.recording_active:
                self.status = "start_tracking"
                self.recording_active = True
        else:
            # Start or update the timer when no valid center point is detected
            if self.start_time is None:
                self.start_time = time.time()  # First detection time
            else:
                end_time = time.time()
                self.time_diff = end_time - self.start_time  # Time difference in seconds
                self.get_logger().info(f"Time without valid detection: {self.time_diff:.2f} seconds")

            # If the time without valid center_point is more than 5 seconds, stop recording
            if self.time_diff and self.time_diff > 5.0:
                self.status = 'return'
                action_to_robot = '1'  # Publish '1' to stop the recording
                self.publish_action(action_to_robot)
                self.recording_active = False  # Stop recording
                self.get_logger().info(f"Recording stopped after {self.time_diff:.2f} seconds of no detection.")
                self.start_time = None  # Reset start time
                self.time_diff = None  # Reset time_diff

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if not self.recording_active:
            self.insert_captured_image(self.status, frame)
            return

        # Save image and stop recording if the status is 'return'
        if self.status == "return":
            self.get_logger().info(f"Saving final image with status: {self.status}")
            self.insert_captured_image(self.status, frame)
            self.recording_active = False  # Stop recording
            self.get_logger().info(f"Recording stopped after {self.time_diff:.2f} seconds.")
            self.start_time = None  # Reset start time
            self.time_diff = None  # Reset time_diff
            return

        if self.status == "start_tracking":
            self.get_logger().info(f"Saving image with status: {self.status}")
            self.insert_captured_image(self.status, frame)
            self.status = "tracking"  # Transition to tracking status

        elif self.status == "tracking":
            self.get_logger().info(f"Saving image with status: {self.status}")
            self.insert_captured_image(self.status, frame)

        elif self.status in ["miss", "capture", "return", "homeout", "patorol"]:
            if self.status == 'miss':
                action_to_robot = '1'  # Publish '1' to stop the recording
                self.publish_action(action_to_robot)
            elif self.status == 'capture':
                action_to_robot = '1'  # Publish '1' to stop the recording
                self.publish_action(action_to_robot)
                self.status = 'capture'
            elif self.status == 'patorol':
                action_to_robot = '1'  # Publish '1' to stop the recording
                self.publish_action(action_to_robot)
                self.status = 'return'
            self.recording_active = False

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
