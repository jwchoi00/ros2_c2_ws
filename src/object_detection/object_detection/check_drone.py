import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO
from geometry_msgs.msg import Point
from interface.msg import CenterPoint
from geometry_msgs.msg import Twist

# from moveit_interfaces.msg import DetectionArray, Detection

### 실시간성을 높이기 위한 코드 개발 (내일)

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('check_drone')
        # 사전에 생성한 YOLO모델을 불러오기
        self.pub_cmd_vel=self.create_publisher(Twist,'/cmd_vel',10)
        self.model = YOLO("/home/g1/ros2_c2_ws/src/object_detection/resource/best_ver2.pt")

        # # Camera 왜곡 보정 파라메터
        # self.K = np.array([[1537.87246, 0, 656.024384], [0, 1570.34693, 618.027499], [0, 0, 1]])
        # self.d = np.array([0.156609014, -0.487498585, 0.0537193345, 0.00294416872, 3.06628289])
        self.cap = cv2.VideoCapture(0) #USB camera Read
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        # # yolo 결과값 전송
        self.publisher_ = self.create_publisher(CompressedImage, 'compressed_image', 10)
        #self.pub_center_point = self.create_publisher(Point,'/center_point',10)
        self.pub_center_point = self.create_publisher(CenterPoint,'/center_point',10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.center_p = Twist()
        self.center_x = 0.0
        self.center_y = 0.0
        
            
    def timer_callback(self):
        # numpy array로 변경
        
        ret, frame = self.cap.read()
        if frame is not None:
            self.get_logger().info(f"Received frame of size: {frame.shape}")
            results = self.model(frame, conf=0.85)  # 정확도 0.5 이상만
            detection_data = []
            h, w = frame.shape[:2]
            self.center_x = 0.0
            self.center_y = 0.0
            # 오브젝트 체크
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()  # box 좌표 넣기
                    self.center_x = ((x1 + x2) / 2) - (w/2) # 카메라 중심 픽셀 부터 박스 중심 좌표 계산
                    self.center_y = ((y1 + y2) / 2) - (2*(h/3))
                    confidence = box.conf[0].item()  # 정확도
                    class_id = int(box.cls[0].item())  # class id
                    detection_data.append({
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'center_x': self.center_x, 'center_y': self.center_y,
                        'confidence': confidence, 'class_id': class_id
                    })
                    ## ==== center point pub
                    # 박스 그리기
                    cv2.rectangle(frame, (int(w/2-30), int(2*(h/3)-10)), (int(w/2+30), int(2*(h/3)+10)), (0, 0, 255), 2)
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    label = f"Class: {class_id}, Conf: {confidence:.2f}"
                    cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # if results.
            # self.center_x = 0.0
            # self.center_y = 0.0
                
            CenterPointPub = CenterPoint()
            CenterPointPub.centerx = self.center_x
            CenterPointPub.centery = self.center_y
            self.pub_center_point.publish(CenterPointPub)
            # print(detection_data)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]  # Quality set to 90
        _, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        # Prepare the CompressedImage message
        compressed_image = CompressedImage()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.format = 'jpeg'
        compressed_image.data = encoded_image.tobytes()

        # Publish the compressed image
        self.publisher_.publish(compressed_image)
        self.get_logger().info('Publishing compressed image')


def main(args=None):
    rclpy.init(args=args)
    compressed_image_subscriber = CompressedImageSubscriber()
    rclpy.spin(compressed_image_subscriber)
    compressed_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()