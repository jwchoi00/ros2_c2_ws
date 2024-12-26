import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageSubscriber(Node):

    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'compressed_image',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        # Convert the byte array to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        
        # Decode the image
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            # Display the image using OpenCV
            cv2.imshow("Compressed Image", frame)

            # Wait for key press to close the window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Closing image window')
                cv2.destroyAllWindows()
        else:
            self.get_logger().error('Failed to decode the image')

def main(args=None):
    rclpy.init(args=args)
    compressed_image_subscriber = CompressedImageSubscriber()
    rclpy.spin(compressed_image_subscriber)
    compressed_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
