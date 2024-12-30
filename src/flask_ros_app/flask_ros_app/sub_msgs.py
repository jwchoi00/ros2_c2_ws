import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ActionSubscriberNode(Node):
    def __init__(self):
        super().__init__('action_subscriber_node')
        # Create a subscriber for the 'robot_actions' topic
        self.subscription = self.create_subscription(
            String,  # Message type
            'robot_actions',  # Topic name
            self.listener_callback,  # Callback method
            10  # QoS (Quality of Service) depth
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received message
        action = msg.data
        self.get_logger().info(f"Received action: {action}")

        # Take actions based on the received message
        if action == "miss":
            self.handle_miss()
        elif action == "capture":
            self.handle_capture()
        elif action == "return":
            self.handle_return()
        else:
            self.get_logger().warn(f"Unknown action: {action}")

    def handle_miss(self):
        # Implement logic for handling the "miss" action
        self.get_logger().info("Handling miss action...")

    def handle_capture(self):
        # Implement logic for handling the "capture" action
        self.get_logger().info("Handling capture action...")

    def handle_return(self):
        # Implement logic for handling the "return" action
        self.get_logger().info("Handling return action...")

def main(args=None):
    rclpy.init(args=args)

    # Initialize the subscriber node
    action_subscriber_node = ActionSubscriberNode()

    # Spin the node to keep it alive and processing incoming messages
    rclpy.spin(action_subscriber_node)

    # Shutdown the node
    action_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
