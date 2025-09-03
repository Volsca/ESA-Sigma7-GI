import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IntMsgSub(Node):
    def __init__(self):
        # Initialize the ros2 node with the name "interface_msg_sub"
        super().__init__("interface_msg_sub")

        # Create a subscriber to the "controller_msg_topic" topic
        # 'String' is the message type containing the message in str type
        # 'controller_msg_topic' is the topic name
        # 'self.msg_callback' is the function to call when a new message is received
        self.subscription_pose = self.create_subscription(
            String, "controller_msg_topic", self.msg_callback, 10
        )

        self.latest_msg = None

    def msg_callback(self, msg):
        # This function is called whenever a new message is received on the "controller_msg_topic"
        self.get_logger().info(f"Received message: {msg.data}")
        self.latest_msg = msg.data


def main(args=None):
    rclpy.init(args=args)
    msg_subscriber = IntMsgSub()
    rclpy.spin(msg_subscriber)
    msg_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
