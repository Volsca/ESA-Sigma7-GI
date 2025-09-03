import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class ForcePub(Node):
    def __init__(self):
        # Initialize the ros2 node with the name "force_pub"
        super().__init__("force_pub")

        # Create a subscriber to the "controller_pose_topic" topic
        # 'PoseStamped' is the message type containing position data with a time stamp
        # 'controller_pose_topic' is the topic name
        # 'self.pose_callback' is the function to call when a new message is received
        self.publisher_ = self.create_publisher(
            PoseStamped, "controller_pose_topic", 10
        )


def main(args=None):
    rclpy.init(args=args)
    force_pub = ForcePub()
    rclpy.spin(force_pub)
    force_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
