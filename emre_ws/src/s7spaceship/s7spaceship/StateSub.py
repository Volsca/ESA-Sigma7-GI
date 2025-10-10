import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class StateSub(Node):
    def __init__(self):
        # Initialize the ros2 node with the name "state_subscriber"
        super().__init__("state_subscriber")

        # Create a subscriber to the "controller_pose_topic" topic
        # 'PoseStamped' is the message type containing position data with a time stamp
        # 'controller_pose_topic' is the topic name
        # 'self.pose_callback' is the function to call when a new message is received
        self.subscription_pose = self.create_subscription(
            PoseStamped, "controller_pose_topic", self.pose_callback, 10
        )
        self.subscription_twist = self.create_subscription(
            TwistStamped, "controller_twist_topic", self.twist_callback, 10
        )
        self.latest_pose = None
        self.latest_twist = None

    def twist_callback(self, msg):
        self.latest_twist = msg
        # self.get_logger().info(f'Received Twist @ {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

    def pose_callback(self, msg):
        # This function is called whenever a new message is received on the "controller_pose_topic"
        self.latest_pose = msg.pose


def main(args=None):
    rclpy.init(args=args)
    state_subscriber = StateSub()
    rclpy.spin(state_subscriber)
    state_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
