import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class ForcePub(Node):
    def __init__(self):
        # Initialize the ros2 node with the name "force_pub"
        super().__init__("force_pub")

        # Create a publisher to  to the "controller_force_topic" topic
        # 'WrenchStamped' is the message type containing force data with a time stamp
        # 'controller_force_topic' is the topic name
        self.publisher_ = self.create_publisher(
            WrenchStamped, "controller_force_topic", 1
        )

        # Timer for publishing wrench messages at 500Hz
        self.timer = self.create_timer(0.002, self.timer_callback)

    def timer_callback(self):
        msg = WrenchStamped()
        # Populate the message fields with appropriate data

        now = self.get_clock().now().to_msg()
        msg.header.stamp = now

        # Example force and torque values (replace with actual computation)
        dx = 0.0  # Force in x-direction
        dy = 0.0  # Force in y-direction
        dz = 0.0  # Force in z-direction
        ta = 0.0  # Torque around x-axis
        tb = 0.0  # Torque around y-axis
        tg = 0.0  # Torque around z-axis

        msg.wrench.force.x = dx
        msg.wrench.force.y = dy
        msg.wrench.force.z = dz
        msg.wrench.torque.x = ta
        msg.wrench.torque.y = tb
        msg.wrench.torque.z = tg

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published Wrench @ {now.sec}.{now.nanosec}: "
            f"Force=({dx:.2f}, {dy:.2f}, {dz:.2f}), "
            f"Torque=({ta:.2f}, {tb:.2f}, {tg:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    force_pub = ForcePub()
    rclpy.spin(force_pub)
    force_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
