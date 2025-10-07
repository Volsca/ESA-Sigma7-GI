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

        now = self.get_clock().now().to_msg()
        msg.header.stamp = now

        # Get force values from class attributes (set externally)
        dx = getattr(self, "force_x", 0.0)
        dy = getattr(self, "force_y", 0.0)
        dz = getattr(self, "force_z", 0.0)
        ta = getattr(self, "torque_x", 0.0)
        tb = getattr(self, "torque_y", 0.0)
        tg = getattr(self, "torque_z", 0.0)

        msg.wrench.force.x = dx
        msg.wrench.force.y = dy
        msg.wrench.force.z = dz
        msg.wrench.torque.x = ta
        msg.wrench.torque.y = tb
        msg.wrench.torque.z = tg

        try:
            self.publisher_.publish(msg)
            if any([dx, dy, dz, ta, tb, tg]):
                self.get_logger().info(
                    f"Published force wrench (fx={msg.wrench.force.x}, fy={msg.wrench.force.y}, fz={msg.wrench.force.z}, "
                    f"tx={msg.wrench.torque.x}, ty={msg.wrench.torque.y}, tz={msg.wrench.torque.z})"
                )
                self.set_forces(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f"Failed to publish force wrench: {e}")
            return

    def set_forces(self, fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0):
        """Set the force and torque values to be published"""
        self.force_x = fx
        self.force_y = fy
        self.force_z = fz
        self.torque_x = tx
        self.torque_y = ty
        self.torque_z = tz


def main(args=None):
    rclpy.init(args=args)
    force_pub = ForcePub()
    rclpy.spin(force_pub)
    force_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
