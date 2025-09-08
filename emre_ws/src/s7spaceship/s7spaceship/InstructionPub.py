import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InstructionPub(Node):
    def __init__(self):
        # Initialize the ros2 node with the name "instruction_pub"
        super().__init__("instruction_pub")

        # Create a publisher to  to the "control_instruction_topic" topic
        # 'String' is the message type containing the instruction data in str type
        # 'control_instruction_topic' is the topic name
        self.publisher_ = self.create_publisher(
            String, "control_instruction_topic", 1
        )

        # Timer for publishing wrench messages at 100Hz
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        msg = String()
        # Populate the message fields with appropriate data
        msg.data = "Control instruction data"

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published instruction: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    instruction_pub = InstructionPub()
    rclpy.spin(instruction_pub)
    instruction_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
