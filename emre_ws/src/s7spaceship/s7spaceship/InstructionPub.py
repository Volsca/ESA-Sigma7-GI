import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InstructionPub(Node):
    def __init__(self):
        # Initialize the ros2 node with the name "instruction_pub"
        super().__init__("instruction_pub")

        # Create a publisher to the "control_instruction_topic" topic
        # 'String' is the message type containing the instruction data in str type
        # 'control_instruction_topic' is the topic name
        self.publisher_ = self.create_publisher(
            String, "control_instruction_topic", 1
        )

    def sendInstruction(self, msg: String):
        try:
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published instruction: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish instruction: {e}")
            return


def main(args=None):
    rclpy.init(args=args)
    instruction_pub = InstructionPub()
    rclpy.spin(instruction_pub)
    instruction_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
