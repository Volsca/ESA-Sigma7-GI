# Spaceship.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped, PoseStamped

try:
    from .ForcePub import ForcePub
    from .InstructionPub import InstructionPub
    from .StateSub import StateSub
    from .IntMsgSub import IntMsgSub
except ImportError:
    # fallback when running locally as a plain script
    from s7spaceship.ForcePub import ForcePub
    from s7spaceship.InstructionPub import InstructionPub
    from s7spaceship.StateSub import StateSub
    from s7spaceship.IntMsgSub import IntMsgSub


class Spaceship(Node):
    """
    Bridge between UI-facing topics and your controller topics.
    Uses your existing publisher/subscriber nodes without guessing internals.
    """

    def __init__(self):
        super().__init__("spaceship")

        # --- Create your nodes (publishers/subscribers) ---
        self.force_node = ForcePub()
        self.instr_node = InstructionPub()
        self.state_node = StateSub()
        self.intmsg_node = IntMsgSub()

        # --- UI -> Controller (inputs from user/UI) ---
        self.ui_instr_sub = self.create_subscription(
            String, "/ui/instruction", self._ui_instruction_cb, 10
        )
        self.ui_force_sub = self.create_subscription(
            WrenchStamped, "/ui/force", self._ui_force_cb, 10
        )

        # --- Controller -> UI (outbound to user/UI) ---
        # We subscribe to controller topics and republish to /ui/* for the UI to consume.
        self.ui_state_pub = self.create_publisher(PoseStamped, "/ui/state", 10)
        self.ui_msg_pub = self.create_publisher(String, "/ui/message", 10)

        self.ctrl_pose_sub = self.create_subscription(
            PoseStamped, "controller_pose_topic", self._ctrl_pose_cb, 10
        )
        self.ctrl_msg_sub = self.create_subscription(
            String, "controller_msg_topic", self._ctrl_msg_cb, 10
        )

        self.get_logger().info(
            "Spaceship bridge online.\n"
            " UI in  -> /ui/instruction (String) → control_instruction_topic\n"
            " UI in  -> /ui/force (WrenchStamped) → controller_force_topic\n"
            " UI out <- /ui/state (PoseStamped) ← controller_pose_topic\n"
            " UI out <- /ui/message (String) ← controller_msg_topic"
        )

    # ---------- UI -> Controller ----------
    def _ui_instruction_cb(self, msg: String):
        # Forward via your InstructionPub's publisher_
        try:
            self.instr_node.publisher_.publish(msg)
            self.get_logger().debug(f"Forwarded UI instruction: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to forward instruction: {e}")

    def _ui_force_cb(self, msg: WrenchStamped):
        # Forward via your ForcePub's publisher_
        try:
            self.force_node.publisher_.publish(msg)
            self.get_logger().debug(
                "Forwarded UI force wrench "
                f"(fx={msg.wrench.force.x}, fy={msg.wrench.force.y}, fz={msg.wrench.force.z}, "
                f"tx={msg.wrench.torque.x}, ty={msg.wrench.torque.y}, tz={msg.wrench.torque.z})"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to forward force: {e}")

    # ---------- Controller -> UI ----------
    def _ctrl_pose_cb(self, msg: PoseStamped):
        self.ui_state_pub.publish(msg)

    def _ctrl_msg_cb(self, msg: String):
        self.ui_msg_pub.publish(msg)


def main():
    rclpy.init()

    spaceship = Spaceship()

    executor = MultiThreadedExecutor()
    # Add the bridge first
    executor.add_node(spaceship)
    # Also spin your attached nodes
    executor.add_node(spaceship.force_node)
    executor.add_node(spaceship.instr_node)
    executor.add_node(spaceship.state_node)
    executor.add_node(spaceship.intmsg_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        # Clean up
        spaceship.destroy_node()
        spaceship.force_node.destroy_node()
        spaceship.instr_node.destroy_node()
        spaceship.state_node.destroy_node()
        spaceship.intmsg_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
