# Spaceship.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

try:
    from .ForcePub import ForcePub
    from .InstructionPub import InstructionPub
    from .StateSub import StateSub
    from .IntMsgSub import IntMsgSub
except ImportError:
    # fallback when running locally as a plain script
    from ForcePub import ForcePub
    from InstructionPub import InstructionPub
    from StateSub import StateSub
    from IntMsgSub import IntMsgSub


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

        self.get_logger().info(
            "Spaceship bridge online.\n"
            " UI in  -> /ui/instruction (String) → control_instruction_topic\n"
            " UI in  -> /ui/force (WrenchStamped) → controller_force_topic\n"
            " UI out <- /ui/state (PoseStamped) ← controller_pose_topic\n"
            " UI out <- /ui/message (String) ← controller_msg_topic"
        )

    # ---------- UI -> Controller ----------
    def sendinstr(self, msg: String):
        # Forward via your InstructionPub's publisher_
        try:
            self.instr_node.sendInstruction(msg)
            self.get_logger().debug(f"Forwarded UI instruction: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to forward instruction: {e}")

    def sendforce(self, fx, fy, fz, tx, ty, tz):
        # Forward via your ForcePub's publisher_
        try:
            self.force_node.set_forces(fx, fy, fz, tx, ty, tz)
            self.get_logger().debug(
                "Forwarded UI force wrench "
                f"(fx={fx}, fy={fy}, fz={fz}, tx={tx}, ty={ty}, tz={tz})"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to forward force: {e}")

    # ---------- Controller -> UI ----------
    def _ctrl_pose_cb(self, msg: PoseStamped):
        msg.pose = self.state_node.latest_pose
        self.get_logger().debug(f"Received controller pose: {msg.pose}")

    def _ctrl_msg_cb(self, msg: String):
        msg.data = self.intmsg_node.latest_msg
        self.get_logger().debug(f"Received controller message: {msg.data}")


def main():
    rclpy.init()
    spaceship = Spaceship()

    executor = MultiThreadedExecutor()
    executor.add_node(spaceship)
    executor.add_node(spaceship.force_node)
    executor.add_node(spaceship.instr_node)
    executor.add_node(spaceship.state_node)
    executor.add_node(spaceship.intmsg_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down spaceship bridge...")
    finally:
        # Make sure the executor stops pulling callbacks
        executor.shutdown()

        # (Optional) remove nodes from executor before destroying
        for n in (
            spaceship.force_node,
            spaceship.instr_node,
            spaceship.state_node,
            spaceship.intmsg_node,
            spaceship,
        ):
            try:
                executor.remove_node(n)
            except Exception:
                pass  # safe to ignore if already removed

        # Destroy child nodes first, then the bridge
        for n in (
            spaceship.force_node,
            spaceship.instr_node,
            spaceship.state_node,
            spaceship.intmsg_node,
            spaceship,
        ):
            try:
                n.destroy_node()
            except Exception:
                pass  # ignore teardown-time races

        # Explicitly shut down the exact context used by these nodes
        try:
            rclpy.shutdown(context=spaceship.context)
        except Exception:
            # Context may already be shutting down due to SIGINT
            pass


def sanity_test(spaceship):
    # quick sanity test
    spaceship.sendinstr(String(data="TEST_INSTRUCTION"))
    spaceship.sendforce(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)


if __name__ == "__main__":
    main()
