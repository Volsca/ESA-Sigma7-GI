# TODO Communicate with RViz using publisher and subscriber
# TODO Better initilisation of spaceship comments

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

        self.get_logger().info("Spaceship bridge online...\n")

    def centering(self):
        # Latest received messages
        self.latest_pose = self.state_node.latest_pose
        self.latest_twist = self.state_node.latest_twist

        # Control gains
        self.kp = 180
        self.kd = 25

        # Derivative and Proportional coefficients for angular axis alpha beta and gamma
        self.kpa = 1.2
        self.kda = 0.1
        self.kpb = 0.55
        self.kdb = 0.14
        self.kpg = 0.3
        self.kdg = 0.06

        if self.latest_pose is None or self.latest_twist is None:
            return (0.0,) * 6  # Return zeros if either is missing

        # Positions from pose
        px = self.latest_pose.pose.position.x
        py = self.latest_pose.pose.position.y
        pz = self.latest_pose.pose.position.z

        # Linear velocities from twist
        vx = self.latest_twist.twist.linear.x
        vy = self.latest_twist.twist.linear.y
        vz = self.latest_twist.twist.linear.z

        # For simplicity, we assume no rotation tracking in this version
        da = db = dg = 0.0  # Orientation error placeholder
        va = vb = vg = 0.0  # Angular velocity

        # PD control law
        # The 0.0s represent the center position, this can be altered to change the centering position
        dx = self.kp * (0.0 - px) + self.kd * (0.0 - vx)
        dy = self.kp * (0.0 - py) + self.kd * (0.0 - vy)
        dz = self.kp * (0.0 - pz) + self.kd * (0.0 - vz)

        ta = self.kpa * (0.0 - da) + self.kda * (0.0 - va)
        tb = self.kpb * (0.0 - db) + self.kdb * (0.0 - vb)
        tg = self.kpg * (0.0 - dg) + self.kdg * (0.0 - vg)
        force = dx, dy, dz, ta, tb, tg

        self.sendforce(force)

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
