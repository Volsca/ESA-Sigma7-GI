# TODO Communicate with RViz using publisher and subscriber
# TODO Better initilisation of spaceship comments

import sys
import threading
import tty
import termios
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
import csv


class Spaceship(Node):
    """
    Bridge between UI-facing topics and your controller topics.
    Uses your existing publisher/subscriber nodes without guessing internals.
    """

    def __init__(self):
        super().__init__("spaceship")

        # --- Create your nodes (publishers/subscribers) ---
        self.force_node = ForcePub(self)
        self.instr_node = InstructionPub(self)
        self.state_node = StateSub(self)
        self.intmsg_node = IntMsgSub(self)

        self.get_logger().info("Spaceship bridge online...\n")

        # Sorry Emre, I'm setting up a PI control system instead of the PD.
        """
        # Control gains
        self.kp = 0  # low to start, will increase depending on
        self.kd = 0

        # Derivative and Proportional coefficients for angular axis alpha beta and gamma

        self.kpa = 0
        self.kda = 0
        self.kpb = 0
        self.kdb = 0
        self.kpg = 0
        self.kdg = 0"""
        """self.kpa = 1.2
        self.kda = 0.1
        self.kpb = 0.55
        self.kdb = 0.14
        self.kpg = 0.3
        self.kdg = 0.06"""

        # ---------- Control Parameters ----------
        # (no kd, as derivative amplifies delay instability)
        # Will need to implement anti windup for integral term later
        self.kp = 50 # Proportional gain
        self.ki = 0.1  # Integral gain
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0

        self.centering_enabled = False
        self.mode = "Centering"
        self.shutdown = False
        self.get_logger().info("Press SPACE to toggle centering.")
        self.receivedframelist = []
        self.sentframelist = []
        self.create_timer(0.001, self.sigmamode)  # 1000 Hz

    # ---------- UI -> Controller ----------
    def sendinstr(self, msg: String):
        # Forward via your InstructionPub's publisher_
        try:
            self.instr_node.sendInstruction(msg)
            # self.get_logger().info(f"Forwarded UI instruction: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to forward instruction: {e}")

    def sendforce(self, fx, fy, fz, tx, ty, tz, frame_id=None):
        self.force_node.set_forces(fx, fy, fz, tx, ty, tz, frame_id)

    def centering(self):
        if self.centering_enabled and self.state_node.latest_odo is not None:
            self.latest_odo = self.state_node.latest_odo
            # Commented out to test PI control
            """ px = self.latest_odo.pose.pose.position.x
            py = self.latest_odo.pose.pose.position.y
            pz = self.latest_odo.pose.pose.position.z

            da = self.latest_odo.pose.pose.orientation.x
            db = self.latest_odo.pose.pose.orientation.y
            dg = self.latest_odo.pose.pose.orientation.z

            # Linear velocities from twist
            vx = self.latest_odo.twist.twist.linear.x
            vy = self.latest_odo.twist.twist.linear.y
            vz = self.latest_odo.twist.twist.linear.z

            va = self.latest_odo.twist.twist.angular.x
            vb = self.latest_odo.twist.twist.angular.y
            vg = self.latest_odo.twist.twist.angular.z

            # PD control law
            # The 0.0s represent the center position, this can be altered to change the centering position
            dx = self.kp * (0.0 - px) + self.kd * (0.0 - vx)
            dy = self.kp * (0.0 - py) + self.kd * (0.0 - vy)
            dz = self.kp * (0.0 - pz) + self.kd * (0.0 - vz)

            ta = self.kpa * (0.0 - da) + self.kda * (0.0 - va)
            tb = self.kpb * (0.0 - db) + self.kdb * (0.0 - vb)
            tg = self.kpg * (0.0 - dg) + self.kdg * (0.0 - vg) """

            # ---------- Linear Control ----------
            centering_position = 0.0  # placeholder for custom centering spring
            px = self.latest_odo.pose.pose.position.x
            py = self.latest_odo.pose.pose.position.y
            pz = self.latest_odo.pose.pose.position.z

            dx = (
                self.kp * (centering_position - px) + self.ki * self.integral_x
            )
            dy = (
                self.kp * (centering_position - py) + self.ki * self.integral_y
            )
            dz = (
                self.kp * (centering_position - pz) + self.ki * self.integral_z
            )

            ta = tb = tg = 0.0  # No angular control for now

            # update integral terms
            # dt = 0.002s because 500Hz
            self.integral_x += (centering_position - px) * 0.002
            self.integral_y += (centering_position - py) * 0.002
            self.integral_z += (centering_position - pz) * 0.002

            self.get_logger().info(f"integral values : x:{self.integral_x}, y:{self.integral_y}, z:{self.integral_z}")

            force = dx, dy, dz, ta, tb, tg
            #self.get_logger().debug(f"Calculated force: {force}")
            self.receivedframelist.append(self.latest_odo.header.frame_id)
            self.sendforce(
                *force, self.latest_odo.header.frame_id
            )  # unpack to six positional args
        else:
            # If centering is disabled, send nothing
            self.get_logger().debug(
                "Centering disabled or not received odometry."
            )
            return

    def sigmamode(self, mode=None):
        if self.mode == "Centering":
            self.centering()

    # ---------- Controller -> UI ----------
    def _ctrl_pose_cb(self, msg: PoseStamped):
        msg.pose = self.state_node.latest_pose
        # self.get_logger().debug(f"Received controller pose: {msg.pose}")

    def _ctrl_msg_cb(self, msg: String):
        msg.data = self.intmsg_node.latest_msg
        # self.get_logger().debug(f"Received controller message: {msg.data}")


# TODO Test with S7 to see if its receiving
class ForcePub:
    def __init__(self, node: Node):
        self.node = node
        self.publisher_ = node.create_publisher(
            WrenchStamped, "controller_wrench_topic", 10
        )
        # cached wrench (optional)
        self.force_x = self.force_y = self.force_z = 0.0
        self.torque_x = self.torque_y = self.torque_z = 0.0
        # optional high-rate publisher owned by the SAME node
        # self._timer = node.create_timer(0.002, self._timer_cb)  # 500 Hz

    def set_forces(
        self, fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0, frame_id=None
    ):
        self.force_x, self.force_y, self.force_z = fx, fy, fz
        self.torque_x, self.torque_y, self.torque_z = tx, ty, tz
        # Forward via your ForcePub's publisher_
        msg = WrenchStamped()

        now = self.node.get_clock().now().to_msg()
        msg.header.stamp = now
        if frame_id is None:
            frame_id = "NOT_SET"
        else:
            msg.header.frame_id = frame_id
        if self.node.centering_enabled:
            msg.wrench.force.x = self.force_x
            msg.wrench.force.y = self.force_y
            msg.wrench.force.z = self.force_z

            msg.wrench.torque.x = self.torque_x
            msg.wrench.torque.y = self.torque_y
            msg.wrench.torque.z = self.torque_z
        else:
            msg.wrench.force.x = 0.0
            msg.wrench.force.y = 0.0
            msg.wrench.force.z = 0.0

            msg.wrench.torque.x = 0.0
            msg.wrench.torque.y = 0.0
            msg.wrench.torque.z = 0.0
        if not self.node.shutdown:
            try:
                self.publisher_.publish(msg)
            except Exception as e:
                self.node.get_logger().error(f"Failed to forward force: {e}")
            # light logging only when meaningfully non-zero

            x = abs(round(float(msg.wrench.force.x), 6))
            y = abs(round(float(msg.wrench.force.y), 6))
            z = abs(round(float(msg.wrench.force.z), 6))
            tx = abs(round(float(msg.wrench.torque.x), 6))
            ty = abs(round(float(msg.wrench.torque.y), 6))
            tz = abs(round(float(msg.wrench.torque.z), 6))

            #self.node.get_logger().info(
            #    f"Published force wrench (fx={x}, fy={y}, fz={z}, "
            #    f"tx={tx}, ty={ty}, tz={tz})"
            #)
            #self.node.get_logger().info(
            #    f"Published wrench message with message id: {msg.header.frame_id}"
            #)
            self.node.sentframelist.append(msg.header.frame_id)


# TODO Test with S7 to see if this is receiving
class StateSub:
    def __init__(self, node: Node):
        self.node = node
        # Create a subscriber to the "controller_pose_topic" topic
        # 'PoseStamped' is the message type containing position data with a time stamp
        # 'controller_pose_topic' is the topic name
        # 'self.pose_callback' is the function to call when a new message is received
        self.subscription_pose = self.node.create_subscription(
            Odometry, "controller_odometry_topic", self.pose_callback, 10
        )
        self.latest_odo = None

    def pose_callback(self, msg):
        # This function is called whenever a new message is received on the "controller_odometry_topic"
        self.latest_odo = msg
        #self.node.get_logger().debug(
        #    f"Received Pose @ {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        #)


# TODO Needs general testing
class InstructionPub:
    def __init__(self, node: Node = None):
        # Initialize the ros2 node with the name "instruction_pub"
        self.node = node

        # Create a publisher to  to the "control_instruction_topic" topic
        # 'String' is the message type containing the instruction data in str type
        # 'control_instruction_topic' is the topic name
        self.publisher_ = self.node.create_publisher(
            String, "control_instruction_topic", 1
        )

    def sendInstruction(self, msg: String):
        try:
            self.publisher_.publish(msg)
            #self.node.get_logger().info(f"Published instruction: {msg.data}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to publish instruction: {e}")
            return


# TODO Needs general testing
class IntMsgSub:
    def __init__(self, node: Node):
        # Initialize the ros2 node with the name "interface_msg_sub"
        self.node = node

        # Create a subscriber to the "controller_msg_topic" topic
        # 'String' is the message type containing the message in str type
        # 'controller_msg_topic' is the topic name
        # 'self.msg_callback' is the function to call when a new message is received
        self.subscription_pose = self.node.create_subscription(
            String, "controller_msg_topic", self.msg_callback, 10
        )

        self.latest_msg = None

    def msg_callback(self, msg):
        # This function is called whenever a new message is received on the "controller_msg_topic"
        self.node.get_logger().info(f"Received message: {msg.data}")
        self.latest_msg = msg.data


# TODO Tested and works
# TODO Add extra features to activate different modes for S7
def KeyInput(spaceship):
    """
    Runs in a background thread.
    Press SPACE to toggle centering (no Enter needed).
    Press Ctrl+C in the main terminal to quit the node as usual.
    """

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # Puts the terminal into "cbreak" mode, which means input is available to the program immediately (without waiting for Enter). This is essential for real-time key detection.
        tty.setcbreak(fd)
        while not spaceship.shutdown:
            ch = sys.stdin.read(1)
            if ch == " ":
                spaceship.centering_enabled = not spaceship.centering_enabled
                spaceship.get_logger().info(
                    f"Centering {'ENABLED' if spaceship.centering_enabled else 'DISABLED'} (space)."
                )
            if ch == "q":
                spaceship.get_logger().info(
                    "Quit command received, shutting down..."
                )
                spaceship.centering_enabled = False
                print("Shutting down spaceship bridge...")
                # Save framelist to CSV
                try:
                    with open("framelist.csv", "w", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow(["Received Frames", "Sent Frames"])
                        max_len = max(
                            len(spaceship.receivedframelist),
                            len(spaceship.sentframelist),
                        )
                        for i in range(max_len):
                            received = (
                                spaceship.receivedframelist[i]
                                if i < len(spaceship.receivedframelist)
                                else ""
                            )
                            sent = (
                                spaceship.sentframelist[i]
                                if i < len(spaceship.sentframelist)
                                else ""
                            )
                            writer.writerow([received, sent])
                        spaceship.get_logger().info(
                            f"Saved {len(spaceship.sentframelist)} sent and {len(spaceship.receivedframelist)} received frames to framelist.csv"
                        )
                except Exception as e:
                    spaceship.get_logger().error(
                        f"Failed to save framelist: {e}"
                    )
                spaceship.shutdown = True
                time.sleep(1)  # let KeyInput loop exit + restore TTY
                try:
                    spaceship.destroy_node()
                except Exception:
                    pass
                # Guard against double shutdown
                try:
                    if rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    except Exception as e:
        try:
            spaceship.get_logger().error(f"KeyInput error: {e}")
        except Exception:
            pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# Loops and shutsdown properly now
def main():
    rclpy.init()
    spaceship = Spaceship()

    # keyboard thread (optional)
    threading.Thread(target=KeyInput, args=(spaceship,), daemon=True).start()
    # sys library: Provides access to system-specific parameters and functions.
    # sys.stdin: Standard input stream (usually the keyboard).
    # sys.stdin.fileno(): Returns the file descriptor (an integer handle) for the input stream. This is needed for low-level terminal operations.
    fd = sys.stdin.fileno()
    # termios library: Allows you to configure terminal I/O settings on Unix systems.
    # termios.tcgetattr(fd): Gets the current terminal settings for the file descriptor fd and saves them. This is so you can restore them later.
    old_settings = termios.tcgetattr(fd)

    try:
        rclpy.spin(spaceship)  # ONLY ONE NODE TO SPIN
    except KeyboardInterrupt:
        spaceship.centering_enabled = False
        print("Shutting down spaceship bridge...")
        # Save framelist to CSV
        try:
            with open("framelist.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Received Frames", "Sent Frames"])
                max_len = max(
                    len(spaceship.receivedframelist),
                    len(spaceship.sentframelist),
                )
                for i in range(max_len):
                    received = (
                        spaceship.receivedframelist[i]
                        if i < len(spaceship.receivedframelist)
                        else ""
                    )
                    sent = (
                        spaceship.sentframelist[i]
                        if i < len(spaceship.sentframelist)
                        else ""
                    )
                    writer.writerow([received, sent])
                spaceship.get_logger().info(
                    f"Saved {len(spaceship.sentframelist)} sent and {len(spaceship.receivedframelist)} received frames to framelist.csv"
                )
        except Exception as e:
            spaceship.get_logger().error(f"Failed to save framelist: {e}")

        spaceship.shutdown = True

        time.sleep(1)
    finally:
        time.sleep(0.5)  # let KeyInput loop exit + restore TTY
        try:
            spaceship.destroy_node()
        except Exception as e:
            print(f"Error: {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # Guard against double shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error: {e}")
        finally:
            # Restore the terminal settings when done
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
