#!/usr/bin/env python3
"""Publishes every frame of openpilot's can stream, undecoded, as a standard can_msgs/Frame on can_frames.

The can stream holds the frames the Comma's panda receives and the frames it sends, from all buses, in the order
pandad read them. header.frame_id names the bus. CAN FD frames longer than 8 bytes don't fit a can_msgs/Frame, so
can_fd_frames carries every frame as a ros2_socketcan_msgs/FdFrame: the complete stream, in order, for CAN FD cars.
"""

import math
import sys
import threading
import time

import numpy as np
import rclpy
import cereal.messaging as messaging
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile

try:
    from can_msgs.msg import Frame
except ImportError:
    Frame = None
try:
    from ros2_socketcan_msgs.msg import FdFrame
except ImportError:
    FdFrame = None

NODE_NAME = "openpilot_can_frame_publisher"
# pandad adds these to the bus number in src. Sent (CAN_RETURNED_BUS_OFFSET): a frame the panda sends on that bus,
# openpilot's own or one it forwards between buses 0 and 2, reported when the panda queues it for sending.
# Blocked (CAN_REJECTED_BUS_OFFSET): a frame from openpilot that the panda's safety checks kept off the bus.
SENT_OFFSET = 0x80
BLOCKED_OFFSET = 0xC0
# Classic CAN data length; can_msgs/Frame always carries 8 data bytes, dlc says how many are used
CLASSIC_LEN = 8


def frame_id_of(src):
    if src >= BLOCKED_OFFSET:
        return f"can{src - BLOCKED_OFFSET}_blocked"
    if src >= SENT_OFFSET:
        return f"can{src - SENT_OFFSET}_sent"
    return f"can{src}"


class OpenPilotCanFramePublisher(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # Parameters
        self.declare_parameter("can_port", "can")

        # One message per frame, and a 10 ms batch holds the frames of all buses: keep a few batches for slow readers
        qos = QoSProfile(depth=1000)
        self.frame_pub = self.create_publisher(Frame, 'can_frames', qos)
        self.fd_frame_pub = self.create_publisher(FdFrame, 'can_fd_frames', qos) if FdFrame is not None else None
        if self.fd_frame_pub is None:
            self.get_logger().warning(
                "ros2_socketcan_msgs is not installed, so can_fd_frames is not published and CAN FD frames longer "
                "than 8 bytes are left out (sudo apt install ros-$ROS_DISTRO-ros2-socketcan-msgs)"
            )
        # header.frame_id per pandad src, built on first use
        self.frame_ids = {}
        self.last_invalid_warning = -math.inf

        topic_name = self.get_parameter("can_port").value
        self.cereal_thread = threading.Thread(target=self.cereal_loop, args=(topic_name,), daemon=True)
        self.cereal_thread.start()

        self.get_logger().info("OpenPilot CAN frame publisher is ready.")

    def cereal_loop(self, topic_name):
        # Subscribed from the thread that reads: msgq wakes the thread that subscribed when a batch arrives, and any
        # other thread only notices on its next 100 ms poll, so the batches would come in bursts
        sock = messaging.sub_sock(topic_name)
        self.get_logger().info(f"Subscribed to openpilot messaging topic: {topic_name}")
        last_error = -math.inf
        while rclpy.ok():
            # Everything inside the try, so a batch that doesn't convert drops only itself
            try:
                message = messaging.recv_one(sock)
                if message:
                    self.process_can(message)
            except Exception as exc:
                # Publishing fails once Ctrl+C has shut ROS down; that isn't worth an error
                if not rclpy.ok():
                    break
                # At most one line every 5 s, so a stream of bad batches doesn't flood the log
                if time.monotonic() - last_error > 5.0:
                    last_error = time.monotonic()
                    self.get_logger().error(f"Failed to process {topic_name}: {exc}")

    def process_can(self, message):
        # Thousands of frames per second: build messages only for the topics something subscribes to
        publish_frames = self.frame_pub.get_subscription_count() > 0
        publish_fd_frames = self.fd_frame_pub is not None and self.fd_frame_pub.get_subscription_count() > 0
        if not (publish_frames or publish_fd_frames):
            return
        # pandad marks a batch invalid when reading from the panda failed; the frames after the failure are lost
        if not message.valid and time.monotonic() - self.last_invalid_warning > 5.0:
            self.last_invalid_warning = time.monotonic()
            self.get_logger().warning("openpilot lost CAN frames: pandad had an error reading them from the panda")

        # The frames of a batch carry no time of their own, so all get the time the batch arrived
        stamp = self.get_clock().now().to_msg()
        for can_data in message.can:
            dat = can_data.dat
            address = can_data.address
            src = can_data.src
            frame_id = self.frame_ids.get(src)
            if frame_id is None:
                frame_id = self.frame_ids[src] = frame_id_of(src)
            # cereal has no extended flag; pandad sends every ID from 0x800 up as extended, so read them the same way
            is_extended = address >= 0x800

            if publish_frames and len(dat) <= CLASSIC_LEN:
                msg = Frame()
                msg.header.stamp = stamp
                msg.header.frame_id = frame_id
                msg.id = address
                msg.is_extended = is_extended
                msg.dlc = len(dat)
                # A numpy array of exactly 8 bytes; the generated setter doesn't take bytes
                msg.data = np.frombuffer(dat.ljust(CLASSIC_LEN, b"\0"), dtype=np.uint8)
                self.frame_pub.publish(msg)
            if publish_fd_frames:
                msg = FdFrame()
                msg.header.stamp = stamp
                msg.header.frame_id = frame_id
                msg.id = address
                msg.is_extended = is_extended
                msg.len = len(dat)
                msg.data = dat
                self.fd_frame_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    if Frame is None:
        rclpy.logging.get_logger(NODE_NAME).fatal(
            "can_msgs is not installed, so no CAN frames are published. Install it with "
            "sudo apt install ros-$ROS_DISTRO-can-msgs; with the Docker version, use an image built from this "
            "checkout (image_name:=openpilot_ros2_bridge:local image_source:=build)"
        )
        rclpy.try_shutdown()
        return 1
    node = OpenPilotCanFramePublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
