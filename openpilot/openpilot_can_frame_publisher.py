#!/usr/bin/env python3
"""Publishes every CAN frame the Comma receives, undecoded, as a standard can_msgs/Frame on can_frames.

The frames come from openpilot's can service: all buses in one stream, in the order pandad read them from the panda.
header.frame_id names the bus. CAN FD frames longer than 8 bytes don't fit a can_msgs/Frame, so they go to
can_fd_frames as ros2_socketcan_msgs/FdFrame instead.
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
# pandad adds these to the bus number in src for frames openpilot sent (CAN_RETURNED_BUS_OFFSET, the panda confirming
# it put the frame on the bus) and for frames the panda's safety blocked (CAN_REJECTED_BUS_OFFSET, never on the bus)
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
                "ros2_socketcan_msgs is not installed, so CAN FD frames longer than 8 bytes are not published "
                "(sudo apt install ros-$ROS_DISTRO-ros2-socketcan-msgs)"
            )
        # header.frame_id per pandad src, built on first use
        self.frame_ids = {}

        topic_name = self.get_parameter("can_port").value
        sock = messaging.sub_sock(topic_name)
        self.get_logger().info(f"Subscribed to openpilot messaging topic: {topic_name}")
        self.cereal_thread = threading.Thread(target=self.cereal_loop, args=(sock, topic_name), daemon=True)
        self.cereal_thread.start()

        self.get_logger().info("OpenPilot CAN frame publisher is ready.")

    def cereal_loop(self, sock, topic_name):
        last_error = -math.inf
        while rclpy.ok():
            # Everything inside the try, so a batch that doesn't convert drops only itself
            try:
                message = messaging.recv_one(sock)
                if message:
                    self.process_can(message)
            except Exception as exc:
                # At most one line every 5 s, so a stream of bad batches doesn't flood the log
                if time.monotonic() - last_error > 5.0:
                    last_error = time.monotonic()
                    self.get_logger().error(f"Failed to process {topic_name}: {exc}")

    def process_can(self, message):
        # Thousands of frames per second: build messages only while something subscribes
        publish_frames = self.frame_pub.get_subscription_count() > 0
        publish_fd_frames = self.fd_frame_pub is not None and self.fd_frame_pub.get_subscription_count() > 0
        if not (publish_frames or publish_fd_frames):
            return

        # The frames of a batch carry no time of their own, so all get the time the batch arrived
        stamp = self.get_clock().now().to_msg()
        for can_data in message.can:
            dat = can_data.dat
            if len(dat) <= CLASSIC_LEN:
                if not publish_frames:
                    continue
                msg = Frame()
                msg.dlc = len(dat)
                # A numpy array of exactly 8 bytes; the generated setter doesn't take bytes
                msg.data = np.frombuffer(dat.ljust(CLASSIC_LEN, b"\0"), dtype=np.uint8)
                publisher = self.frame_pub
            else:
                if not publish_fd_frames:
                    continue
                msg = FdFrame()
                msg.len = len(dat)
                msg.data = dat
                publisher = self.fd_frame_pub

            src = can_data.src
            frame_id = self.frame_ids.get(src)
            if frame_id is None:
                frame_id = self.frame_ids[src] = frame_id_of(src)
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.id = can_data.address
            # cereal has no extended flag; pandad sends every ID from 0x800 up as extended, so read them the same way
            msg.is_extended = can_data.address >= 0x800
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    if Frame is None:
        rclpy.logging.get_logger(NODE_NAME).fatal(
            "can_msgs is not installed (sudo apt install ros-$ROS_DISTRO-can-msgs); not publishing CAN frames"
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
