#!/usr/bin/env python3
"""Sends drive commands from ROS 2 to openpilot's joystick mode on the Comma.

Takes JoystickCommand messages from joystick_command and sends openpilot's testJoystick message at 100 Hz over
ZMQ, on the port openpilot's cereal bridge uses for it. On the Comma, scripts/comma_joystick_relay.py (started by
the launch with start_control:=true) passes it on to joystickd, which turns it into carControl.
"""

import math
import sys
import threading
import time

import rclpy
import zmq
from cereal import log
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from openpilot_ros2_bridge.msg import JoystickCommand

SERVICE = 'testJoystick'
# joystickd sets actuators.accel = 4 * axis 0 and actuators.torque = axis 1, after clipping both axes to -1..1
MAX_ACCEL = 4.0
# Longest control_timeout: the newest command is re-sent until then, which also keeps joystickd's own 0.2 s reset
# from acting, so a dead node's last command must not last long
MAX_TIMEOUT = 0.5


def bridge_port(endpoint):
    # Same as get_port() in openpilot's cereal/messaging/bridge_zmq.cc, so openpilot's own bridge finds it too
    value = 0xcbf29ce484222325
    for byte in endpoint.encode():
        value ^= byte
        value = (value * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return 8023 + value % (65535 - 8023)


def clip(value, limit):
    return max(-limit, min(limit, value))


class OpenPilotJoystickSender(Node):

    def __init__(self):
        super().__init__('openpilot_joystick_sender')

        # Parameters
        self.declare_parameter('control_timeout', 0.25, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('port', 0)
        self.declare_parameter('bind_address', '*')
        timeout = float(self.get_parameter('control_timeout').value)
        self.timeout = min(max(timeout, 0.02), MAX_TIMEOUT)
        if self.timeout != timeout:
            self.get_logger().warning(f"control_timeout must be 0.02 to {MAX_TIMEOUT} s; using {self.timeout} s")
        rate = float(self.get_parameter('rate').value)
        port = int(self.get_parameter('port').value) or bridge_port(SERVICE)
        address = f"tcp://{self.get_parameter('bind_address').value}:{port}"

        self.lock = threading.Lock()
        self.axes = (0.0, 0.0)
        self.received = -math.inf
        self.receiving = False
        self.stop_reported = True  # nothing to report before the first command
        self.last_start_log = -math.inf
        self.last_stop_log = -math.inf
        self.last_rejected = -math.inf

        self.zmq_context = zmq.Context()
        self.sock = self.zmq_context.socket(zmq.PUB)
        self.sock.setsockopt(zmq.LINGER, 0)
        # Only the newest command matters: never queue old ones behind a slow link
        self.sock.setsockopt(zmq.CONFLATE, 1)
        # A small kernel buffer too: after a Wi-Fi stall, it would deliver everything it queued in a burst
        self.sock.setsockopt(zmq.SNDBUF, 4096)
        try:
            self.sock.bind(address)
        except zmq.ZMQError as exc:
            self.get_logger().fatal(f"Can't listen on {address} for the Comma: {exc}")
            raise

        # Best effort matches both reliable and best-effort publishers, e.g. ros2 topic pub
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JoystickCommand, 'joystick_command', self.on_command, qos)
        self.create_timer(1.0 / rate, self.send)

        self.get_logger().info(
            f"Sending {SERVICE} to the Comma from {address}; without a command for {self.timeout} s it gets "
            "accel 0 and steer 0."
        )
        self.get_logger().info("OpenPilot joystick sender is ready.")

    def on_command(self, msg):
        if not (math.isfinite(msg.accel) and math.isfinite(msg.steer)):
            if time.monotonic() - self.last_rejected > 5.0:
                self.last_rejected = time.monotonic()
                self.get_logger().warning(f"Ignoring joystick commands that aren't finite (accel {msg.accel}, steer {msg.steer})")
            return
        with self.lock:
            self.axes = (clip(msg.accel, MAX_ACCEL) / MAX_ACCEL, clip(msg.steer, 1.0))
            self.received = time.monotonic()

    def send(self):
        now = time.monotonic()
        with self.lock:
            age = now - self.received
            fresh = age <= self.timeout
            axes = self.axes if fresh else (0.0, 0.0)
        self.report(fresh, age, now)
        # openpilot 0.9.5 to 0.9.7 read buttons[0] and fail without it; joystickd ignores it
        event = log.Event.new_message(
            logMonoTime=time.monotonic_ns(), valid=True, testJoystick={'axes': list(axes), 'buttons': [False]}
        )
        try:
            self.sock.send(event.to_bytes(), zmq.NOBLOCK)
        except zmq.Again:
            pass

    def report(self, fresh, age, now):
        # Log when commands start and stop, each at most every 5 s, because a publisher slower than control_timeout
        # flips between the two with every message. A stop that lasts 1 s is always logged.
        if fresh:
            if not self.receiving and now - self.last_start_log > 5.0:
                self.last_start_log = now
                self.get_logger().info("Receiving joystick commands")
            self.stop_reported = False
        elif not self.stop_reported and (now - self.last_stop_log > 5.0 or age > 1.0):
            self.stop_reported = True
            self.last_stop_log = now
            self.get_logger().warning(
                f"No joystick command for {self.timeout} s; sending accel 0 and steer 0 until the next one"
            )
        self.receiving = fresh

    def destroy_node(self):
        self.sock.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotJoystickSender()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
