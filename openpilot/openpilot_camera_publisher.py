#!/usr/bin/env python3

import rclpy
import zmq
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from cereal import log_from_bytes

class OpenPilotCameraPublisher(Node):
    def __init__(self):
        super().__init__("openpilot_camera_publisher")

        # Parameters
        self.declare_parameter("openpilot_ip", "127.0.0.1")
        self.declare_parameter("camera_port", 8002)
        self.declare_parameter("camera_name", "roadCamera")
        self.declare_parameter("image_format", "jpeg")

        openpilot_ip = self.get_parameter("openpilot_ip").value
        camera_port = self.get_parameter("camera_port").value
        self.camera_name = self.get_parameter("camera_name").value
        self.image_format = self.get_parameter("image_format").value

        # Establish connction with Comma
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{openpilot_ip}:{camera_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.camera_publisher = self.create_publisher(CompressedImage, "output", 1)
        self.create_timer(0.01, self.poll_socket)

    def poll_socket(self):
        while True:
            try:
                byte_message = self.socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break

            if byte_message is None:
                continue

            timestamp = self.get_clock().now().to_msg()
            message = log_from_bytes(byte_message)
            encode_data = getattr(message, f"{self.camera_name}EncodeData")

            if hasattr(encode_data, 'header'):
                data = encode_data.header + encode_data.data
            else:
                data = encode_data.data

            msg = CompressedImage()
            msg.header.stamp = timestamp
            msg.header.frame_id = f"openpilot_{self.camera_name}"
            msg.format = self.image_format
            msg.data = data

            self.camera_publisher.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = OpenPilotCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()