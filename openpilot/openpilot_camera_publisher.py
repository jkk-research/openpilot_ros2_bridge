#!/usr/bin/env python3

import array
import sys
import threading
from fractions import Fraction

import av
import rclpy
import zmq
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage, Image
from cereal_compat import log_from_bytes

# openpilot renamed the camera services over time, so subscribe to every known name.
CAMERA_SERVICES = {
    "road": ("narrowRoadEncodeData", "roadEncodeData"),
    "wide_road": ("wideRoadEncodeData",),
    "driver": ("cabinEncodeData", "driverEncodeData"),
    "q_road": ("qNarrowRoadEncodeData", "qRoadEncodeData"),
}

V4L2_BUF_FLAG_KEYFRAME = 8


def zmq_port(service):
    # Same FNV-1a based port mapping as the openpilot cereal messaging bridge.
    hash_value = 0xcbf29ce484222325
    for byte in service.encode():
        hash_value ^= byte
        hash_value = (hash_value * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return 8023 + hash_value % (65535 - 8023)


class OpenPilotCameraPublisher(Node):
    def __init__(self):
        super().__init__("openpilot_camera_publisher")

        # Parameters
        self.declare_parameter("openpilot_ip", "127.0.0.1")
        self.declare_parameter("camera", "road")
        self.declare_parameter("scale", 1.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("jpeg_quality", 80)

        openpilot_ip = self.get_parameter("openpilot_ip").value
        self.camera = self.get_parameter("camera").value
        self.scale = float(self.get_parameter("scale").value)
        jpeg_quality = min(max(int(self.get_parameter("jpeg_quality").value), 1), 100)
        # Map JPEG quality 1-100 to the MJPEG encoder's qscale 31-2.
        self.jpeg_qscale = str(2 + round((100 - jpeg_quality) * 29 / 99))

        if self.camera not in CAMERA_SERVICES:
            raise ValueError(f"Unknown camera '{self.camera}', expected one of {', '.join(CAMERA_SERVICES)}")

        # Connect directly to the cereal bridge running on the Comma device
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        for service in CAMERA_SERVICES[self.camera]:
            self.socket.connect(f"tcp://{openpilot_ip}:{zmq_port(service)}")

        self.frame_id = f"openpilot_{self.camera}_camera"
        self.image_pub = self.create_publisher(Image, f"{self.camera}_camera/image_raw", 1)
        self.compressed_pub = self.create_publisher(CompressedImage, f"{self.camera}_camera/image_raw/compressed", 1)

        self.decoder = None
        self.jpeg_encoder = None
        self.last_encode_id = None
        self.frame_count = 0

        self.get_logger().info(
            f"Streaming {self.camera} camera from {openpilot_ip} ({', '.join(CAMERA_SERVICES[self.camera])})"
        )
        self.thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.thread.start()

    def receive_loop(self):
        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)
        while rclpy.ok():
            if not poller.poll(100):
                continue
            try:
                self.process_message(self.socket.recv())
            except Exception as exc:
                self.get_logger().error(f"Failed to process {self.camera} camera frame: {exc}")
                self.decoder = None

    def process_message(self, byte_message):
        message = log_from_bytes(byte_message)
        # Ports are derived from a hash of the service name, so ignore anything that is not video.
        if not message.which().endswith("EncodeData"):
            return
        encode_data = getattr(message, message.which())
        idx = encode_data.idx
        is_keyframe = bool(idx.flags & V4L2_BUF_FLAG_KEYFRAME)

        # A dropped packet breaks the inter-frame chain; wait for the next keyframe.
        if self.last_encode_id is not None and idx.encodeId != self.last_encode_id + 1:
            self.decoder = None
        self.last_encode_id = idx.encodeId

        if self.decoder is None:
            if not is_keyframe:
                return
            self.decoder = av.CodecContext.create("h264" if str(idx.type).endswith("H264") else "hevc", "r")
            self.decoder.thread_type = "SLICE"
            self.decoder.thread_count = 4
            self.decoder.flags |= av.codec.context.Flags.low_delay

        data = encode_data.data
        if is_keyframe and len(encode_data.header) > 0:
            data = encode_data.header + data

        stamp = self.get_clock().now().to_msg()
        for frame in self.decoder.decode(av.Packet(data)):
            self.publish_frame(frame, stamp)

    def publish_frame(self, frame, stamp):
        publish_image = self.image_pub.get_subscription_count() > 0
        publish_compressed = self.compressed_pub.get_subscription_count() > 0
        if not (publish_image or publish_compressed):
            return

        width = frame.width
        height = frame.height
        if self.scale != 1.0:
            width = max(2, int(round(width * self.scale / 2)) * 2)
            height = max(2, int(round(height * self.scale / 2)) * 2)

        if publish_image:
            bgr = frame.reformat(width=width, height=height, format="bgr24").to_ndarray()
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height = height
            msg.width = width
            msg.encoding = "bgr8"
            msg.step = width * 3
            msg.data = array.array("B", bgr.tobytes())
            self.image_pub.publish(msg)

        if publish_compressed:
            if self.jpeg_encoder is None or (self.jpeg_encoder.width, self.jpeg_encoder.height) != (width, height):
                self.jpeg_encoder = av.CodecContext.create("mjpeg", "w")
                self.jpeg_encoder.width = width
                self.jpeg_encoder.height = height
                self.jpeg_encoder.pix_fmt = "yuvj420p"
                self.jpeg_encoder.time_base = Fraction(1, 20)
                self.jpeg_encoder.options = {"qmin": self.jpeg_qscale, "qmax": self.jpeg_qscale}

            jpeg_frame = frame.reformat(width=width, height=height, format="yuvj420p")
            jpeg_frame.pts = self.frame_count
            self.frame_count += 1
            for packet in self.jpeg_encoder.encode(jpeg_frame):
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                msg.format = "bgr8; jpeg compressed bgr8"
                msg.data = array.array("B", bytes(packet))
                self.compressed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotCameraPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
