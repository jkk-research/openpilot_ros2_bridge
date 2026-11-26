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
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from cereal_compat import log_from_bytes

# openpilot renamed the camera services over time, so subscribe to every known name.
CAMERA_SERVICES = {
    "road": ("narrowRoadEncodeData", "roadEncodeData"),
    "wide_road": ("wideRoadEncodeData",),
    "driver": ("cabinEncodeData", "driverEncodeData"),
    "q_road": ("qNarrowRoadEncodeData", "qRoadEncodeData"),
}
# The road camera's state, which names its image sensor
ROAD_CAMERA_STATE_SERVICES = ("narrowRoadCameraState", "roadCameraState")

# openpilot's camera intrinsics for the Comma 3, 3X and 4 (DEVICE_CAMERAS in common/transformations/camera.py): the
# size of the full resolution streams (camerad bins OS04C10 frames to half size; that holds from openpilot 0.9.8),
# and the focal length in pixels of the road camera and of the fisheye lens the wide road and driver cameras share.
# openpilot has no distortion model; it treats every camera as a pinhole camera.
SENSOR_CAMERAS = {
    "ar0231/ox03c10": {"size": (1928, 1208), "road": 2648.0, "fisheye": 567.0},
    "os04c10": {"size": (1344, 760), "road": 1522.0 * 3 / 4, "fisheye": 567.0 / 4 * 3},
}
# openpilot's ImageSensor names; unknown is an AR0231 from before openpilot reported the sensor
SENSORS = {"unknown": "ar0231/ox03c10", "ar0231": "ar0231/ox03c10", "ox03c10": "ar0231/ox03c10", "os04c10": "os04c10"}
LENSES = {"road": "road", "q_road": "road", "wide_road": "fisheye", "driver": "fisheye"}

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

        # q_road is a 526x330 copy of the road camera, and that size doesn't tell which sensor took it; the full
        # resolution streams tell it by their size, like openpilot's driver monitoring does
        self.state_socket = None
        self.road_sensor = None
        if self.camera == "q_road":
            self.state_socket = context.socket(zmq.SUB)
            self.state_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            for service in ROAD_CAMERA_STATE_SERVICES:
                self.state_socket.connect(f"tcp://{openpilot_ip}:{zmq_port(service)}")

        self.frame_id = f"openpilot_{self.camera}_camera"
        self.image_pub = self.create_publisher(Image, f"{self.camera}_camera/image_raw", 1)
        self.compressed_pub = self.create_publisher(CompressedImage, f"{self.camera}_camera/image_raw/compressed", 1)
        self.info_pub = self.create_publisher(CameraInfo, f"{self.camera}_camera/camera_info", 1)

        self.decoder = None
        self.jpeg_encoder = None
        self.last_encode_id = None
        self.frame_count = 0
        # ((camera model, width, height), CameraInfo) of the last frame, and what was logged about the camera model
        self.camera_info_cache = None
        self.camera_model_note = None

        self.get_logger().info(
            f"Streaming {self.camera} camera from {openpilot_ip} ({', '.join(CAMERA_SERVICES[self.camera])})"
        )
        self.thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.thread.start()

    def receive_loop(self):
        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)
        if self.state_socket is not None:
            poller.register(self.state_socket, zmq.POLLIN)
        while rclpy.ok():
            ready = dict(poller.poll(100))
            if self.state_socket in ready:
                try:
                    self.process_state(self.state_socket.recv())
                except Exception as exc:
                    self.get_logger().error(f"Failed to process the road camera state: {exc}")
            if self.socket in ready:
                try:
                    self.process_message(self.socket.recv())
                except Exception as exc:
                    self.get_logger().error(f"Failed to process {self.camera} camera frame: {exc}")
                    self.decoder = None

    def process_state(self, byte_message):
        message = log_from_bytes(byte_message)
        # Same field number under both names, so the bundled schema reads the newer name as the older one
        if message.which() not in ROAD_CAMERA_STATE_SERVICES:
            return
        field = getattr(message, message.which()).sensor
        try:
            sensor = str(field)
        except RuntimeError:
            # A sensor newer than the bundled schema has no name here
            sensor = f"unknown sensor #{field.raw}"
        if sensor != self.road_sensor:
            self.road_sensor = sensor
            self.get_logger().info(f"Road camera sensor: {sensor}")

    def camera_model(self, frame_width, frame_height):
        for model, camera in SENSOR_CAMERAS.items():
            if camera["size"] == (frame_width, frame_height):
                return model
        return SENSORS.get(self.road_sensor) if self.camera == "q_road" else None

    def camera_info(self, frame_width, frame_height, width, height):
        model = self.camera_model(frame_width, frame_height)
        if model is None:
            note = (f"No camera_info for the {self.camera} camera yet: its {frame_width}x{frame_height} frames don't "
                    + ("tell the sensor, and the road camera state hasn't named one it knows"
                       if self.camera == "q_road" else "match a stream size the bridge has openpilot's intrinsics for"))
            if note != self.camera_model_note:
                self.camera_model_note = note
                self.get_logger().warning(note)
            return None
        key = (model, width, height)
        if self.camera_info_cache is not None and self.camera_info_cache[0] == key:
            return self.camera_info_cache[1]

        camera = SENSOR_CAMERAS[model]
        full_width, full_height = camera["size"]
        focal_length = camera[LENSES[self.camera]]
        # openpilot's K at the published size: every stream shows the sensor's whole frame, scaled. The principal
        # point is the image center as openpilot puts it, at width / 2 and height / 2.
        fx = focal_length * width / full_width
        fy = focal_length * height / full_height
        cx, cy = width / 2.0, height / 2.0
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_cache = (key, info)

        note = f"{self.camera} camera_info: {model} intrinsics, focal length {fx:.1f} x {fy:.1f} px at {width}x{height}"
        if note != self.camera_model_note:
            self.camera_model_note = note
            self.get_logger().info(note)
        return info

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
        publish_info = self.info_pub.get_subscription_count() > 0
        if not (publish_image or publish_compressed or publish_info):
            return

        width = frame.width
        height = frame.height
        if self.scale != 1.0:
            width = max(2, int(round(width * self.scale / 2)) * 2)
            height = max(2, int(round(height * self.scale / 2)) * 2)

        if publish_info:
            info = self.camera_info(frame.width, frame.height, width, height)
            if info is not None:
                # The same stamp as the images, so image_transport subscribers pair them
                info.header.stamp = stamp
                self.info_pub.publish(info)

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
