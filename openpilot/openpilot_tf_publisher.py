#!/usr/bin/env python3

import math
import threading

import rclpy
import cereal.messaging as messaging
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import TransformBroadcaster


def quaternion_from_rpy(roll, pitch, yaw):
    # openpilot's rot_from_euler convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class OpenPilotTfPublisher(Node):
    """Broadcasts base_frame -> openpilot (calibrated frame, model outputs) -> openpilot_device from openpilot calibration."""

    def __init__(self):
        super().__init__("openpilot_tf_publisher")

        # Parameters (numeric ones accept ints too, e.g. device_y:=0 from a launch file)
        numeric = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("calibration_port", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("openpilot_frame", "openpilot")
        self.declare_parameter("device_frame", "openpilot_device")
        self.declare_parameter("device_x", 0.0, numeric)
        self.declare_parameter("device_y", 0.0, numeric)
        self.declare_parameter("device_z", 0.0, numeric)
        self.declare_parameter("use_calibration_height", True)
        self.declare_parameter("base_frame_height", 0.0, numeric)
        self.declare_parameter("rate", 20.0, numeric)

        self.base_frame = self.get_parameter("base_frame").value
        self.openpilot_frame = self.get_parameter("openpilot_frame").value
        self.device_frame = self.get_parameter("device_frame").value
        self.device_xyz = [float(self.get_parameter(f"device_{axis}").value) for axis in "xyz"]
        self.use_calibration_height = self.get_parameter("use_calibration_height").value
        self.base_frame_height = float(self.get_parameter("base_frame_height").value)

        self.lock = threading.Lock()
        self.rpy = None
        self.height = None
        self.cal_status = None

        self.broadcaster = TransformBroadcaster(self)

        calibration_port = self.get_parameter("calibration_port").value or self.default_calibration_port()
        sock = messaging.sub_sock(calibration_port)
        self.get_logger().info(f"Subscribed to openpilot messaging topic: {calibration_port}")
        threading.Thread(target=self.cereal_loop, args=(sock,), daemon=True).start()

        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.publish_transforms)
        self.get_logger().info(
            f"Broadcasting {self.base_frame} -> {self.openpilot_frame} -> {self.device_frame}")

    @staticmethod
    def default_calibration_port():
        # some openpilot versions publish calibration as extrinsicsCalibration instead of liveCalibration
        if "extrinsicsCalibration" in messaging.SERVICE_LIST:
            return "extrinsicsCalibration"
        return "liveCalibration"

    def cereal_loop(self, sock):
        while rclpy.ok():
            message = messaging.recv_one(sock)
            if message:
                try:
                    self.process_calibration(getattr(message, message.which()))
                except Exception as exc:
                    self.get_logger().error(f"Failed to process calibration: {exc}")

    def process_calibration(self, calibration):
        rpy = [float(v) for v in calibration.rpyCalib]
        if len(rpy) != 3:
            return

        fieldnames = calibration.schema.fieldnames
        height = list(calibration.height) if "height" in fieldnames else []
        cal_status = str(calibration.calStatus)

        with self.lock:
            self.rpy = rpy
            self.height = float(height[0]) if height else None

        if cal_status != self.cal_status:
            self.cal_status = cal_status
            self.get_logger().info(
                f"Calibration {cal_status}: rpy={[round(v, 4) for v in rpy]} height={self.height}")

    def make_transform(self, stamp, parent, child, translation, rotation):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = translation
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = rotation
        return transform

    def publish_transforms(self):
        with self.lock:
            rpy, height = self.rpy, self.height

        x, y, z = self.device_xyz
        if self.use_calibration_height and height is not None:
            z = height - self.base_frame_height

        stamp = self.get_clock().now().to_msg()
        # openpilot frames are forward-right-down, ROS frames forward-left-up: 180 deg about x
        transforms = [self.make_transform(stamp, self.base_frame, self.openpilot_frame, (x, y, z), (1.0, 0.0, 0.0, 0.0))]

        if rpy is not None:
            # rpyCalib is device_from_calib; the device pose in the calibrated frame is its inverse
            qx, qy, qz, qw = quaternion_from_rpy(*rpy)
            transforms.append(self.make_transform(stamp, self.openpilot_frame, self.device_frame, (0.0, 0.0, 0.0), (-qx, -qy, -qz, qw)))

        self.broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
