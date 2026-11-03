#!/usr/bin/env python3

import sys
import rclpy
import numpy as np
import threading
import cereal.messaging as messaging
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32MultiArray

class OpenPilotDriverMonitoringPublisher(Node):

    def __init__(self):
        super().__init__("openpilot_driver_monitoring_publisher")

        # Parameters
        self.declare_parameter("driver_state_port", "driverStateV2")
        self.declare_parameter("driver_monitoring_state_port", "driverMonitoringState")

        # Publishers for the driver monitoring model output (driverStateV2)
        self.driver_state_pub = self.create_publisher(Float32MultiArray, 'driver_state', 1)
        self.face_orientation_pub = self.create_publisher(Float32MultiArray, 'driver_face_orientation', 1)
        self.face_orientation_std_pub = self.create_publisher(Float32MultiArray, 'driver_face_orientation_std', 1)
        self.face_position_pub = self.create_publisher(Float32MultiArray, 'driver_face_position', 1)
        self.face_position_std_pub = self.create_publisher(Float32MultiArray, 'driver_face_position_std', 1)
        self.face_probs_pub = self.create_publisher(Float32MultiArray, 'driver_face_probs', 1)

        # Publishers for the driver monitoring policy (driverMonitoringState)
        self.monitoring_state_pub = self.create_publisher(Float32MultiArray, 'driver_monitoring_state', 1)
        self.monitoring_vision_pub = self.create_publisher(Float32MultiArray, 'driver_monitoring_vision', 1)
        self.monitoring_wheeltouch_pub = self.create_publisher(Float32MultiArray, 'driver_monitoring_wheeltouch', 1)
        self.monitoring_legacy_state_pub = self.create_publisher(Float32MultiArray, 'driver_monitoring_legacy_state', 1)

        subscriptions = [
            (self.get_parameter("driver_state_port").value, self.process_driver_state),
            (self.get_parameter("driver_monitoring_state_port").value, self.process_driver_monitoring_state),
        ]

        self.cereal_threads = []
        for topic_name, handler in subscriptions:
            sock = messaging.sub_sock(topic_name)
            self.get_logger().info(f"Subscribed to openpilot messaging topic: {topic_name}")
            thread = threading.Thread(target=self.cereal_loop, args=(sock, topic_name, handler), daemon=True)
            thread.start()
            self.cereal_threads.append(thread)

        self.get_logger().info("OpenPilot driver monitoring publisher is ready.")

    def publish_array(self, data, publisher):
        data = np.asarray(data, dtype=np.float32)
        if data.size == 0:
            return
        if data.ndim == 0:
            data = data.reshape(1)
        multiarray = Float32MultiArray()
        multiarray.layout.dim = [
            MultiArrayDimension(
                label=f"dim{i}",
                size=data.shape[i],
                stride=int(np.prod(data.shape[i + 1:])) if i < data.ndim - 1 else 1,
            )
            for i in range(data.ndim)
        ]
        multiarray.data = data.flatten().tolist()
        publisher.publish(multiarray)

    def stack_series(self, *series):
        arrays = [np.asarray(values, dtype=np.float32) for values in series]
        if not arrays:
            return None

        min_len = min(array.size for array in arrays)
        if min_len == 0:
            return None

        return np.stack([array[:min_len] for array in arrays], axis=0)

    def to_float(self, value):
        try:
            return float(value)
        except (TypeError, ValueError):
            raw_value = getattr(value, "raw", None)
            if raw_value is not None:
                return float(raw_value)
            return float(int(value))

    def optional_float(self, struct, field):
        # Fields that only exist in some openpilot versions are published as NaN.
        value = getattr(struct, field, None)
        return np.nan if value is None else self.to_float(value)

    def cereal_loop(self, sock, topic_name, handler):
        while rclpy.ok():
            message = messaging.recv_one(sock)
            if message:
                try:
                    handler(message)
                except Exception as exc:
                    self.get_logger().error(f"Failed to process {topic_name}: {exc}")

    def process_driver_state(self, message):
        driver_state = message.driverStateV2
        # Row 0 is the left-hand drive seat, row 1 the right-hand drive seat.
        drivers = (driver_state.leftDriverData, driver_state.rightDriverData)

        self.publish_array(
            [
                float(driver_state.frameId),
                float(driver_state.modelExecutionTime),
                float(driver_state.gpuExecutionTime),
                float(driver_state.wheelOnRightProb),
            ],
            self.driver_state_pub,
        )

        for field, publisher in (
            ("faceOrientation", self.face_orientation_pub),
            ("faceOrientationStd", self.face_orientation_std_pub),
            ("facePosition", self.face_position_pub),
            ("facePositionStd", self.face_position_std_pub),
        ):
            stacked = self.stack_series(*(getattr(driver, field) for driver in drivers))
            if stacked is not None:
                self.publish_array(stacked, publisher)

        self.publish_array(
            [
                [
                    float(driver.faceProb),
                    float(driver.leftEyeProb),
                    float(driver.rightEyeProb),
                    float(driver.leftBlinkProb),
                    float(driver.rightBlinkProb),
                    float(driver.sunglassesProb),
                    self.optional_float(driver, "phoneProb"),
                    self.optional_float(driver, "sleepProb"),
                ]
                for driver in drivers
            ],
            self.face_probs_pub,
        )

    def process_driver_monitoring_state(self, message):
        # Older openpilot versions publish a different driverMonitoringState layout.
        monitoring_state = getattr(message, message.which())
        if not hasattr(monitoring_state, "visionPolicyState"):
            self.process_legacy_driver_monitoring_state(monitoring_state)
            return

        self.publish_array(
            [
                self.to_float(monitoring_state.alertLevel),
                self.to_float(monitoring_state.activePolicy),
                float(monitoring_state.isRHD),
                float(monitoring_state.lockout),
                float(monitoring_state.lockoutCount),
                float(monitoring_state.lockoutMinutesRemaining),
                float(monitoring_state.alert3Count),
                float(monitoring_state.noResponseCount),
                float(monitoring_state.noResponseForceDecel),
                float(monitoring_state.alwaysOn),
                float(monitoring_state.alwaysOnLockout),
                float(monitoring_state.rhdCalibration.calibratedPercent),
                float(monitoring_state.rhdCalibration.offset),
            ],
            self.monitoring_state_pub,
        )

        vision = monitoring_state.visionPolicyState
        self.publish_array(
            [
                float(vision.awarenessPercent),
                float(vision.awarenessStep),
                float(vision.isDistracted),
                float(vision.faceDetected),
                float(vision.distractedTypes.pose),
                float(vision.distractedTypes.eye),
                float(vision.distractedTypes.phone),
                float(vision.distractedTypes.sleep),
                float(vision.pose.pitch),
                float(vision.pose.yaw),
                float(vision.pose.calibrated),
                float(vision.pose.uncertainty),
                float(vision.pose.pitchCalib.calibratedPercent),
                float(vision.pose.pitchCalib.offset),
                float(vision.pose.yawCalib.calibratedPercent),
                float(vision.pose.yawCalib.offset),
                float(vision.wheeltouchFallbackPercent),
                float(vision.uncertainOffroadAlertPercent),
            ],
            self.monitoring_vision_pub,
        )

        wheeltouch = monitoring_state.wheeltouchPolicyState
        self.publish_array(
            [
                float(wheeltouch.awarenessPercent),
                float(wheeltouch.awarenessStep),
                float(wheeltouch.driverInteracting),
            ],
            self.monitoring_wheeltouch_pub,
        )

    def process_legacy_driver_monitoring_state(self, monitoring_state):
        self.publish_array(
            [
                float(monitoring_state.faceDetected),
                float(monitoring_state.isDistracted),
                float(monitoring_state.distractedType),
                float(monitoring_state.awarenessStatus),
                float(monitoring_state.awarenessActive),
                float(monitoring_state.awarenessPassive),
                float(monitoring_state.stepChange),
                float(monitoring_state.posePitchOffset),
                float(monitoring_state.posePitchValidCount),
                float(monitoring_state.poseYawOffset),
                float(monitoring_state.poseYawValidCount),
                float(monitoring_state.isLowStd),
                float(monitoring_state.hiStdCount),
                float(monitoring_state.isActiveMode),
                float(monitoring_state.isRHD),
            ],
            self.monitoring_legacy_state_pub,
        )

def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotDriverMonitoringPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
