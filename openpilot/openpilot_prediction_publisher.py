#!/usr/bin/env python3

import rclpy
import numpy as np
import zmq
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension
from lexus_platform.msg import Float32MultiArrayStamped

from cereal import log_from_bytes

class OpenPilotPredictionPublisher(Node):

    def __init__(self):
        super().__init__("openpilot_prediction_publisher")

        # Parameters
        self.declare_parameter("openpilot_ip", "127.0.0.1")
        self.declare_parameter("modelv2_port", 8009)

        openpilot_ip = self.get_parameter("openpilot_ip").value
        modelv2_port = self.get_parameter("modelv2_port").value

        # Establish connction with Comma
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{openpilot_ip}:{modelv2_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Publishers
        self.position_pub = self.create_publisher(Float32MultiArrayStamped, 'position', 1)
        self.position_std_pub = self.create_publisher(Float32MultiArrayStamped, 'position_std', 1)
        self.orientation_pub = self.create_publisher(Float32MultiArrayStamped, 'orientation', 1)
        self.velocity_pub = self.create_publisher(Float32MultiArrayStamped, 'velocity', 1)
        self.orientation_rate_pub = self.create_publisher(Float32MultiArrayStamped, 'orientation_rate', 1)
        self.acceleration_pub = self.create_publisher(Float32MultiArrayStamped, 'acceleration', 1)
        self.lane_lines_pub = self.create_publisher(Float32MultiArrayStamped, 'lane_lines', 1)
        self.lane_lines_std_pub = self.create_publisher(Float32MultiArrayStamped, 'lane_lines_std', 1)
        self.lane_lines_probs_pub = self.create_publisher(Float32MultiArrayStamped, 'lane_line_probs', 1)
        self.road_edges_pub = self.create_publisher(Float32MultiArrayStamped, 'road_edges', 1)
        self.road_edges_std_pub = self.create_publisher(Float32MultiArrayStamped, 'road_edges_std', 1)
        self.desire_prediction_pub = self.create_publisher(Float32MultiArrayStamped, 'desire_prediction', 1)
        self.desire_state_pub = self.create_publisher(Float32MultiArrayStamped, 'desire_state', 1)

        self.create_timer(0.01, self.poll_predictions)
        
    def publish_array(self, data, timestamp, publisher):
        data = np.asarray(data, dtype=np.float32)
        multiarray = Float32MultiArrayStamped()
        multiarray.header.stamp = timestamp
        multiarray.header.frame_id = "openpilot"
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

    def poll_predictions(self):
        while True:
            try:
                byte_message = self.socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break

            if byte_message is None:
                continue
            
            timestamp = self.get_clock().now().to_msg()
            message = log_from_bytes(byte_message)
            modelV2 = message.modelV2

            # position
            self.publish_array([modelV2.position.x, modelV2.position.y, modelV2.position.z, modelV2.position.t], timestamp, self.position_pub)

            # lane lines
            lane_lines_data = []
            for i in range(4):
                lane_lines_data.append([modelV2.laneLines[i].x, modelV2.laneLines[i].y, modelV2.laneLines[i].z, modelV2.laneLines[i].t])

            self.publish_array(lane_lines_data, timestamp, self.lane_lines_pub)

            # lane line standard deviations
            self.publish_array(modelV2.laneLineStds, timestamp, self.lane_lines_std_pub)

            # lane line probabilities
            self.publish_array(modelV2.laneLineProbs, timestamp, self.lane_lines_probs_pub)

            # road edges
            road_edges_data = []
            for i in range(2):
                road_edges_data.append([modelV2.roadEdges[i].x, modelV2.roadEdges[i].y, modelV2.roadEdges[i].z, modelV2.roadEdges[i].t])

            self.publish_array(road_edges_data, timestamp, self.road_edges_pub)

            # road edge standard deviations
            self.publish_array(modelV2.roadEdgeStds, timestamp, self.road_edges_std_pub)

            # position standard deviation
            self.publish_array([modelV2.position.xStd, modelV2.position.yStd, modelV2.position.zStd, modelV2.position.t], timestamp, self.position_std_pub)

            # orientation
            self.publish_array([modelV2.orientation.x, modelV2.orientation.y, modelV2.orientation.z, modelV2.orientation.t], timestamp, self.orientation_pub)

            # velocity
            self.publish_array([modelV2.velocity.x, modelV2.velocity.y, modelV2.velocity.z, modelV2.velocity.t], timestamp, self.velocity_pub)

            # orientation rate
            self.publish_array([modelV2.orientationRate.x, modelV2.orientationRate.y, modelV2.orientationRate.z, modelV2.orientationRate.t], timestamp, self.orientation_rate_pub)

            # acceleration
            self.publish_array([modelV2.acceleration.x, modelV2.acceleration.y, modelV2.acceleration.z, modelV2.acceleration.t], timestamp, self.acceleration_pub)

            # desire prediction
            self.publish_array(modelV2.meta.desirePrediction, timestamp, self.desire_prediction_pub)

            # desire state
            self.publish_array(modelV2.meta.desireState, timestamp, self.desire_state_pub)

if __name__ == '__main__':
    rclpy.init()
    node = OpenPilotPredictionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()