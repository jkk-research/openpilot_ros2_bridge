#!/usr/bin/env python3

import rclpy
import numpy as np
import cereal.messaging as messaging
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32MultiArray

class OpenPilotPredictionPublisher(Node):

    def __init__(self):
        super().__init__("openpilot_prediction_publisher")

        print("Starting OpenPilot Prediction Publisher Node")

        # Parameters
        self.declare_parameter("modelv2_port", "modelV2")

        modelv2_port = self.get_parameter("modelv2_port").value

        # OpenPilot publishes modelV2 through its messaging bus; subscribe directly.
        self.modelv2_sock = messaging.sub_sock(modelv2_port)

        print(f"Subscribed to OpenPilot messaging topic: {modelv2_port}")

        # Publishers
        self.position_pub = self.create_publisher(Float32MultiArray, 'position', 1)
        self.position_std_pub = self.create_publisher(Float32MultiArray, 'position_std', 1)
        self.orientation_pub = self.create_publisher(Float32MultiArray, 'orientation', 1)
        self.velocity_pub = self.create_publisher(Float32MultiArray, 'velocity', 1)
        self.orientation_rate_pub = self.create_publisher(Float32MultiArray, 'orientation_rate', 1)
        self.acceleration_pub = self.create_publisher(Float32MultiArray, 'acceleration', 1)
        self.lane_lines_pub = self.create_publisher(Float32MultiArray, 'lane_lines', 1)
        self.lane_lines_std_pub = self.create_publisher(Float32MultiArray, 'lane_lines_std', 1)
        self.lane_lines_probs_pub = self.create_publisher(Float32MultiArray, 'lane_line_probs', 1)
        self.road_edges_pub = self.create_publisher(Float32MultiArray, 'road_edges', 1)
        self.road_edges_std_pub = self.create_publisher(Float32MultiArray, 'road_edges_std', 1)
        self.desire_prediction_pub = self.create_publisher(Float32MultiArray, 'desire_prediction', 1)
        self.desire_state_pub = self.create_publisher(Float32MultiArray, 'desire_state', 1)

        self.create_timer(0.01, self.poll_predictions)

        print("OpenPilot Prediction Publisher Node is ready and polling for predictions.")
        
    def publish_array(self, data, publisher):
        data = np.asarray(data, dtype=np.float32)
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

    def stack_xyz_t(self, xyz_t_data, fallback_t=None):
        x = np.asarray(xyz_t_data.x, dtype=np.float32)
        y = np.asarray(xyz_t_data.y, dtype=np.float32)
        z = np.asarray(xyz_t_data.z, dtype=np.float32)

        if len(xyz_t_data.t) > 0:
            t = np.asarray(xyz_t_data.t, dtype=np.float32)
        elif fallback_t is not None and len(fallback_t) > 0:
            t = np.asarray(fallback_t, dtype=np.float32)
        else:
            t = np.arange(len(x), dtype=np.float32)

        n = min(len(x), len(y), len(z), len(t))
        if n == 0:
            return np.zeros((4, 0), dtype=np.float32)

        return np.stack((x[:n], y[:n], z[:n], t[:n]), axis=0)

    def poll_predictions(self):
        message = messaging.recv_one(self.modelv2_sock)
        if not message:
            return

        print("Received modelV2 message, processing predictions...")

        modelV2 = message.modelV2
        fallback_t = modelV2.position.t

        # position
        self.publish_array(self.stack_xyz_t(modelV2.position), self.position_pub)

        # lane lines
        lane_lines = [self.stack_xyz_t(modelV2.laneLines[i], fallback_t=fallback_t) for i in range(4)]
        lane_min_len = min(arr.shape[1] for arr in lane_lines)
        lane_lines_data = np.stack([arr[:, :lane_min_len] for arr in lane_lines], axis=0)
        self.publish_array(lane_lines_data, self.lane_lines_pub)

        # lane line standard deviations
        self.publish_array(modelV2.laneLineStds, self.lane_lines_std_pub)

        # lane line probabilities
        self.publish_array(modelV2.laneLineProbs, self.lane_lines_probs_pub)

        # road edges
        road_edges = [self.stack_xyz_t(modelV2.roadEdges[i], fallback_t=fallback_t) for i in range(2)]
        road_min_len = min(arr.shape[1] for arr in road_edges)
        road_edges_data = np.stack([arr[:, :road_min_len] for arr in road_edges], axis=0)
        self.publish_array(road_edges_data, self.road_edges_pub)

        # road edge standard deviations
        self.publish_array(modelV2.roadEdgeStds, self.road_edges_std_pub)

        # position standard deviation
        self.publish_array([modelV2.position.xStd, modelV2.position.yStd, modelV2.position.zStd, modelV2.position.t], self.position_std_pub)

        # orientation
        self.publish_array(self.stack_xyz_t(modelV2.orientation), self.orientation_pub)

        # velocity
        self.publish_array(self.stack_xyz_t(modelV2.velocity), self.velocity_pub)

        # orientation rate
        self.publish_array(self.stack_xyz_t(modelV2.orientationRate), self.orientation_rate_pub)

        # acceleration
        self.publish_array(self.stack_xyz_t(modelV2.acceleration), self.acceleration_pub)

        # desire prediction
        self.publish_array(modelV2.meta.desirePrediction, self.desire_prediction_pub)

        # desire state
        self.publish_array(modelV2.meta.desireState, self.desire_state_pub)

if __name__ == '__main__':
    rclpy.init()
    node = OpenPilotPredictionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()