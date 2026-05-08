#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

class OpenPilotPredictionVisualizer(Node):

    def __init__(self):
        super().__init__("openpilot_prediction_visualizer")
        qos_profile = QoSProfile(depth=1)

        # Publishers
        self.openpilot_plan_pub = self.create_publisher(MarkerArray, 'plan_markers', qos_profile)
        self.openpilot_lanes_pub = self.create_publisher(MarkerArray, 'lanes_markers', qos_profile)

        # Subscribers
        self.create_subscription(Float32MultiArray, 'position', self.position_callback, qos_profile)
        self.create_subscription(Float32MultiArray, 'lane_lines', self.lane_lines_callback, qos_profile)
        self.create_subscription(Float32MultiArray, 'lane_line_probs', self.lane_lines_probs_callback, qos_profile)

        self.latest_lane_line_probs = np.zeros(4, dtype=np.float32)

    def marker_header(self):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id='openpilot')

    def position_callback(self, msg):
        position = float32_multiarray_to_numpy(msg)
        if position.ndim != 2 or position.shape[0] < 3:
            return

        plan_points = []
        for x, y, z, _ in position.T:
            point_openpilot = Point(x=float(x), y=float(y), z=float(z))
            plan_points.append(point_openpilot)

        plan_marker_array = MarkerArray()

        marker = Marker()
        marker.header = self.marker_header()
        marker.ns = "Openpilot plan"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker.points = plan_points
        plan_marker_array.markers.append(marker)

        self.openpilot_plan_pub.publish(plan_marker_array)

    def lane_lines_probs_callback(self, msg):
        lane_lines_probs = float32_multiarray_to_numpy(msg).flatten()
        if lane_lines_probs.size == 0:
            return
        self.latest_lane_line_probs = lane_lines_probs

    def lane_lines_callback(self, lane_lines_msg):
        lane_lines = float32_multiarray_to_numpy(lane_lines_msg)
        if lane_lines.ndim != 3 or lane_lines.shape[1] < 3:
            return
        lane_lines_probs = self.latest_lane_line_probs

        lanes_marker_array = MarkerArray()

        lane_count = min(4, lane_lines.shape[0])
        for i in range(lane_count):
            lane_points = []
            for x, y, z, _ in lane_lines[i].T:
                point_openpilot = Point(x=float(x), y=float(y), z=float(z))
                lane_points.append(point_openpilot)

            marker = Marker()
            marker.header = self.marker_header()
            marker.ns = "Openpilot lane"
            marker.id = i+1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            prob = float(lane_lines_probs[i]) if i < lane_lines_probs.size else 0.0
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.7, a=1.0)
            marker.points = lane_points
            lanes_marker_array.markers.append(marker)
        
        self.openpilot_lanes_pub.publish(lanes_marker_array)

def float32_multiarray_to_numpy(multiarray):
    dims = tuple(map(lambda x: x.size, multiarray.layout.dim))
    data = multiarray.data[multiarray.layout.data_offset:]
    return np.array(data, dtype=np.float32).reshape(dims)

if __name__ == '__main__':
    rclpy.init()
    node = OpenPilotPredictionVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()