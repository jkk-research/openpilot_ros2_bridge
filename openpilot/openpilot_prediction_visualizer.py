#!/usr/bin/env python3

import rclpy
import numpy as np
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from lexus_platform.msg import Float32MultiArrayStamped

class OpenPilotPredictionVisualizer(Node):

    def __init__(self):
        super().__init__("openpilot_prediction_visualizer")
        qos_profile = QoSProfile(depth=1)

        # Publishers
        self.openpilot_plan_pub = self.create_publisher(MarkerArray, 'plan_markers', qos_profile)
        self.openpilot_lanes_pub = self.create_publisher(MarkerArray, 'lanes_markers', qos_profile)

        # Subscribers
        self.create_subscription(Float32MultiArrayStamped, 'position', self.position_callback, qos_profile)
        self.lane_lines_sub = message_filters.Subscriber(self, Float32MultiArrayStamped, 'lane_lines', qos_profile=qos_profile)
        self.lane_lines_probs_sub = message_filters.Subscriber(self, Float32MultiArrayStamped, 'lane_line_probs', qos_profile=qos_profile)

        self.ts = message_filters.TimeSynchronizer([self.lane_lines_sub, self.lane_lines_probs_sub], queue_size=2)
        self.ts.registerCallback(self.lane_lines_callback)

    def position_callback(self, msg):
        position = float32_multiarray_to_numpy(msg)

        plan_points = []
        for x, y, z, t in position.T:
            point_openpilot= Point(x=x,y=y,z=z)
            plan_points.append(point_openpilot)

        plan_marker_array = MarkerArray()

        marker = Marker()
        marker.header = msg.header
        marker.ns = "Openpilot plan"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        marker.points = plan_points
        plan_marker_array.markers.append(marker)

        self.openpilot_plan_pub.publish(plan_marker_array)


    def lane_lines_callback(self, lane_lines_msg, lane_lines_probs_msg):
        lane_lines = float32_multiarray_to_numpy(lane_lines_msg)
        lane_lines_probs = float32_multiarray_to_numpy(lane_lines_probs_msg)

        lanes_marker_array = MarkerArray()

        for i in range(4):
            lane_points = []
            for x, y, z, t in lane_lines[i].T:
                point_openpilot = Point(x=x,y=y,z=z)
                lane_points.append(point_openpilot)

            marker = Marker()
            marker.header = lane_lines_msg.header
            marker.ns = "Openpilot lane"
            marker.id = i+1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.color = ColorRGBA(0.0, 1.0, 0.7, lane_lines_probs[i])
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