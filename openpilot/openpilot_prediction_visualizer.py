#!/usr/bin/env python3

import sys

import rclpy
import math
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

from openpilot_ros2_bridge.msg import ModelV2

class OpenPilotPredictionVisualizer(Node):

    def __init__(self):
        super().__init__("openpilot_prediction_visualizer")
        qos_profile = QoSProfile(depth=1)

        # Markers are drawn in openpilot's calibrated frame (forward, right, down)
        self.declare_parameter("openpilot_frame", "openpilot")
        self.openpilot_frame = self.get_parameter("openpilot_frame").value

        # Publishers
        self.openpilot_plan_pub = self.create_publisher(MarkerArray, 'plan_markers', qos_profile)
        self.openpilot_lanes_pub = self.create_publisher(MarkerArray, 'lanes_markers', qos_profile)
        self.openpilot_edges_pub = self.create_publisher(MarkerArray, 'road_edges_markers', qos_profile)
        self.openpilot_leads_pub = self.create_publisher(MarkerArray, 'leads_markers', qos_profile)

        # Subscribers
        self.create_subscription(ModelV2, 'modelV2', self.model_callback, qos_profile)

    def marker_header(self):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=self.openpilot_frame)

    def model_callback(self, msg):
        self.publish_plan(msg.position)
        self.publish_lane_lines(msg.lane_lines, msg.lane_line_probs)
        self.publish_road_edges(msg.road_edges)
        self.publish_leads(msg.leads_v3)

    def publish_plan(self, position):
        plan_points = [openpilot_point(x, y, z) for x, y, z in zip(position.x, position.y, position.z)]
        if not plan_points:
            return

        plan_marker_array = MarkerArray()

        marker = Marker()
        marker.header = self.marker_header()
        marker.ns = "Openpilot plan"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        # make plan line wide and green
        marker.scale.x = 2.0
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker.points = plan_points
        plan_marker_array.markers.append(marker)

        # add sampled larger points along the plan for stronger visualization
        num_points = len(plan_points)
        if num_points > 0:
            step = max(1, num_points // 12)
            for idx in range(0, num_points, step):
                p = plan_points[idx]
                sphere = Marker()
                sphere.header = self.marker_header()
                sphere.ns = "Openpilot plan point"
                sphere.id = 1000 + idx
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = 0.25
                sphere.scale.y = 0.25
                sphere.scale.z = 0.25
                sphere.color = ColorRGBA(r=0.0, g=0.8, b=0.0, a=1.0)
                sphere.pose.position = p
                plan_marker_array.markers.append(sphere)

        self.openpilot_plan_pub.publish(plan_marker_array)

    def publish_lane_lines(self, lane_lines, lane_lines_probs):
        if not lane_lines:
            return

        lanes_marker_array = MarkerArray()

        lane_count = min(4, len(lane_lines))
        for i in range(lane_count):
            lane = lane_lines[i]
            lane_points = [openpilot_point(x, y, z) for x, y, z in zip(lane.x, lane.y, lane.z)]
            # only visualize lane if its probability >= 30%
            prob = float(lane_lines_probs[i]) if i < len(lane_lines_probs) else 0.0
            if prob < 0.3:
                continue

            marker = Marker()
            marker.header = self.marker_header()
            marker.ns = "Openpilot lane"
            marker.id = i+1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.12
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.7, a=1.0)
            marker.points = lane_points
            lanes_marker_array.markers.append(marker)
        
        self.openpilot_lanes_pub.publish(lanes_marker_array)

    def publish_road_edges(self, road_edges):
        if not road_edges:
            return

        edges_marker_array = MarkerArray()
        edge_count = min(2, len(road_edges))
        for i in range(edge_count):
            edge = road_edges[i]
            edge_points = [openpilot_point(x, y, z) for x, y, z in zip(edge.x, edge.y, edge.z)]

            # Always visualize road edges regardless of std/probability
            alpha = 1.0

            marker = Marker()
            marker.header = self.marker_header()
            marker.ns = "Openpilot road edge"
            marker.id = i + 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            # make road edges visually stronger
            marker.scale.x = 0.18
            marker.color = ColorRGBA(r=0.0, g=0.45, b=1.0, a=min(1.0, alpha + 0.15))
            marker.points = edge_points
            edges_marker_array.markers.append(marker)

            # add larger spheres along the road edge for emphasis
            edge_len = len(edge_points)
            if edge_len > 0:
                step_e = max(1, edge_len // 8)
                for j in range(0, edge_len, step_e):
                    ep = edge_points[j]
                    edge_sphere = Marker()
                    edge_sphere.header = self.marker_header()
                    edge_sphere.ns = "Openpilot road edge point"
                    edge_sphere.id = 200 + i * 100 + j
                    edge_sphere.type = Marker.SPHERE
                    edge_sphere.action = Marker.ADD
                    edge_sphere.pose.orientation.w = 1.0
                    edge_sphere.scale.x = 0.18
                    edge_sphere.scale.y = 0.18
                    edge_sphere.scale.z = 0.18
                    edge_sphere.color = ColorRGBA(r=0.0, g=0.55, b=1.0, a=min(1.0, alpha + 0.25))
                    edge_sphere.pose.position = ep
                    edges_marker_array.markers.append(edge_sphere)

        self.openpilot_edges_pub.publish(edges_marker_array)

    def publish_leads(self, leads):
        if not leads:
            return

        leads_marker_array = MarkerArray()

        # simple color palette
        colors = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                  ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
                  ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0), ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)]

        for i, lead in enumerate(leads):
            vs = lead.v
            points = [openpilot_point(x, y, 0.0) for x, y in zip(lead.x, lead.y)]

            color = colors[i % len(colors)]

            # thicker line showing predicted trajectory
            line_marker = Marker()
            line_marker.header = self.marker_header()
            line_marker.ns = "Openpilot lead"
            line_marker.id = 10000 + i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.22
            line_marker.color = color
            line_marker.points = points
            leads_marker_array.markers.append(line_marker)

            # larger, semi-transparent sphere at the predicted current position
            if len(points) > 0:
                p0 = points[0]
                sphere_marker = Marker()
                sphere_marker.header = self.marker_header()
                sphere_marker.ns = "Openpilot lead point"
                sphere_marker.id = 11000 + i
                sphere_marker.type = Marker.SPHERE
                sphere_marker.action = Marker.ADD
                sphere_marker.pose.orientation.w = 1.0
                sphere_marker.scale.x = 0.6
                sphere_marker.scale.y = 0.4
                sphere_marker.scale.z = 0.4
                sphere_marker.color = ColorRGBA(r=color.r, g=color.g, b=color.b, a=0.9)
                sphere_marker.pose.position = p0
                leads_marker_array.markers.append(sphere_marker)

                # vehicle footprint cube at the predicted position, oriented along heading
                cube_marker = Marker()
                cube_marker.header = self.marker_header()
                cube_marker.ns = "Openpilot lead cube"
                cube_marker.id = 12000 + i
                cube_marker.type = Marker.CUBE
                cube_marker.action = Marker.ADD
                # default orientation
                cube_marker.pose.orientation.w = 1.0
                # approximate vehicle size (length x width x height)
                length = 4.0
                width = 1.8
                height = 1.5
                cube_marker.scale.x = length
                cube_marker.scale.y = width
                cube_marker.scale.z = height
                cube_marker.color = ColorRGBA(r=color.r * 0.9, g=color.g * 0.9, b=color.b * 0.9, a=0.6)
                # compute orientation from first two predicted points when available
                if len(points) > 1:
                    dx = points[1].x - p0.x
                    dy = points[1].y - p0.y
                    yaw = math.atan2(dy, dx)
                    qz = math.sin(yaw / 2.0)
                    qw = math.cos(yaw / 2.0)
                    cube_marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
                cube_marker.pose.position = p0
                leads_marker_array.markers.append(cube_marker)

                # add rectangular footprint outline (four corners) for clearer vehicle shape
                if len(points) > 1:
                    hx = length / 2.0
                    hy = width / 2.0
                    corners = []
                    for cx, cy in [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy), (hx, hy)]:
                        rx = cx * math.cos(yaw) - cy * math.sin(yaw)
                        ry = cx * math.sin(yaw) + cy * math.cos(yaw)
                        corners.append(Point(x=p0.x + rx, y=p0.y + ry, z=p0.z - 0.02))

                    foot_marker = Marker()
                    foot_marker.header = self.marker_header()
                    foot_marker.ns = "Openpilot lead footprint"
                    foot_marker.id = 12500 + i
                    foot_marker.type = Marker.LINE_STRIP
                    foot_marker.action = Marker.ADD
                    foot_marker.pose.orientation.w = 1.0
                    foot_marker.scale.x = 0.06
                    foot_marker.color = ColorRGBA(r=0.05, g=0.05, b=0.05, a=0.9)
                    foot_marker.points = corners
                    leads_marker_array.markers.append(foot_marker)

                # arrow indicating immediate heading (from first to second predicted point)
                if len(points) > 1:
                    arrow_marker = Marker()
                    arrow_marker.header = self.marker_header()
                    arrow_marker.ns = "Openpilot lead arrow"
                    arrow_marker.id = 13000 + i
                    arrow_marker.type = Marker.ARROW
                    arrow_marker.action = Marker.ADD
                    arrow_marker.pose.orientation.w = 1.0
                    arrow_marker.scale.x = 0.2  # shaft diameter
                    arrow_marker.scale.y = 0.45  # head diameter
                    arrow_marker.scale.z = 0.45  # head length
                    arrow_marker.color = ColorRGBA(r=color.r, g=color.g, b=color.b, a=0.9)
                    arrow_marker.points = [p0, points[1]]
                    leads_marker_array.markers.append(arrow_marker)

                # text label with lead index and first predicted speed (if available)
                v0 = float(vs[0]) if (hasattr(vs, '__len__') and len(vs) > 0) else 0.0
                text_marker = Marker()
                text_marker.header = self.marker_header()
                text_marker.ns = "Openpilot lead text"
                text_marker.id = 14000 + i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.35
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
                text_marker.text = f"Lead {i}  v={v0:.1f} m/s"
                # place label slightly above the sphere
                text_marker.pose.position.x = p0.x
                text_marker.pose.position.y = p0.y
                text_marker.pose.position.z = p0.z - 0.7
                leads_marker_array.markers.append(text_marker)

        self.openpilot_leads_pub.publish(leads_marker_array)


def openpilot_point(x, y, z):
    # ModelV2 carries openpilot's own values, already in its calibrated frame
    return Point(x=float(x), y=float(y), z=float(z))

def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotPredictionVisualizer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)