#!/usr/bin/env python3

import importlib
import math
import re
import sys
import threading
import time

import rclpy
import cereal.messaging as messaging
from cereal import log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_parser.definition import AbstractGenericString, AbstractSequence, BasicType, NamespacedType

import openpilot_ros2_bridge.msg as op_msgs

# openpilot service -> (ROS message, port parameter). Each is published on a topic named after the service.
SERVICES = [
    ('carState', 'CarState', 'car_state_port'),
    ('carControl', 'CarControl', 'car_control_port'),
    ('carOutput', 'CarOutput', 'car_output_port'),
    ('carParams', 'CarParams', 'car_params_port'),
    ('modelV2', 'ModelV2', 'modelv2_port'),
    ('longitudinalPlan', 'LongitudinalPlan', 'longitudinal_plan_port'),
    ('lateralPlan', 'LateralPlan', 'lateral_plan_port'),
    ('driverStateV2', 'DriverStateV2', 'driver_state_port'),
    ('driverMonitoringState', 'DriverMonitoringState', 'driver_monitoring_state_port'),
    ('extrinsicsCalibration', 'ExtrinsicsCalibration', None),
    ('radarState', 'RadarState', None),
    ('radarTracks', 'RadarData', None),
]
DRIVER_MONITORING = {'driverStateV2', 'driverMonitoringState'}
RADAR = {'radarState', 'radarTracks'}
# Older openpilot versions' names for a service, published on a topic of that name
OLD_NAMES = {'extrinsicsCalibration': 'liveCalibration', 'radarTracks': 'liveTracks'}
# Every top-level message starts with these; they come from the Event, not from the struct
EVENT_FIELDS = {'header', 'log_mono_time', 'valid'}
FLOAT_TYPES = {'float', 'double', 'long double'}
CAPNP_FLOATS = {'float32', 'float64'}
CAPNP_INTS = {'int8', 'int16', 'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64'}


def snake(name):
    # Same conversion as scripts/generate_msgs.py, which named the message fields
    s = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s).lower()


def message_class(namespaced_type):
    module = importlib.import_module('.'.join(namespaced_type.namespaces))
    return getattr(module, namespaced_type.name)


def fill_nan(msg):
    # Every float of a nested message the running openpilot doesn't have, so it reads as missing rather than 0
    for field, slot in zip(msg.get_fields_and_field_types(), msg.SLOT_TYPES):
        if isinstance(slot, BasicType) and slot.typename in FLOAT_TYPES:
            setattr(msg, field, math.nan)
        elif isinstance(slot, NamespacedType):
            fill_nan(getattr(msg, field))
    return msg


def same_layout(slot, proto):
    # Whether an openpilot field still has the layout the message field was generated from; e.g.
    # radarState.radarErrors was a list of enums before openpilot 0.10
    if proto.which() == 'group':
        return isinstance(slot, NamespacedType)
    capnp_type = proto.slot.type
    kind = capnp_type.which()
    if kind == 'list':
        if not isinstance(slot, AbstractSequence):
            return False
        elem = capnp_type.list.elementType.which()
        if elem in ('struct', 'list'):
            return isinstance(slot.value_type, NamespacedType)
        if elem == 'text':
            return isinstance(slot.value_type, AbstractGenericString)
        return isinstance(slot.value_type, BasicType)
    if kind == 'struct':
        return isinstance(slot, NamespacedType)
    if kind == 'text':
        return isinstance(slot, AbstractGenericString)
    return isinstance(slot, BasicType)


def coverage(msg_type, struct_schema):
    # How many of the message's own fields the running openpilot's struct has
    names = {snake(name) for name in struct_schema.fields}
    own = [field for field in msg_type.get_fields_and_field_types() if field not in EVENT_FIELDS]
    return sum(field in names for field in own), len(own)


class CapnpToRos:
    """Fills the generated ROS messages from the openpilot structs they mirror, matching fields by name."""

    def __init__(self):
        self.plans = {}

    def plan(self, msg_type, struct):
        key = (msg_type, struct.schema.node.id)
        if key in self.plans:
            return self.plans[key]
        capnp_fields = struct.schema.fields
        by_snake = {snake(name): name for name in capnp_fields}
        steps = []
        for field, slot in zip(msg_type.get_fields_and_field_types(), msg_type.SLOT_TYPES):
            name = by_snake.get(field)
            if name is not None and not same_layout(slot, capnp_fields[name].proto):
                name = None
            if name is None:
                # Not in this openpilot version, or in another layout: floats read NaN, also inside a nested
                # message; everything else keeps its default
                if isinstance(slot, BasicType) and slot.typename in FLOAT_TYPES:
                    steps.append((field, None, 'nan', None))
                elif isinstance(slot, NamespacedType):
                    steps.append((field, None, 'nan_msg', None))
                continue
            proto = capnp_fields[name].proto
            if proto.which() == 'group':
                steps.append((field, name, 'msg', None))
                continue
            capnp_type = proto.slot.type
            kind = capnp_type.which()
            if kind == 'list':
                elem = capnp_type.list.elementType.which()
                if elem == 'enum':
                    steps.append((field, name, 'enum_list', None))
                elif elem in ('struct', 'list'):
                    elem_class = message_class(slot.value_type) if isinstance(slot, AbstractSequence) else None
                    steps.append((field, name, 'msg_list' if elem == 'struct' else 'list_list', elem_class))
                else:
                    steps.append((field, name, 'list', None))
            elif kind == 'struct':
                steps.append((field, name, 'msg', None))
            elif kind == 'enum':
                steps.append((field, name, 'enum', None))
            elif kind == 'text':
                steps.append((field, name, 'str', None))
            elif kind == 'bool':
                steps.append((field, name, 'bool', None))
            elif kind in CAPNP_FLOATS:
                steps.append((field, name, 'float', None))
            elif kind in CAPNP_INTS:
                steps.append((field, name, 'int', None))
        self.plans[key] = steps
        return steps

    def fill(self, msg, struct):
        for field, name, kind, elem_class in self.plan(type(msg), struct):
            if kind == 'nan':
                setattr(msg, field, math.nan)
                continue
            if kind == 'nan_msg':
                fill_nan(getattr(msg, field))
                continue
            value = getattr(struct, name)
            if kind == 'float':
                setattr(msg, field, float(value))
            elif kind == 'int':
                setattr(msg, field, int(value))
            elif kind == 'bool':
                setattr(msg, field, bool(value))
            elif kind == 'str':
                setattr(msg, field, str(value))
            elif kind == 'enum':
                setattr(msg, field, value.raw)
            elif kind == 'list':
                setattr(msg, field, list(value))
            elif kind == 'enum_list':
                setattr(msg, field, [v.raw for v in value])
            elif kind == 'msg':
                self.fill(getattr(msg, field), value)
            elif kind == 'msg_list':
                setattr(msg, field, [self.fill(elem_class(), v) for v in value])
            elif kind == 'list_list':
                setattr(msg, field, [elem_class(data=list(row)) for row in value])
        return msg


class OpenPilotTypedPublisher(Node):

    def __init__(self):
        super().__init__("openpilot_typed_publisher")

        # Parameters
        for _, _, port in SERVICES:
            if port:
                self.declare_parameter(port, '')
        self.declare_parameter("driver_monitoring", True, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("radar", False, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("openpilot_frame", "openpilot")
        self.openpilot_frame = self.get_parameter("openpilot_frame").value

        self.converter = CapnpToRos()
        self.cereal_threads = []
        for service, msg_name, port in SERVICES:
            if service in DRIVER_MONITORING and not self.enabled("driver_monitoring"):
                continue
            if service in RADAR and not self.enabled("radar"):
                continue
            source = self.get_parameter(port).value if port else ''
            if not source:
                source = service
            if service in OLD_NAMES and source not in messaging.SERVICE_LIST:
                source = OLD_NAMES[service]
            if source not in messaging.SERVICE_LIST or source not in log.Event.schema.fields:
                self.get_logger().info(f"Skipping {service}: this openpilot version has no {source} service")
                continue
            if log.Event.schema.fields[source].proto.slot.type.which() != 'struct':
                # liveTracks before 2024 was a list of another struct
                self.get_logger().info(f"Skipping {service}: this openpilot version's {source} has an older layout")
                continue

            msg_type = getattr(op_msgs, msg_name)
            have, total = coverage(msg_type, log.Event.schema.fields[source].schema)
            if have * 2 < total:
                # An older layout of the struct, e.g. driverMonitoringState before openpilot 0.11.1: publishing
                # it would give a message of defaults that looks like real data
                hint = ' publish_arrays:=true publishes it as driver_monitoring_legacy_state.' if service == 'driverMonitoringState' else ''
                self.get_logger().warning(
                    f"Skipping {service}: this openpilot version's {source} has only {have} of the message's "
                    f"{total} fields, an older layout.{hint}"
                )
                continue
            # carParams only arrives every 50 s, so late subscribers get the last one
            if service == 'carParams':
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            else:
                qos = 10
            publisher = self.create_publisher(msg_type, source if service in OLD_NAMES else service, qos)
            frame_id = self.openpilot_frame if service == 'modelV2' else ''

            sock = messaging.sub_sock(source)
            self.get_logger().info(f"Subscribed to openpilot messaging topic: {source}")
            thread = threading.Thread(
                target=self.cereal_loop, args=(sock, source, msg_type, publisher, frame_id), daemon=True
            )
            thread.start()
            self.cereal_threads.append(thread)

        self.get_logger().info("OpenPilot typed publisher is ready.")

    def enabled(self, parameter):
        return str(self.get_parameter(parameter).value).lower() in ('true', '1')

    def cereal_loop(self, sock, source, msg_type, publisher, frame_id):
        last_error = -math.inf
        while rclpy.ok():
            # Everything inside the try, so a frame that doesn't decode (e.g. from a newer openpilot) drops only itself
            try:
                message = messaging.recv_one(sock)
                if not message or message.which() != source:
                    continue
                msg = self.converter.fill(msg_type(), getattr(message, source))
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = frame_id
                msg.log_mono_time = message.logMonoTime
                msg.valid = message.valid
                publisher.publish(msg)
            except Exception as exc:
                # At most one line per service every 5 s: after a version mismatch every message fails
                if time.monotonic() - last_error > 5.0:
                    last_error = time.monotonic()
                    self.get_logger().error(f"Failed to process {source}: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotTypedPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
