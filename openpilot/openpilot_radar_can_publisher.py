#!/usr/bin/env python3
"""Decodes the car's radar CAN messages with opendbc and publishes every frame as a CanMessage on radar_can.

By default it decodes the radar messages openpilot itself reads for the car: the DBC, bus and messages of the
CAN parser in opendbc's RadarInterface for the fingerprint in carParams. It also decodes the blind spot messages
the rear corner radars send, the ones openpilot reads for carState.leftBlindspot and rightBlindspot.
"""

import importlib
import math
import sys
import threading
import time

import rclpy
import bridge_messaging as messaging
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from openpilot_ros2_bridge.msg import CanMessage

# Per opendbc brand, the messages its carstate.py reads the blind spot status from, over all its DBC variants.
# Decoded on every bus they arrive on: which one depends on the car and the harness. openpilot only reads them on
# cars it found blind spot monitoring on, but they're decoded whenever the car sends them.
BLIND_SPOT_CAN = {
    'byd': ['BSD_RADAR'],
    'chrysler': ['BSM_1', 'BSM_LEFT', 'BSM_RIGHT'],
    'ford': ['Side_Detect_L_Stat', 'Side_Detect_R_Stat'],
    'gm': ['BCMBlindSpotMonitor'],
    'honda': ['BSM_STATUS_LEFT', 'BSM_STATUS_RIGHT'],
    'hyundai': ['LCA11', 'ADAS_CMD_50_50ms'],
    'mazda': ['BSM'],
    'subaru': ['BSD_RCTA'],
    'tesla': ['DAS_status'],
    'toyota': ['BSM'],
    'volkswagen': ['SWA_01', 'SWA_1', 'MEB_Side_Assist_01'],
}
BLIND_SPOT_BUSES = (0, 1, 2)


def brand_of(car_params):
    return str(car_params.brand if 'brand' in car_params.schema.fieldnames else car_params.carName)


def make_decoder(dbc_name, dbc, names, bus):
    from opendbc.can.dbc import SignalType
    from opendbc.can.parser import CANParser

    topics = {}
    for name in names:
        msg = dbc.name_to_msg[name]
        # Checksums and counters only matter to the parser's validity checks
        topics[msg.address] = (name, [sig.name for sig in msg.sigs.values() if sig.type == SignalType.DEFAULT])
    # NaN frequency: publish whatever arrives, without alive checks
    return CANParser(dbc_name, [(name, float("nan")) for name in names], bus), dbc_name, bus, topics


class OpenPilotRadarCanPublisher(Node):

    def __init__(self):
        super().__init__("openpilot_radar_can_publisher")

        # Parameters
        self.declare_parameter("car_params_port", "carParams")
        self.declare_parameter("can_port", "can")
        self.declare_parameter("can_dbc", "auto")
        self.declare_parameter("can_bus", -1)
        self.declare_parameter("can_messages", "default")
        self.declare_parameter("blind_spot", True, ParameterDescriptor(dynamic_typing=True))
        self.blind_spot = str(self.get_parameter("blind_spot").value).lower() in ('true', '1')

        # Every decoded frame; a 10 ms batch can hold a frame of every radar track
        self.can_pub = self.create_publisher(CanMessage, 'radar_can', 100)

        # (CANParser, dbc, bus, {address: (message name, signal names)})
        self.decoder = None
        self.setup_key = None
        self.setup_lock = threading.Lock()
        # The same for the blind spot messages, one per DBC and bus
        self.blind_spot_decoders = []
        self.blind_spot_key = None
        if self.get_parameter("can_dbc").value != "auto":
            self.setup_decoding(None, "fixed")

        self.cereal_threads = []
        for topic_name, handler in [
            (self.get_parameter("car_params_port").value, self.process_car_params),
            (self.get_parameter("can_port").value, self.process_can),
        ]:
            sock = messaging.sub_sock(topic_name)
            self.get_logger().info(f"Subscribed to openpilot messaging topic: {topic_name}")
            thread = threading.Thread(target=self.cereal_loop, args=(sock, topic_name, handler), daemon=True)
            thread.start()
            self.cereal_threads.append(thread)

        self.get_logger().info("OpenPilot radar CAN publisher is ready.")

    def cereal_loop(self, sock, topic_name, handler):
        last_error = -math.inf
        while rclpy.ok():
            # Everything inside the try, so a frame that doesn't decode drops only itself
            try:
                message = messaging.recv_one(sock)
                if message:
                    handler(message)
            except Exception as exc:
                # At most one line per topic every 5 s, so a stream of bad frames doesn't flood the log
                if time.monotonic() - last_error > 5.0:
                    last_error = time.monotonic()
                    self.get_logger().error(f"Failed to process {topic_name}: {exc}")

    def process_car_params(self, message):
        car_params = message.carParams
        if self.get_parameter("can_dbc").value == "auto":
            # Rebuilt when the Comma reports another car
            fields = car_params.schema.fieldnames
            key = (str(car_params.carFingerprint), int(car_params.flags) if 'flags' in fields else 0,
                   bool(car_params.radarUnavailable))
            self.setup_decoding(car_params, key)
        if self.blind_spot:
            self.setup_blind_spot(car_params)

    def setup_blind_spot(self, car_params):
        brand, fingerprint = brand_of(car_params), str(car_params.carFingerprint)
        with self.setup_lock:
            if (brand, fingerprint) == self.blind_spot_key:
                return
            self.blind_spot_key = (brand, fingerprint)
            self.blind_spot_decoders = []
            try:
                self.blind_spot_decoders, description = self.build_blind_spot_decoders(brand, fingerprint)
                if self.blind_spot_decoders:
                    self.get_logger().info(f"Decoding blind spot CAN messages: {description}")
                else:
                    self.get_logger().info(f"Not decoding blind spot CAN messages: {description}")
            except ImportError as exc:
                self.get_logger().warning(f"Blind spot CAN decoding needs opendbc's Python CAN parser: {exc}")
            except Exception as exc:
                self.get_logger().error(f"Blind spot CAN decoding disabled: {exc}")

    def build_blind_spot_decoders(self, brand, fingerprint):
        from opendbc.can.dbc import DBC
        from opendbc.car.values import PLATFORMS

        candidates = BLIND_SPOT_CAN.get(brand)
        if not candidates:
            return [], f"none known for brand {brand!r}"
        platform = PLATFORMS.get(fingerprint)
        if platform is None:
            return [], f"unknown fingerprint {fingerprint!r}"
        # dbc_dict keys are opendbc Bus enums ("pt", "radar", ...) in current versions, plain strings before; the
        # powertrain DBC first, so a message in several DBCs comes from that one
        dbc_names = sorted(platform.config.dbc_dict.items(), key=lambda kv: getattr(kv[0], "value", kv[0]) != "pt")
        by_dbc = {}
        for name in candidates:
            for _, dbc_name in dbc_names:
                dbc, names = by_dbc.setdefault(dbc_name, (DBC(dbc_name), []))
                if name in dbc.name_to_msg:
                    names.append(name)
                    break
        decoders = [make_decoder(dbc_name, dbc, names, bus)
                    for dbc_name, (dbc, names) in by_dbc.items() if names for bus in BLIND_SPOT_BUSES]
        if not decoders:
            return [], f"{fingerprint}'s DBCs have none of {', '.join(candidates)}"
        found = [f"{','.join(names)} ({dbc_name})" for dbc_name, (_, names) in by_dbc.items() if names]
        return decoders, f"{'; '.join(found)} on buses {','.join(map(str, BLIND_SPOT_BUSES))}"

    def setup_decoding(self, car_params, key):
        with self.setup_lock:
            if key == self.setup_key:
                return
            self.setup_key = key
            self.decoder = None
            try:
                self.decoder, description = self.build_decoder(car_params)
                if self.decoder is None:
                    self.get_logger().warning(f"Not decoding radar CAN messages: {description}")
                else:
                    self.get_logger().info(f"Decoding radar CAN messages: {description}")
            except ImportError as exc:
                self.get_logger().warning(f"Radar CAN decoding needs opendbc's Python CAN parser: {exc}")
            except FileNotFoundError as exc:
                self.get_logger().error(f"Radar CAN decoding disabled, missing DBC: {exc}")
            except Exception as exc:
                self.get_logger().error(f"Radar CAN decoding disabled: {exc}")

    def build_decoder(self, car_params):
        from opendbc.can.dbc import DBC

        dbc_name = self.get_parameter("can_dbc").value
        bus = self.get_parameter("can_bus").value
        messages = self.get_parameter("can_messages").value

        radar_addresses = None
        if dbc_name == "auto":
            fingerprint = str(car_params.carFingerprint)
            if car_params.radarUnavailable:
                return None, f"openpilot reads no radar for {fingerprint} (carParams.radar_unavailable)"
            brand = brand_of(car_params)
            try:
                # Not through opendbc.car.car_helpers, which needs packages the bridge doesn't have
                radar_interface = importlib.import_module(f"opendbc.car.{brand}.radar_interface").RadarInterface
            except ModuleNotFoundError:
                return None, f"opendbc has no radar interface for brand {brand!r}; set radar_can_dbc"
            # The parser openpilot's radard input comes from; the radar interfaces only read plain carParams fields
            radar_parser = radar_interface(car_params).rcp
            if radar_parser is None:
                return None, f"openpilot reads no radar CAN messages for {fingerprint}"
            dbc_name = radar_parser.dbc_name
            radar_addresses = sorted(radar_parser.message_states)
            if bus < 0:
                bus = radar_parser.bus
        elif bus < 0:
            # The radar bus of most cars with a single panda
            bus = 1

        dbc = DBC(dbc_name)
        if messages == "default":
            if radar_addresses is not None:
                names = [dbc.addr_to_msg[address].name for address in radar_addresses]
            else:
                names = [msg.name for msg in sorted(dbc.addr_to_msg.values(), key=lambda m: m.address)]
        else:
            names = list(dict.fromkeys(name.strip() for name in messages.split(",") if name.strip()))
            unknown = [name for name in names if name not in dbc.name_to_msg]
            if unknown:
                self.get_logger().warning(f"Not in DBC {dbc_name}, skipping: {', '.join(unknown)}")
            names = [name for name in names if name in dbc.name_to_msg]
        if not names:
            return None, f"no messages to decode in DBC {dbc_name}"

        description = f"dbc={dbc_name} bus={bus} messages={len(names)} ({names[0]} to {names[-1]})"
        return make_decoder(dbc_name, dbc, names, bus), description

    def process_can(self, message):
        decoders = ([self.decoder] if self.decoder is not None else []) + self.blind_spot_decoders
        if not decoders:
            return
        frames = [(frame.address, frame.dat, frame.src) for frame in message.can]
        stamp = self.get_clock().now().to_msg()
        for decoder in decoders:
            self.publish_decoded(decoder, message, frames, stamp)

    def publish_decoded(self, decoder, message, frames, stamp):
        parser, dbc_name, bus, topics = decoder
        for address in parser.update([(message.logMonoTime, frames)]):
            name, signals = topics[address]
            # Every valid frame of this batch, not only the last one
            samples = parser.vl_all[address]
            for i in range(len(next(iter(samples.values()), ()))):
                can_msg = CanMessage(dbc=dbc_name, bus=bus, address=address, name=name, signal_names=signals,
                                     values=[float(samples[signal][i]) for signal in signals])
                can_msg.header.stamp = stamp
                can_msg.log_mono_time = message.logMonoTime
                can_msg.valid = message.valid
                self.can_pub.publish(can_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotRadarCanPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
