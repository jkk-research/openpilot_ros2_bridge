#!/usr/bin/env python3

import math
import sys
import threading
import time

import rclpy
import numpy as np
import bridge_messaging as messaging
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

from openpilot_ros2_bridge.msg import CanMessage

# Default drivetrain messages decoded from the raw CAN bus, per opendbc brand. A brand's list spans all its DBC
# variants. role: the platform's dbc_dict entry that holds them; bus: the CAN bus they arrive on with a single panda,
# with exceptions in bus_by_dbc and default_can_bus().
DRIVETRAIN_CAN = {
    'byd': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['DRIVE_STATE', 'PEDAL'],
    },
    'chrysler': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['ECM_1', 'ECM_5', 'ESP_8', 'Transmission_Status', 'GEAR', 'EPB_1', 'ACCEL_GAS', 'BRAKE_1'],
    },
    'ford': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['EngVehicleSpThrottle', 'EngBrakeData', 'ABS_BrkBst_Data', 'BrakeSnData_4', 'PowertrainData_10',
            'VehicleOperatingModes', 'DesiredTorqBrk'],
    },
    'gm': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['ECMEngineStatus', 'AcceleratorPedal2', 'ECMAcceleratorPos', 'ECMPRDNL2', 'EBCMRegenPaddle',
            'BECMBatteryVoltageCurrent', 'BCMGeneralPlatformStatus'],
    },
    'honda': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['ENGINE_DATA', 'POWERTRAIN_DATA', 'GAS_PEDAL_2', 'VSA_STATUS', 'GEARBOX_AUTO', 'GEARBOX_CVT',
            'EPB_STATUS', 'GEARBOX_ALT_2'],
    },
    'hyundai': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['TCS13', 'E_EMS11', 'EMS12', 'EMS11', 'EMS_366', 'ESP12', 'ACCELERATOR', 'ACCELERATOR_ALT',
            'GEAR_SHIFTER', 'BRAKE', 'LVR12', 'ELECT_GEAR', 'TCU12', 'CLU15', 'EMS20', 'FCEV_ACCELERATOR', 'GEAR_ALT',
            'GEAR_ALT_2'],
    },
    'mazda': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['ENGINE_DATA', 'GEAR', 'BRAKE', 'EPB'],
    },
    'mg': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['GW_HSC2_HCU_FrP00', 'EHBS_HSC2_FrP00', 'GW_HSC2_ECM_FrP04'],
    },
    'nissan': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['CRUISE_THROTTLE', 'GAS_PEDAL', 'BRAKE_PEDAL', 'GEARBOX'],
    },
    'psa': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['Dyn_CMM', 'Dyn2_FRE', 'Dyn_EasyMove'],
    },
    'rivian': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['VDM_PropStatus', 'iBESP2', 'ESP_Status', 'ESP_Brake_Pressure', 'VDM_Torque_Front',
            'VDM_Torque_Rear', 'ESP_EpbStatus', 'VDM_ModSw_DrvModPosSts_GW'],
    },
    'subaru': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {'subaru_global_2020_hybrid_generated': 1},
        'messages': ['Throttle', 'Transmission', 'Brake_Pedal', 'Throttle_Hybrid'],
    },
    'tesla': {
        'role': 'party',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['DI_systemStatus', 'DI_torque', 'ESP_status', 'IBST_status', 'DI_state'],
    },
    'toyota': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['ENGINE_RPM', 'GAS_PEDAL', 'GAS_PEDAL_HYBRID', 'BRAKE_MODULE', 'BRAKE', 'GEAR_PACKET',
            'GEAR_PACKET_HYBRID', 'CLUTCH'],
    },
    'volkswagen': {
        'role': 'pt',
        'bus': 0,
        'bus_by_dbc': {},
        'messages': ['Motor_14', 'Motor_20', 'ESP_05', 'Getriebe_11', 'Motor_51', 'ESC_50', 'Motor_EV_01', 'Motor_3',
            'Motor_03', 'Getriebe_1'],
    },
}


class OpenPilotDrivetrainPublisher(Node):

    def __init__(self):
        super().__init__("openpilot_drivetrain_publisher")

        # Parameters
        self.declare_parameter("car_state_port", "carState")
        self.declare_parameter("car_output_port", "carOutput")
        self.declare_parameter("car_params_port", "carParams")
        self.declare_parameter("can_port", "can")
        self.declare_parameter("can_dbc", "auto")
        self.declare_parameter("can_bus", -1)
        self.declare_parameter("can_messages", "default")
        self.declare_parameter("publish_arrays", False, ParameterDescriptor(dynamic_typing=True))
        self.publish_arrays = str(self.get_parameter("publish_arrays").value).lower() in ('true', '1')

        # Publishers; carParams only arrives every 50 s, so late subscribers get the last one
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.car_pub = self.create_publisher(String, 'drivetrain_car', latched)
        # Every decoded frame; a 10 ms batch can hold several
        self.can_pub = self.create_publisher(CanMessage, 'can', 50)
        if self.publish_arrays:
            self.state_pub = self.create_publisher(Float32MultiArray, 'drivetrain_state', 1)
            self.command_pub = self.create_publisher(Float32MultiArray, 'drivetrain_command', 1)
            self.params_pub = self.create_publisher(Float32MultiArray, 'drivetrain_params', latched)

        # (CANParser, dbc, bus, {address: (array publisher or None, message name, signal names, layout label)})
        self.can_decoder = None
        self.can_description = "can=off"
        self.can_publishers = {}
        # The car the decoder was built for; a fixed drivetrain_can_dbc is set up once, without carParams
        self.can_setup_key = None
        self.can_setup_lock = threading.Lock()
        if self.get_parameter("can_dbc").value != "auto":
            self.setup_can_decoding(None, "fixed")

        subscriptions = [
            (self.get_parameter("car_params_port").value, self.process_car_params),
            (self.get_parameter("can_port").value, self.process_can),
        ]
        if self.publish_arrays:
            subscriptions += [
                (self.get_parameter("car_state_port").value, self.process_car_state),
                (self.get_parameter("car_output_port").value, self.process_car_output),
            ]

        self.cereal_threads = []
        for topic_name, handler in subscriptions:
            sock = messaging.sub_sock(topic_name)
            self.get_logger().info(f"Subscribed to openpilot messaging topic: {topic_name}")
            thread = threading.Thread(target=self.cereal_loop, args=(sock, topic_name, handler), daemon=True)
            thread.start()
            self.cereal_threads.append(thread)

        self.get_logger().info("OpenPilot drivetrain publisher is ready.")

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

    def to_float(self, value):
        try:
            return float(value)
        except (TypeError, ValueError):
            raw_value = getattr(value, "raw", None)
            if raw_value is not None:
                return float(raw_value)
            return float(int(value))

    def field(self, struct, *names, default=float("nan")):
        # First of `names` present in this openpilot's schema; fields get renamed or dropped between versions
        fieldnames = struct.schema.fieldnames
        for name in names:
            if name in fieldnames:
                return getattr(struct, name)
        return default

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

    def process_car_state(self, message):
        car_state = message.carState

        self.publish_array(
            [
                self.to_float(car_state.gearShifter),
                float(car_state.vEgo),
                float(car_state.aEgo),
                float(car_state.vEgoRaw),
                float(car_state.standstill),
                float(car_state.gasPressed),
                float(car_state.brakePressed),
                float(self.field(car_state, 'regenBraking')),
                float(car_state.parkingBrake),
                float(self.field(car_state, 'brakeHoldActive')),
                float(car_state.cruiseState.enabled),
                float(car_state.cruiseState.available),
                float(self.field(car_state, 'fuelGauge')),
                float(self.field(car_state, 'charging')),
                float(self.field(car_state, 'engineRpm')),
                float(self.field(car_state, 'clutchPressed')),
            ],
            self.state_pub,
        )

    def process_car_output(self, message):
        actuators = message.carOutput.actuatorsOutput

        self.publish_array(
            [
                float(actuators.accel),
                float(actuators.gas),
                float(actuators.brake),
                self.to_float(actuators.longControlState),
                float(actuators.speed),
            ],
            self.command_pub,
        )

    def process_car_params(self, message):
        car_params = message.carParams
        brand = str(self.field(car_params, 'brand', 'carName', default=''))
        fingerprint = str(car_params.carFingerprint)

        if self.publish_arrays:
            self.publish_car_params_array(car_params)
        if self.get_parameter("can_dbc").value == "auto":
            # Rebuilt when the Comma reports another car
            self.setup_can_decoding(car_params, (brand, fingerprint, int(self.field(car_params, 'flags', default=0))))
        self.car_pub.publish(String(data=f"brand={brand} fingerprint={fingerprint} {self.can_description}"))

    def publish_car_params_array(self, car_params):
        self.publish_array(
            [
                self.to_float(car_params.transmissionType),
                float(car_params.mass),
                float(car_params.wheelbase),
                float(car_params.centerToFront),
                float(car_params.steerRatio),
                float(self.field(car_params, 'wheelSpeedFactor')),
                float(car_params.minEnableSpeed),
                float(car_params.openpilotLongitudinalControl),
                float(car_params.pcmCruise),
                float(self.field(car_params, 'stopAccel')),
                float(self.field(car_params, 'longitudinalActuatorDelay', 'longitudinalActuatorDelayUpperBound')),
            ],
            self.params_pub,
        )

    def setup_can_decoding(self, car_params, key):
        with self.can_setup_lock:
            if key == self.can_setup_key:
                return
            self.can_setup_key = key
            self.can_decoder = None
            self.can_description = "can=off"
            try:
                self.can_decoder, self.can_description = self.build_can_decoder(car_params)
            except ImportError as exc:
                self.get_logger().warning(f"Drivetrain CAN decoding needs opendbc's Python CAN parser: {exc}")
            except FileNotFoundError as exc:
                self.get_logger().error(
                    f"Drivetrain CAN decoding disabled, missing DBC: {exc}. On openpilot 0.11.0 and older, "
                    "opendbc's *_generated DBC files only exist after a full openpilot build."
                )
            except Exception as exc:
                self.get_logger().error(f"Drivetrain CAN decoding disabled: {exc}")

    def default_can_bus(self, brand, config, dbc_name, car_params):
        if config is None:
            # Without a known car only the DBC-wide exceptions apply
            return next((c['bus_by_dbc'][dbc_name] for c in DRIVETRAIN_CAN.values() if dbc_name in c['bus_by_dbc']), 0)
        bus = config['bus_by_dbc'].get(dbc_name, config['bus'])
        if car_params is None:
            return bus
        # Same powertrain bus choices as the brands' CanBus classes in opendbc
        flags = int(self.field(car_params, 'flags', default=0))
        if brand == 'honda':
            from opendbc.car.honda.values import HondaFlags
            if flags & HondaFlags.BOSCH and not flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD):
                bus = 1
        elif brand == 'hyundai' and dbc_name == 'hyundai_canfd_generated':
            from opendbc.car.hyundai.values import HyundaiFlags
            # Named CANFD_LKA_STEERING in opendbc before April 2026
            lka_steering = getattr(HyundaiFlags, 'CANFD_LKA_STEER_MSG', None) or HyundaiFlags.CANFD_LKA_STEERING
            bus = 1 if flags & lka_steering else 0
        elif brand == 'nissan' and car_params.carFingerprint == 'NISSAN_ALTIMA':
            bus = 1
        return bus

    def build_can_decoder(self, car_params):
        from opendbc.can.dbc import DBC, SignalType
        from opendbc.can.parser import CANParser

        dbc_name = self.get_parameter("can_dbc").value
        bus = self.get_parameter("can_bus").value
        messages = self.get_parameter("can_messages").value
        brand = str(self.field(car_params, 'brand', 'carName', default='')) if car_params is not None else None
        fingerprint = str(car_params.carFingerprint) if car_params is not None else None
        config = DRIVETRAIN_CAN.get(brand)

        if dbc_name == "auto":
            if config is None:
                return None, f"can=off (no drivetrain defaults for brand {brand!r}; set drivetrain_can_dbc)"
            from opendbc.car.values import PLATFORMS
            platform = PLATFORMS.get(fingerprint)
            if platform is None:
                return None, f"can=off (unknown fingerprint {fingerprint!r}; set drivetrain_can_dbc)"
            # dbc_dict keys are opendbc Bus enums ("pt", "party", ...) in current versions, plain strings before
            dbcs = {getattr(role, "value", role): name for role, name in platform.config.dbc_dict.items()}
            dbc_name = dbcs.get(config['role'])
            if dbc_name is None:
                return None, f"can=off ({fingerprint} has no {config['role']} DBC)"

        if bus < 0:
            bus = self.default_can_bus(brand, config, dbc_name, car_params)

        dbc = DBC(dbc_name)
        if messages == "default":
            # A brand's defaults span all its DBC variants, so keep the ones this DBC defines
            candidates = config['messages'] if config else [m for c in DRIVETRAIN_CAN.values() for m in c['messages']]
            names = [name for name in dict.fromkeys(candidates) if name in dbc.name_to_msg]
        else:
            names = list(dict.fromkeys(name.strip() for name in messages.split(",") if name.strip()))
            unknown = [name for name in names if name not in dbc.name_to_msg]
            if unknown:
                self.get_logger().warning(f"Not in DBC {dbc_name}, skipping: {', '.join(unknown)}")
            names = [name for name in names if name in dbc.name_to_msg]

        topics = {}
        for name in list(names):
            msg = dbc.name_to_msg[name]
            # Checksums and counters only matter to the parser's validity checks
            signals = [sig.name for sig in msg.sigs.values() if sig.type == SignalType.DEFAULT]
            publisher = None
            if self.publish_arrays:
                topic = f"drivetrain_can/{name.lower()}"
                try:
                    if topic not in self.can_publishers:
                        # Depth 10: two frames of a message can arrive in one 10 ms batch
                        self.can_publishers[topic] = self.create_publisher(Float32MultiArray, topic, 10)
                except Exception as exc:
                    self.get_logger().warning(f"Skipping {name}, not usable as a topic name: {exc}")
                    names.remove(name)
                    continue
                publisher = self.can_publishers[topic]
            topics[msg.address] = (publisher, name, signals, ",".join(signals))
        if not names:
            return None, f"can=off (no drivetrain messages in DBC {dbc_name})"

        # NaN frequency: publish whatever arrives, without alive checks
        parser = CANParser(dbc_name, [(name, float("nan")) for name in names], bus)
        description = f"dbc={dbc_name} bus={bus} messages={','.join(names)}"
        self.get_logger().info(f"Decoding drivetrain CAN messages: {description}")
        return (parser, dbc_name, bus, topics), description

    def process_can(self, message):
        decoder = self.can_decoder
        if decoder is None:
            return
        parser, dbc_name, bus, topics = decoder

        frames = [(frame.address, frame.dat, frame.src) for frame in message.can]
        stamp = self.get_clock().now().to_msg()
        for address in parser.update([(message.logMonoTime, frames)]):
            publisher, name, signals, label = topics[address]
            # Every valid frame of this batch, not only the last one
            samples = parser.vl_all[address]
            for i in range(len(next(iter(samples.values()), ()))):
                values = [float(samples[signal][i]) for signal in signals]
                can_msg = CanMessage(dbc=dbc_name, bus=bus, address=address, name=name, signal_names=signals, values=values)
                can_msg.header.stamp = stamp
                can_msg.log_mono_time = message.logMonoTime
                can_msg.valid = message.valid
                self.can_pub.publish(can_msg)
                if publisher is not None:
                    multiarray = Float32MultiArray()
                    # The label names the signals in data order
                    multiarray.layout.dim = [MultiArrayDimension(label=label, size=len(signals), stride=1)]
                    multiarray.data = values
                    publisher.publish(multiarray)

def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotDrivetrainPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
