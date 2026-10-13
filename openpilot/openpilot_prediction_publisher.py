#!/usr/bin/env python3

import sys
import rclpy
import numpy as np
import threading
import cereal.messaging as messaging
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32MultiArray

class OpenPilotPredictionPublisher(Node):

    def __init__(self):
        super().__init__("openpilot_prediction_publisher")

        # Parameters
        self.declare_parameter("modelv2_port", "modelV2")
        self.declare_parameter("longitudinal_plan_port", "longitudinalPlan")
        self.declare_parameter("lateral_plan_port", "lateralPlan")
        self.declare_parameter("car_state_port", "carState")
        self.declare_parameter("car_control_port", "carControl")

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
        self.road_edges_probs_pub = self.create_publisher(Float32MultiArray, 'road_edges_probs', 1)
        self.leads_v3_pub = self.create_publisher(Float32MultiArray, 'leads_v3', 1)
        self.leads_v3_metadata_pub = self.create_publisher(Float32MultiArray, 'leads_v3_metadata', 1)
        self.desire_prediction_pub = self.create_publisher(Float32MultiArray, 'desire_prediction', 1)
        self.desire_state_pub = self.create_publisher(Float32MultiArray, 'desire_state', 1)
        self.model_meta_state_pub = self.create_publisher(Float32MultiArray, 'model_meta_state', 1)
        self.model_disengage_predictions_pub = self.create_publisher(Float32MultiArray, 'model_disengage_predictions', 1)
        self.temporal_pose_trans_pub = self.create_publisher(Float32MultiArray, 'temporal_pose_trans', 1)
        self.temporal_pose_trans_std_pub = self.create_publisher(Float32MultiArray, 'temporal_pose_trans_std', 1)
        self.temporal_pose_rot_pub = self.create_publisher(Float32MultiArray, 'temporal_pose_rot', 1)
        self.temporal_pose_rot_std_pub = self.create_publisher(Float32MultiArray, 'temporal_pose_rot_std', 1)
        self.desired_curvature_pub = self.create_publisher(Float32MultiArray, 'desired_curvature', 1)
        self.model_confidence_pub = self.create_publisher(Float32MultiArray, 'model_confidence', 1)
        self.longitudinal_plan_pub = self.create_publisher(Float32MultiArray, 'longitudinal_plan', 1)
        self.longitudinal_plan_state_pub = self.create_publisher(Float32MultiArray, 'longitudinal_plan_state', 1)
        self.lateral_plan_pub = self.create_publisher(Float32MultiArray, 'lateral_plan', 1)
        self.lateral_plan_state_pub = self.create_publisher(Float32MultiArray, 'lateral_plan_state', 1)
        self.lateral_plan_solver_x_pub = self.create_publisher(Float32MultiArray, 'lateral_plan_solver_state_x', 1)
        self.lateral_plan_solver_u_pub = self.create_publisher(Float32MultiArray, 'lateral_plan_solver_state_u', 1)
        self.car_state_motion_pub = self.create_publisher(Float32MultiArray, 'car_state_motion', 1)
        self.car_state_wheel_speeds_pub = self.create_publisher(Float32MultiArray, 'car_state_wheel_speeds', 1)
        self.car_state_steering_pub = self.create_publisher(Float32MultiArray, 'car_state_steering', 1)
        self.car_state_cruise_pub = self.create_publisher(Float32MultiArray, 'car_state_cruise', 1)
        self.car_state_flags_pub = self.create_publisher(Float32MultiArray, 'car_state_flags', 1)
        self.car_state_status_pub = self.create_publisher(Float32MultiArray, 'car_state_status', 1)
        self.car_control_actuators_pub = self.create_publisher(Float32MultiArray, 'car_control_actuators', 1)
        self.car_control_orientation_pub = self.create_publisher(Float32MultiArray, 'car_control_orientation', 1)
        self.car_control_hud_pub = self.create_publisher(Float32MultiArray, 'car_control_hud', 1)
        self.car_control_flags_pub = self.create_publisher(Float32MultiArray, 'car_control_flags', 1)

        subscriptions = [
            (self.get_parameter("modelv2_port").value, self.process_model_v2),
            (self.get_parameter("longitudinal_plan_port").value, self.process_longitudinal_plan),
            (self.get_parameter("lateral_plan_port").value, self.process_lateral_plan),
            (self.get_parameter("car_state_port").value, self.process_car_state),
            (self.get_parameter("car_control_port").value, self.process_car_control),
        ]

        self.cereal_threads = []
        for topic_name, handler in subscriptions:
            sock = messaging.sub_sock(topic_name)
            self.get_logger().info(f"Subscribed to openpilot messaging topic: {topic_name}")
            thread = threading.Thread(target=self.cereal_loop, args=(sock, topic_name, handler), daemon=True)
            thread.start()
            self.cereal_threads.append(thread)

        self.get_logger().info("OpenPilot prediction publisher is ready.")
        
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

    def stack_xyz_t(self, xyz_t_data, fallback_t=None):
        x = -np.asarray(xyz_t_data.x, dtype=np.float32)
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

    def cereal_loop(self, sock, topic_name, handler):
        while rclpy.ok():
            message = messaging.recv_one(sock)
            if message:
                try:
                    handler(message)
                except Exception as exc:
                    self.get_logger().error(f"Failed to process {topic_name}: {exc}")

    def process_model_v2(self, message):
        modelV2 = message.modelV2
        fallback_t = modelV2.position.t

        self.publish_array(self.stack_xyz_t(modelV2.position), self.position_pub)

        lane_lines = [self.stack_xyz_t(line, fallback_t=fallback_t) for line in list(modelV2.laneLines)[:4]]
        if lane_lines:
            lane_min_len = min(arr.shape[1] for arr in lane_lines)
            if lane_min_len > 0:
                self.publish_array(np.stack([arr[:, :lane_min_len] for arr in lane_lines], axis=0), self.lane_lines_pub)

        self.publish_array(modelV2.laneLineStds, self.lane_lines_std_pub)
        self.publish_array(modelV2.laneLineProbs, self.lane_lines_probs_pub)

        road_edges = [self.stack_xyz_t(edge, fallback_t=fallback_t) for edge in list(modelV2.roadEdges)[:2]]
        if road_edges:
            road_min_len = min(arr.shape[1] for arr in road_edges)
            if road_min_len > 0:
                self.publish_array(np.stack([arr[:, :road_min_len] for arr in road_edges], axis=0), self.road_edges_pub)

        self.publish_array(modelV2.roadEdgeStds, self.road_edges_std_pub)

        try:
            road_edge_stds = np.asarray(modelV2.roadEdgeStds, dtype=np.float32)
            road_edge_probs = np.clip(1.0 - road_edge_stds, 0.0, 1.0)
            self.publish_array(road_edge_probs, self.road_edges_probs_pub)
        except Exception:
            pass

        try:
            if hasattr(modelV2, 'leadsV3') and len(modelV2.leadsV3) > 0:
                leads_v3 = []
                leads_v3_metadata = []
                for lead in modelV2.leadsV3:
                    x = -np.asarray(lead.x, dtype=np.float32)
                    xStd = np.asarray(lead.xStd, dtype=np.float32)
                    y = np.asarray(lead.y, dtype=np.float32)
                    yStd = np.asarray(lead.yStd, dtype=np.float32)
                    v = np.asarray(lead.v, dtype=np.float32)
                    vStd = np.asarray(lead.vStd, dtype=np.float32)
                    a = np.asarray(lead.a, dtype=np.float32)
                    aStd = np.asarray(lead.aStd, dtype=np.float32)
                    t = np.asarray(lead.t, dtype=np.float32) if len(lead.t) > 0 else np.asarray(fallback_t, dtype=np.float32)

                    n = min(len(x), len(xStd), len(y), len(yStd), len(v), len(vStd), len(a), len(aStd), len(t))
                    if n == 0:
                        arr = np.zeros((9, 0), dtype=np.float32)
                    else:
                        arr = np.stack((x[:n], xStd[:n], y[:n], yStd[:n], v[:n], vStd[:n], a[:n], aStd[:n], t[:n]), axis=0)
                    leads_v3.append(arr)
                    leads_v3_metadata.append([self.to_float(lead.prob), self.to_float(lead.probTime)])

                if len(leads_v3) > 0:
                    lead_min_len = min(arr.shape[1] for arr in leads_v3)
                    if lead_min_len > 0:
                        self.publish_array(np.stack([arr[:, :lead_min_len] for arr in leads_v3], axis=0), self.leads_v3_pub)
                    self.publish_array(np.asarray(leads_v3_metadata, dtype=np.float32), self.leads_v3_metadata_pub)
        except Exception:
            pass

        position_std = self.stack_series(
            modelV2.position.xStd,
            modelV2.position.yStd,
            modelV2.position.zStd,
            modelV2.position.t,
        )
        if position_std is not None:
            self.publish_array(position_std, self.position_std_pub)
        self.publish_array(self.stack_xyz_t(modelV2.orientation), self.orientation_pub)
        self.publish_array(self.stack_xyz_t(modelV2.velocity), self.velocity_pub)
        self.publish_array(self.stack_xyz_t(modelV2.orientationRate), self.orientation_rate_pub)
        self.publish_array(self.stack_xyz_t(modelV2.acceleration), self.acceleration_pub)
        self.publish_array(modelV2.meta.desirePrediction, self.desire_prediction_pub)
        self.publish_array(modelV2.meta.desireState, self.desire_state_pub)

        self.publish_array(
            [
                self.to_float(modelV2.meta.engagedProb),
                float(modelV2.meta.hardBrakePredicted),
                self.to_float(modelV2.meta.laneChangeState),
                self.to_float(modelV2.meta.laneChangeDirection),
            ],
            self.model_meta_state_pub,
        )

        disengage_predictions = self.stack_series(
            modelV2.meta.disengagePredictions.t,
            modelV2.meta.disengagePredictions.brakeDisengageProbs,
            modelV2.meta.disengagePredictions.gasDisengageProbs,
            modelV2.meta.disengagePredictions.steerOverrideProbs,
            modelV2.meta.disengagePredictions.brake3MetersPerSecondSquaredProbs,
            modelV2.meta.disengagePredictions.brake4MetersPerSecondSquaredProbs,
            modelV2.meta.disengagePredictions.brake5MetersPerSecondSquaredProbs,
            modelV2.meta.disengagePredictions.gasPressProbs,
            modelV2.meta.disengagePredictions.brakePressProbs,
        )
        if disengage_predictions is not None:
            self.publish_array(disengage_predictions, self.model_disengage_predictions_pub)

        self.publish_array(modelV2.temporalPose.trans, self.temporal_pose_trans_pub)
        self.publish_array(modelV2.temporalPose.transStd, self.temporal_pose_trans_std_pub)
        self.publish_array(modelV2.temporalPose.rot, self.temporal_pose_rot_pub)
        self.publish_array(modelV2.temporalPose.rotStd, self.temporal_pose_rot_std_pub)
        self.publish_array([self.to_float(modelV2.action.desiredCurvature)], self.desired_curvature_pub)
        self.publish_array([self.to_float(modelV2.confidence)], self.model_confidence_pub)

    def process_longitudinal_plan(self, message):
        plan = message.longitudinalPlan
        longitudinal_plan = self.stack_series(plan.speeds, plan.accels, plan.jerks)
        if longitudinal_plan is not None:
            self.publish_array(longitudinal_plan, self.longitudinal_plan_pub)

        self.publish_array(
            [
                float(plan.hasLead),
                float(plan.fcw),
                self.to_float(plan.longitudinalPlanSource),
                float(plan.processingDelay),
                float(plan.aTarget),
                float(plan.shouldStop),
                float(plan.allowThrottle),
                float(plan.allowBrake),
                float(plan.solverExecutionTime),
            ],
            self.longitudinal_plan_state_pub,
        )

    def process_lateral_plan(self, message):
        plan = message.lateralPlan
        lateral_plan = self.stack_series(plan.dPathPoints, plan.psis, plan.curvatures, plan.curvatureRates)
        if lateral_plan is not None:
            self.publish_array(lateral_plan, self.lateral_plan_pub)

        self.publish_array(
            [
                float(plan.mpcSolutionValid),
                self.to_float(plan.desire),
                self.to_float(plan.laneChangeState),
                self.to_float(plan.laneChangeDirection),
                float(plan.useLaneLines),
                float(plan.solverExecutionTime),
                float(plan.solverCost),
            ],
            self.lateral_plan_state_pub,
        )

        solver_state_x = [np.asarray(values, dtype=np.float32) for values in plan.solverState.x]
        if solver_state_x:
            min_len = min(values.size for values in solver_state_x)
            if min_len > 0:
                self.publish_array(np.stack([values[:min_len] for values in solver_state_x], axis=0), self.lateral_plan_solver_x_pub)
        self.publish_array(plan.solverState.u, self.lateral_plan_solver_u_pub)

    def process_car_state(self, message):
        car_state = message.carState

        self.publish_array(
            [
                float(car_state.vEgo),
                float(car_state.aEgo),
                float(car_state.vEgoRaw),
                float(car_state.vEgoCluster),
                float(car_state.vCruise),
                float(car_state.vCruiseCluster),
                float(car_state.yawRate),
                float(car_state.engineRpm),
                float(car_state.cumLagMs),
                float(car_state.fuelGauge),
            ],
            self.car_state_motion_pub,
        )

        self.publish_array(
            [
                float(car_state.wheelSpeeds.fl),
                float(car_state.wheelSpeeds.fr),
                float(car_state.wheelSpeeds.rl),
                float(car_state.wheelSpeeds.rr),
            ],
            self.car_state_wheel_speeds_pub,
        )

        self.publish_array(
            [
                float(car_state.steeringAngleDeg),
                float(car_state.steeringAngleOffsetDeg),
                float(car_state.steeringRateDeg),
                float(car_state.steeringTorque),
                float(car_state.steeringTorqueEps),
                float(car_state.gas),
                float(car_state.brake),
            ],
            self.car_state_steering_pub,
        )

        self.publish_array(
            [
                float(car_state.cruiseState.enabled),
                float(car_state.cruiseState.speed),
                float(car_state.cruiseState.speedCluster),
                float(car_state.cruiseState.available),
                float(car_state.cruiseState.speedOffset),
                float(car_state.cruiseState.standstill),
                float(car_state.cruiseState.nonAdaptive),
            ],
            self.car_state_cruise_pub,
        )

        self.publish_array(
            [
                float(car_state.canValid),
                float(car_state.canTimeout),
                float(car_state.standstill),
                float(car_state.gasPressed),
                float(car_state.brakePressed),
                float(car_state.regenBraking),
                float(car_state.parkingBrake),
                float(car_state.brakeHoldActive),
                float(car_state.steeringPressed),
                float(car_state.steerFaultTemporary),
                float(car_state.steerFaultPermanent),
                float(car_state.invalidLkasSetting),
                float(car_state.stockAeb),
                float(car_state.stockFcw),
                float(car_state.espDisabled),
                float(car_state.accFaulted),
                float(car_state.carFaultedNonCritical),
                float(car_state.espActive),
                float(car_state.vehicleSensorsInvalid),
                float(car_state.lowSpeedAlert),
                float(car_state.leftBlinker),
                float(car_state.rightBlinker),
                float(car_state.genericToggle),
                float(car_state.doorOpen),
                float(car_state.seatbeltUnlatched),
                float(car_state.clutchPressed),
                float(car_state.leftBlindspot),
                float(car_state.rightBlindspot),
                float(car_state.charging),
            ],
            self.car_state_flags_pub,
        )

        self.publish_array(
            [
                self.to_float(car_state.gearShifter),
                float(car_state.canErrorCounter),
            ],
            self.car_state_status_pub,
        )

    def process_car_control(self, message):
        car_control = message.carControl

        self.publish_array(
            [
                float(car_control.enabled),
                float(car_control.latActive),
                float(car_control.longActive),
                float(car_control.actuators.accel),
                float(car_control.actuators.gas),
                float(car_control.actuators.brake),
                float(car_control.actuators.steer),
                float(car_control.actuators.steeringAngleDeg),
                float(car_control.actuators.curvature),
                float(car_control.actuators.speed),
                float(car_control.actuators.steerOutputCan),
                self.to_float(car_control.actuators.longControlState),
            ],
            self.car_control_actuators_pub,
        )

        orientation_data = self.stack_series(car_control.orientationNED, car_control.angularVelocity)
        if orientation_data is not None:
            self.publish_array(orientation_data, self.car_control_orientation_pub)

        self.publish_array(
            [
                float(car_control.hudControl.speedVisible),
                float(car_control.hudControl.setSpeed),
                float(car_control.hudControl.lanesVisible),
                float(car_control.hudControl.leadVisible),
                self.to_float(car_control.hudControl.visualAlert),
                self.to_float(car_control.hudControl.audibleAlert),
                float(car_control.hudControl.rightLaneVisible),
                float(car_control.hudControl.leftLaneVisible),
                float(car_control.hudControl.rightLaneDepart),
                float(car_control.hudControl.leftLaneDepart),
                float(car_control.hudControl.leadDistanceBars),
            ],
            self.car_control_hud_pub,
        )

        self.publish_array(
            [
                float(car_control.leftBlinker),
                float(car_control.rightBlinker),
                float(car_control.cruiseControl.cancel),
                float(car_control.cruiseControl.resume),
                float(car_control.cruiseControl.override),
            ],
            self.car_control_flags_pub,
        )

def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotPredictionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)