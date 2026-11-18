# Generated from openpilot car.capnp:CarParams by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/carParams.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint8 STEER_CONTROL_TYPE_TORQUE=0
uint8 STEER_CONTROL_TYPE_ANGLE=1
uint8 STEER_CONTROL_TYPE_CURVATURE=2
uint8 TRANSMISSION_TYPE_UNKNOWN=0
uint8 TRANSMISSION_TYPE_AUTOMATIC=1
uint8 TRANSMISSION_TYPE_MANUAL=2
uint8 TRANSMISSION_TYPE_DIRECT=3
uint8 TRANSMISSION_TYPE_CVT=4
uint8 FINGERPRINT_SOURCE_CAN=0
uint8 FINGERPRINT_SOURCE_FW=1
uint8 FINGERPRINT_SOURCE_FIXED=2
uint8 NETWORK_LOCATION_FWD_CAMERA=0
uint8 NETWORK_LOCATION_GATEWAY=1

string brand
string car_fingerprint
bool fuzzy_fingerprint
bool not_car
bool pcm_cruise
uint32 flags
bool alpha_longitudinal_available
float32 min_enable_speed
float32 min_steer_speed
bool steer_at_standstill
SafetyConfig[] safety_configs
int16 alternative_experience
float32 max_lateral_accel
bool auto_resume_sng
float32 mass
float32 wheelbase
float32 center_to_front
float32 steer_ratio
float32 steer_ratio_rear
float32 rotational_inertia
float32 tire_stiffness_factor
float32 tire_stiffness_front
float32 tire_stiffness_rear
LongitudinalPIDTuning longitudinal_tuning
float32 steer_limit_timer
uint8 steer_control_type
bool radar_unavailable
float32 stop_accel
float32 steer_actuator_delay
float32 longitudinal_actuator_delay
bool openpilot_longitudinal_control
string car_vin
bool dashcam_only
bool passive
uint8 transmission_type
CarFw[] car_fw
float32 radar_delay
uint8 fingerprint_source
uint8 network_location
float32 wheel_speed_factor
bool sec_oc_required
bool sec_oc_key_available
