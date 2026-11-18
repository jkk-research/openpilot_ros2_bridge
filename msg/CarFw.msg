# Generated from openpilot car.capnp:CarParams.CarFw by scripts/generate_msgs.py; do not edit by hand.

uint8 ECU_EPS=0
uint8 ECU_ABS=1
uint8 ECU_FWD_RADAR=2
uint8 ECU_FWD_CAMERA=3
uint8 ECU_ENGINE=4
uint8 ECU_UNKNOWN=5
uint8 ECU_DSU=6
uint8 ECU_PARKING_ADAS=7
uint8 ECU_TRANSMISSION=8
uint8 ECU_SRS=9
uint8 ECU_GATEWAY=10
uint8 ECU_HUD=11
uint8 ECU_COMBINATION_METER=12
uint8 ECU_VSA=13
uint8 ECU_PROGRAMMED_FUEL_INJECTION=14
uint8 ECU_ELECTRIC_BRAKE_BOOSTER=15
uint8 ECU_SHIFT_BY_WIRE=16
uint8 ECU_DEBUG=17
uint8 ECU_HYBRID=18
uint8 ECU_ADAS=19
uint8 ECU_HVAC=20
uint8 ECU_CORNER_RADAR=21
uint8 ECU_EPB=22
uint8 ECU_TELEMATICS=23
uint8 ECU_BODY=24

uint8 ecu
uint32 address
uint8 sub_address
uint32 response_address
string brand
uint8 bus
bool logging
bool obd_multiplexing
