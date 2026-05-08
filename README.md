# ROS2 bridge for Openpilot
ROS2 bridge for C3/X and C4's  running Openpilot


## Set up Comma device for streaming

1. *[PC running ROS2]* Install and set-up the openpilot enviroment \
Follow the instructions from official repository: https://github.com/commaai/openpilot/tree/master/tools#native-setup-on-ubuntu-2404-and-macos

2. *[Comma3/X or Comma 4]* Once you have a Comma 3 or Comma 4 in a car, ssh in to the device and start Cereal bridge: \
    ```
    cd openpilot/cereal/messaging/
    ./bridge
    ```
3. *[PC running ROS2]* Get back to the Openpilot enviroment on your PC and start the bridge
    ```
    cd openpilot
    source .venv/bin/activate
    cd openpilot/cereal/messaging/
    ./bridge [ip adress of the comma device] "carControl","modelV2"
    ```
    alternatively you can add more services to the bridge

4. Launch the ROS2 scripts

