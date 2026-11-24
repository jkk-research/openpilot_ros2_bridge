from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def _script_command(script_name, ros_arguments):
    share_dir = get_package_share_directory('openpilot_ros2_bridge')
    script_path = f'{share_dir}/scripts/{script_name}'
    return [
        'bash',
        '-lc',
        [
            'source /opt/ros/',
            LaunchConfiguration('ros_distro'),
            '/setup.bash && ',
            'OPENPILOT_ROOT="',
            LaunchConfiguration('openpilot_root'),
            '" && ',
            'export PYTHONPATH="',
            LaunchConfiguration('bridge_venv'),
            '/lib/python3.12/site-packages:',
            LaunchConfiguration('openpilot_root'),
            ':',
            LaunchConfiguration('openpilot_root'),
            '/openpilot:',
            LaunchConfiguration('openpilot_root'),
            '/opendbc_repo:',
            LaunchConfiguration('openpilot_root'),
            '/msgq_repo:${PYTHONPATH:-}" && ',
            # exec, so launch's SIGINT/SIGTERM reach the node instead of orphaning it under bash
            'exec python3 "',
            script_path,
            '" --ros-args -r __ns:=/',
            LaunchConfiguration('namespace'),
            ' ',
            *ros_arguments,
        ],
    ]


def _camera_publishers(context):
    if not IfCondition(LaunchConfiguration('start_camera')).evaluate(context):
        return []

    cameras = [camera.strip() for camera in LaunchConfiguration('cameras').perform(context).split(',') if camera.strip()]
    return [
        ExecuteProcess(
            cmd=_script_command(
                'openpilot_camera_publisher.py',
                [
                    '-r __node:=openpilot_', camera, '_camera_publisher',
                    ' -p camera:=', camera,
                    ' -p openpilot_ip:=', LaunchConfiguration('camera_ip'),
                    ' -p scale:=', LaunchConfiguration('camera_scale'),
                    ' -p jpeg_quality:=', LaunchConfiguration('camera_jpeg_quality'),
                ],
            ),
            output='screen',
        )
        for camera in cameras
    ]


def _both(first, second):
    # True when both launch arguments are set to true (true/1, any case)
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration(first), "'.lower() in ('true', '1') and '",
        LaunchConfiguration(second), "'.lower() in ('true', '1')",
    ]))


def generate_launch_description():
    # The radar services join the bridged ones with start_radar:=true (liveTracks is radarTracks' older name)
    bridge_topics = PythonExpression([
        "'", LaunchConfiguration('bridge_topics'), "' + (',radarState,radarTracks,liveTracks' if '",
        LaunchConfiguration('start_radar'), "'.lower() in ('true', '1') else '')",
    ])

    local_bridge = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_bridge')),
        cmd=[
            'bash',
            '-lc',
            [
                'OPENPILOT_ROOT="',
                LaunchConfiguration('openpilot_root'),
                '" && ',
                'if [ -d "$OPENPILOT_ROOT/cereal/messaging" ]; then OPENPILOT_CEREAL_DIR="$OPENPILOT_ROOT/cereal/messaging"; elif [ -d "$OPENPILOT_ROOT/openpilot/cereal/messaging" ]; then OPENPILOT_CEREAL_DIR="$OPENPILOT_ROOT/openpilot/cereal/messaging"; else echo "Could not find openpilot cereal/messaging under $OPENPILOT_ROOT" >&2; exit 1; fi && ',
                'cd "$OPENPILOT_CEREAL_DIR" && exec ./bridge "',
                LaunchConfiguration('comma_ip'),
                '" "',
                bridge_topics,
                '"',
            ],
        ],
        output='screen',
    )

    remote_bridge = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_remote_bridge')),
        cmd=[
            'bash',
            '-lc',
            [
                'ssh ',
                LaunchConfiguration('remote_bridge_target'),
                ' \'',
                LaunchConfiguration('remote_bridge_command'),
                '\'',
            ],
        ],
        output='screen',
    )

    typed_publisher = ExecuteProcess(
        cmd=_script_command(
            'openpilot_typed_publisher.py',
            [
                '-p modelv2_port:=', LaunchConfiguration('modelv2_port'),
                ' -p longitudinal_plan_port:=', LaunchConfiguration('longitudinal_plan_port'),
                ' -p lateral_plan_port:=', LaunchConfiguration('lateral_plan_port'),
                ' -p car_state_port:=', LaunchConfiguration('car_state_port'),
                ' -p car_control_port:=', LaunchConfiguration('car_control_port'),
                ' -p car_output_port:=', LaunchConfiguration('car_output_port'),
                ' -p car_params_port:=', LaunchConfiguration('car_params_port'),
                ' -p driver_state_port:=', LaunchConfiguration('driver_state_port'),
                ' -p driver_monitoring_state_port:=', LaunchConfiguration('driver_monitoring_state_port'),
                ' -p driver_monitoring:=', LaunchConfiguration('start_driver_monitoring'),
                ' -p radar:=', LaunchConfiguration('start_radar'),
                ' -p openpilot_frame:=', LaunchConfiguration('openpilot_frame'),
            ],
        ),
        output='screen',
    )

    # The Float32MultiArray topics, only with publish_arrays:=true
    prediction_publisher = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('publish_arrays')),
        cmd=_script_command(
            'openpilot_prediction_publisher.py',
            [
                '-p modelv2_port:=', LaunchConfiguration('modelv2_port'),
                ' -p longitudinal_plan_port:=', LaunchConfiguration('longitudinal_plan_port'),
                ' -p lateral_plan_port:=', LaunchConfiguration('lateral_plan_port'),
                ' -p car_state_port:=', LaunchConfiguration('car_state_port'),
                ' -p car_control_port:=', LaunchConfiguration('car_control_port'),
            ],
        ),
        output='screen',
    )

    prediction_visualizer = ExecuteProcess(
        cmd=_script_command(
            'openpilot_prediction_visualizer.py',
            ['-p openpilot_frame:=', LaunchConfiguration('openpilot_frame')],
        ),
        output='screen',
    )

    tf_publisher = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_tf')),
        cmd=_script_command(
            'openpilot_tf_publisher.py',
            [
                '-p base_frame:=', LaunchConfiguration('base_frame'),
                ' -p openpilot_frame:=', LaunchConfiguration('openpilot_frame'),
                ' -p device_frame:=', LaunchConfiguration('device_frame'),
                ' -p device_x:=', LaunchConfiguration('device_x'),
                ' -p device_y:=', LaunchConfiguration('device_y'),
                ' -p device_z:=', LaunchConfiguration('device_z'),
                ' -p use_calibration_height:=', LaunchConfiguration('use_calibration_height'),
                ' -p base_frame_height:=', LaunchConfiguration('base_frame_height'),
            ],
        ),
        output='screen',
    )

    driver_monitoring_publisher = ExecuteProcess(
        condition=_both('publish_arrays', 'start_driver_monitoring'),
        cmd=_script_command(
            'openpilot_driver_monitoring_publisher.py',
            [
                '-p driver_state_port:=', LaunchConfiguration('driver_state_port'),
                ' -p driver_monitoring_state_port:=', LaunchConfiguration('driver_monitoring_state_port'),
            ],
        ),
        output='screen',
    )

    drivetrain_publisher = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_drivetrain')),
        cmd=_script_command(
            'openpilot_drivetrain_publisher.py',
            [
                '-p car_state_port:=', LaunchConfiguration('car_state_port'),
                ' -p car_output_port:=', LaunchConfiguration('car_output_port'),
                ' -p car_params_port:=', LaunchConfiguration('car_params_port'),
                ' -p can_port:=', LaunchConfiguration('can_port'),
                ' -p can_dbc:=', LaunchConfiguration('drivetrain_can_dbc'),
                ' -p can_bus:=', LaunchConfiguration('drivetrain_can_bus'),
                ' -p can_messages:=', LaunchConfiguration('drivetrain_can_messages'),
                ' -p publish_arrays:=', LaunchConfiguration('publish_arrays'),
            ],
        ),
        output='screen',
    )

    # The car's radar CAN messages, decoded with opendbc, only with start_radar_can:=true
    radar_can_publisher = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_radar_can')),
        cmd=_script_command(
            'openpilot_radar_can_publisher.py',
            [
                '-p car_params_port:=', LaunchConfiguration('car_params_port'),
                ' -p can_port:=', LaunchConfiguration('can_port'),
                ' -p can_dbc:=', LaunchConfiguration('radar_can_dbc'),
                ' -p can_bus:=', LaunchConfiguration('radar_can_bus'),
                ' -p can_messages:=', LaunchConfiguration('radar_can_messages'),
                ' -p blind_spot:=', LaunchConfiguration('radar_can_blind_spot'),
            ],
        ),
        output='screen',
    )

    # Drive commands from ROS 2 to openpilot's joystick mode, only with start_control:=true
    joystick_sender = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_control')),
        cmd=_script_command(
            'openpilot_joystick_sender.py',
            ['-p control_timeout:=', LaunchConfiguration('control_timeout')],
        ),
        output='screen',
    )

    # Its Comma side, over SSH like the remote bridge (the Docker launch starts it from the host instead)
    remote_control = ExecuteProcess(
        condition=_both('start_control', 'start_remote_bridge'),
        cmd=[
            'bash',
            f"{get_package_share_directory('openpilot_ros2_bridge')}/scripts/start_comma_joystick_relay.sh",
            LaunchConfiguration('remote_bridge_target'),
            LaunchConfiguration('comma_openpilot_dir'),
            LaunchConfiguration('control_host_ip'),
        ],
        # ssh ends after about 15 s without the Comma (ServerAliveInterval); connect again once it's back
        respawn=True,
        respawn_delay=5.0,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('ros_distro', default_value='jazzy'),
        DeclareLaunchArgument('openpilot_root', default_value='/workspace/openpilot'),
        DeclareLaunchArgument(
            'bridge_venv',
            default_value=EnvironmentVariable(
                'OPENPILOT_ROS2_BRIDGE_VENV',
                default_value='/workspace/ros2_ws/src/openpilot_ros2_bridge/.venv',
            ),
        ),
        DeclareLaunchArgument('comma_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument(
            'bridge_topics',
            default_value='modelV2,carControl,carState,longitudinalPlan,lateralPlan,liveCalibration,extrinsicsCalibration,driverStateV2,driverMonitoringState,carOutput,carParams,can',
        ),
        DeclareLaunchArgument('namespace', default_value='openpilot'),
        DeclareLaunchArgument('start_tf', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('openpilot_frame', default_value='openpilot'),
        DeclareLaunchArgument('device_frame', default_value='openpilot_device'),
        DeclareLaunchArgument('device_x', default_value='0.0'),
        DeclareLaunchArgument('device_y', default_value='0.0'),
        DeclareLaunchArgument('device_z', default_value='0.0'),
        DeclareLaunchArgument('use_calibration_height', default_value='true'),
        DeclareLaunchArgument('base_frame_height', default_value='0.0'),
        DeclareLaunchArgument('start_remote_bridge', default_value='false'),
        DeclareLaunchArgument('remote_bridge_target', default_value='comma@127.0.0.1'),
        DeclareLaunchArgument(
            'remote_bridge_command',
            default_value='cd /data/openpilot/openpilot/cereal/messaging 2>/dev/null || cd /data/openpilot/cereal/messaging && exec ./bridge',
        ),
        DeclareLaunchArgument('comma_openpilot_dir', default_value='/data/openpilot'),
        DeclareLaunchArgument('start_control', default_value='false'),
        DeclareLaunchArgument('control_timeout', default_value='0.25'),
        DeclareLaunchArgument('control_host_ip', default_value=''),
        DeclareLaunchArgument('modelv2_port', default_value='modelV2'),
        DeclareLaunchArgument('longitudinal_plan_port', default_value='longitudinalPlan'),
        DeclareLaunchArgument('lateral_plan_port', default_value='lateralPlan'),
        DeclareLaunchArgument('car_state_port', default_value='carState'),
        DeclareLaunchArgument('car_control_port', default_value='carControl'),
        DeclareLaunchArgument('publish_arrays', default_value='false'),
        DeclareLaunchArgument('start_driver_monitoring', default_value='true'),
        DeclareLaunchArgument('driver_state_port', default_value='driverStateV2'),
        DeclareLaunchArgument('driver_monitoring_state_port', default_value='driverMonitoringState'),
        DeclareLaunchArgument('start_drivetrain', default_value='true'),
        DeclareLaunchArgument('car_output_port', default_value='carOutput'),
        DeclareLaunchArgument('car_params_port', default_value='carParams'),
        DeclareLaunchArgument('can_port', default_value='can'),
        DeclareLaunchArgument('drivetrain_can_dbc', default_value='auto'),
        DeclareLaunchArgument('drivetrain_can_bus', default_value='-1'),
        DeclareLaunchArgument('drivetrain_can_messages', default_value='default'),
        DeclareLaunchArgument('start_radar', default_value='false'),
        DeclareLaunchArgument('start_radar_can', default_value='false'),
        DeclareLaunchArgument('radar_can_dbc', default_value='auto'),
        DeclareLaunchArgument('radar_can_bus', default_value='-1'),
        DeclareLaunchArgument('radar_can_messages', default_value='default'),
        DeclareLaunchArgument('radar_can_blind_spot', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('cameras', default_value='road'),
        DeclareLaunchArgument('camera_ip', default_value=LaunchConfiguration('comma_ip')),
        DeclareLaunchArgument('camera_scale', default_value='1.0'),
        DeclareLaunchArgument('camera_jpeg_quality', default_value='80'),
        remote_bridge,
        remote_control,
        local_bridge,
        TimerAction(
            period=2.0,
            actions=[
                typed_publisher,
                prediction_publisher,
                prediction_visualizer,
                tf_publisher,
                driver_monitoring_publisher,
                drivetrain_publisher,
                radar_can_publisher,
                joystick_sender,
                OpaqueFunction(function=_camera_publishers),
            ],
        ),
    ])
