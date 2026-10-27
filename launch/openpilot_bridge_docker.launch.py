from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


_FORWARDED_ARGS = [
    'start_bridge', 'comma_ip', 'bridge_topics',
    'modelv2_port', 'longitudinal_plan_port', 'lateral_plan_port', 'car_state_port', 'car_control_port',
    'start_camera', 'camera_ip', 'camera_port', 'camera_name', 'image_format',
    'namespace', 'start_tf', 'base_frame', 'openpilot_frame', 'device_frame',
    'device_x', 'device_y', 'device_z', 'use_calibration_height', 'base_frame_height',
]


def _launch_args():
    # One argv entry per name:=value for the in-container launch
    args = [[f'{name}:=', LaunchConfiguration(name)] for name in _FORWARDED_ARGS]
    # The Comma side of the bridge is started from the host (it has the SSH keys), never inside the container
    return args + ['start_remote_bridge:=false']


def generate_launch_description():
    package_share = get_package_share_directory('openpilot_ros2_bridge')
    host_runner = f'{package_share}/docker/run_openpilot_bridge_host.sh'

    remote_bridge = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_remote_bridge')),
        cmd=[
            'ssh', '-tt', '-o', 'BatchMode=yes', '-o', 'ServerAliveInterval=5',
            LaunchConfiguration('remote_bridge_target'),
            [
                'if pgrep -x bridge >/dev/null; then echo "cereal bridge already running on the Comma"; exit 0; fi; ',
                LaunchConfiguration('remote_bridge_command'),
            ],
        ],
        output='screen',
    )

    run_container = ExecuteProcess(
        cmd=[
            'bash',
            host_runner,
            LaunchConfiguration('repo_root'),
            LaunchConfiguration('image_name'),
            LaunchConfiguration('image_source'),
            LaunchConfiguration('container_name'),
            LaunchConfiguration('ros_domain_id'),
            LaunchConfiguration('rmw_implementation'),
            LaunchConfiguration('ros_distro'),
            LaunchConfiguration('host_openpilot_root'),
            *_launch_args(),
        ],
        additional_env={
            'COMMA_TARGET': LaunchConfiguration('remote_bridge_target'),
            'COMMA_OPENPILOT_DIR': LaunchConfiguration('comma_openpilot_dir'),
            'PREPARE_OPENPILOT': LaunchConfiguration('prepare_openpilot'),
            'MOUNT_SOURCE': LaunchConfiguration('mount_source'),
            'CYCLONEDDS_URI_VALUE': LaunchConfiguration('cyclonedds_uri'),
        },
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('repo_root', default_value=EnvironmentVariable('OPENPILOT_ROS2_BRIDGE_REPO', default_value='')),
        DeclareLaunchArgument('image_name', default_value='ghcr.io/jkk-research/openpilot_ros2_bridge:latest'),
        DeclareLaunchArgument('image_source', default_value='pull'),
        DeclareLaunchArgument('container_name', default_value='openpilot_ros2_bridge'),
        DeclareLaunchArgument('ros_distro', default_value='jazzy'),
        DeclareLaunchArgument('ros_domain_id', default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='0')),
        DeclareLaunchArgument('rmw_implementation', default_value=EnvironmentVariable('RMW_IMPLEMENTATION', default_value='rmw_fastrtps_cpp')),
        DeclareLaunchArgument('cyclonedds_uri', default_value=EnvironmentVariable('CYCLONEDDS_URI', default_value='')),
        DeclareLaunchArgument('host_openpilot_root', default_value=EnvironmentVariable('OPENPILOT_ROOT', default_value='')),
        DeclareLaunchArgument('prepare_openpilot', default_value='false'),
        DeclareLaunchArgument('mount_source', default_value='false'),
        DeclareLaunchArgument('comma_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('comma_openpilot_dir', default_value='/data/openpilot'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('bridge_topics', default_value='modelV2,carControl,carState,longitudinalPlan,lateralPlan,liveCalibration,extrinsicsCalibration'),
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
        DeclareLaunchArgument('remote_bridge_target', default_value=['comma@', LaunchConfiguration('comma_ip')]),
        DeclareLaunchArgument(
            'remote_bridge_command',
            default_value='cd /data/openpilot/openpilot/cereal/messaging 2>/dev/null || cd /data/openpilot/cereal/messaging && exec ./bridge',
        ),
        DeclareLaunchArgument('modelv2_port', default_value='modelV2'),
        DeclareLaunchArgument('longitudinal_plan_port', default_value='longitudinalPlan'),
        DeclareLaunchArgument('lateral_plan_port', default_value='lateralPlan'),
        DeclareLaunchArgument('car_state_port', default_value='carState'),
        DeclareLaunchArgument('car_control_port', default_value='carControl'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('camera_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('camera_port', default_value='8002'),
        DeclareLaunchArgument('camera_name', default_value='roadCamera'),
        DeclareLaunchArgument('image_format', default_value='jpeg'),
        remote_bridge,
        run_container,
    ])
