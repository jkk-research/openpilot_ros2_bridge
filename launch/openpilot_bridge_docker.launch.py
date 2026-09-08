from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _launch_args():
    return [
        'start_bridge:=', LaunchConfiguration('start_bridge'),
        ' comma_ip:=', LaunchConfiguration('comma_ip'),
        ' bridge_topics:=', LaunchConfiguration('bridge_topics'),
        ' start_remote_bridge:=', LaunchConfiguration('start_remote_bridge'),
        ' remote_bridge_target:=', LaunchConfiguration('remote_bridge_target'),
        ' remote_bridge_command:=', LaunchConfiguration('remote_bridge_command'),
        ' modelv2_port:=', LaunchConfiguration('modelv2_port'),
        ' longitudinal_plan_port:=', LaunchConfiguration('longitudinal_plan_port'),
        ' lateral_plan_port:=', LaunchConfiguration('lateral_plan_port'),
        ' car_state_port:=', LaunchConfiguration('car_state_port'),
        ' car_control_port:=', LaunchConfiguration('car_control_port'),
        ' start_camera:=', LaunchConfiguration('start_camera'),
        ' camera_ip:=', LaunchConfiguration('camera_ip'),
        ' camera_port:=', LaunchConfiguration('camera_port'),
        ' camera_name:=', LaunchConfiguration('camera_name'),
        ' image_format:=', LaunchConfiguration('image_format'),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('openpilot_ros2_bridge')
    host_runner = f'{package_share}/docker/run_openpilot_bridge_host.sh'

    run_container = ExecuteProcess(
        cmd=[
            host_runner,
            LaunchConfiguration('repo_root'),
            LaunchConfiguration('image_name'),
            LaunchConfiguration('container_name'),
            LaunchConfiguration('ros_domain_id'),
            LaunchConfiguration('rmw_implementation'),
            LaunchConfiguration('ros_distro'),
            LaunchConfiguration('host_openpilot_root'),
            *_launch_args(),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('repo_root', default_value=EnvironmentVariable('OPENPILOT_ROS2_BRIDGE_REPO', default_value='.')),
        DeclareLaunchArgument('image_name', default_value='openpilot_ros2_bridge:local'),
        DeclareLaunchArgument('container_name', default_value='openpilot_ros2_bridge'),
        DeclareLaunchArgument('ros_distro', default_value='jazzy'),
        DeclareLaunchArgument('ros_domain_id', default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='0')),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
        DeclareLaunchArgument('host_openpilot_root', default_value=EnvironmentVariable('OPENPILOT_ROOT', default_value='')),
        DeclareLaunchArgument('comma_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('bridge_topics', default_value='modelV2,carControl,carState,longitudinalPlan,lateralPlan'),
        DeclareLaunchArgument('start_remote_bridge', default_value='false'),
        DeclareLaunchArgument('remote_bridge_target', default_value='comma@127.0.0.1'),
        DeclareLaunchArgument('remote_bridge_command', default_value='cd openpilot/cereal/messaging && ./bridge'),
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
        run_container,
    ])
