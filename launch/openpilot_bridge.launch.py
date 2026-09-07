from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
            'source "',
            LaunchConfiguration('openpilot_root'),
            '/.venv/bin/activate" && ',
            'python3 "',
            script_path,
            '" --ros-args ',
            *ros_arguments,
        ],
    ]


def generate_launch_description():
    bridge_topics = LaunchConfiguration('bridge_topics')

    local_bridge = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_bridge')),
        cmd=[
            'bash',
            '-lc',
            [
                'source "',
                LaunchConfiguration('openpilot_root'),
                '/.venv/bin/activate" && ',
                'cd "',
                LaunchConfiguration('openpilot_root'),
                '/cereal/messaging" && ',
                './bridge "',
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

    prediction_publisher = ExecuteProcess(
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
        cmd=_script_command('openpilot_prediction_visualizer.py', []),
        output='screen',
    )

    camera_publisher = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_camera')),
        cmd=_script_command(
            'openpilot_camera_publisher.py',
            [
                '-p openpilot_ip:=', LaunchConfiguration('camera_ip'),
                ' -p camera_port:=', LaunchConfiguration('camera_port'),
                ' -p camera_name:=', LaunchConfiguration('camera_name'),
                ' -p image_format:=', LaunchConfiguration('image_format'),
            ],
        ),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('ros_distro', default_value='jazzy'),
        DeclareLaunchArgument('openpilot_root', default_value='/workspace/openpilot'),
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
        remote_bridge,
        local_bridge,
        TimerAction(period=2.0, actions=[prediction_publisher, prediction_visualizer, camera_publisher]),
    ])
