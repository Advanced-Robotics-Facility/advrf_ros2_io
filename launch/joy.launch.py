"""Start the joystick driver and bind it to selected-joint control."""
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context):
    parameters = {}
    joints = LaunchConfiguration('joints').perform(context)
    if joints:
        names = yaml.safe_load(joints)
        if not isinstance(names, list) or any(not isinstance(name, str) or not name for name in names):
            raise ValueError('joints must be a YAML list of joint names')
        if names:
            parameters['joints'] = names
    namespace = LaunchConfiguration('namespace')
    return [
        Node(package='joy', executable='joy_node', name='joy_node', namespace=namespace,
             output='screen', parameters=[{
                 'device_id': int(LaunchConfiguration('device_id').perform(context)),
                 'deadzone': 0.0,  # Applied once, by joy_control_node.
                 'autorepeat_rate': 50.0,
                 'sticky_buttons': False,
             }]),
        Node(package='advrf_ros2_io', executable='joy_control_node', namespace=namespace,
             output='screen', parameters=[LaunchConfiguration('config')]),
        Node(package='advrf_ros2_io', executable='joint_control_node', namespace=namespace,
             output='screen', parameters=[LaunchConfiguration('config'), parameters]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Namespace of the existing motor controller'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('advrf_ros2_io'), 'config', 'joint_control.yaml'])),
        DeclareLaunchArgument('joints', default_value='', description='Optional joint filter; otherwise discover from rx/joint_states'),
        DeclareLaunchArgument('device_id', default_value='0', description='SDL joystick device index'),
        OpaqueFunction(function=setup),
    ])
