"""Exercise the IO node against mock ROS endpoints, without motor hardware."""
import os
os.environ['ROS_DOMAIN_ID'] = str(219 + (1 if '--split' in __import__('sys').argv else 0) + (2 if '--discover' in __import__('sys').argv else 0) + (4 if '--speed' in __import__('sys').argv else 0) + (8 if '--triggers' in __import__('sys').argv else 0))
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, Bool
from advrf_ros2_interfaces.msg import JointControlEvent


class JointControlNodeTest(unittest.TestCase):
    def test_event_and_joy_bindings(self):
        with tempfile.TemporaryDirectory() as root:
            os.environ['ROS_LOG_DIR'] = root
            rclpy.init()
            node = rclpy.create_node('mock_controller', namespace='/io_test')
            active = node.create_publisher(Bool, "ecat/active", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            active.publish(Bool(data=True))
            commands = []
            feedback = node.create_publisher(JointState, 'rx/joint_states', qos_profile_sensor_data)
            events = node.create_publisher(JointControlEvent, 'joint_control/events', 10)
            axis_input = node.create_publisher(Float64, 'joint_control/axis', 1)
            joy = node.create_publisher(Joy, 'joy', qos_profile_sensor_data)
            subscription = node.create_subscription(JointState, 'joint_commands', commands.append, 10)

            def spin(duration, fresh=True):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    if fresh:
                        feedback.publish(JointState(name=['a', 'b'], position=[1., 2.]))
                    rclpy.spin_once(node, timeout_sec=0.01)

            def event(value):
                events.publish(JointControlEvent(action=getattr(JointControlEvent, value.upper())))
                spin(0.1)

            def joystick(button=None, axis=0., speed=1.):
                buttons = [0] * 6
                if button is not None:
                    buttons[button] = 1
                axes = [max(0., -axis), max(0., axis)] if '--split' in sys.argv else [0., axis]
                if '--speed' in sys.argv:
                    axes = [-axis, 0., 0., 0., 0., 1. - speed]  # Inverted raw axis 5.
                if '--triggers' in sys.argv:
                    axes = [0., 0., 1. - 2. * max(0., -axis) * speed, 0., 0.,
                            1. - 2. * max(0., axis) * speed]
                joy.publish(Joy(axes=axes, buttons=buttons))
                spin(0.06)

            with open(root + '/node.log', 'w+') as log:
                config = get_package_share_directory('advrf_ros2_io') + '/config/joint_control.yaml'
                overrides = []
                if '--triggers' not in sys.argv:
                    overrides = ['-p', 'position_trigger_axes:=false', '-p', 'position_minus_axis:=-1',
                                 '-p', 'position_plus_axis:=-1']
                if '--speed' in sys.argv:
                    overrides += ['-p', 'position_axis:=0', '-p', 'axis_scale:=-1.0',
                                  '-p', 'speed_axis:=5', '-p', 'speed_axis_inverted:=true']
                if '--split' in sys.argv:
                    overrides += [
                        '-p', 'position_minus_axis:=0',
                        '-p', 'position_plus_axis:=1',
                        '-p', 'speed_axis:=-1',
                        '-p', 'axis_scale:=1.0',
                    ]

                if '--speed' not in sys.argv and '--triggers' not in sys.argv:
                    overrides += [
                        '-p', 'speed_axis:=-1',
                        '-p', 'axis_scale:=1.0',
                        '-p', 'position_axis:=1',
                    ]

                common_arguments = [
                    '--ros-args', '--params-file', config,
                    '-r', '__ns:=/io_test',
                    '-p', 'input_timeout:=0.15',
                ]
                control_arguments = [
                    '-p', 'axis_speed:=0.1',
                    '-p', 'feedback_timeout:=0.2',
                ]
                if '--discover' not in sys.argv:
                    control_arguments += ['-p', 'joints:=[a,b]']

                process = subprocess.Popen(
                    [sys.argv[1]] + common_arguments + control_arguments,
                    stdout=log,
                    stderr=log,
                )
                adapter = subprocess.Popen(
                    [sys.argv[2]] + common_arguments + overrides,
                    stdout=log,
                    stderr=log,
                )
                try:
                    deadline = time.monotonic() + 5
                    while (events.get_subscription_count() == 0 or joy.get_subscription_count() == 0) and time.monotonic() < deadline:
                        spin(0.05, fresh='--discover' not in sys.argv)
                    self.assertGreater(events.get_subscription_count(), 0)
                    if '--discover' in sys.argv:
                        events.publish(JointControlEvent(action=3))
                        spin(0.05, fresh=False)
                        events.publish(JointControlEvent(action=JointControlEvent.POSITION_PLUS))
                        spin(0.05, fresh=False)
                        for invalid in [JointState(), JointState(name=[''], position=[1.]),
                                        JointState(name=['x', 'x'], position=[1., 2.]),
                                        JointState(name=['x'], position=[float('nan')]),
                                        JointState(name=['x'])]:
                            feedback.publish(invalid)
                            spin(0.03, fresh=False)
                        self.assertEqual(commands, [])
                    spin(0.2)
                    self.assertEqual(commands, [])
                    # The adapter sends neutral when it discovers bridge state.
                    # Reset arming explicitly before checking the generic input.
                    event('sync')
                    # The generic interface works without joystick messages.
                    events.publish(JointControlEvent(action=255))
                    axis_input.publish(Float64(data=1.0))
                    spin(0.06)
                    self.assertEqual(commands, [], 'Unarmed axis and invalid events must be ignored')
                    axis_input.publish(Float64(data=0.0))
                    spin(0.04)
                    axis_input.publish(Float64(data=0.5))
                    spin(0.06)
                    self.assertGreater(len(commands), 0)
                    axis_input.publish(Float64(data=float('nan')))
                    spin(0.04)
                    count = len(commands)
                    axis_input.publish(Float64(data=1.0))
                    spin(0.06)
                    self.assertEqual(len(commands), count, 'Invalid input must disarm the axis')
                    event('sync')
                    event('position_plus')
                    self.assertEqual(commands[-1].name, ['a'])
                    self.assertAlmostEqual(commands[-1].position[0], 1.01)
                    event('next')
                    event('position_minus')
                    self.assertEqual(commands[-1].name, ['b'])
                    self.assertAlmostEqual(commands[-1].position[0], 1.99)
                    # Removed power action values must not publish commands.
                    count = len(commands)
                    for action in (3, 4):
                        events.publish(JointControlEvent(action=action))
                        spin(0.06)
                    self.assertEqual(len(commands), count)
                    # First sample seeds held buttons; only new presses generate events.
                    joystick(5)
                    joystick()
                    joystick(5)
                    event('position_plus')
                    self.assertEqual(commands[-1].name, ['a'])
                    count = len(commands)
                    joystick()
                    joystick(0)
                    joystick(0)
                    self.assertEqual(len(commands), count)
                    joystick()
                    if '--triggers' in sys.argv:
                        before_buttons = len(commands)
                        joystick(2)
                        joystick()
                        joystick(3)
                        joystick()
                        self.assertEqual(len(commands), before_buttons, 'Position buttons must be disabled')
                    if '--speed' in sys.argv or '--triggers' in sys.argv:
                        before_axis = len(commands)
                        joystick(axis=1., speed=0.)
                        self.assertEqual(len(commands), before_axis)
                        joystick()
                        joystick(axis=1., speed=0.25)
                        slow = commands[-2:]
                        joystick()
                        joystick(axis=1., speed=1.)
                        fast = commands[-2:]
                        def velocity(samples):
                            a, b = samples
                            def stamp(m):
                                return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                            return (b.position[0] - a.position[0]) / (stamp(b) - stamp(a))
                        self.assertAlmostEqual(velocity(slow), (0.1 * (0.25 - 0.15) / 0.85) if '--triggers' in sys.argv else 0.025, delta=0.005)
                        self.assertAlmostEqual(velocity(fast), 0.1, delta=0.015)
                        joystick()
                    before_axis = len(commands)
                    joystick(axis=1.)
                    self.assertGreater(len(commands), before_axis)
                    self.assertGreater(commands[-1].position[0], 1.)
                    joystick()
                    before_axis = len(commands)
                    joystick(axis=-1.)
                    self.assertGreater(len(commands), before_axis)
                    self.assertLess(commands[-1].position[0], 1.)
                    spin(0.3)
                    count = len(commands)
                    spin(0.1)
                    self.assertEqual(len(commands), count, 'Axis must stop after missing input')
                    spin(0.3, fresh=False)
                    events.publish(JointControlEvent(action=JointControlEvent.POSITION_PLUS))
                    spin(0.1, fresh=False)
                    self.assertEqual(len(commands), count, 'Stale feedback must block motion')
                    self.assertIsNone(process.poll())
                finally:
                    adapter.terminate()
                    adapter.wait(timeout=5)
                    process.terminate()
                    process.wait(timeout=5)
                    node.destroy_node()
                    rclpy.shutdown()
                    log.seek(0)
                    print(log.read())


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
