"""Joystick activation against a fake bridge, on an isolated DDS domain."""
import os
os.environ['ROS_DOMAIN_ID'] = '186'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import subprocess
import sys
import tempfile
import time
import unittest
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool


class JoystickActiveTest(unittest.TestCase):
    def test_toggle_without_gating_axes(self):
        with tempfile.TemporaryDirectory() as tmp:
            os.environ['ROS_LOG_DIR'] = tmp
            rclpy.init()
            node = rclpy.create_node('fake_bridge', namespace='/active_test')
            state = node.create_publisher(Bool, 'ecat/active', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            requests, axes = [], []
            def toggle(req, res):
                requests.append(req.data)
                # Inject movement while the service response is still pending.
                joy.publish(Joy(buttons=[1,0,0,0,0,0], axes=[0.0,0.8]))
                time.sleep(0.1)
                state.publish(Bool(data=req.data))
                res.success = True
                res.message = 'mock bridge updated'
                return res
            service = node.create_service(SetBool, 'ecat/set_active', toggle)
            joy = node.create_publisher(Joy, 'joy', 10)
            sub = node.create_subscription(Float64, 'joint_control/axis', lambda msg: axes.append(msg.data), 10)
            state.publish(Bool(data=False))
            process = subprocess.Popen([sys.argv[1], '--ros-args', '-r', '__ns:=/active_test',
                '-p', 'toggle_active_button:=0', '-p', 'power_on_button:=-1', '-p', 'power_off_button:=-1'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            def spin(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)
            def send(pressed=False, axis=0.0):
                joy.publish(Joy(buttons=[int(pressed),0,0,0,0,0], axes=[0.0,axis]))
                spin(0.08)
            try:
                end = time.monotonic()+5
                while joy.get_subscription_count()==0 and time.monotonic()<end: spin(0.05)
                self.assertGreater(joy.get_subscription_count(), 0)
                spin(0.15)
                send(pressed=True)  # A held at connection must not toggle.
                self.assertEqual(requests, [])
                send(axis=0.8)
                self.assertGreater(axes[-1], 0, "Inactive bridge must not gate joystick data")
                send(); send(pressed=True); spin(0.15)
                self.assertEqual(requests, [True])
                self.assertGreater(axes[-1], 0, 'Pending service request must not gate joystick data')
                send(pressed=True); send(pressed=True)
                self.assertEqual(requests, [True])
                send(); send(axis=0.8)
                self.assertGreater(axes[-1], 0)
                send(); send(pressed=True); spin(0.15)
                self.assertEqual(requests, [True, False])
                send(axis=0.8)
                self.assertGreater(axes[-1], 0, "Inactive bridge must not gate joystick data")
                send(); send(pressed=True); spin(0.15)
                self.assertEqual(requests, [True, False, True])
                # External/watchdog state change must replace any locally inferred state.
                state.publish(Bool(data=False)); spin(0.1)
                send(); send(pressed=True); spin(0.15)
                self.assertEqual(requests, [True, False, True, True])
                state.publish(Bool(data=False))
                spin(0.1)
                send(axis=0.8)
                self.assertGreater(axes[-1], 0, 'External deactivation must not reset the joystick axis')
                self.assertIsNone(process.poll())
            finally:
                process.terminate(); process.wait(timeout=5)
                node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
