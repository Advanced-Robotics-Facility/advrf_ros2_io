"""Check URDF axis placement and replacing/clearing the selected-joint marker."""
import importlib.machinery
import importlib.util
from pathlib import Path
import unittest
from visualization_msgs.msg import Marker

loader = importlib.machinery.SourceFileLoader('selected_marker', str(
    Path(__file__).resolve().parents[1] / 'scripts/selected_joint_marker_node'))
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)
URDF = '''<robot name="test">
<joint name="yaw" type="revolute"><child link="arm"/><axis xyz="0 0 2"/></joint>
<joint name="slide" type="prismatic"><child link="claw"/><axis xyz="1 0 0"/></joint>
<joint name="fixed" type="fixed"><child link="base"/></joint>
</robot>'''


class SelectionMarkerTest(unittest.TestCase):
    def test_urdf_axis_and_frame(self):
        joints = module.joint_frames(URDF)
        self.assertNotIn('fixed', joints)
        markers = module.markers_for(joints, 'yaw').markers
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertEqual(len(markers), 4)
        self.assertTrue(all(m.header.frame_id == 'arm' and m.frame_locked for m in markers[1:]))
        arrow = markers[2]
        self.assertAlmostEqual(arrow.points[1].z, 0.22)
        self.assertEqual(arrow.points[1].x, 0)
        self.assertEqual(markers[3].text, 'yaw')

    def test_new_selection_and_unknown_clear_previous(self):
        joints = module.joint_frames(URDF)
        markers = module.markers_for(joints, 'slide').markers
        self.assertEqual(markers[2].header.frame_id, 'claw')
        self.assertAlmostEqual(markers[2].points[1].x, 0.22)
        for name in ['', 'unknown']:
            markers = module.markers_for(joints, name).markers
            self.assertEqual(len(markers), 1)
            self.assertEqual(markers[0].action, Marker.DELETEALL)


if __name__ == '__main__':
    unittest.main()
