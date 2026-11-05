import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from system.f1tenth_gazebo_replay.generate import strip_mesh, create_world
from system.f1tenth_gazebo_replay.map_mesh import select_points, floor_z


class WorldTests(unittest.TestCase):
    def test_strip_mesh_preserves_metric_vertices_and_closes_loop(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'strip.obj'
            left = np.array([[0, 0], [2, 0], [2, 2], [0, 2]])
            right = np.array([[-1, -1], [3, -1], [3, 3], [-1, 3]])
            strip_mesh(path, left, right, .01, (.1, -.2, -.7))
            lines = path.read_text().splitlines()
            vertices = np.array([[float(v) for v in line.split()[1:]] for line in lines if line.startswith('v ')])
            np.testing.assert_allclose(vertices[:, :2], np.vstack((left, right)))
            np.testing.assert_allclose(vertices[:,2], vertices[:,0]*.1-vertices[:,1]*.2-.69)
            self.assertEqual(sum(line.startswith('f ') for line in lines), 8)

    def test_map_sampling_preserves_xyz_and_filters_relative_to_sloping_floor(self):
        points = np.array([[0,0,1.], [0.01,0,1.01], [10,0,2.5], [10,0,5.], [np.nan,0,0]])
        selected = select_points(points, [.1,0,1.])
        np.testing.assert_allclose(selected, points[[0,2]])
        np.testing.assert_allclose(floor_z(selected[:,:2], [.1,0,1.]), [1,2])

    def test_world_is_labeled_replay_and_car_has_no_collision(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            create_world(output, np.array([0., 0., 0.]), np.array([8., 5.]), 1.)
            world = ET.parse(output/'world.sdf').getroot().find('world')
            self.assertEqual(world.attrib['name'], 'acados_trajectory_replay')
            car = world.find("model[@name='orca_replay']")
            self.assertEqual(car.find('plugin').attrib['name'], 'replay::Replay')
            self.assertEqual(car.findall('.//collision'), [])
            self.assertIn('trajectory.csv', car.find('plugin/trajectory_file').text)


if __name__ == '__main__': unittest.main()
