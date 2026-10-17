"""Physical mapping cases: ramps, observed steps, unsupported holes and clutter."""
from pathlib import Path
import math
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from terrain_grid import TerrainConfig, TerrainGrid
from lio_odometry import body_odometry


def floor_points(slope=0.0):
    x,y = np.meshgrid(np.arange(-1.975,2.0,0.05),np.arange(-1.975,2.0,0.05))
    return np.column_stack([x.ravel(),y.ravel(),(-0.4+slope*x).ravel()])


def build(points):
    grid = TerrainGrid()
    for _ in range(3):
        grid.integrate(points,(0,0,0))
    return grid,grid.render()


def cell(data,x,y,key='traversability'):
    ix,iy = np.floor((np.array([x,y])-data['origin'])/data['resolution']).astype(int)
    return data[key][iy,ix]


class TerrainTests(unittest.TestCase):
    def test_flat_floor_is_free_and_unobserved_hole_stays_unknown(self):
        p = floor_points()
        p = p[~((p[:,0] > 0.3)&(p[:,0] < 1.1)&(np.abs(p[:,1]) < .4))]
        _,data = build(p)
        self.assertGreater(np.count_nonzero(data['occupancy'] == 0),4000)
        self.assertEqual(cell(data,.7,0),-1)
        self.assertEqual(cell(data,-.7,0),0)
        self.assertFalse(np.any(data['cost'] == 100))

    def test_gentle_ramp_is_ground_while_steep_ramp_has_cost(self):
        for degrees,expected_blocked in [(5,False),(25,True)]:
            _,data = build(floor_points(math.tan(math.radians(degrees))))
            self.assertGreater(np.count_nonzero(data['occupancy'] == 0),1000)
            slopes = data['slope_deg'][np.isfinite(data['slope_deg'])]
            self.assertAlmostEqual(float(np.median(slopes)),degrees,delta=2)
            self.assertEqual(cell(data,0,0,'cost') == 100,expected_blocked)

    def test_wall_is_occupied_but_high_overhang_is_not(self):
        p = floor_points()
        x,y = np.meshgrid(np.arange(.525,.8,.05),np.arange(-.975,1,.05))
        ceiling = np.column_stack([x.ravel(),y.ravel(),np.full(x.size,2.0)])
        z,y = np.meshgrid(np.arange(-.25,.5,.05),np.arange(-.975,1,.05))
        wall = np.column_stack([np.full(z.size,-.525),y.ravel(),z.ravel()])
        _,data = build(np.vstack([p,ceiling,wall]))
        self.assertEqual(cell(data,-.525,.025,'occupancy'),100)
        self.assertEqual(cell(data,.625,.025,'occupancy'),0)

    def test_observed_step_gets_terrain_hazard(self):
        p = floor_points()
        p[p[:,0] > .6,2] += .2
        _,data = build(p)
        near = np.arange(.4,.85,.05)
        self.assertTrue(any(cell(data,x,.025,'cost') == 100 for x in near))

    def test_single_scan_outliers_do_not_establish_an_obstacle(self):
        grid = TerrainGrid()
        p = floor_points()
        grid.integrate(np.vstack([p,[[.025,.025,.2]]]),(0,0,0))
        for _ in range(2):
            grid.integrate(p,(0,0,0))
        self.assertEqual(cell(grid.render(),.025,.025,'occupancy'),0)

    def test_wall_view_preserves_ground_without_importing_low_obstacles(self):
        grid = TerrainGrid()
        floor = floor_points()
        low = np.array([[.625,.025,-.1]])
        high = np.array([[-.625,.025,.3]])
        for _ in range(3):
            grid.integrate(np.vstack([floor,low,high]),(0,0,0))
            grid.integrate_walls(high,(0,0,0))
        data = grid.render()
        self.assertEqual(cell(data,.625,.025,'occupancy'),100)
        self.assertEqual(cell(data,.625,.025,'walls'),0)
        self.assertEqual(cell(data,-.625,.025,'walls'),100)
        self.assertEqual(cell(data,1.025,.025,'walls'),0)

    def test_save_preserves_origin_and_terrain_layers(self):
        grid,data = build(floor_points())
        with tempfile.TemporaryDirectory() as directory:
            result = grid.save(Path(directory)/'map')
            self.assertTrue(Path(result).exists())
            saved = np.load(Path(directory)/'map.npz')
            np.testing.assert_array_equal(saved['occupancy'],data['occupancy'])
            self.assertEqual(str(saved['frame']),'odom')
            self.assertTrue((Path(directory)/'map_traversability.pgm').exists())


class OdometryTests(unittest.TestCase):
    def test_turn_in_place_removes_imu_lever_arm_velocity(self):
        # Rotating base at a stationary origin: displaced IMU has world velocity.
        r = (.1,.0,.2)
        p,q,v,w = body_odometry((.1,0,.2),(0,0,0,1),(0,.2,0),(0,0,2),r,(0,0,0,1))
        np.testing.assert_allclose(p,[0,0,0],atol=1e-12)
        np.testing.assert_allclose(v,[0,0,0],atol=1e-12)
        np.testing.assert_allclose(w,[0,0,2])

    def test_world_velocity_is_rotated_to_body_axes(self):
        q = (0,0,math.sqrt(.5),math.sqrt(.5))
        _,_,v,_ = body_odometry((0,0,0),q,(0,1,0),(0,0,0),(0,0,0),(0,0,0,1))
        np.testing.assert_allclose(v,[1,0,0],atol=1e-12)


if __name__ == '__main__':
    unittest.main()
