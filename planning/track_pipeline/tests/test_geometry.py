import unittest
import numpy as np
from planning.track_pipeline.geometry import extract_loop, resample_closed, ray_boundaries, track_limits, validate_strip, mutual_crossings

class GeometryTests(unittest.TestCase):
    def test_annulus_centre_comes_from_space_not_driven_line(self):
        r=.05; origin=np.array([-7.,-7.]); yy,xx=np.mgrid[:280,:280]
        radius=np.hypot(origin[0]+(xx+.5)*r,origin[1]+(yy+.5)*r)
        free=(radius>3)&(radius<5)
        theta=np.linspace(0,2*np.pi,200,endpoint=False)
        driven=np.c_[4.7*np.cos(theta),4.7*np.sin(theta)]
        loop=extract_loop(free,r,origin,driven)
        centre=resample_closed(loop,.05,.15)[1]
        self.assertLess(abs(np.mean(np.linalg.norm(centre,axis=1))-4),.08)
        self.assertGreater(np.mean(np.linalg.norm(centre-driven[0],axis=1)),1)

    def test_closed_circle_has_metric_spacing_and_curvature(self):
        theta=np.linspace(0,2*np.pi,180,endpoint=False)
        s,xy,yaw,kappa,length=resample_closed(np.c_[3*np.cos(theta),3*np.sin(theta)],.05,0)
        self.assertAlmostEqual(length,6*np.pi,places=2)
        self.assertLess(np.ptp(np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)),.001)
        self.assertTrue(np.allclose(kappa,1/3,atol=.003))
        self.assertLess(s[-1],length)

    def test_missing_wall_is_explicit_not_invented(self):
        free=np.ones((100,100),bool);obs=np.zeros_like(free)
        xy=np.array([[2.5,2.5]]);yaw=np.array([0.])
        left,right,ls,rs=ray_boundaries(xy,yaw,obs,free,.05,np.zeros(2),2.)
        self.assertEqual(ls[0],0);self.assertEqual(rs[0],0)
        self.assertLessEqual(np.linalg.norm(left-xy),2.)

class StripTests(unittest.TestCase):
    def test_crossing_limits_are_detected(self):
        a=np.array([[0.,0.],[2.,0.],[2.,2.],[0.,2.]])
        b=a+np.array([1.,1.])
        self.assertEqual(mutual_crossings(a,b),2)

    def test_obstacle_between_vertices_is_detected(self):
        free=np.ones((100,100),bool);free[45:55,45:55]=False
        centre=np.array([[1.,2.5],[4.,2.5],[4.,3.],[1.,3.]])
        left=centre+np.array([0.,.1]);right=centre-np.array([0.,.1])
        result=validate_strip(centre,left,right,free,.05,np.zeros(2))
        self.assertGreater(result['strip_samples_outside_usable'],0)

if __name__=='__main__':unittest.main()
