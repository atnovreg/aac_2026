import unittest
import numpy as np
import casadi as ca
from control.acados_mpcc.slash_model import slash_rhs,slash_symbolic_rhs,slash_forces,load_parameters
class SlashTests(unittest.TestCase):
 def setUp(self):self.p=load_parameters()
 def test_parameters_and_zero_speed_finite(self):
  self.assertAlmostEqual(self.p['lf']+self.p['lr'],.335)
  self.assertGreater(self.p['mass'],2.)
  self.assertTrue(np.isfinite(slash_rhs(np.zeros(7),np.zeros(3),self.p)).all())
 def test_numeric_symbolic_match(self):
  x=ca.MX.sym('x',7);u=ca.MX.sym('u',3);f=ca.Function('slash_test',[x,u],[slash_symbolic_rhs(x,u,self.p)])
  for v in [.1,1.,4.,10.]:
   state=np.array([1.,2.,.3,v,.2,.4,3.]);control=np.array([.7,.12,3.])
   np.testing.assert_allclose(slash_rhs(state,control,self.p),np.array(f(state,control)).ravel(),atol=1e-10)
 def test_rear_drive_combined_traction_and_steering_symmetry(self):
  p=self.p;fzr=p['mass']*9.80665*p['lf']/p['wheelbase'];fzf=p['mass']*9.80665*p['lr']/p['wheelbase']
  for steering in [-.4,0.,.4]:
   x=np.array([0,0,0,5,2,1,0.]);forces=slash_forces(x,np.array([1.,steering,5.]),p)
   self.assertLessEqual(np.hypot(forces[0],forces[2]),p['mu']*fzr+1e-9)
   self.assertLessEqual(abs(forces[1]),p['mu']*fzf+1e-9)
  x=np.array([0,0,0,3,0,0,0.]);l=slash_rhs(x,np.array([0.,.1,3.]),p);r=slash_rhs(x,np.array([0.,-.1,3.]),p)
  self.assertGreater(l[5],0);self.assertAlmostEqual(l[5],-r[5])
 def test_braking_does_not_launch_stationary_vehicle(self):
  force=slash_forces(np.zeros(7),np.array([-1.,0.,0.]),self.p)
  self.assertAlmostEqual(force[0],0.)
 def test_brake_current_is_independent_of_drive_current(self):
  x=np.array([0.,0.,0.,4.,0.,0.,0.]);u=np.array([-1.,0.,0.])
  low=dict(self.p,motor_brake_current_limit=5.)
  high=dict(self.p,motor_brake_current_limit=30.)
  self.assertLess(abs(slash_forces(x,u,low)[0]),abs(slash_forces(x,u,high)[0]))
 def test_mechanical_rpm_limit_bounds_drive_envelope(self):
  p=dict(self.p,motor_max_rpm=1000.)
  v=1000.*2*np.pi/60/p['gear_ratio']*p['wheel_radius']*1.01
  x=np.array([0.,0.,0.,v,0.,0.,0.])
  self.assertAlmostEqual(slash_forces(x,np.array([1.,0.,0.]),p)[0],0.)
 def test_partial_parameter_overrides_preserve_stock_gearing(self):
  p=load_parameters({'mass':4.})
  self.assertEqual(p['mass'],4.)
  self.assertAlmostEqual(p['gear_ratio'],86/23*2.72)

 def test_braking_is_dissipative_and_symbolic_matches(self):
  p=dict(self.p,motor_brake_current_limit=12.)
  x=ca.MX.sym('brake_x',7);u=ca.MX.sym('brake_u',3)
  f=ca.Function('brake_test',[x,u],[slash_symbolic_rhs(x,u,p)])
  for speed in [-2.,-.01,0.,.01,2.,15.]:
   state=np.array([0.,0.,0.,speed,.1,.05,0.]);control=np.array([-.8,.1,0.])
   self.assertLessEqual(slash_forces(state,control,p)[0]*speed,0.)
   np.testing.assert_allclose(slash_rhs(state,control,p),np.array(f(state,control)).ravel(),atol=1e-10)
 def test_invalid_geometry_rejected(self):
  with self.assertRaises(ValueError):load_parameters({'lf':.3})
