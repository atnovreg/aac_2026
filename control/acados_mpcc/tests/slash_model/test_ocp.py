import tempfile,unittest
from pathlib import Path
import numpy as np
import casadi as ca
from scipy.integrate import solve_ivp
from tools.acados_mpcc.controller import default_config,build_ocp,state_bounds
from tools.acados_mpcc.track import Track
from tools.acados_mpcc.slash_model import load_parameters,slash_rhs
from tools.acados_mpcc.model import INPUT_SCALE,increment_limits

class SlashOcpTests(unittest.TestCase):
 def test_generated_prediction_matches_independent_plant_and_slash_limits(self):
  cfg=default_config();cfg.update(model='slash_58276_74',model_parameters=load_parameters(),solver='SQP_RTI',hessian='GAUSS_NEWTON',speed_max=4.)
  track=Track.from_csv('tracks/geonji_slash_both_inner/track.csv')
  with tempfile.TemporaryDirectory() as directory:ocp,scale=build_ocp(track,cfg,Path(directory))
  ref=track.reference(0);x=np.r_[ref['center'],ref['yaw'],3.,.1,.2,0.];u=np.array([.1,.03,3.]);z=np.r_[x,np.zeros(3)]/scale
  f=ca.Function('slash_prediction_test',[ocp.model.x,ocp.model.u],[ocp.model.disc_dyn_expr])
  predicted=np.asarray(f(z,u/INPUT_SCALE)).ravel()*scale
  plant=solve_ivp(lambda t,x:slash_rhs(x,u,cfg['model_parameters']),(0,cfg['dt']),x,rtol=1e-11,atol=1e-13).y[:,-1]
  np.testing.assert_allclose(predicted[:7],plant,atol=1e-7)
  lo,hi=state_bounds(track,cfg);self.assertEqual(lo[7],-1.);self.assertEqual(hi[8],.45)
  np.testing.assert_allclose(increment_limits(cfg)[:2],[.2,.05/.35])
