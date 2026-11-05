"""Additional solver-only geometric guard; validation corridor stays unchanged."""
from soft_runtime import SoftController
import numpy as np


class _GuardedSolver:
    def __init__(self,solver,margin):
        self.solver,self.margin=solver,margin

    def constraints_set(self,stage,field,value):
        if field=='lh': value=np.asarray(value)+self.margin
        elif field=='uh': value=np.asarray(value)-self.margin
        return self.solver.constraints_set(stage,field,value)

    def __getattr__(self,name):
        return getattr(self.solver,name)


class GuardedSoftController(SoftController):
    def __init__(self,track,config,build_dir):
        margin=float(config['solver_geometry_margin_m'])
        if not np.isfinite(margin) or margin<=0 or margin>=min(track.width_left.min(),track.width_right.min()):
            raise ValueError('Solver-only guard must fit every side of the runtime corridor')
        super().__init__(track,config,build_dir)
        self.solver=_GuardedSolver(self.solver,margin)
