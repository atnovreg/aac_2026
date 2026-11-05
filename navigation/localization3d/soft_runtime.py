"""Offline-only MPCC soft-zone cost; existing hard constraint is preserved."""
import casadi as ca
import numpy as np
from tools.acados_mpcc import controller as original_controller


def extend_ocp(ocp,weight):
    if weight<=0: raise ValueError('Positive soft weight required')
    limits=ca.MX.sym('preferred_limits',4)  # lower, upper, lower-band, upper-band
    h=ocp.model.con_h_expr
    residual=ca.vertcat(ca.fmax(limits[0]-h,0)/limits[2],
                       ca.fmax(h-limits[1],0)/limits[3])
    ocp.model.p=ca.vertcat(ocp.model.p,limits)
    ocp.model.name+='_soft'
    ocp.name=ocp.model.name
    ocp.model.cost_y_expr=ca.vertcat(ocp.model.cost_y_expr,residual)
    ocp.model.cost_y_expr_e=ca.vertcat(ocp.model.cost_y_expr_e,residual)
    for field in ['W','W_e']:
        matrix=np.pad(getattr(ocp.cost,field),((0,2),(0,2)))
        matrix[-2,-2]=matrix[-1,-1]=2*weight
        setattr(ocp.cost,field,matrix)
    ocp.cost.yref=np.r_[ocp.cost.yref,0.,0.]
    ocp.cost.yref_e=np.r_[ocp.cost.yref_e,0.,0.]
    ocp.parameter_values=np.r_[ocp.parameter_values,-.2,.2,.3,.3]


class SoftProfile:
    def __init__(self,s,length,lower,upper,band_lower,band_upper):
        self.s=np.asarray(s,dtype=float)
        self.length=float(length)
        self.values=np.column_stack([lower,upper,band_lower,band_upper])
        if self.values.shape!=(len(self.s),4) or not np.isfinite(self.values).all():
            raise ValueError('Invalid preferred profile shape/values')
        if self.s[0]!=0 or np.any(np.diff(self.s)<=0) or self.length<=self.s[-1]:
            raise ValueError('Invalid periodic progress')
        if np.any(self.values[:,0]>=self.values[:,1]) or np.any(self.values[:,2:]<=0):
            raise ValueError('Invalid signed interval or band normalizer')
        self.closed_s=np.r_[self.s,self.length]
        self.closed_values=np.vstack([self.values,self.values[0]])

    @classmethod
    def load(cls,path):
        with np.load(path) as d:
            return cls(d['s'],float(d['length']),d['lower'],d['upper'],d['band_lower'],d['band_upper'])

    def at(self,progress):
        wrapped=np.asarray(progress)%self.length
        return np.stack([np.interp(wrapped,self.closed_s,self.closed_values[:,j]) for j in range(4)],axis=-1)


class _ParameterAdapter:
    def __init__(self,solver,owner,profile):
        self._solver,self._owner,self._profile=solver,owner,profile

    def set(self,stage,field,value):
        if field=='p':
            if len(value)!=4: raise ValueError('Expected legacy centre/normal parameters')
            progress=self._owner.prediction[stage,6]
            value=np.r_[value,self._profile.at(progress)]
        return self._solver.set(stage,field,value)

    def __getattr__(self,name):
        return getattr(self._solver,name)


class SoftController(original_controller.Controller):
    def __init__(self,track,config,build_dir):
        if config.get('runtime_backend','python')!='python':
            raise ValueError('This experiment uses Python stage API; old C-batch has four parameters')
        profile=SoftProfile.load(config['soft_profile'])
        build=original_controller.build_ocp
        def modified_build(*args,**kwargs):
            ocp,scale=build(*args,**kwargs)
            extend_ocp(ocp,config.get('soft_weight',.03))
            return ocp,scale
        original_controller.build_ocp=modified_build
        try:
            super().__init__(track,config,build_dir)
        finally:
            original_controller.build_ocp=build
        self.solver=_ParameterAdapter(self.solver,self,profile)
