"""Public-spec-based Slash 2WD, unidentified tires and equivalent VESC current.

7 states match the MPCC interface; u=[normalized equivalent motor current,
road-wheel steering angle, virtual progress speed]. Negative drive input is
rear braking, not a reverse-drive command. Rear drive/braking only.
Static axle loads and a smooth combined-force envelope are approximations.
"""
from pathlib import Path
from tools.workspace import find_workspace
import numpy as np
import yaml

SPEC=find_workspace(__file__)/'config/vehicles/slash_58276_74.yaml'

def load_parameters(overrides=None):
    """Merge experiment overrides with the audited specification, then validate."""
    p = yaml.safe_load(SPEC.read_text())['parameters'].copy()
    overrides = dict(overrides or {})
    p.update(overrides)
    if any(k in overrides for k in ('mass', 'length', 'width')) and 'yaw_inertia' not in overrides:
        # Same uniform-body approximation as the baseline, not a measured inertia.
        p['yaw_inertia'] = p['mass'] * (p['length']**2 + p['width']**2) / 12
    if not all(np.isfinite(v) for v in p.values()):
        raise ValueError('Slash model parameters must be finite')
    positive = ('wheelbase', 'mass', 'lf', 'lr', 'yaw_inertia', 'wheel_radius',
                'motor_kv', 'gear_ratio', 'battery_voltage', 'motor_current_limit',
                'motor_max_rpm', 'brake_speed_regularization', 'mu', 'low_speed_regularization')
    if any(p[k] <= 0 for k in positive) or p['motor_brake_current_limit'] < 0:
        raise ValueError('Invalid positive model parameter or negative brake limit')
    if not np.isclose(p['lf'] + p['lr'], p['wheelbase']):
        raise ValueError('lf + lr must equal wheelbase')
    if not 0 < p['drivetrain_efficiency'] <= 1:
        raise ValueError('Drivetrain efficiency must be in (0, 1]')
    return p

def _forces(x,u,p,sin,cos,tanh,sqrt,atan2,maximum,select):
    vx,vy,r=x[3],x[4],x[5];delta=u[1];g=9.80665
    fzf=p['mass']*g*p['lr']/p['wheelbase'];fzr=p['mass']*g*p['lf']/p['wheelbase']
    forward=sqrt(vx*vx+p['low_speed_regularization']**2)
    af=delta-atan2(vy+p['lf']*r,forward);ar=-atan2(vy-p['lr']*r,forward)
    kt=60/(2*np.pi*p['motor_kv'])
    # Spec-bounded speed envelope; its linear torque taper remains an assumption.
    rpm_limit=min(p['motor_kv']*p['battery_voltage'],p.get('motor_max_rpm',50000.))
    no_load=rpm_limit*2*np.pi/60/p['gear_ratio']*p['wheel_radius']
    drive_current=u[0]*p['motor_current_limit']*maximum(0.,1-vx/no_load)
    # Smooth dissipative brake model: no reverse thrust from rest.
    brake_current=u[0]*p.get('motor_brake_current_limit',p['motor_current_limit'])*tanh(vx/p.get('brake_speed_regularization',.2))
    effective_current=select(u[0]>=0,drive_current,brake_current)
    requested=effective_current*kt*p['gear_ratio']*p['drivetrain_efficiency']/p['wheel_radius']
    rear_cap=p['mu']*fzr
    fx=rear_cap*tanh(requested/rear_cap)
    fyf=p['mu']*fzf*tanh(p['tire_shape_front']*af)
    fyr=sqrt(maximum(rear_cap**2-fx**2,0.))*tanh(p['tire_shape_rear']*ar)
    return fx,fyf,fyr

def slash_forces(x,u,p):
    return np.array(_forces(x,u,p,np.sin,np.cos,np.tanh,np.sqrt,np.arctan2,np.maximum,np.where))

def _rhs(x,u,p,sin,cos,tanh,sqrt,atan2,maximum,select,vector):
    fx,fyf,fyr=_forces(x,u,p,sin,cos,tanh,sqrt,atan2,maximum,select)
    yaw,vx,vy,r=x[2],x[3],x[4],x[5];delta=u[1]
    drag=p['rolling_coefficient']*p['mass']*9.80665*tanh(vx/.2)+p['aero_coefficient']*vx*sqrt(vx*vx+.01)
    return vector([vx*cos(yaw)-vy*sin(yaw),vx*sin(yaw)+vy*cos(yaw),r,
        (fx-fyf*sin(delta)-drag)/p['mass']+vy*r,
        (fyf*cos(delta)+fyr)/p['mass']-vx*r,
        (p['lf']*fyf*cos(delta)-p['lr']*fyr)/p['yaw_inertia'],u[2]])

def slash_rhs(x,u,p):
    return _rhs(x,u,p,np.sin,np.cos,np.tanh,np.sqrt,np.arctan2,np.maximum,np.where,np.asarray)

def slash_symbolic_rhs(x,u,p):
    import casadi as ca
    return _rhs(x,u,p,ca.sin,ca.cos,ca.tanh,ca.sqrt,ca.atan2,ca.fmax,ca.if_else,lambda a:ca.vertcat(*a))
