# ORCA dynamics adapted from alexliniger/MPCC SimTimeStep.m and getModelParams.m.
# Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
"""Original ORCA 1:43 nonlinear bicycle model and fixed-step integration.

State: [X, Y, psi, vx, vy, yaw_rate, progress].
Input: [motor_duty, steering_angle, virtual_progress_speed].
This deliberately retains the upstream abs(vx) slip convention without adding
a low-speed floor. Derivatives are singular at vx=vy=yaw_rate=0; initialize the
optimizer at a nonzero forward speed, as the original baseline does.
"""

import numpy as np


ORCA_PARAMS = {
    "m": 0.041, "Iz": 27.8e-6, "lf": 0.029, "lr": 0.033,
    "Cm1": 0.287, "Cm2": 0.0545, "Cr0": 0.0518, "Cr2": 0.00035,
    "Br": 3.3852, "Cr": 1.2691, "Dr": 0.1737,
    "Bf": 2.579, "Cf": 1.2, "Df": 0.192,
    "L": 0.12, "W": 0.06,
}

INPUT_SCALE = np.array([1.0, 0.35, 6.0])


def _rhs(x, u, sin, cos, atan, atan2, absolute, vector):
    p = ORCA_PARAMS
    phi, vx, vy, omega = x[2], x[3], x[4], x[5]
    duty, delta = u[0], u[1]
    alpha_f = -atan2(p["lf"] * omega + vy, absolute(vx)) + delta
    alpha_r = atan2(p["lr"] * omega - vy, absolute(vx))
    f_fy = p["Df"] * sin(p["Cf"] * atan(p["Bf"] * alpha_f))
    f_ry = p["Dr"] * sin(p["Cr"] * atan(p["Br"] * alpha_r))
    f_rx = p["Cm1"] * duty - p["Cm2"] * duty * vx - p["Cr0"] - p["Cr2"] * vx**2
    return vector([
        vx * cos(phi) - vy * sin(phi),
        vy * cos(phi) + vx * sin(phi),
        omega,
        (f_rx - f_fy * sin(delta) + p["m"] * vy * omega) / p["m"],
        (f_ry + f_fy * cos(delta) - p["m"] * vx * omega) / p["m"],
        (f_fy * p["lf"] * cos(delta) - f_ry * p["lr"]) / p["Iz"],
        u[2],
    ])


def orca_rhs(x, u):
    """Evaluate the original continuous-time ODE for one numeric state/input."""
    x, u = np.asarray(x, dtype=float), np.asarray(u, dtype=float)
    if x.shape != (7,) or u.shape != (3,):
        raise ValueError("ORCA state must have 7 elements and input 3 elements")
    return _rhs(x, u, np.sin, np.cos, np.arctan, np.arctan2, np.abs, np.asarray)


def symbolic_orca_rhs(x, u):
    """Return a CasADi expression using the same physical ODE as orca_rhs."""
    import casadi as ca

    return _rhs(x, u, ca.sin, ca.cos, ca.atan, ca.atan2, ca.fabs,
                lambda values: ca.vertcat(*values))


def rk4_step(x, u, dt=0.02, substeps=4):
    """Integrate the numeric ORCA model while holding the physical input fixed."""
    if dt <= 0 or substeps < 1 or int(substeps) != substeps:
        raise ValueError("RK4 requires positive dt and a positive integer substeps")
    state = np.asarray(x, dtype=float).copy()
    h = float(dt) / int(substeps)
    for _ in range(int(substeps)):
        k1 = orca_rhs(state, u)
        k2 = orca_rhs(state + 0.5 * h * k1, u)
        k3 = orca_rhs(state + 0.5 * h * k2, u)
        k4 = orca_rhs(state + h * k3, u)
        state += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return state


def model_label(config):
    return ('Slash VXL 2WD 58276-74 / provisional VESC current model'
            if config.get('model')=='slash_58276_74' else 'Liniger ORCA 1:43')


def configured_rhs(x,u,config,symbolic=False):
    if config.get('model')=='slash_58276_74':
        from .slash_model import slash_rhs,slash_symbolic_rhs
        return (slash_symbolic_rhs if symbolic else slash_rhs)(x,u,config['model_parameters'])
    return symbolic_orca_rhs(x,u) if symbolic else orca_rhs(x,u)


def configured_rk4(x,u,config):
    if config.get('model')!='slash_58276_74':
        return rk4_step(x,u,config['dt'],config['prediction_substeps'])
    state=np.array(x,float).copy();h=config['dt']/config['prediction_substeps']
    for _ in range(config['prediction_substeps']):
        k1=configured_rhs(state,u,config);k2=configured_rhs(state+h*k1/2,u,config)
        k3=configured_rhs(state+h*k2/2,u,config);k4=configured_rhs(state+h*k3,u,config)
        state+=h*(k1+2*k2+2*k3+k4)/6
    return state


def increment_limits(config):
    if config.get('model')=='slash_58276_74':
        p=config['model_parameters']
        return np.array([p['current_slew_per_second']/max(p['motor_current_limit'],p.get('motor_brake_current_limit',p['motor_current_limit']))*config['dt'],p['steering_rate_limit']*config['dt']/INPUT_SCALE[1],5.])
    return np.array([1.,1.,5.])
