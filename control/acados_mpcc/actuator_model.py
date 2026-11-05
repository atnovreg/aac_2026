"""Prediction of 10ms command queues and interval-mean actuator response.

Augmented physical state: vehicle[7], previous request[3], drive/steering
endpoints[2], pending drive commands (oldest first), pending steering commands.
The nonlinear plant holds each interval mean for one 10ms tick, matching the
virtual plant approximation. These are assumed actuator parameters, not a
hardware identification or an electrical VESC model.
"""
import math

import numpy as np

from .model import configured_rhs


TICK = .01


def layout(config):
    p = config.get('actuator_prediction')
    if p is None:
        return None
    if config.get('model') != 'slash_58276_74':
        raise ValueError('Actuator prediction requires the Slash model')
    counts = []
    for name in ('drive_delay_s', 'steering_delay_s'):
        value = p[name]
        count = value/TICK
        if not math.isfinite(value) or value < 0 or count > 100 or abs(count-round(count)) > 1e-9:
            raise ValueError('Prediction delays must be 0..100 integer 10ms ticks')
        counts.append(round(count))
    for name in ('drive_tau_s', 'steering_tau_s'):
        if not math.isfinite(p[name]) or p[name] < 0:
            raise ValueError('Invalid prediction time constant')
    ticks = config['dt']/TICK
    if not math.isfinite(ticks) or ticks < 1 or abs(ticks-round(ticks)) > 1e-9:
        raise ValueError('Prediction timestep must contain integer 10ms ticks')
    substeps = config.get('actuator_prediction_substeps', config['prediction_substeps'])
    if not math.isfinite(substeps) or substeps < 1 or substeps > 16 or substeps != int(substeps):
        raise ValueError('Prediction integrator substeps must be an integer in1..16')
    return (*counts, 12+sum(counts))


def response_coefficients(tau):
    if tau == 0:
        return 1., 1.
    ratio = TICK/tau
    one_minus_decay = -math.expm1(-ratio)
    mean_fraction = (ratio*(.5+ratio*(-1/6+ratio/24)) if ratio < 1e-4
                     else 1-one_minus_decay/ratio)
    return one_minus_decay, mean_fraction


def augmented_step(physical, command, config, dt=None, symbolic=False):
    """Advance a held request, including already committed delayed commands."""
    import casadi as ca
    dimensions = layout(config)
    if dimensions is None:
        raise ValueError('No actuator prediction configured')
    drive_n, steer_n, nx = dimensions
    dt = config['dt'] if dt is None else dt
    if abs(dt/TICK-round(dt/TICK)) > 1e-9 or dt <= 0:
        raise ValueError('Augmented integration requires integer 10ms ticks')
    p = config['actuator_prediction']
    state = physical[:7]
    endpoints = [physical[10], physical[11]]
    queues = [[physical[12+i] for i in range(drive_n)],
              [physical[12+drive_n+i] for i in range(steer_n)]]
    coefficients = [response_coefficients(p['drive_tau_s']),
                    response_coefficients(p['steering_tau_s'])]
    vector = ca.vertcat if symbolic else lambda *a: np.asarray(a)
    for _ in range(round(dt/TICK)):
        applied = []
        for j in range(2):
            target = queues[j].pop(0) if queues[j] else command[j]
            if (drive_n, steer_n)[j]:
                queues[j].append(command[j])
            endpoint_fraction, mean_fraction = coefficients[j]
            applied.append(endpoints[j]+(target-endpoints[j])*mean_fraction)
            endpoints[j] += (target-endpoints[j])*endpoint_fraction
        u = vector(*applied, command[2])
        substeps = int(config.get('actuator_prediction_substeps', config['prediction_substeps']))
        h = TICK/substeps
        for _ in range(substeps):
            k1 = configured_rhs(state, u, config, symbolic)
            k2 = configured_rhs(state+h*k1/2, u, config, symbolic)
            k3 = configured_rhs(state+h*k2/2, u, config, symbolic)
            k4 = configured_rhs(state+h*k3, u, config, symbolic)
            state = state+h*(k1+2*k2+2*k3+k4)/6
    return vector(*[state[i] for i in range(7)],
                  *[command[i] for i in range(3)], *endpoints, *queues[0], *queues[1])
