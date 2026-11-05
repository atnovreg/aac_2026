"""Sensor-estimator-to-controller boundary; no plant or GT input exists."""
import numpy as np
from estimator import mpcc_state


def estimated_feedback(estimate,track,previous_progress,previous_yaw,now):
    if estimate.last_accepted_time is None or now-estimate.last_accepted_time>.5:
        raise RuntimeError('localization is stale')
    progress=track.project(estimate.pose[:2,3],previous_progress)
    state=mpcc_state(estimate.pose,estimate.velocity_map,estimate.gyro,progress)
    state[2]+=2*np.pi*round((previous_yaw-state[2])/(2*np.pi))
    if not np.isfinite(state).all():raise RuntimeError('invalid estimated controller state')
    return state
