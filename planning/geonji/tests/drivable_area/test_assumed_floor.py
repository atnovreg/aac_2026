import numpy as np
from planning.geonji.assume_route_floor import fill_route_gaps

def test_fill_is_bounded_obstacles_win_and_evidence_unchanged():
    observed=np.zeros((9,9),bool);observed[4,1]=True
    obstacle=np.zeros_like(observed);obstacle[4,4]=True
    original=observed.copy()
    candidate,assumed,distance=fill_route_gaps(observed,obstacle,np.array([0.,0.]),1.,np.array([[1.5,4.5],[7.5,4.5]]),1.)
    assert assumed[4,3] and assumed[3,3]
    assert not candidate[4,4]
    assert not assumed[1,3]
    assert not assumed[4,1]
    assert np.array_equal(observed,original)
    assert np.all(distance[assumed]<=1.)
