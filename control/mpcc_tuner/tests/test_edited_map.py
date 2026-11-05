"""Catch map/track mismatches when an edited course and old history coexist."""
import copy
from io import BytesIO
import unittest
import numpy as np
from PIL import Image
from tools.mpcc_tuner.config import catalog, simulation_config, ROOT
from tools.mpcc_tuner.assets import map_png

EDITED = ROOT/'results/건지광장/map_edit/20260917_inner_focus'

class EditedMapTests(unittest.TestCase):
    def test_default_experiment_uses_edited_track_with_same_controller(self):
        request=copy.deepcopy(catalog()['defaults'])
        self.assertEqual(request['track'],'edited_inner')
        edited=simulation_config(request,'/tmp/map-test')
        request['track']='both_inner'
        original=simulation_config(request,'/tmp/map-test')
        self.assertEqual(edited.pop('track'),str(EDITED/'track_both_inner'))
        self.assertEqual(original.pop('track'),str(ROOT/'tracks/geonji_slash_both_inner'))
        self.assertEqual(edited,original)

    def test_removed_obstacle_is_floor_only_on_edited_map(self):
        # Known user-selected bottom spot: x=2.7,y=-1.1. Old=obstacle, new=floor.
        with np.load(EDITED/'drivable_area/area.npz') as d:
            col,row=np.floor((np.array([2.7,-1.1])-d['origin'])/float(d['resolution'])).astype(int)
            row=d['candidate'].shape[0]-1-row
        old=np.asarray(Image.open(BytesIO(map_png('both_inner'))))
        new=np.asarray(Image.open(BytesIO(map_png('edited_inner'))))
        self.assertEqual(old[row,col].tolist(),[210,165,137])
        self.assertEqual(new[row,col].tolist(),[248,249,244])
        # Central structure remains an obstacle on both maps.
        with np.load(EDITED/'drivable_area/area.npz') as d:
            col,row=np.floor((np.array([0.,15.])-d['origin'])/float(d['resolution'])).astype(int)
            row=d['candidate'].shape[0]-1-row
        np.testing.assert_array_equal(new[row,col],old[row,col])

if __name__=='__main__': unittest.main()
