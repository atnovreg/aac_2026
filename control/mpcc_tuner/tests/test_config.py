import copy
import unittest
from tools.mpcc_tuner.config import catalog, validated_request, simulation_config

class ConfigTests(unittest.TestCase):
    def request(self):
        return copy.deepcopy(catalog()['defaults'])

    def test_weight_mapping_preserves_other_values_and_original(self):
        request=self.request();request['weights']['contour']=1.0
        checked=validated_request(request)
        config=simulation_config(checked,'/tmp/isolated-run')
        self.assertEqual(config['weights']['contour'],1.0)
        self.assertEqual(config['weights']['lag'],1000.0)
        self.assertEqual(config['weights']['increment'],[.01,1.0,.001])
        self.assertEqual(config['solver'],'SQP_RTI')
        self.assertEqual(config['levenberg_marquardt'],0.1)
        self.assertEqual(catalog()['defaults']['weights']['contour'],.1)
        self.assertEqual(config['output'],'/tmp/isolated-run')

    def test_reject_invalid_input_without_starting_solver(self):
        for key,value in [('contour',float('nan')),('lag',float('inf')),('input_progress',0),('contour',-1),('contour',True),('progress','0.02')]:
            with self.subTest(key=key,value=value):
                request=self.request();request['weights'][key]=value
                with self.assertRaises(ValueError): validated_request(request)
        for change in [{'track':'../../foo'},{'speed_max':26.9},{'output':'/tmp/forbidden'}]:
            request=self.request();request.update(change)
            with self.assertRaises(ValueError): validated_request(request)

    def test_approximately_60mph_cap_reaches_solver_without_changing_model(self):
        request=self.request();request['speed_max']=26.8
        config=simulation_config(request,'/tmp/high-speed-cap')
        self.assertEqual(config['speed_max'],26.8)
        self.assertAlmostEqual(config['progress_speed_max'],40.2)
        self.assertEqual(catalog()['speed_limits']['max'],26.8)
        self.assertEqual(catalog()['defaults']['speed_max'],4.)
        self.assertNotIn('model_parameters',config)

    def test_missing_or_unknown_weights_rejected(self):
        request=self.request();del request['weights']['lag']
        with self.assertRaises(ValueError): validated_request(request)
        request=self.request();request['weights']['surprise']=1
        with self.assertRaises(ValueError): validated_request(request)

if __name__=='__main__': unittest.main()
