"""Compatibility launch entrypoint for the IPM BEV node."""

import importlib.util
from pathlib import Path


def generate_launch_description():
    deploy_path = Path(__file__).with_name('deploy.py')
    spec = importlib.util.spec_from_file_location(
        'ipm_bev_node_deploy', deploy_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
