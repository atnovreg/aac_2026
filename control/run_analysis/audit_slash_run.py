"""Post-run checks of physical footprint and provisional axle force budgets."""
from pathlib import Path
import argparse,json
import numpy as np
import yaml
from control.run_analysis.audit_geonji_simulation import main as map_audit
from planning.geonji.build_geonji_track import box_clearance
from control.acados_mpcc.slash_model import slash_forces

def main(run):
 map_audit(run)
 cfg=yaml.safe_load((run/'config.yaml').read_text());p=cfg['model_parameters'];data=np.load(run/'simulation.npz');dense=data['dense'];a=data['trajectory']
 base=Path('results/건지광장/drivable_area/083146_v5_slash_footprint');d=np.load(base/'area.npz');radius=float(np.load(base/'vehicle_clearance.npz')['radius'])
 distance=box_clearance(d['candidate'],dense[:,1:3],float(d['resolution']),d['origin'])
 # Every dense RK45 point corresponds to a held control from its parent step.
 control=np.repeat(a[:,8:11],5,axis=0);forces=np.array([slash_forces(x[1:8],u,p) for x,u in zip(dense,control)])
 rear=p['mass']*9.80665*p['lf']/p['wheelbase']*p['mu'];front=p['mass']*9.80665*p['lr']/p['wheelbase']*p['mu']
 rear_use=np.hypot(forces[:,0],forces[:,2])/rear;front_use=abs(forces[:,1])/front
 assert len(control)==len(dense)
 record={'footprint_circle_with_5cm_margin_m':radius,'minimum_center_to_forbidden_cell_m':float(distance.min()),'minimum_extra_clearance_beyond_circle_m':float(distance.min()-radius),'footprint_clearance_violations':int((distance<radius-1e-9).sum()),'max_rear_combined_force_fraction':float(rear_use.max()),'max_front_lateral_force_fraction':float(front_use.max()),'max_abs_sideslip_deg':float(np.max(abs(np.arctan2(dense[:,5],dense[:,4])))*180/np.pi),'max_equivalent_motor_current_A':float(abs(a[:,8]).max()*p['motor_current_limit']),'vehicle_parameters_unidentified':True}
 assert rear_use.max()<=1+1e-8 and front_use.max()<=1+1e-8
 assert record['footprint_clearance_violations']==0
 (run/'physical_validation.json').write_text(json.dumps(record,indent=2)+'\n')
 np.savetxt(run/'vesc_simulated_commands.csv',np.c_[a[:,0],a[:,8]*p['motor_current_limit'],a[:,9]],delimiter=',',header='time_s,equivalent_motor_current_A,road_wheel_steering_rad',comments='')
 print(json.dumps(record),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);main(p.parse_args().run)
