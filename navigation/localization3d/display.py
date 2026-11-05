"""Prepare three full-SE3 pose circles in an isolated Gazebo comparison world."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from system.f1tenth_gazebo_replay.generate import child,visual
from system.f1tenth_gazebo_replay.map_mesh import write_stl

HERE=Path(__file__).resolve().parent
BASE=HERE/'baseline'


def ring(path,radius):
    angles=np.linspace(0,2*np.pi,81)
    inner=np.c_[(radius-.045)*np.cos(angles),(radius-.045)*np.sin(angles),np.zeros(81)]
    outer=np.c_[(radius+.045)*np.cos(angles),(radius+.045)*np.sin(angles),np.zeros(81)]
    triangles=[]
    for i in range(80):triangles.extend([[inner[i],outer[i],outer[i+1]],[inner[i],outer[i+1],inner[i+1]]])
    write_stl(path,np.asarray(triangles))


def trail(path,xyz,width=.025):
    triangles=[]
    for a,b in zip(xyz[:-1],xyz[1:]):
        d=b-a;n=np.array([-d[1],d[0],0.]);n/=max(np.linalg.norm(n),1e-12);n*=width/2
        triangles.extend([[a-n,b-n,a+n],[a+n,b-n,b+n]])
    write_stl(path,np.asarray(triangles))


def plugin(model,file):
    p=child(model,'plugin',filename='libPose3DReplay.so',name='localization_display::Pose3DReplay')
    child(p,'trajectory_file','/experiment/results/display/'+file)


def main(args):
    run=args.run.resolve();a=np.load(run/'loop.npz')
    assert json.loads((run/'summary.json').read_text())['completed_lap']
    checked=json.loads((HERE/'results/audit.json').read_text())
    assert checked['status']=='PASS' and Path(checked['run']).resolve()==run
    out=HERE/'results/display';out.mkdir(exist_ok=False)
    tree=ET.parse(BASE/'gazebo_equiv100/world.sdf');world=tree.getroot().find('world')
    world.set('name','geonji_localization3d_20261104')
    template=world.find("model[@name='slash_replay']");world.remove(template)
    for name in ['preferred_region.stl','soft_band.stl','raceline.stl']:
        shutil.copy2(BASE/'gazebo_equiv100'/name,out/name)
    for e in tree.iter():
        if e.tag=='uri' and e.text.startswith('/experiment/gazebo_equiv100/'):
            e.text=e.text.replace('/experiment/gazebo_equiv100/','/experiment/results/display/')
    choices=[('gt','GT','#2470f2','0.05 0.3 1 1',.38),
             ('odometry','ODO','#f09a15','1 .45 .02 1',.50),
             ('estimate','SCAN','#14a751','.03 .9 .22 1',.62)]
    for key,label,_,color,radius in choices:
        poses=a[key]
        euler=Rotation.from_matrix(poses[:,:3,:3]).as_euler('xyz')
        euler[:,2]=np.unwrap(euler[:,2])
        values=np.c_[a['time'],poses[:,:3,3],euler]
        csv=key+'.csv'
        np.savetxt(out/csv,values,delimiter=',',header='t,x,y,z,roll,pitch,yaw',comments='',fmt='%.15g')
        ring(out/(key+'_ring.stl'),radius)
        model=child(world,'model',name='marker_'+label.lower())
        child(model,'pose',' '.join(map(str,values[0,1:])))
        child(model,'static','false')
        link=child(model,'link',name='ring');child(link,'gravity','false')
        inertial=child(link,'inertial');child(inertial,'mass','.01');inertia=child(inertial,'inertia')
        for axis in ['ixx','iyy','izz']:child(inertia,axis,'.001')
        v=visual(link,label,'mesh',{'uri':'/experiment/results/display/'+key+'_ring.stl'},color)
        child(v.find('material'),'emissive',color)
        visual(link,'heading','box',{'size':'.45 .045 .03'},color,'.2 0 0 0 0 0')
        plugin(model,csv)
        trail(out/(key+'_trail.stl'),poses[:,:3,3]+[0,0,.03])
        static=child(world,'model',name='trail_'+label.lower());child(static,'static','true')
        visual(child(static,'link',name='trail'),label,'mesh',{'uri':'/experiment/results/display/'+key+'_trail.stl'},color)
    car=copy.deepcopy(template);car.set('name','vehicle_gt')
    for old in list(car.findall('plugin')):car.remove(old)
    body=car.find('link')
    for v in list(body.findall('visual')):
        if v.get('name') in ['position_marker','marker_stem','heading_marker']:body.remove(v)
    child(body,'pose','0 0 -.115 0 0 0')
    plugin(car,'gt.csv');world.append(car)
    ET.indent(tree);tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    report=dict(source_run=str(run),mode='Replay of3Dvirtualsensor/estimator/MPCCclosedloop; notnativeGazebosensors',
        speed=1.,hold_seconds=0.,repeat='Replay restarts after the recorded singlelap; not a new computed lap',
        legend={'blue':'GT','orange':'uncorrected wheel/IMU odometry','green':'3Dscan-matched estimate'},
        circles='Different radii are display-only, not uncertainty/error radii; circle centres preserve estimated XYZ',
        pose_dimensions='fullXYZrollpitchyaw; no2Dprojection in sensor/registration',
        simulation_time_s=float(a['time'][-1]))
    (out/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(out/'world.sdf')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    main(p.parse_args())
