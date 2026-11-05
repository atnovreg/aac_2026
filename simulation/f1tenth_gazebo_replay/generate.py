import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from tools.workspace import find_workspace
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from .map_mesh import floor_z, map_meshes, write_stl


def strip_mesh(path, left, right, z, plane=(0.,0.,0.)):
    """Write an unscaled closed triangle ribbon, with faces visible from above."""
    n = len(left)
    vertices = np.vstack((left, right))
    heights = floor_z(vertices, plane) + z
    xyz = np.c_[vertices, heights]
    faces = []
    lines = [f'v {x:.9f} {y:.9f} {h:.6f}' for (x, y), h in zip(vertices, heights)]
    for i in range(n):
        j = (i+1) % n
        for a, b, c in ((i, j, n+i), (j, n+j, n+i)):
            ab, ac = vertices[b]-vertices[a], vertices[c]-vertices[a]
            cross = ab[0]*ac[1]-ab[1]*ac[0]
            if cross < 0: b, c = c, b
            lines.append(f'f {a+1} {b+1} {c+1}')
            faces.append((a,b,c))
    path.write_text('\n'.join(lines)+'\n')
    write_stl(path.with_suffix('.stl'), xyz[np.asarray(faces)])


def child(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None: element.text = str(text)
    return element


def visual(link, name, kind, dimensions, color, pose='0 0 0 0 0 0'):
    element = child(link, 'visual', name=name)
    child(element, 'pose', pose)
    geometry = child(child(element, 'geometry'), kind)
    for key, value in dimensions.items(): child(geometry, key, value)
    material = child(element, 'material')
    child(material, 'ambient', color); child(material, 'diffuse', color)
    child(element, 'cast_shadows', 'false')
    return element


def create_world(output, initial_pose, center, speed, plane=(0.,0.,0.), meshes=(), vehicle=None):
    sdf = ET.Element('sdf', version='1.9')
    world = child(sdf, 'world', name='acados_trajectory_replay')
    child(world, 'gravity', '0 0 0')
    physics = child(world, 'physics', name='replay_clock', type='ignored')
    child(physics, 'max_step_size', '.01'); child(physics, 'real_time_factor', '1')
    for filename, name in [('gz-sim-physics-system', 'gz::sim::systems::Physics'),
                           ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
                           ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster')]:
        child(world, 'plugin', filename=filename, name=name)
    scene = child(world, 'scene')
    child(scene, 'ambient', '.8 .8 .8 1'); child(scene, 'background', '.92 .94 .97 1')
    child(scene, 'shadows', 'false')
    light = child(world, 'light', name='sun', type='directional')
    child(light, 'pose', '0 0 20 0 0 0'); child(light, 'direction', '-.2 .1 -1')
    child(light, 'diffuse', '.9 .9 .9 1')
    gui = child(world, 'gui', fullscreen='0')
    view = child(gui, 'plugin', filename='MinimalScene', name='3D View')
    settings = child(view, 'gz-gui')
    child(settings, 'title', '3D View')
    child(settings, 'property', 'false', key='showTitleBar', type='bool')
    child(settings, 'property', 'docked', key='state', type='string')
    child(view, 'engine', 'ogre2'); child(view, 'scene', 'scene')
    child(view, 'grid', 'false')
    child(view, 'ambient_light', '.8 .8 .8'); child(view, 'background_color', '.92 .94 .97')
    child(view, 'camera_pose', f'{center[0]} {center[1]-18} 22 0 .885 1.57079632679')
    for filename in ['GzSceneManager', 'InteractiveViewControl', 'CameraTracking', 'EntityContextMenuPlugin']:
        settings = child(child(gui, 'plugin', filename=filename, name=filename), 'gz-gui')
        for key, kind, value in [('state','string','floating'), ('width','double','5'), ('height','double','5'), ('showTitleBar','bool','false')]:
            child(settings, 'property', value, key=key, type=kind)
    control = child(gui, 'plugin', filename='WorldControl', name='Playback')
    settings = child(control, 'gz-gui')
    for key, kind, value in [('state','string','floating'), ('height','double','72'), ('showTitleBar','bool','false')]:
        child(settings, 'property', value, key=key, type=kind)
    anchors = child(settings, 'anchors', target='3D View')
    child(anchors, 'line', own='left', target='left'); child(anchors, 'line', own='bottom', target='bottom')
    child(control, 'play_pause', 'true'); child(control, 'use_event', 'true')
    cloud = child(world, 'model', name='measured_3d_map')
    child(cloud, 'static', 'true')
    cloud_link = child(cloud, 'link', name='original_pcd_returns')
    for name, color in meshes:
        visual(cloud_link, name, 'mesh', {'uri':str((output/name).resolve())}, color)
    ground = child(world, 'model', name='ground')
    child(ground, 'static', 'true')
    link = child(ground, 'link', name='ground')
    visual(link, 'ground', 'plane', {'normal':'0 0 1', 'size':'80 80'}, '.82 .85 .89 1', f'0 0 {plane[2]-3} 0 0 0')
    track = child(world, 'model', name='measured_track_limits')
    child(track, 'static', 'true')
    link = child(track, 'link', name='track')
    for name, color in [('left', '.1 .9 .6 1'),
                        ('right', '1 .48 .12 1'), ('center', '.25 .7 1 1')]:
        visual(link, name, 'mesh', {'uri':str((output/(name+'.stl')).resolve())}, color)
    car = child(world, 'model', name='slash_replay' if vehicle else 'orca_replay')
    x, y, yaw = initial_pose
    child(car, 'pose', f'{x} {y} {floor_z([x,y], plane)+.035} 0 0 {yaw}')
    child(car, 'static', 'false')
    link = child(car, 'link', name='body')
    child(link, 'gravity', 'false')
    inertial = child(link, 'inertial'); child(inertial, 'mass', '.041')
    inertia = child(inertial, 'inertia')
    for axis in ['ixx', 'iyy', 'izz']: child(inertia, axis, '.0001')
    visual(link, 'chassis', 'box', {'size':'.12 .06 .025'}, '1 .15 .06 1')
    visual(link, 'cabin', 'box', {'size':'.045 .045 .02'}, '.06 .1 .15 1', '0 0 .022 0 0 0')
    for xwheel in [-.033, .029]:
        for ywheel in [-.034, .034]:
            visual(link, f'wheel_{xwheel}_{ywheel}', 'cylinder', {'radius':'.015', 'length':'.012'},
                   '.025 .025 .025 1', f'{xwheel} {ywheel} -.018 1.57079632679 0 0')
    if vehicle:
        # Replay-only schematic: use the saved model envelope, not an ORCA-sized icon.
        for element in list(link.findall('visual')):
            link.remove(element)
        inertial.find('mass').text = str(vehicle['mass'])
        visual(link, 'chassis', 'box', {'size':f"{vehicle['length']} {vehicle['width']} .08"},
               '1 .15 .06 1', '0 0 .10 0 0 0')
        visual(link, 'cabin', 'box', {'size':'.20 .18 .08'}, '.06 .1 .15 1', '0 0 .18 0 0 0')
        for xwheel in [-vehicle['lr'], vehicle['lf']]:
            for ywheel in [-vehicle['width']/2+.02, vehicle['width']/2-.02]:
                visual(link, f'wheel_{xwheel}_{ywheel}', 'cylinder',
                       {'radius':str(vehicle['wheel_radius']), 'length':'.04'}, '.025 .025 .025 1',
                       f"{xwheel} {ywheel} {vehicle['wheel_radius']-.035} 1.57079632679 0 0")
    visual(link, 'position_marker', 'sphere', {'radius':'.18'}, '1 .12 .05 1', '0 0 2.5 0 0 0')
    visual(link, 'marker_stem', 'cylinder', {'radius':'.012', 'length':'2.45'}, '1 .25 .05 1', '0 0 1.25 0 0 0')
    visual(link, 'heading_marker', 'box', {'size':'.38 .035 .025'}, '1 1 1 1', '.2 0 2.5 0 0 0')
    plugin = child(car, 'plugin', filename='libTrajectoryReplay.so', name='replay::Replay')
    child(plugin, 'trajectory_file', str((output/'trajectory.csv').resolve()))
    child(plugin, 'floor_plane', ' '.join(map(str, plane)))
    child(plugin, 'speed', speed); child(plugin, 'hold_seconds', '2')
    ET.indent(sdf)
    ET.ElementTree(sdf).write(output/'world.sdf', encoding='utf-8', xml_declaration=True)


def generate(run, output, speed, map_height=2.0, map_file=None):
    summary = json.loads((run/'summary.json').read_text())
    if not summary.get('completed_lap'): raise ValueError('Select a completed acados lap')
    root = find_workspace(__file__)
    provenance = json.loads((run/'provenance.json').read_text())
    # Existing runs record host or /ws paths; resolve the workspace-relative track.
    original = Path(provenance['track'])
    track_dir = original if original.exists() else root/'tracks'/original.name
    digest = hashlib.sha256((track_dir/'track.csv').read_bytes()).hexdigest()
    if digest != provenance['track_sha256']: raise ValueError('Track differs from recorded simulation input')
    track = np.genfromtxt(track_dir/'track.csv', delimiter=',', names=True)
    data = np.genfromtxt(run/'trajectory.csv', delimiter=',', names=True)
    if len(data) < 2 or np.any(np.diff(data['t']) <= 0): raise ValueError('Invalid recorded trajectory time')
    samples = np.c_[data['t'], data['x'], data['y'], data['yaw']]
    if not np.isfinite(samples).all(): raise ValueError('Nonfinite recorded poses')
    output.mkdir(parents=True, exist_ok=False)
    np.savetxt(output/'trajectory.csv', samples, delimiter=',', header='t,x,y,yaw', comments='', fmt='%.12g')
    center = np.c_[track['x'], track['y']]
    normal = np.c_[-np.sin(track['yaw']), np.cos(track['yaw'])]
    left = center + normal*track['width_left'][:,None]
    right = center - normal*track['width_right'][:,None]
    validation = json.loads((track_dir/'validation.json').read_text())
    plane = validation.get('floor_plane')
    if plane is None:
        source = Path(json.loads((track_dir/'extraction.json').read_text())['source'])
        if not source.is_absolute(): source = root/source
        with np.load(source/'area.npz') as area:
            plane = area['plane'].tolist()
    config = yaml.safe_load((run/'config.yaml').read_text()) if (run/'config.yaml').exists() else {}
    vehicle = config.get('model_parameters') if config.get('model') == 'slash_58276_74' else None
    meshes, map_info = map_meshes(map_file or track_dir/'map.pcd', output, plane, map_height)
    strip_mesh(output/'road.obj', left, right, .006, plane)
    for name, points in [('left', left), ('right', right), ('center', center)]:
        width = .035 if name != 'center' else .02
        strip_mesh(output/(name+'.obj'), points+normal*width/2, points-normal*width/2, .04, plane)
    create_world(output, samples[0,1:], (center.min(axis=0)+center.max(axis=0))/2, speed, plane, meshes, vehicle)
    # Fit the plaza overview to the actual route extent.
    if vehicle:
        tree = ET.parse(output/'world.sdf')
        midpoint = (center.min(axis=0)+center.max(axis=0))/2
        span = float(np.ptp(center, axis=0).max())
        tree.find('.//camera_pose').text = f'{midpoint[0]} {midpoint[1]-span*.62} {span*.85} 0 .94 1.57079632679'
        tree.write(output/'world.sdf', encoding='utf-8', xml_declaration=True)
    manifest = {'map':map_info, 'floor_plane':plane, 'source_run':str(run), 'track':str(track_dir), 'track_sha256':digest,
                'trajectory_sha256':hashlib.sha256((run/'trajectory.csv').read_bytes()).hexdigest(),
                'mode':'recorded trajectory replay; no vehicle physics or live MPCC', 'speed':speed,
                'recorded_start_s':float(samples[0,0]), 'recorded_end_s':float(samples[-1,0]),
                'vehicle_visual_size_m':[vehicle['length'],vehicle['width']] if vehicle else [.12,.06],
                'vehicle_visual':'schematic body; no suspension or collision physics', 'position_marker_radius_m':.18}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return output/'world.sdf'


def main():
    parser = argparse.ArgumentParser(description='Create a Gazebo replay world from a completed acados lap')
    parser.add_argument('--run', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--map-file', type=Path, help='Alternative PCD already in the track camera_init frame')
    parser.add_argument('--map-height', type=float, default=2.0, help='Display returns up to this height above fitted floor (m)')
    parser.add_argument('--speed', type=float, default=1.)
    args = parser.parse_args()
    if not np.isfinite(args.map_height) or args.map_height <= 0: parser.error('map-height must be positive and finite')
    if not np.isfinite(args.speed) or args.speed <= 0: parser.error('speed must be positive and finite')
    root = find_workspace(__file__)
    if args.run is None:
        candidates = []
        for file in sorted((root/'results/acados_mpcc').glob('*/summary.json')):
            summary = json.loads(file.read_text())
            if summary.get('completed_lap') and summary.get('solver') == 'SQP_RTI': candidates.append(file.parent)
        if not candidates: parser.error('No completed RTI run found; specify --run')
        args.run = candidates[-1]
    output = args.output or root/'results/gazebo_replay'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    print(generate(args.run.resolve(), output.resolve(), args.speed, args.map_height, args.map_file))


if __name__ == '__main__': main()
