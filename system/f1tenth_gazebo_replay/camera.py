"""Set overview after the GUI has advertised its camera service."""
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


def main():
    world = ET.parse(sys.argv[1])
    x,y,z,roll,pitch,yaw = map(float, world.find('.//camera_pose').text.split())
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    qx,qy,qz,qw=sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy
    request=f'pose: {{position: {{x: {x} y: {y} z: {z}}} orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}'
    # Service advertisement can precede creation of the render camera.
    # A published pose confirms the scene is ready to accept a move request.
    try:
        subprocess.run(['gz','topic','-e','-t','/gui/camera/pose','-n','1'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=45, check=True)
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
        print('Camera pose not available; leaving the view unchanged.', file=sys.stderr)
        return
    for _ in range(30):
        result=subprocess.run(['gz','service','-s','/gui/move_to/pose','--reqtype','gz.msgs.GUICamera','--reptype','gz.msgs.Boolean','--timeout','1000','--req',request], capture_output=True, text=True, timeout=5)
        if 'data: true' in result.stdout:
            print('Gazebo map overview camera ready.', flush=True)
            return
        time.sleep(.5)
    print('Camera service unavailable; adjust the Gazebo view manually.', file=sys.stderr)


if __name__ == '__main__': main()
