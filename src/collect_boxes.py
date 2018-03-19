from robot import Robot
import rgslib as lib
import numpy as np
import time

print('Initialising robot...')

r = Robot()
c = lib.MotionController(r)
v = lib.VisionController(r)
s = lib.GameState(0)

print('GO!')

try:
    while True:
        t0 = time.time()
        m = [m for m in v.markers_blocking() if s.get_marker_type(m) == 'FRIENDLY']
        print(time.time() - t0)
        if m:
            if len(m) > 0:
                m0 = sorted(m, key=lambda x: x.spherical.distance_metres)[0]
                id = m0.id
                
                t0 = v.last_marker_time
                def bad_angle_interrupt():
                    markers = v.markers
                    id_markers = [m for m in markers if m.id == id]
                    if len(id_markers) == 0:
                        print("Didn't see marker")
                        return 0
                    else:
                        m0 = id_markers[0]
                        angle = m0.spherical.rot_y_degrees
                        distance = m0.spherical.distance_metres * 100
                        print("Saw marker at", angle)
                        if np.abs(angle) > np.abs(np.arctan2(20, distance))*(180 / np.pi) + 1 and np.abs(angle) > 5 and v.last_marker_time > t0:
                            return 'ANGLE TOO WIDE {}'.format(angle)
                        else:
                            return 0
                
                angle = m0.spherical.rot_y_degrees
                distance = m0.spherical.distance_metres * 0.881 * 100
                print('Found box, angle/distance:', angle, distance)
                if np.abs(angle) > 5:
                    c.rotate(angle, speed=0.25)
                c.move(distance + 60, speed=0.35, interrupts=[bad_angle_interrupt])
        else:
            c.rotate(30, speed=0.3)

except:
    c.speed = 0
    raise