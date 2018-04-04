from robot import Robot
import rgslib as lib
import numpy as np
import time
import atexit

start_time = time.time()

def time_since_start():
	return time.time() - start_time


r = Robot()
s = lib.GameState(2)
c = lib.MotionController(r, s)
v = lib.VisionController(r, s)

def exit():
	c.speed = 0
atexit.register(exit)

c.close_barrier()

# 730 = Distance from centre of starting zone to centre of opposite zone
c.move(500, speed=0.99, coast=True)

c.open_barrier()

# Collect boxes until time since start exceeds certain value, or we can no longer see any boxes after full rotation

print('GO!')
TURN_ANGLE = 30
TURNS_BEFORE_EXIT = 360 / TURN_ANGLE

turns_since_box = 0
while True:
	if (time.time() - start_time) > 120:
		print('Exiting at 2 mins')
		break
	t0 = time.time()
	m = [m for m in v.markers_blocking() if s.get_marker_type(m) == 'FRIENDLY']
	print(time.time() - t0)
	if m:
		turns_since_box = 0
		m = [m for m in v.markers_semiblocking() if s.get_marker_type(m) == 'FRIENDLY']
		if len(m) > 0:
			m0 = sorted(m, key=lambda x: x.spherical.distance_metres)[0]
			id = m0.id

			t0 = v.last_marker_time

			def bad_angle_interrupt():
				markers = v.markers
				id_markers = [m for m in markers if m.id == id]
				if len(id_markers) == 0:
					return 0
				else:
					m0 = id_markers[0]
					angle = m0.spherical.rot_y_degrees
					distance = m0.spherical.distance_metres * 100
					if np.abs(angle) > np.abs(np.arctan2(20, distance))*(180 / np.pi) + 1 and np.abs(angle) > 5 and v.last_marker_time > t0:
						return 'ANGLE TOO WIDE {}'.format(angle)
					else:
						return 0

			angle = m0.spherical.rot_y_degrees
			distance = m0.spherical.distance_metres * 0.881 * 100
			print('Found box, angle/distance:', angle, distance)
			if np.abs(angle) > 5:
				c.rotate(angle, speed=0.25)
			else:
				c.rotate(angle, speed=0.15)
			if not bad_angle_interrupt():
				c.move(distance + 60, speed=0.6, interrupts=[bad_angle_interrupt])
	else:
		c.rotate(30)
		turns_since_box += 1
		if turns_since_box > TURNS_BEFORE_EXIT:
			print('Rotated 360 without seeing anything, exit.')
			break

c.close_barrier()


# Make sure the gamestate position is updated before moving
v.markers_blocking()
print(s.robot_pos, s.robot_rot)
c.move_to(s.friendly_zone_middle())
print(s.robot_pos, s.robot_rot)

print("wedidit.jpg")
del v
del c
del s
del r