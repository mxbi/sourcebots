from robot import Robot
import rgslib as lib
from rgslib import eagle
import numpy as np
import time
import atexit

start_time = time.time()

def time_since_start():
	return time.time() - start_time

r = Robot()
s = lib.GameState(0)
c = lib.MotionController(r, s)
v = lib.VisionController(r, s)

eagle.EagleThread(s, v).start()

def exit():
	c.speed = 0
atexit.register(exit)

c.barrier_plough()

# 730 = Distance from centre of starting zone to centre of opposite zone
c.move(600, speed=0.99, coast=True)
# c.move_to((80, 300), coast=True)
# # c.move_to((350, 300), coast=True)
# c.rotate(-90)

c.barrier_open()

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
		m = m# [m for m in v.markers_semiblocking() if s.get_marker_type(m) == 'FRIENDLY']
		if len(m) > 0:
			m0 = sorted(m, key=lambda x: x.spherical.distance_metres)[0]
			id = m0.id
			s.box_id = id

			t0 = v.last_marker_time

			def bad_angle_interrupt():
				markers = v.markers
				id_markers = [m for m in markers if m.id == id]
				if len(id_markers) == 0:
					return 0
				else:
					m0 = id_markers[0]
					angle = m0.spherical.rot_y_degrees
					distance = m0.spherical.distance_metres * lib.VISION_DISTANCE_FACTOR
					if np.abs(angle) > np.abs(np.arctan2(20, distance))*(180 / np.pi) + 1 and np.abs(angle) > 5 and v.last_marker_time > t0:
						return 'ANGLE TOO WIDE {}'.format(angle)
					else:
						return 0

			angle = m0.spherical.rot_y_degrees
			distance = m0.spherical.distance_metres * lib.VISION_DISTANCE_FACTOR
			print('Found box, angle/distance:', angle, distance)
			if np.abs(angle) > 5:
					c.rotate(angle, speed=0.25, verbose=1)
			else:
				c.rotate(angle, speed=0.18, verbose=1)
			if not bad_angle_interrupt():
				c.barrier_open()
				if distance < 20:
					time.sleep(0.4) # Wait for barrier to open
				ret = c.move(distance, speed=0.7, interrupts=[bad_angle_interrupt], verbose=1)
				c.barrier_reverse_plough()
			s.box_id = None
	else:
		c.rotate(-30, speed=0.6, verbose=1)
		turns_since_box += 1
		if turns_since_box > TURNS_BEFORE_EXIT:
			print('Rotated 360 without seeing anything, exit.')
			break

c.barrier_close()

# Make sure the gamestate position is updated before moving
v.markers_blocking()
print(s.robot_pos, s.robot_rot)
# intersecting = c.check_destination(s.friendly_zone_middle())
# if intersecting:
# 	print("[Salv] Motion intersecting pillar")
# 	# c.move_to(s.opposite_zone_middle())
# 	# print(s.robot_pos, s.robot_rot)
c.move_to(s.friendly_zone_middle())
print(s.robot_pos, s.robot_rot)

# Make sure we are most definitely 100% where we need to be, for real this time
pos, _ = s.robot_state_blocking()

# Where we should currently be
home_zone_rectangle = s.home_zone_rectangle.shrink(40)

# Oh, no! We're not where we're supposed to be!
while not home_zone_rectangle.is_point_inside(pos):
	c.move_to(s.friendly_zone_middle())
	pos, _ = s.robot_state_blocking()

print("wedidit.jpg")
del v
del c
del s
del r
