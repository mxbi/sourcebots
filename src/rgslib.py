import threading

import numpy as np
import time
import contextlib
import sys
import os

# Define units
ms = 0.001

# Define constants
RE_PER_CM = 11.13  # Rotary encodes per cm moved - ~1% error
RE_LATENCY = 60 * ms  # Rotary encoder function lag in milliseconds - currently pulled out of my ass
RE_PREDICT_TIME = 200 * ms  # Stop checking rotary encoder and use the current velocity to predict when this many seconds remain
RE_MOVE_OFFSET = 8  # Breaking distance to always subtract in cm

# Angle callibration

# The difference in rotary encoder readings necessary to turn a degree. TODO: Find better value
RE_PER_DEGREE = 8.50
# Experimentally determined value which determines how much of an effect angular velocity has on when we need to stop. TODO: Find better value
# Unit is seconds
ROTATION_K = 0.14

FAST_MOVE_SPEED = 1
FAST_ROTATE_SPEED = 0.5

VELOCITY_UPDATE_ALPHA = 0.25 # Update rate for velocity (v1 = alpha * d(RE)/dt + (1 -alpha) * v0)

# Use 'with nostdout():' to silence output.
@contextlib.contextmanager
def nostdout():
	with open(os.devnull, "w") as devnull:
		old_stdout = sys.stdout
		sys.stdout = devnull
		try:
			yield
		finally:
			sys.stdout = old_stdout


def wait_until(t):
	"Wait until `t` in unix epoch time"
	time.sleep(max(0, t - time.time()))

class MotionController:
	def __init__(self, robot):
		self.r = robot
		self.arduino = self.r.servo_board

		self.reset_state()
		self._update_re()

	# On destruction, set speed to 0
	def __del__(self, robot):
		self.speed = 0

	# Update the rotary encoder time
	def update_re(self):
		t0 = time.time()
		with nostdout():
			self.left_re, self.right_re = [int(i) for i in self.arduino.direct_command('r')]
		self.re = np.array([self.left_re, self.right_re])
		self.re_time = time.time() - RE_LATENCY  # Estimate actual time measurement was taken

	# Backwards-compatibility
	def _update_re(self):
		self.update_re()

	@property
	def mleft(self):
		return self.r.motor_board.m0

	@mleft.setter
	def mleft(self, val):
		self.r.motor_board.m0 = val

	@property
	def mright(self):
		return self.r.motor_board.m1

	@mright.setter
	def mright(self, val):
		self.r.motor_board.m1 = val

	@property
	def speed(self):
		return (self.mleft, self.mright)

	@speed.setter  # Dark magic, when "self.speed = 1" is called, update both motors
	def speed(self, speed):
		t0 = time.time()
		self.r.motor_board.m0, self.r.motor_board.m1 = speed, speed

	def _aminusb(self, arr):
		return arr[0] - arr[1]

	def reset_state(self):
		self.pos = np.zeros(2)
		self.rot = np.float64(0)

	def cos(self, v):
		return np.cos(v * (np.pi / 180))

	def sin(self, v):
		return np.sin(v * (np.pi / 180))

	def move(self, distance, speed=FAST_MOVE_SPEED, interrupts=[]):
		"""Accurately move `distance` cm using rotary encoders. Can be negative"""

		sign = np.sign(distance)
		distance -= RE_MOVE_OFFSET * sign
		distances = []
		self._update_re()
		initial_re = self.re.copy()
		initial_time = self.re_time
		velocity = None

		try:
			self.speed = speed * sign

			vs = []
			while True:
				# Save last rotary encoder values for comparison
				old_re = self.re.copy()
				old_re_time = self.re_time

				self._update_re()

				# Note this is a vector because re/old_re is a vector
				total_distance = (self.re - initial_re) / RE_PER_CM
				distances.append(total_distance)

				re_time_delta = self.re_time - old_re_time

				# Update the velocity with some weight to get a smoother more accurate measurement
				# Instead of resetting the velocity every time
				v = (self.re - old_re) / RE_PER_CM / re_time_delta
				if velocity is None:
					velocity = v
				velocity = VELOCITY_UPDATE_ALPHA * v + (1 - VELOCITY_UPDATE_ALPHA) * velocity

				breaking = 0
				for ifunc in interrupts:
					ret = ifunc()
					if ret != 0:
						print('INTERRUPT', ifunc, ret)
						breaking = 1
						end_time = time.time()
						break
				if breaking == 1: break

				alpha = 0.05
				# The 'sign' factor is there so that instead of speeding up the slower wheel,
				# the faster wheel will slow down when distance < 0
				v = speed - sign * alpha * np.maximum(0, sign * (total_distance - total_distance[::-1]))
				vs.append(v)
				self.mleft = v[0]
				self.mright = v[1]

				time_remaining = (distance - total_distance.mean()) / velocity.mean()

				print('[RobotController] Distance {}cm Velocity {}cm/s ETA {}s'.format(total_distance, velocity, round(time_remaining, 4)))

				# Because "re_time" is corrected for the latency of the re function, this pseudo-works out the time when the movement should actually finish,
				# not when the rotary encoder says the movement should finish (which would be 60ms or so off)
				if RE_PREDICT_TIME > time_remaining >= 0:
					end_time = self.re_time + time_remaining
					break

			wait_until(end_time)
			self.speed = 0
		except:
			self.speed = 0 # If something crashes here, turn off the motors so it doesn't attack the wall
			raise

		t0 = time.time()

		# Temporary
		# TODO: Exit when it actually stops moving (when velocity goes to 0)
		for i in range(10):
			self._update_re()
			total_distance = (self.re - initial_re) / RE_PER_CM
			distances.append(total_distance)
			print(time.time() - t0)
			print('[RobotController] Finished - travelled {}cm'.format((self.re - initial_re) / RE_PER_CM))

		distance = np.mean(distances[-1])
		self.pos[0] += distance * self.sin(self.rot)
		self.pos[1] += distance * self.cos(self.rot)

		return distances, vs

	# Turns the angle in degrees, where clockwise is positive and anticlockwise is negative
	def rotate(self, angle, speed=FAST_ROTATE_SPEED):
		sign = np.sign(angle)  # -1 if negative, 1 if positive, 0 if 0

		angles = []
		self._update_re()
		initial_re = self.re.copy()
		initial_time = self.re_time
		velocity = None

		try:
			self.mleft = speed * sign
			self.mright = -speed * sign
			t0 = time.time()

			while True:
				# Save last rotary encoder values for comparison
				old_re = self.re.copy()
				old_re_time = self.re_time

				self._update_re()

				# Note this is a vector because re/old_re is a vector
				angle_travelled = self._aminusb(self.re - initial_re) / RE_PER_DEGREE
				angles.append(angle_travelled)

				re_time_delta = self.re_time - old_re_time
				# Angular velocity in degrees per second
				v = self._aminusb(self.re - old_re) / RE_PER_DEGREE / re_time_delta
				if velocity is None:
					velocity = v
				velocity = VELOCITY_UPDATE_ALPHA * v + (1 - VELOCITY_UPDATE_ALPHA) * velocity

				# If we were to start braking now, what angle would we have travelled?
				final_angle = angle_travelled + ROTATION_K * velocity
				# How much less is that than the angle we're aiming for?
				undershoot = angle - final_angle

				# How much extra time is necessary to travel this amount?
				time_remaining = undershoot / velocity

				print('[RobotController] Rotation {}deg Velocity {}deg/s ETA {}s'.format(angle_travelled, velocity, round(time_remaining, 4)))

				# When the time remaining is small enough, stop checking rotary encoders and just wait out this extra time
				# If this time is negative, either this can be for two reasons:
				#  - The sign of velocity is wrong - the robot is moving the wrong way (should never happen)
				#  - The robot will overshoot instead of undershoot if it brakes now, so we should stop ASAP to prevent overshooting by too much
				# For now, assume it's the second case and stop if time_remaining is negative
				if time_remaining < RE_PREDICT_TIME and time.time() - t0 > 0.1:
					if np.sign(velocity) != sign: # Turning the wrong way!
						# Log so if this ever happens, it'll be easier to debug
						print('[RobotController] Warning: Turning the wrong direction!')
					end_time = time.time() + time_remaining
					break

			# Wait the extra time to minimise inaccuracy, if necessary
			wait_until(end_time)
			self.speed = 0
		except:
			self.speed = 0
			raise

		# Temporary
		# TODO: Exit when it actually stops moving (when velocity goes to 0)
		for i in range(10):
			t0 = time.time()
			self._update_re()
			# total_distance = (self.re - initial_re) / RE_PER_CM
			# distances.append(total_distance)
			print('[RobotController] Finished - travelled {}deg'.format(self._aminusb(self.re - initial_re) / RE_PER_DEGREE))

		self.rot += self._aminusb(self.re - initial_re) / RE_PER_DEGREE

		return angles


class VisionController:
	# Gets passed the Robot() instance so it can use the cameras
	def __init__(self, robot):
		self.r = robot
		self.camera = robot.camera

		# Can't see any markers initially
		self.markers = []

		# The number of times the markers have been updated my the marker thread.
		# Useful for blocking
		self.marker_update_count = 0
		self.last_marker_time = 0
		self.last_marker_duration = 0

		# Start thread which runs forever
		self.thread = MarkerThread(self).start()
		# TODO: Gracefully stop this thread on user request or object destruction

	def markers_semiblocking(self, time_threshold=None):
		"""Wait until the current see() call has finished and return that. Note that the photo may have been taken before markers_semiblocking was called.
		Optionally, time_threshold (unix epoch) can be passed. This will return the first processed frame that was taken after that time."""
		old_marker_count = self.marker_update_count

		if time_threshold:
			while self.last_marker_time < time_threshold:
				time.sleep(0.001)

		else:
			while self.marker_update_count < old_marker_count + 1:
				time.sleep(0.001)

		return self.markers

	def markers_blocking(self):
		"""Wait until the next see() call has finished and return that."""
		old_marker_count = self.marker_update_count
		# Wait for markers to be updated twice since calling the function

		while self.marker_update_count < old_marker_count + 2:
			time.sleep(0.001) # Microsleep while blocking to prevent 100% cpu usage

		return self.markers


# Thread which continuously updates the vision controller's 'markers' field with the latest reading of markers
class MarkerThread(threading.Thread):
	def __init__(self, vision_controller):
		self.vision_controller = vision_controller
		threading.Thread.__init__(self)

	def run(self):
		vision_controller = self.vision_controller
		camera = vision_controller.camera
		while True:
			t0 = time.time()

			vision_controller.markers = camera.see()

			vision_controller.marker_update_count += 1
			vision_controller.last_marker_time = time.time()
			vision_controller.last_marker_duration = time.time() - t0

class GameState:
	"""GameState calculates and stores estimates locations of all objects of interest in the game as well as localising the robot itself."""
	def __init__(self, friendly_zone):
		from robot import WALL, COLUMN, TOKEN_ZONE_0, TOKEN_ZONE_1, TOKEN_ZONE_2, TOKEN_ZONE_3
		self.marker_id_mapping = {'WALL': WALL, 'COLUMN': COLUMN, 'TOKEN_ZONE_0': TOKEN_ZONE_0, 'TOKEN_ZONE_1': TOKEN_ZONE_1, 'TOKEN_ZONE_2': TOKEN_ZONE_2, 'TOKEN_ZONE_3': TOKEN_ZONE_3}
		self._init_wall_positions()
		self.robot_pos = [None, None]

		assert friendly_zone in [0, 1, 2, 3]
		self.friendly_zone = friendly_zone

		# self._init_marker_types()

	def _init_wall_positions(self):
		# (x, y) of all the wall markers - see https://docs.sourcebots.co.uk/rules.pdf
		self.wall_positions = {}
		# Bottom wall of markers
		for marker, x in [(20, 1), (19, 2), (18, 3), (17, 4), (16, 5), (15, 6), (16, 7)]:
			self.wall_positions[marker] = (x*100, 0)
		# Top wall of markers
		for marker, x in [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7)]:
			self.wall_positions[marker] = (x*100, 800)
		# Right wall of markers
		for marker, y in zip([13, 12, 11, 10, 9, 8], [1, 2, 3, 4, 5, 6, 7]):
			self.wall_positions[marker] = (800, y*100)
		for marker, y in zip([21, 22, 23, 24, 25, 26, 27], [1, 2, 3, 4, 5, 6, 7]):
			self.wall_positions[marker] = (0, y*100)

	def get_marker_type(self, marker):
		friendly_marker = self.marker_id_mapping['TOKEN_ZONE_{}'.format(self.friendly_zone)]
		if marker.id in self.marker_id_mapping['WALL']:
			return 'WALL'
		elif marker.id in self.marker_id_mapping['COLUMN']:
			return 'COLUMN'
		elif marker.id in friendly_marker:
			return 'FRIENDLY'
		else:
			return 'ENEMY'

	def _update_robot_position(self, position):
		# Kalman Filter?
		raise NotImplementedError

	def report_vision_marker(self, marker):
		if marker in self.wall_positions:
			raise NotImplementedError
		raise NotImplementedError
