import numpy as np
import time
import contextlib
import threading
import sys
import os
import itertools
from collections import defaultdict

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

VELOCITY_UPDATE_ALPHA = 0.5 # Update rate for velocity (v1 = alpha * d(RE)/dt + (1 -alpha) * v0
ACTIVE_CORRECTION_ALPHA = 0.05 # Intensity of active correction

VISION_DISTANCE_FACTOR = 0.88 * 100 # Actual cms per sourcebots-metre

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

def handle_arduino_bullshit():
	raise NotImplementedError

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
		if hasattr(speed, '__len__'):
			if len(speed) == 2: # Allow vectors
				self.r.motor_board.m0, self.r.motor_board.m1 = speed
			else:
				raise ValueError("Tried setting speed with {} len {} ????".format(speed, len(speed)))
		else:
			self.r.motor_board.m0, self.r.motor_board.m1 = speed, speed

	def _aminusb(self, arr):
		return arr[0] - arr[1]

	def reset_state(self):
		self.pos = np.zeros(2)
		# rot is degrees anticlockwise from the positive x axis
		self.rot = np.float64(0)

	def cos(self, v):
		return np.cos(v * (np.pi / 180))

	def sin(self, v):
		return np.sin(v * (np.pi / 180))

	# Normalises an angle such that it is in the range (-180, 180].
	# This means, for example:
	#  - normalise_angle(270) = -90
	#  - normalise_angle(450) = 90
	#  - normalise_angle(180) = 180
	def normalise_angle(theta):
		return 180 - ((180 - theta) % 360)

	def move(self, distance, speed=FAST_MOVE_SPEED, interrupts=[], verbose=1):
		"""Accurately move `distance` cm using rotary encoders. Can be negative

		Parameters:
		distance: cms to move
		speed (default: {FAST_MOVE_SPEED}): Base motor power to move
		interrupts: List of functions to call in a loop while moving - if any of the functions return a non-zero exit code the robot will stop.
		verbose (default: 1): Verbosity level. 0->No messages, 1->Info messages, 2->All debug messages

		Returns:
		dict: logs
		"""

		logs = defaultdict(list)
		logs['start_time'] = time.time()

		sign = np.sign(distance)
		distance -= RE_MOVE_OFFSET * sign
		distances = []
		self._update_re()
		initial_re = self.re.copy()
		velocity = None

		message = "[RobotController] Move {} with motor power {}".format(distance, speed * sign)
		logs['debug'].append(message)
		if verbose:
			print(message)

		if distance < 0.1:
			return logs

		try:
			self.speed = speed * sign

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
				if breaking == 1:
					break

				# The 'sign' factor is there so that instead of speeding up the slower wheel,
				# the faster wheel will slow down when distance < 0
				power = speed - sign * ACTIVE_CORRECTION_ALPHA * np.maximum(0, sign * (total_distance - total_distance[::-1]))
				self.speed = power

				time_remaining = (distance - total_distance.mean()) / velocity.mean()

				message = '[RobotController] Distance {}cm Velocity {}cm/s ETA {}s'.format(total_distance, velocity, round(time_remaining, 4))
				logs['debug'].append(message)
				if verbose >= 2:
					print(message)

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

		escape_velocity = 1
		distance_travelled = None
		while np.mean(v) > escape_velocity or distance_travelled is None:
			old_re = self.re.copy()
			old_re_time = self.re_time
			self._update_re()

			distance_travelled = (self.re - initial_re) / RE_PER_CM
			logs['distances'].append(distance_travelled)

			re_time_delta = self.re_time - old_re_time
			v = (self.re - old_re) / RE_PER_CM / re_time_delta

		message = '[RobotController] Finished - travelled {}cm'.format(distance_travelled)
		logs['debug'].append(message)
		if verbose:
			print(message)

		distance = np.mean(logs['distances'][-1])
		self.pos[0] += distance * self.cos(self.rot)
		self.pos[1] += distance * self.sin(self.rot)

		return logs

	# Turns the angle in degrees, where clockwise is positive and anticlockwise is negative
	def rotate(self, angle, speed=FAST_ROTATE_SPEED, verbose=1):
		"""Accurately rotate `angle` degrees using rotary encoders. Can be negative

		Parameters:
		angle: angle to turn in degrees
		speed (default: {FAST_ROTATE_SPEED}): Base motor power to use
		verbose (default: 1): Verbosity level. 0->No messages, 1->Info messages, 2->All debug messages

		Returns:
		dict: logs
		"""
		sign = np.sign(angle)  # -1 if negative, 1 if positive, 0 if 0

		angles = []
		self._update_re()
		initial_re = self.re.copy()
		velocity = None

		if angle < 0.1:
			return

		try:
			self.mleft = speed * sign
			self.mright = -speed * sign

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
				# If this time is negative, this can be for two reasons:
				#  - The sign of velocity is wrong - the robot is moving the wrong way temporarily
				#  - The robot will overshoot instead of undershoot if it brakes now, so we should stop ASAP to prevent overshooting by too much
				# If it's the first case, we can just repeat the while loop again until the robot's velocity is corrected.
				# If it's the second case, and the sign of the velocity is correct, we should stop
				if time_remaining < RE_PREDICT_TIME and np.sign(velocity) == sign:
					end_time = time.time() + time_remaining
					break

			# Wait the extra time to minimise inaccuracy, if necessary
			wait_until(end_time)
			self.speed = 0
		except:
			self.speed = 0
			raise

		# Wait until we stop moving to get a good position estimate
		escape_velocity = 1
		angle_travelled = None
		while v > escape_velocity or angle_travelled is None:
			old_re = self.re.copy()
			old_re_time = self.re_time
			self._update_re()

			re_time_delta = self.re_time - old_re_time
			v = self._aminusb(self.re - old_re) / RE_PER_DEGREE / re_time_delta
			angle_travelled = self._aminusb(self.re - initial_re) / RE_PER_DEGREE
			angles.append(angle_travelled)

		print('[RobotController] Finished - travelled {}deg'.format(angle_travelled))

		# Rotation is anticlockwise whereas angle is clockwise - so subtract
		self.rot -= angles[-1]

		return angles


class VisionController:
	# Gets passed the Robot() instance so it can use the cameras
	def __init__(self, robot, gamestate=None):
		self.r = robot
		self.camera = robot.camera
		self.gamestate = gamestate
		if self.gamestate is None:
			print('[VisionController][WARN] No GameState passed to VisionController, functionality will be reduced')

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
			if vision_controller.gamestate is not None:
				vision_controller.gamestate.report_vision_markers(vision_controller.markers)

			vision_controller.marker_update_count += 1
			vision_controller.last_marker_time = time.time()
			vision_controller.last_marker_duration = time.time() - t0
