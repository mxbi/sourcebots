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
RE_PER_DEGREE = 8.02  # The difference in rotary encoder readings necessary to turn a degree. TODO: Find better value
RE_ROTATE_OFFSET = 3  # The amount, in degrees, by which the robot tends to overshoot

FAST_MOVE_SPEED = 1
FAST_ROTATE_SPEED = 0.5


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


class RobotController:
	def __init__(self, robot):
		self.r = robot
		self.arduino = self.r.servo_board

		self.reset_state()
		self._update_re()

	def _update_re(self):
		t0 = time.time()
		# print(t0)
		with nostdout():
			# if 1:
			self.left_re, self.right_re = [int(i) for i in self.arduino.direct_command('r')]
		self.re = np.array([self.left_re, self.right_re])
		# print(self.re)
		# print(id(self.re))
		self.re_time = time.time() - RE_LATENCY  # Estimate actual time measurement was taken

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

	def move(self, distance, speed=FAST_MOVE_SPEED):
		sign = np.sign(distance)

		"Accurately move `distance` cm using rotary encoders. Can be negative"
		# TODO: Actively correct for drift during movement
		distance -= RE_MOVE_OFFSET * sign
		distances = []
		self._update_re()
		initial_re = self.re.copy()
		initial_time = self.re_time

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
			velocity = (self.re - old_re) / RE_PER_CM / re_time_delta

			alpha = 0.05
			# The 'sign' factor is there so that instead of speeding up the slower wheel,
			# the faster wheel will slow down when distance < 0
			v = speed - sign * alpha * np.maximum(0, sign * (total_distance - total_distance[::-1]))
			print(v)
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

		angle -= RE_ROTATE_OFFSET * sign
		angles = []
		self._update_re()
		initial_re = self.re.copy()
		initial_time = self.re_time

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
			velocity = self._aminusb(self.re - old_re) / RE_PER_DEGREE / re_time_delta

			time_remaining = (angle - angle_travelled) / velocity

			print('[RobotController] Rotation {}deg Velocity {}deg/s ETA {}s'.format(angle_travelled, velocity, round(time_remaining, 4)))

			if RE_PREDICT_TIME > time_remaining >= 0:
				end_time = self.re_time + time_remaining
				break

		wait_until(end_time)
		self.speed = 0

		# Temporary
		# TODO: Exit when it actually stops moving (when velocity goes to 0)
		for i in range(10):
			t0 = time.time()
			self._update_re()
			# total_distance = (self.re - initial_re) / RE_PER_CM
			# distances.append(total_distance)
			print(time.time() - t0)
			print('[RobotController] Finished - travelled {}deg'.format(self._aminusb(self.re - initial_re) / RE_PER_DEGREE))

		self.rot += self._aminusb(self.re - initial_re) / RE_PER_DEGREE

		return angles
