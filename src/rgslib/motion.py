import numpy as np
import time
# from robot import COAST  # PLEASE STAY COMMENTED THIS TIME
from collections import defaultdict

from . import trig
from . import nostdout, wait_until, timeout
from . import RE_LATENCY, FAST_MOVE_SPEED, RE_MOVE_OFFSET, RE_PER_CM, VELOCITY_UPDATE_ALPHA, ACTIVE_CORRECTION_ALPHA, \
	RE_PREDICT_TIME, FAST_ROTATE_SPEED, RE_PER_DEGREE, ROTATION_K

COAST = 'coast'

class MotionController:
	def __init__(self, robot, gamestate=None):
		self.r = robot
		self.arduino = self.r.servo_board

		self._update_re()

		self.is_barrier_open = False

		self.gamestate = gamestate
		if gamestate is None:
			print('[MotionController][WARN] No GameState passed to MotionController, functionality will be reduced')

	# On destruction, set speed to 0
	def __del__(self):
		self.r.speed = 0

	# Update the rotary encoder time
	def update_re(self):
		for i in range(5):
			try:
				with timeout(1):
					self.left_re, self.right_re = [int(i) for i in self.arduino.direct_command('r')]
				break
			except KeyboardInterrupt:
				raise
			except Exception as e:
				print('[MotionController][ERROR] Arduino failed to respond to rotary encoder request with "{}", retrying...'.format(e))
		self.re = np.array([self.left_re, self.right_re])
		self.re_time = time.time() - RE_LATENCY  # Estimate actual time measurement was taken

	# Don't hurt me salv
	def update_re_return_ultrasound(self):
		for i in range(5):
			try:
				with timeout(1):
					self.left_re, self.right_re, ultrasound_distance = [int(float(i)) for i in self.arduino.direct_command('u')]
				break
			except KeyboardInterrupt:
				raise
			except Exception as e:
				print('[MotionController][ERROR] Arduino failed to respond to rotary encoder+ultrasound request with "{}", retrying...'.format(e))
#		print(ultrasound_distance)
		self.re = np.array([self.left_re, self.right_re])
		self.re_time = time.time() - RE_LATENCY  # Estimate actual time measurement was taken
		return ultrasound_distance / 10

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
		return self.mleft, self.mright

	@speed.setter  # Dark magic, when "self.speed = 1" is called, update both motors
	def speed(self, speed):
		if hasattr(speed, '__len__') and speed != COAST:
			if len(speed) == 2:  # Allow vectors
				try:
					self.r.motor_board.m0, self.r.motor_board.m1 = speed
				except ValueError as e:
					print('[MotionController][WTF] Motor board complained with {}, asking again nicely'.format(e))
					time.sleep(0.05)
					self.r.motor_board.m0, self.r.motor_board.m1 = speed
			else:
				raise ValueError("Tried setting speed with {} len {} ????".format(speed, len(speed)))
		else:
			#print('Set speed to {}'.format(speed))
			try:
				self.r.motor_board.m0, self.r.motor_board.m1 = speed, speed
			except ValueError as e:
				print('[MotionController][WTF] Motor board complained with {}, asking again nicely'.format(e))
				time.sleep(0.05)
				self.r.motor_board.m0, self.r.motor_board.m1 = speed, speed

	def _aminusb(self, arr):
		return arr[0] - arr[1]

	def barrier_open(self):
		self.arduino.direct_command('servo', 100, 0)
		self.is_barrier_open = True

	def barrier_close(self):
		self.arduino.direct_command('servo', 30, 0)
		self.is_barrier_open = False

	def barrier_plough(self):
		self.arduino.direct_command('servo', 40, 0)
		self.is_barrier_open = False

	def barrier_reverse_plough(self):
		self.arduino.direct_command('servo', 0, 0)
		self.is_barrier_open = False

	def move(self, distance, speed=FAST_MOVE_SPEED, interrupts=[], verbose=1, coast=False, ultrasound_interrupt_distance=None):
		"""Accurately move `distance` cm using rotary encoders. Can be negative

		Parameters:
		distance: cms to move
		speed (default: {FAST_MOVE_SPEED}): Base motor power to move
		interrupts: List of functions to call in a loop while moving - if any of the functions return a non-zero exit code the robot will stop.
		coast (default: False): Ramp down at the end of the movement instead of suddenly braking. Useful when going really fast.
		verbose (default: 1): Verbosity level. 0->No messages, 1->Info messages, 2->All debug messages

		Returns:
		dict: logs
		"""

		initial_pos = self.gamestate.robot_pos if self.gamestate is not None else None
		initial_rot = self.gamestate.robot_rot if self.gamestate is not None else None

		logs = defaultdict(list)
		logs['start_time'] = time.time()

		sign = np.sign(distance)
		distance -= RE_MOVE_OFFSET * sign
		distances = []
		self._update_re()
		initial_re = self.re.copy()
		velocity = None

		message = "[MotionController] Move {} with motor power {}".format(distance, speed * sign)
		logs['debug'].append(message)
		if verbose:
			print(message)

		if np.abs(distance) <= RE_MOVE_OFFSET:
			return logs

		try:
			self.speed = speed * sign

			while True:
				# Save last rotary encoder values for comparison
				old_re = self.re.copy()
				old_re_time = self.re_time

				# Ultrasound interrupts
				broke_ultrasound = 0
				if ultrasound_interrupt_distance:
					while True:
						us = self.update_re_return_ultrasound()
						if broke_ultrasound > 100:
							print('[MotionController.move][WARN] Ultrasound blocking 20 times, exiting early!!'.format(us))
							return
						elif us < ultrasound_interrupt_distance:
							self.speed = 0
							broke_ultrasound += 1
							print('[MotionController.move] Ultrasound detected something with distance {}, pausing'.format(us))
						else:
							if broke_ultrasound:
								self.speed = speed
							break
				else:
					self.update_re()

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

				message = '[MotionController] Distance {}cm Velocity {}cm/s ETA {}s'.format(total_distance, velocity, round(time_remaining, 4))
				logs['debug'].append(message)
				if verbose >= 2:
					print(message)

				# Because "re_time" is corrected for the latency of the re function, this pseudo-works out the time when the movement should actually finish,
				# not when the rotary encoder says the movement should finish (which would be 60ms or so off)
				if RE_PREDICT_TIME > time_remaining >= 0:
					end_time = self.re_time + time_remaining
					break

			wait_until(end_time)
			self.speed = COAST if coast else 0
		except:
			self.speed = COAST if coast else 0  # If something crashes here, turn off the motors so it doesn't attack the wall
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

		message = '[MotionController] Finished - travelled {}cm'.format(distance_travelled)
		logs['debug'].append(message)
		if verbose:
			print(message)

		distance = np.mean(logs['distances'][-1])
		if initial_pos is not None and initial_rot is not None:
			self.gamestate.robot_pos = initial_pos + trig.to_cartesian_degrees(initial_rot, distance)
			self.gamestate.vision_waits = 1

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
		logs = defaultdict(list)

		initial_rot = self.gamestate.robot_rot if self.gamestate is not None else None

		sign = np.sign(angle)  # -1 if negative, 1 if positive, 0 if 0

		angles = []
		self._update_re()
		initial_re = self.re.copy()
		velocity = None

		message = "[MotionController] Rotate {} with motor power {}".format(angle, speed * sign)
		logs['debug'].append(message)
		if verbose:
			print(message)

		if np.abs(angle) < 0.1:
			print("[MotionController][WARN] That was a pretty small angle you just asked me to move")
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

				if verbose >= 2:
					print('[MotionController] Rotation {}deg Velocity {}deg/s ETA {}s'.format(angle_travelled, velocity, round(time_remaining, 4)))

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

		print('[MotionController] Finished - travelled {}deg'.format(angle_travelled))

		# Rotation is anticlockwise whereas angle is clockwise - so subtract
		if initial_rot is not None:
			self.gamestate.robot_rot = initial_rot - angles[-1]
			self.gamestate.vision_waits = 1

		return angles

	# Warning: does not handle pillars
	def move_to(self, target_pos, rotate_speed=0.25, move_speed=FAST_MOVE_SPEED, coast=False):
		epsilon_angle = 3

		# Where we currently are
		current_pos, current_rot = self.gamestate.robot_state_blocking()

		# How we need to move
		motion = target_pos - current_pos

		# Which way we need to be facing
		target_rot, distance = trig.to_polar_degrees(motion)

		while True:
			# How much we need to turn to face that direction
			angle = trig.normalise_angle_degrees(target_rot - current_rot)

			if np.abs(angle) < epsilon_angle:
				break

			print(target_rot, current_rot, target_rot - current_rot, angle)
			self.rotate(-angle, speed=rotate_speed)
			update_time = self.gamestate.last_pos_update_time
			_, current_rot = self.gamestate.robot_state_blocking()
			#if self.gamestate.last_pos_update_time == update_time:
			#	print('[GameState.move_to] Could not see markers, moving 90 degrees and retrying')
			#	# self.move(-10, speed=-0.1)
			#	self.rotate(90, speed=rotate_speed)
			#	_, current_rot = self.gamestate.robot_state_blocking()

		# angle is anticlockwise, rotate accepts clockwise, so -
		self.move(distance, speed=move_speed, coast=coast)

	def check_destination(self, target_pos, rotate_speed=0.25, move_speed=FAST_MOVE_SPEED, coast=False, verbose=1):
		target_pos = np.array(target_pos)

		# Where we currently are
		current_pos, current_rot = self.gamestate.robot_state_blocking()

		movement_line = (current_pos, target_pos)
		bad_zones = [zone for zone in self.gamestate.zones_to_avoid if zone.is_crossed_by(movement_line)]

		return bool(bad_zones)

	# # Warning: does not handle pillars
	def move_to(self, target_pos, rotate_speed=0.25, move_speed=FAST_MOVE_SPEED, coast=False, verbose=1):
		"""Move to precise co-ordinates. Will dynamically avoid obstacles

		Parameters:
		target_pos: Co-ordinates to move to
		rotate_speed (default: 0.25): Base motor power to use when rotating
		move_speed (default: {FAST_MOVE_SPEED}): Base motor power to use when moving
		coast (default: True): Whether to slowly stop at the end - note this leads to overshot distances but avoids sudden breaking. Passed through to move()
		verbose (default: 1): Verbosity level. 0->No messages, 1->Info messages, 2->All debug messages

		Returns:
		dict: logs
		"""
		target_pos = np.array(target_pos)
		epsilon_angle = 3 # TODO: Improve this angle, maybe using atan2 to figure out acceptable distance

		message = "[MotionController.move_to] Going to {} with speeds {}/{}".format(target_pos, move_speed, rotate_speed)
		if verbose:
			print(message)

		# Where we currently are
		current_pos, current_rot = self.gamestate.robot_state_blocking()

		wall_eps = 25 # Prevent moving within 25cm of a wall using this method
		bad_end = any(pillar.is_point_inside(target_pos) for pillar in self.gamestate.pillars) or any(c > (800 - wall_eps) or c < (wall_eps) for c in target_pos)

		if bad_end:
			print('[MotionController.move_to][ERROR] Illegal position {} requested in move_to(), ignoring.'.format(target_pos))
			pass

		# Whether we start inside a pillar
		bad_start = any(pillar.is_point_inside(current_pos) for pillar in self.gamestate.pillars)

		movement_line = (current_pos, target_pos)
		bad_zones = [zone for zone in self.gamestate.zones_to_avoid if zone.is_crossed_by(movement_line)]

		# If we never hit any pillars
		if not bad_zones or bad_start:
			if bad_start:
				print('[MotionController.move_to][WARN] Starting inside a pillar, continuing as normal.')

			# How we need to move
			motion = target_pos - current_pos

			# Which way we need to be facing
			target_rot, distance = trig.to_polar_degrees(motion)

			while True:
				# How much we need to turn to face that direction
				angle = trig.normalise_angle_degrees(target_rot - current_rot)

				# Keep turning until we are within a certain range of the target
				if np.abs(angle) < epsilon_angle:
					break

				# angle is anticlockwise, rotate accepts clockwise, so -
				self.rotate(-angle, speed=rotate_speed)

				# Update our rotation if we see any more markers
				_, current_rot = self.gamestate.robot_state_blocking()

			self.move(distance, speed=move_speed)
		else:
			first_bad_zone = bad_zones[0]
			alternative_route = first_bad_zone.alternative_route(movement_line)

			if verbose:
				print('[MotionController.move_to] No direct path in move_to(), taking detour:', alternative_route)
			for start, end in alternative_route:
				# These may still go through other pillars (though unlikely), but we've stopped it going through at least
				# one pillar and we'll handle each of the other pillars recursively
				self.move_to(end, rotate_speed, move_speed, coast=coast, verbose=verbose)

		message = "[MotionController.move_to] Finished moving to {}, estimated position {}".format(target_pos, self.gamestate.robot_pos)
		if verbose:
			print(message)
