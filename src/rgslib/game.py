import numpy as np
import itertools

from . import VISION_DISTANCE_FACTOR

class GameState:
	"""GameState calculates and stores estimates locations of all objects of interest in the game as well as localising the robot itself."""
	def __init__(self, friendly_zone):
		from robot import WALL, COLUMN, TOKEN_ZONE_0, TOKEN_ZONE_1, TOKEN_ZONE_2, TOKEN_ZONE_3
		self.marker_id_mapping = {'WALL': WALL, 'COLUMN': COLUMN, 'TOKEN_ZONE_0': TOKEN_ZONE_0, 'TOKEN_ZONE_1': TOKEN_ZONE_1, 'TOKEN_ZONE_2': TOKEN_ZONE_2, 'TOKEN_ZONE_3': TOKEN_ZONE_3}
		self._init_wall_positions()
		self.robot_pos = [None, None]
		self.robot_rot = None

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

	def report_vision_markers(self, markers):
		useful_markers = [m for m in markers if self.get_marker_type(m) in ['WALL', 'COLUMN']]
		if len(useful_markers) == 0:
			print('[GameState] No anchored markers')
			return
		elif len(useful_markers) == 1:
			print('[GameState] Only one anchored marker')
		else:
			combinations = itertools.combinations(useful_markers, 2)
			positions = []
			rotations = []
			for stuff in combinations:
				pos, rot = self._robot_position(*stuff)
				positions.append(pos)
				rotations.append(rot)
			pos = np.mean(positions, axis=0)
			rot = np.mean(rotations, axis=0)
			print('[GameState] Found {} markers, determined position {} rotation {}'.format(len(useful_markers), pos, rot))
			self.robot_pos = pos
			self.robot_rot = rot
			return pos, rot

	# Given two markers, calculates the position of the robot
	def _robot_position(self, m0, m1):
		# Co-ordinates of each wall marker
		x0, y0 = self.wall_positions[m0.id]
		x1, y1 = self.wall_positions[m1.id]

		# theta is anticlockwise whereas rot_y is clockwise, hence the negative
		theta0 = -m0.spherical.rot_y_radians
		theta1 = -m1.spherical.rot_y_radians

		# Distances between robot and each wall marker, in cm
		r0 = m0.distance_metres * VISION_DISTANCE_FACTOR
		r1 = m1.distance_metres * VISION_DISTANCE_FACTOR

		# Converting to complex numbers makes certain operations easier, in particular rotations
		z0 = x0 + y0*1j
		z1 = x1 + y1*1j

		# phi is the angle between the wall markers
		phi = theta1 - theta0
		# print(phi, theta1, theta0, np.sin(phi), np.abs(z1 - z0))

		# Distance between m0 and m1
		marker_distance = np.abs(z1 - z0)
		# The line that passes through m0 and m1 makes an angle of alpha with the horizontal
		# Calculation using sine rule (uses inaccurate angle)
		alpha_sin = np.arcsin(np.clip((r1 * np.sin(phi)) / marker_distance, -1.0, 1.0))
		# Robust calculation using cosine rule and only distances
		cosalpha = (r0 ** 2 + marker_distance ** 2 - r1 ** 2) / (2 * r0 * marker_distance)
		if np.abs(cosalpha) > 1:
			print('[GameState][WARN] Calculated cosalpha out of bounds, clipping')
		alpha = np.arccos(np.clip(cosalpha, -1.0, 1.0))

		# Select +/- alpha depending on which one is nearest to (inaccurate) sine rule. This finds the correct solution to the cosine rule (as it has two solutions)
		alphas = np.array([alpha, -alpha])
		alpha = alphas[np.argmin(np.abs(alphas - alpha_sin))]

		# The position of the robot as a complex number
		z = z0 + r0 * np.exp(alpha * 1j) * (z1 - z0) / marker_distance

		# The position of the robot as a numpy array
		pos = np.array([z.real, z.imag])

		# The angle the robot is facing measured anticlockwise from the horizontal
		theta = np.angle(z0 - z) - theta0
		theta_degrees = theta * (180 / np.pi)

		return pos, theta_degrees
