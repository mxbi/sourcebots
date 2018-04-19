import numpy as np
import itertools
import time

from . import trig
from . import util
from . import VISION_DISTANCE_FACTOR


class GameState:
	"""GameState calculates and stores estimates locations of all objects of interest in the game as well as localising the robot itself."""
	def __init__(self, friendly_zone):
		try:
			from robot import WALL, COLUMN, TOKEN_ZONE_0, TOKEN_ZONE_1, TOKEN_ZONE_2, TOKEN_ZONE_3
		except ImportError:
			from .game_specific import WALL, COLUMN, TOKEN_ZONE_0, TOKEN_ZONE_1, TOKEN_ZONE_2, TOKEN_ZONE_3
		self.marker_id_mapping = {'WALL': WALL, 'COLUMN': COLUMN, 'TOKEN_ZONE_0': TOKEN_ZONE_0, 'TOKEN_ZONE_1': TOKEN_ZONE_1, 'TOKEN_ZONE_2': TOKEN_ZONE_2, 'TOKEN_ZONE_3': TOKEN_ZONE_3}
		self._init_wall_positions()
		self.robot_pos = [None, None]
		self.robot_rot = None
		self.last_pos_update_time = 0

		assert friendly_zone in [0, 1, 2, 3]
		self.friendly_zone = friendly_zone

		self.vision_updates = 0
		self.box_id = None

		self.init_time = time.time()
		self.vision_waits = 0

		self.pillars = [
			util.Rectangle(381, 419, 181, 219),
			util.Rectangle(181, 219, 381, 419),
			util.Rectangle(381, 419, 581, 619),
			util.Rectangle(581, 619, 381, 419),
		]

		self.home_zone_rectangle_dict = {
			0: util.Rectangle(418.5, 700, 100, 381.5),
			1: util.Rectangle(418.5, 700, 418.5, 700),
			2: util.Rectangle(100, 381.5, 418.5, 700),
			3: util.Rectangle(100, 381.5, 100, 381.5),
		}

		# friendly_zone id vs centre of zone
		self.zone_dict = [
			zone.centre() for _, zone in self.home_zone_rectangle_dict
		]

		self.home_zone_rectangle = self.home_zone_rectangle_dict[friendly_zone]

		# Minimum distance to avoid the edges of pillars by, in cm
		# Should be roughly equal to width of robot so if center avoids expanded zone, then the sides will avoid the pillars
		min_edge_distance = 20
		self.zones_to_avoid = [pillar.expand(min_edge_distance) for pillar in self.pillars]

		# Edges of zones we want to avoid
		self.edges_to_avoid = [edge for pillar in self.pillars for edge in pillar.edges]

	def _init_wall_positions(self):
		# (x, y) of all the wall markers - see https://docs.sourcebots.co.uk/rules.pdf
		self.wall_positions = {}
		# Bottom wall of markers
		for marker, x in [(20, 1), (19, 2), (18, 3), (17, 4), (16, 5), (15, 6), (14, 7)]:
			self.wall_positions[marker] = (x*100, 0)
		# Top wall of markers
		for marker, x in [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7)]:
			self.wall_positions[marker] = (x*100, 800)
		# Right wall of markers
		for marker, y in zip([13, 12, 11, 10, 9, 8, 7], [1, 2, 3, 4, 5, 6, 7]):
			self.wall_positions[marker] = (800, y*100)
		for marker, y in zip([21, 22, 23, 24, 25, 26, 27], [1, 2, 3, 4, 5, 6, 7]):
			self.wall_positions[marker] = (0, y*100)

		# TODO: Set the correct ones for the actual competition
		column_positions = [(400, 619), (419, 600), (400, 582), (382, 600), (600, 419), (619, 400), (600, 382), (582, 400), (400, 219), (419, 200), (400, 182), (382, 200), (200, 419), (219, 400), (200, 382), (182, 400)]
#		column_positions = [(400, 613), (413, 600), (400, 588), (388, 600), (609, 428), (619, 405), (609, 377), (600, 405), (400, 253), (410, 230), (400, 208), (391, 230), (227, 421), (248, 400), (227, 379), (206, 400)]

		for marker, pos in zip(range(28, 44), column_positions):
			self.wall_positions[marker] = pos

	# Given two markers, calculates the position and rotation of the robot
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
		phi = trig.normalise_angle_radians(theta1 - theta0)
		phi_sign = 1 if phi > 0 else -1

		# Distance between m0 and m1
		marker_distance = np.abs(z1 - z0)

		# The line that passes through m0 and m1 makes an angle of alpha with the horizontal
		cosalpha = (r0 ** 2 + marker_distance ** 2 - r1 ** 2) / (2 * r0 * marker_distance)
		if np.abs(cosalpha) > 1:
			print('[GameState][WARN] cosalpha has invalid value {}', cosalpha)
			return None, None
			# cosalpha = np.clip(cosalpha, -1, 1)
		
		alpha = np.arccos(cosalpha) * phi_sign

		# The position of the robot as a complex number
		z = z0 + r0 * np.exp(alpha * 1j) * (z1 - z0) / marker_distance

		# The position of the robot as a numpy array
		pos = np.array([z.real, z.imag])

		# The angle the robot is facing measured anticlockwise from the horizontal
		theta = np.angle(z0 - z) - theta0
		theta_degrees = trig.normalise_angle_degrees(theta * (180/np.pi))

		return pos, theta_degrees

	def elapsed_time(self):
		return time.time() - self.init_time

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

	def friendly_zone_middle(self):
		return self.zone_dict[self.friendly_zone]

	def opposite_zone_middle(self):
		return self.zone_dict[(self.friendly_zone + 2) % 4]

	def robot_state_blocking(self):
		initial_count = self.vision_updates

		while self.vision_updates < initial_count + 2:
			time.sleep(0.001)

		return self.robot_pos, self.robot_rot

	def report_vision_markers(self, markers, verbose=False):
		if self.vision_waits:
			self.vision_waits -= 1
			return

		self.vision_updates += 1
		useful_markers = [m for m in markers if self.get_marker_type(m) in ['WALL', 'COLUMN']]
		if len(useful_markers) == 0:
			if verbose:
				print('[GameState] No anchored markers')
			return
		elif len(useful_markers) == 1:
			if verbose:
				print('[GameState] Only one anchored marker')
		elif len(useful_markers) == 2:
			if verbose:
				print('[GameState] Only two anchored markers')
		else:
			combinations = itertools.combinations(useful_markers, 2)
			positions = []
			rotations = []
			for stuff in combinations:
				pos, rot = self._robot_position(*stuff)
				if pos is not None:
					positions.append(pos)
					rotations.append(rot)
			if verbose:
				print('Combinations:', len(positions))
			pos = np.mean(positions, axis=0)
			rot = trig.angle_degrees(sum(trig.to_cartesian_degrees(angle, 1) for angle in rotations))
			if verbose:
				print('[GameState] Found {} markers, determined position {} rotation {}'.format(len(useful_markers), pos, rot))
			self.last_pos_update_time = time.time()
			self.robot_pos = pos
			self.robot_rot = rot
			return pos, rot
