import numpy as np
from .import trig


def contains_line(edges, edge):
	for x in edges:
		a, b = x
		c, d = edge
		if (a == c).all() and (b == d).all():
			return True

	return False


class Rectangle:
	# Represents a rectangular zone bounded in the y-axis by (south, north) and in the x-axis by (west, east)
	def __init__(self, south, north, west, east):
		self.south, self.north, self.west, self.east = south, north, west, east

		# a b
		# d c
		self.a = np.array([west, north])
		self.b = np.array([east, north])
		self.c = np.array([east, south])
		self.d = np.array([west, south])
		self.corners = [self.a, self.b, self.c, self.d]

		self.north_edge = self.a, self.b
		self.east_edge = self.b, self.c
		self.south_edge = self.c, self.d
		self.west_edge = self.d, self.a

		self.edges = [self.north_edge, self.east_edge, self.south_edge, self.west_edge]

		self.corner_dict = {
			str(self.a): (self.a, (self.west_edge, self.north_edge), np.array([-1, 1])),
			str(self.b): (self.b, (self.north_edge, self.east_edge), np.array([1, 1])),
			str(self.c): (self.c, (self.east_edge, self.south_edge), np.array([1, -1])),
			str(self.d): (self.d, (self.south_edge, self.west_edge), np.array([-1, -1])),
		}

	# Expands the square in all directions by the given amount
	def expand(self, distance):
		return Rectangle(self.south - distance, self.north + distance, self.west - distance, self.east + distance)

	def is_crossed_by(self, line):
		return any(trig.crosses(line, edge) for edge in self.edges)

	def is_point_inside(self, point):
		x, y = point
		return self.west < x < self.east and self.south < y < self.north

	# If the line goes through the rectangle, return an alternate route that avoids this zone
	# Returns a list of lines that the robot should go through instead
	def alternative_route(self, line):
		start, end = line

		# The distance to avoid this zone by
		pad_distance = 10

		bad_edges = [edge for edge in self.edges if trig.crosses(line, edge)]

		if bad_edges is None:
			print('[Rectangle][WARN] Calling alternative route with a line that does not go through zone')
			return [line]

		if len(bad_edges) == 1:
			print('[Rectangle][WARN] Only one edge crossed but alternative_route called')
			return [line]

		for _, (corner, (edge1, edge2), direction) in self.corner_dict.items():
			if contains_line(bad_edges, edge1) and contains_line(bad_edges, edge2):
				# Line goes through two adjacent edges, so if we stay clear of the corner we'll be fine
				#
				# Before:
				#
				# \   _______
				#  \ [      ]
				#   \[      ]
				#    \      ]
				#    [\     ]
				#    [_\____]
				#       \
				#        \
				#         \
				#          \
				#
				# After:
				#
				# |   _______
				# |  [      ]
				# |  [      ]
				# |  [      ]
				# |  [      ]
				# |  [______]
				# |
				#  \
				#   \
				#    \
				middle = corner + direction * pad_distance
				return [(start, middle), (middle, end)]

		# If we get here, that means the line goes through opposite edges

		# If the line goes through south and north edges
		if contains_line(bad_edges, self.north_edge) and contains_line(bad_edges, self.south_edge):
			_, y0 = start
			_, y1 = end

			if y1 > y0:
				# We are going south to north
				choices = [[self.c, self.b], [self.d, self.a]]
			else:
				# We are going north to south
				choices = [[self.b, self.c], [self.a, self.d]]
		else:  # Lines go through east and west edges
			x0, _ = start
			x1, _ = end

			if x1 > x0:
				# We are going east to west
				choices = [[self.a, self.b], [self.d, self.c]]
			else:
				# We are going west to east
				choices = [[self.b, self.a], [self.c, self.d]]

		min_distance = None
		chosen_route = None

		# There are multiple options: we can go to the left or to the right of the pillar, so choose the shortest route
		for choice in choices:
			corner0, corner1 = choice
			_, _, pad0 = self.corner_dict[str(corner0)]
			_, _, pad1 = self.corner_dict[str(corner1)]

			# Go via the corners
			middle0 = corner0 + pad0 * pad_distance
			middle1 = corner1 + pad1 * pad_distance
			potential_route = [(start, middle0), (middle0, middle1), (middle1, end)]

			distance = 0

			for line in potential_route:
				a, b = line
				distance += trig.magnitude(b - a)

			if min_distance is None or distance < min_distance:
				min_distance = distance
				chosen_route = potential_route

		assert chosen_route is not None

		return chosen_route
