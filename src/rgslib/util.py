from .import trig


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
		self.corners = [a, b, c, d]

		self.north_edge = a, b
		self.east_edge = b, c
		self.south_edge = c, d
		self.west_edge = d, a

		self.edges = [self.north_edge, self.east_edge, self.south_edge, self.west_edge]

		self.corner_dict = {
			self.a: ((self.west_edge, self.north_edge), np.array([-1, 1])),
			self.b: ((self.north_edge, self.east_edge), np.array([1, 1])),
			self.c: ((self.east_edge, self.south_edge), np.array([1, -1])),
			self.d: ((self.south_edge, self.west_edge), np.array([-1, -1])),
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
		pad_distance = 5

		bad_edges = [edge for edge in self.edges if trig.crosses(line, edge)]

		if bad_edges is None:
			print('[Rectangle][WARN] Calling alternative route with a line that does not go through zone')
			return [line]

		if len(bad_edges) == 1:
			print('[Rectangle][WARN] Only one edge crossed but alternative_route called')
			return [line]

		for corner, ((edge1, edge2), direction) in self.corner_dict:
			if edge1 in bad_edges and edge2 in bad_edges:
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
		# uhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
