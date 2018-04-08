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

	# Expands the square in all directions by the given amount
	def expand(self, distance):
		return Rectangle(self.south - distance, self.north + distance, self.west - distance, self.east + distance)

	def is_crossed_by(self, line):
		return any(trig.crosses(line, edge) for edge in self.edges)

	def is_point_inside(self, point):
		x, y = point
		return self.west < y < self.east and self.south < x < self.north
