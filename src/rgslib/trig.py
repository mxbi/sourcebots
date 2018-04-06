import numpy as np


# Normalises an angle such that it is in the range (-180, 180].
# This means, for example:
#  - normalise_angle_degrees(270) = -90
#  - normalise_angle_degrees(450) = 90
#  - normalise_angle_degrees(180) = 180
def normalise_angle_degrees(theta):
	return _normalise_angle(theta, 180)


# Normalises an angle into the range (-pi. pi].
def normalise_angle_radians(theta):
	return _normalise_angle(theta, np.pi)


# Normalises an angle into the range (-half, half].
def _normalise_angle(theta, half):
	return half - ((half - theta) % (2*half))


# Returns the anticlockwise angle between the positive horizontal and the 2d vector, in radians
# The returned angle will be in the range [-pi, pi]
def angle_radians(vector):
	if len(vector) != 2:
		raise ValueError()

	return np.arctan2(vector[1], vector[0])


# Returns the anticlockwise angle between the positive horizontal and the 2d vector, in degrees
# The returned angle will be in the range [-180, 180]
def angle_degrees(vector):
	return angle_radians(vector) * (180/np.pi)


def magnitude(vector):
	return np.linalg.norm(vector)


def sin_degrees(theta):
	return np.sin(theta * (np.pi/180))


def cos_degrees(theta):
	return np.cos(theta * (np.pi/180))


def tan_degrees(theta):
	return np.tan(theta * (np.pi/180))


# Converts polar co-ordinates (angle from horizontal, magnitude) into a numpy array (x, y)
def to_cartesian_radians(theta, r):
	return r * np.array([np.cos(theta), np.sin(theta)])


# Converts polar co-ordinates (angle from horizontal, magnitude) into a numpy array (x, y)
def to_cartesian_degrees(theta, r):
	return to_cartesian_radians(theta * (np.pi/180), r)


# Converts numpy array (x, y) into polar co-ordinates (angle from horizontal, magnitude)
def to_polar_radians(vector):
	return angle_radians(vector), magnitude(vector)


# Converts numpy array (x, y) into polar co-ordinates (angle from horizontal, magnitude)
def to_polar_degrees(vector):
	return angle_degrees(vector), magnitude(vector)


# Whether or not one line crosses the other.
# Lines cross only if one line actually passes through the other, such that the points of one line lie either side
# of the other line. Therefore, if both lines are equal or otherwise co-linear, this will return false. Similarly, if
# one end of line1 is located on line0, this will return False.
#
# Both arguments should be a pair of numpy arrays that represent points.
def crosses(line0, line1):
	a, b = line0
	c, d = line1

	ab_crosses_cd = np.sign(np.cross((a - c), (d - c))) == - np.sign(np.cross((b - c), (d - c))) != 0
	cd_crosses_ab = np.sign(np.cross((d - a), (b - a))) == - np.sign(np.cross((c - a), (b - a))) != 0

	return ab_crosses_cd and cd_crosses_ab
