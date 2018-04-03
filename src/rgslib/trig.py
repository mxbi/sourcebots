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
	return angle_degrees(vector) * (180/np.pi)


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
	return to_cartesian_degrees(theta * (np.pi/180), r)


# Converts numpy array (x, y) into polar co-ordinates (angle from horizontal, magnitude)
def to_polar_radians(vector):
	return angle_radians(vector), magnitude(vector)


# Converts numpy array (x, y) into polar co-ordinates (angle from horizontal, magnitude)
def to_polar_degrees(vector):
	return angle_degrees(vector), magnitude(vector)
