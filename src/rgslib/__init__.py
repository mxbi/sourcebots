import contextlib
import os
import sys
import time

# Define units
ms = 0.001

# Define constants
RE_PER_CM = 11.13  # Rotary encodes per cm moved - ~1% error
RE_LATENCY = 60 * ms  # Rotary encoder function lag in milliseconds - currently pulled out of my ass
RE_PREDICT_TIME = 200 * ms  # Stop checking rotary encoder and use the current velocity to predict when this many seconds remain
RE_MOVE_OFFSET = 8  # Breaking distance to always subtract in cm

## Angle callibration
# The difference in rotary encoder readings necessary to turn a degree.
RE_PER_DEGREE = 8.50
# Experimentally determined value which determines how much of an effect angular velocity has on when we need to stop.
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

# Normalises an angle such that it is in the range (-180, 180].
# This means, for example:
#  - normalise_angle(270) = -90
#  - normalise_angle(450) = 90
#  - normalise_angle(180) = 180
def normalise_angle(theta):
	return 180 - ((180 - theta) % 360)

from .motion import MotionController
from .vision import VisionController
from .game import GameState
