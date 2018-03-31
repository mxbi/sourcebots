from robot import Robot
import rgslib as lib
import numpy as np
import time


# Centre of our zone, where we need to return
HOME_ZONE_CENTRE = np.array([232, 232])

start_time = time.time()


def time_since_start():
	return time.time() - start_time


r = Robot()
s = lib.GameState(0)
c = lib.MotionController(r, s)
v = lib.VisionController(r, s)

c.close_barrier()

# 730 = Distance from centre of starting zone to centre of opposite zone
c.move(730)

c.open_barrier()

# Collect boxes until time since start exceeds certain value, or we can no longer see any boxes after full rotation

c.close_barrier()

c.move_to(HOME_ZONE_CENTRE)
