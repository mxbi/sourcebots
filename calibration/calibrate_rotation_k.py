from ..src import rgslib as lib
import robot
import time
import numpy as np

# This python file finds the value for K for the robot rotation equations without user input.
# Note, both A and K will change depending on surface - A must be manually tuned.

test_cases = [0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]

r = robot.Robot()
motion = lib.MotionController(r)

all_k = []
all_re = []
all_v = []
for power in test_cases:
	print('## Testing at power {}'.format(power))

	for multiplier in [1, -1, 1, -1]:
		motion.mleft = power * multiplier
		motion.mright = -power * multiplier

		time.sleep(1)

		# Initial sample
		motion.update_re()
		re = motion.re.copy()

		# Start ~1s velocity measurement
		t0 = time.time()
		time.sleep(1)

		motion.update_re()
		velocity = np.abs(re - motion.re).sum() / (time.time() - t0)  # Total rotary encodes per second

		# Calculate RE delta when stopping
		motion.update_re()
		total_re_before = np.abs(re - motion.re).sum()
		motion.speed = 0
		time.sleep(1)

		motion.update_re()
		total_re_after = np.abs(re - motion.re).sum()

		re_delta = total_re_after - total_re_before

		K = re_delta / velocity
		print('Velocity = {} K = {}'.format(power * multiplier, K))
		all_k.append(K)
		all_re.append(re_delta)
		all_v.append(velocity)

# mean
K = np.mean(all_re / all_v)
mae = np.mean(np.abs(K - np.array(all_k)))
print('Finished! Found K = {} MAE = {}'.format(K, mae))
