import numpy as np
import time
import contextlib
import sys
import os

# Define units
ms = 0.001

# Define constants
RE_PER_CM = 11.13       # Rotary encodes per cm moved - ~1% error
RE_LATENCY = 60 * ms # Rotary encoder function lag in milliseconds - currently pulled out of my ass
RE_PREDICT_TIME = 200 * ms # Stop checking rotary encoder and use the current velocity to predict when this many seconds remain
RE_MOVE_OFFSET = 8 # Breaking distance to always subtract in cm
RE_PER_DEGREE = 8.02 # The difference in rotary encoder readings necessary to turn a degree. TODO: Find better value

FAST_MOVE_SPEED = 1
FAST_ROTATE_SPEED = 0.5

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

def wait_until(t):
    "Wait until `t` in unix epoch time"
    time.sleep(max(0, t - time.time()))

class RobotController:
    def __init__(self, robot):
        self.r = robot
        self.arduino = self.r.servo_board

        self._update_re()

    def _update_re(self):
        t0 = time.time()
        with nostdout():
            self.left_re, self.right_re = [int(i) for i in self.arduino.direct_command('r')]
        self.re = np.array([self.left_re, self.right_re])
        self.re_time = time.time() - RE_LATENCY # Estimate actual time measurement was taken

    @property
    def mleft(self): return self.r.motor_board.m0

    @mleft.setter
    def mleft(self, val): self.r.motor_motor_board.m0 = val

    @property
    def mright(self): return self.r.motor_board.m1

    @mright.setter
    def mright(self, val): self.r.motor_motor_board.m1 = val

    @property
    def speed(self):
        return (self.mleft, self.mright)

    @speed.setter # Dark magic, when "self.speed = 1" is called, update both motors
    def speed(self, speed):
        self.r.motor_board.m0, self.r.motor_board.m1 = speed, speed

    def move(self, distance, speed=FAST_MOVE_SPEED):
        "Accurately move `distance` cm using rotary encoders. Can be negative"
        # TODO: Actively correct for drift during movement
        distance -= RE_MOVE_OFFSET
        distances = []
        self._update_re()
        initial_re = self.re.copy()
        initial_time = self.re_time

        movement_speed = speed
        if distance > 0:
            self.speed = movement_speed
        else:
            self.speed = -movement_speed

        while True:
            # Save last rotary encoder values for comparison
            old_re = self.re.copy()
            old_re_time = self.re_time

            self._update_re()

            # Note this is a vector because re/old_re is a vector
            total_distance = (self.re - initial_re) / RE_PER_CM
            distances.append(total_distance)

            re_time_delta = self.re_time - old_re_time
            velocity = (self.re - old_re) / RE_PER_CM / re_time_delta

            time_remaining = (distance - total_distance.mean()) / velocity.mean()
            # Because "re_time" is corrected for the latency of the re function, this pseudo-works out the time when the movement should actually finish,
            # not when the rotary encoder says the movement should finish (which would be 60ms or so off)
            if time_remaining < 0:
                time_remaining = 5
            end_time = self.re_time + time_remaining

            print('[RobotController] Distance {}cm Velocity {}cm/s ETA {}s'.format(total_distance, velocity, round(time_remaining, 4)))

            if time_remaining < RE_PREDICT_TIME:
                break

        wait_until(end_time)
        self.speed = 0

        t0 = time.time()

        # Temporary
        # TODO: Exit when it actually stops moving (when velocity goes to 0)
        for i in range(10):
            self._update_re()
            total_distance = (self.re - initial_re) / RE_PER_CM
            distances.append(total_distance)
            print(time.time() - t0)
            print('[RobotController] Finished - travelled {}cm'.format((self.re - initial_re) / RE_PER_CM))

        return distances

    def _aminusb(self, arr):
        return arr[0] - arr[1]

    # Turns the angle in degrees, where clockwise is positive and anticlockwise is negative
    def rotate(self, angle, speed=FAST_ROTATE_SPEED):
        angles = []
        self._update_re()
        initial_re = self.re.copy()
        initial_time = self.re_time

        if angle < 0:
            speed = -speed

        self.mleft = speed
        self.mright = -speed

        while True:
            # Save last rotary encoder values for comparison
            old_re = self.re.copy()
            old_re_time = self.re_time

            self._update_re()

            # Note this is a vector because re/old_re is a vector
            angle_travelled = self._aminusb(self.re - initial_re) / RE_PER_DEGREE
            angles.append(angle_travelled)

            re_time_delta = self.re_time - old_re_time
            velocity = self._aminusb(self.re - old_re) / RE_PER_DEGREE / re_time_delta

            time_remaining = (angle - angle_travelled) / velocity

            if time_remaining < 0:  # TODO: Something that isn't an ugly hack
                time_remaining = 5
            end_time = self.re_time + time_remaining

            print('[RobotController] Rotation {}deg Velocity {}deg/s ETA {}s'.format(angle_travelled, velocity, round(time_remaining, 4)))

            if time_remaining < RE_PREDICT_TIME:
                break

        wait_until(end_time)
        self.speed = 0

        return angles