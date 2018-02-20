import numpy as np
import time
import copy

# Define units
ms = 0.001

# Define constants
RE_PER_CM = 11.13       # Rotary encodes per cm moved - ~1% error
RE_LATENCY = 60 * ms # Rotary encoder function lag in milliseconds - currently pulled out of my ass
RE_PREDICT_TIME = 100 * ms # Stop checking rotary encoder and use the current velocity to predict when this many seconds remain

FAST_MOVE_SPEED = 1

def wait_until(time):
    "Wait until `time` in unix epoch time"
    time.sleep(time.time() - time)

class RobotController:
    def __init__(self, robot):
        self.r = robot
        self.mleft = self.r.motor_board.m0
        self.mright = self.r.motor_board.m1

        self.arduino = self.servo_board

        self._update_re()

    def _update_re(self):
        self.left_re, self.right_re = self.arduino.direct_command('r')
        self.re = np.array([self.left_re, self.right_re])
        self.re_time = time.time() - RE_LATENCY # Estimate actual time measurement was taken

    @property
    def speed(self):
        return [self.mleft, self.mright]

    @speed.setter # Dark magic, when "self.speed = 1" is called, update both motors
    def speed(self, speed):
        self.mleft, self.mright = speed

    def move(self, distance, movement_speed=FAST_MOVE_SPEED):
        "Accurately move `distance` cm using rotary encoders. Can be negative"
        # TODO: Actively correct for drift during movement
        self._update_re()
        initial_re = self.re.copy()
        initial_time = self.re_time

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

            re_time_delta = self.last_re_time - old_re_time
            velocity = (self.re - old_re) / RE_PER_CM / re_time_delta

            # TODO: How to deal with vectors? Right now I just work based on the left rotary encoder
            time_remaining = (distance - total_distance[0]) / velocity[0]
            # Because "re_time" is corrected for the latency of the re function, this pseudo-works out the time when the movement should actually finish,
            # not when the rotary encoder says the movement should finish (which would be 60ms or so off)
            end_time = self.re_time + time_remaining

            print('[RobotController] Distance travelled {}cm Velocity {}cm/s Time remaining {}s'.format(total_distance, velocity, time_remaining))

            if time_remaining < RE_PREDICT_TIME:
                break

        wait_until(end_time)
        self.speed = 0

        self._update_re()

        print('[RobotController] Finished - travelled {}cm'.format(self.re))

        return self.re
