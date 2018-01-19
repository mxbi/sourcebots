import numpy as np
import time
import copy

class RobotController:
    def __init__(self, robot):
        self.r = robot
        self.mleft = self.r.motor_board.m0
        self.mright = self.r.motor_board.m1

        self.ramp_up_speed = 0.05 # Speed increase allowed every 10ms
        self.ramp_down_speed = 0.1 # Speed decrease allowed every 10ms

    def _set_speed_both(self, speed):
        self.mleft, self.mright = speed, speed

    def _get_speed_both(self):
        return [self.mleft, self.mright]

    # def _ramp_to_speed(self, target_speed, set_func=self._set_speed_both, get_func=self._get_speed_both):
    #     assert motor in ['left', 'right', 'both']
    #     set_func = {'left': self._set_speed_left, 'right': self._set_speed_right, 'both': self.set_speed_both}[motor]
    #
    #     current_speed = get_speed(both)
    #     while motor < target_speed:
    #         t0 = time.time()
    #         for motor in
    #             if motor + self.ramp_up_speed >= target_speed: # If the speed is less than one step away from reaching the target, go
    #                 set_func(target_speed)
    #                 break
    #             else: # Otherwise, slowly ramp up the speed
    #                 motor += self.ramp_up_speed
    #
    #         time.sleep(0.01 - (time.time() - t0)) # Sleep 10ms minus the time spent in the loop for better timing control
    #
    # def _ramp_to_speed(self, motors, )
