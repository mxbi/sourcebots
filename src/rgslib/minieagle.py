import time
import numpy as np
import threading
import sys
import traceback
import os
import pickle

def now():
    """Returns the current time as a string in the format 'YYYY_MM_DD_HH_MM_SS'. Useful for timestamping filenames etc."""
    return time.strftime("%Y_%m_%d__%H_%M_%S")

# Thread which continuously updates the vision controller's 'markers' field with the latest reading of markers
class OfflineEagleThread(threading.Thread):
	def __init__(self, gamestate, vision, framerate=5):
		self.gamestate = gamestate
		self.vision = vision
		self.framerate = framerate
		self.start_time = time.time()

		threading.Thread.__init__(self)
		self.pickle_list = []

		self.runfile = 'runs/run_{}-{}.eagle'.format(now(), framerate)
		try:
			os.mkdir('runs')
		except:
			pass

	def run(self):
			while True:
				t0 = time.time()
				pickle_data = pickle.dump((t0, self.gamestate, self.vision.markers), open(self.runfile, 'ab'))
				time.sleep(max(0, 1 / self.framerate - (time.time() - t0)))  # Wait remaining frametime
