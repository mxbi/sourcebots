import time
import threading

class VisionController:
	# Gets passed the Robot() instance so it can use the cameras
	def __init__(self, robot, gamestate=None):
		self.r = robot
		self.camera = robot.camera
		self.gamestate = gamestate
		if self.gamestate is None:
			print('[VisionController][WARN] No GameState passed to VisionController, functionality will be reduced')

		# Can't see any markers initially
		self.markers = []

		# The number of times the markers have been updated my the marker thread.
		# Useful for blocking
		self.marker_update_count = 0
		self.last_marker_time = 0
		self.last_marker_duration = 0

		# Start thread which runs forever
		self.thread = MarkerThread(self).start()
		# TODO: Gracefully stop this thread on user request or object destruction

	def markers_semiblocking(self, time_threshold=None):
		"""Wait until the current see() call has finished and return that. Note that the photo may have been taken before markers_semiblocking was called.
		Optionally, time_threshold (unix epoch) can be passed. This will return the first processed frame that was taken after that time."""
		old_marker_count = self.marker_update_count

		if time_threshold:
			while self.last_marker_time < time_threshold:
				time.sleep(0.001)

		else:
			while self.marker_update_count < old_marker_count + 1:
				time.sleep(0.001)

		return self.markers

	def markers_blocking(self):
		"""Wait until the next see() call has finished and return that."""
		old_marker_count = self.marker_update_count
		# Wait for markers to be updated twice since calling the function

		while self.marker_update_count < old_marker_count + 2:
			time.sleep(0.001) # Microsleep while blocking to prevent 100% cpu usage

		return self.markers


# Thread which continuously updates the vision controller's 'markers' field with the latest reading of markers
class MarkerThread(threading.Thread):
	def __init__(self, vision_controller):
		self.vision_controller = vision_controller
		threading.Thread.__init__(self)

	def run(self):
		vision_controller = self.vision_controller
		camera = vision_controller.camera
		while True:
			t0 = time.time()

			vision_controller.markers = camera.see()
			if vision_controller.gamestate is not None:
				vision_controller.gamestate.report_vision_markers(vision_controller.markers)

			vision_controller.marker_update_count += 1
			vision_controller.last_marker_time = time.time()
			vision_controller.last_marker_duration = time.time() - t0
