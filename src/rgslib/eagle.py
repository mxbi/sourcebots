from rgslib import game
import rgslib
import pygame
import time
import numpy as np
import threading

WHITE = np.array((255, 255, 255))
BLACK = np.array((0, 0, 0))
DISPLAY_SIZE = (1000, 1000)

# Convert rgslib coordinates to pixel position
def transform(c):
	return (c[0] + 100, 900 - c[1])

# Create co-ordinates of a square with top-left corner position and size
def box(c, size):
	return [c, (c[0] + size, c[1]), (c[0] + size, c[1] + size), (c[0], c[1] + size)]

def draw_arena(screen, s, vision):
	font = pygame.font.SysFont("roboto", 20)
	bold_font = pygame.font.SysFont("roboto", 20, bold=True)

	pygame.draw.rect(screen, WHITE, pygame.Rect(0, 0, *DISPLAY_SIZE))
	pygame.draw.lines(screen, BLACK, True, [(100, 100), (100, 900), (900, 900), (900, 100)])

	# Starting zone boxes
	# Team boxes
	zone_corners = [(200, 600), (200, 382), (419, 382), (419, 600)]
	zone_size = 182
	for corner in zone_corners:
		corners = box(transform(corner), zone_size)
		pygame.draw.lines(screen, WHITE*0.85, True, corners)

	# Team boxes
	zone_corners = [(100, 700), (100, 382), (419, 382), (419, 700)]
	zone_size = 282
	for corner in zone_corners:
		corners = box(transform(corner), zone_size)
		pygame.draw.lines(screen, WHITE*0.7, True, corners)

	# Columns
	columns = [(182, 419), (582, 419), (382, 619), (382, 219)] # Top-left corner
	column_size = 37
	for c in columns:
		c = transform(c)
		corners = box(c, column_size) #[c, (c[0]+column_size, c[1]), (c[0]+column_size, c[1]+column_size), (c[0], c[1]+column_size)] # Infer all corners from top-left corner (by adding column_size)
		pygame.draw.lines(screen, BLACK, True, corners)

	# Draw marker text
	for marker, pos in s.wall_positions.items():
		x, y = transform(pos)
		# Offset wall marker labels so they don't intersect the rectangle
		if y == 100: y -= 15
		if y == 900: y += 15
		if x == 100: x -= 15
		if x == 900: x += 15

		if marker in [marker.id for marker in vision.markers]:
			text = bold_font.render(str(marker), True, (0, 30, 0))
		else:
			text = font.render(str(marker), True, (0, 90, 0))
		screen.blit(text, (x - text.get_width() // 2, y - text.get_height() // 2))

def draw_robot(screen, s, vision):
	rot = s.robot_rot
	rot /= (180 / np.pi)  # Transform to radians
	rot = (np.pi/2) - rot  # Transform to clockwise from y direction
	pos = transform(s.robot_pos)  # Transform co-ordinates to pixel values

	# Rotation matrix, when multiplied with a vector (x, y) rotates it by rot degrees about the origin.
	# The origin here is the centre of the robot
	rot_matrix = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])

	# We assume that the centre of rotation is the centre of the robot, actually it's at the camera but shhh
	corners = np.array([[25, -25], [-25, -25], [-25, 25], [25, 25]])
	corners = pos + np.dot(rot_matrix, corners.T).T
	pygame.draw.polygon(screen, (0, 0, 255), corners)

	# Draw FOV cone
	fov = 22 * (np.pi/180)
	vertices = np.array([[-2000*np.sin(fov), -25-(2000*np.cos(fov))], [0, -25], [2000*np.sin(fov), -25-(2000*np.cos(fov))]])
	vertices = pos + np.dot(rot_matrix, vertices.T).T
	pygame.draw.aalines(screen, (200, 200, 255), False, vertices, 1)

	# Draw line pointing ahead
	vertices = np.array([[0, -35], [0, -300]])
	vertices = pos + np.dot(rot_matrix, vertices.T).T
	pygame.draw.aaline(screen, (200, 200, 255), *vertices, 1)

	# Draw a triangle and apply the rotation matrix again
	vertices = np.array([[-10, -25], [0, -35], [10, -25]])
	vertices = pos + np.dot(rot_matrix, vertices.T).T
	# top_middle = np.mean(robot_corners[0:2], axis=0).astype(int)
	pygame.draw.lines(screen, (0, 0, 255), False, vertices, 2)

	for marker in vision.markers:
		if hasattr(vision, 'fake'):
			if np.abs(marker.spherical.rot_y_degrees * (np.pi/180)) > fov:
				continue
		marker_type = s.get_marker_type(marker)
		if marker_type == 'FRIENDLY':
			colour = (0, 150, 0) if marker.id == s.box_id else (0, 220, 0)
		elif marker_type == 'ENEMY':
			colour = (255, 0, 0)
		else:
			continue
		box_thickness = 5 if marker.id == s.box_id else 2
		marker_pos = s.robot_pos + rgslib.trig.to_cartesian_degrees(s.robot_rot - marker.spherical.rot_y_degrees, marker.distance_metres * rgslib.VISION_DISTANCE_FACTOR)
		pygame.draw.lines(screen, colour, True, box(transform(marker_pos - [5, 5]), 10), box_thickness)

# Thread which continuously updates the vision controller's 'markers' field with the latest reading of markers
class EagleThread(threading.Thread):
	def __init__(self, gamestate, vision, framerate=30):
		self.gamestate = gamestate
		self.vision = vision
		self.framerate = framerate

		threading.Thread.__init__(self)
		pygame.init()
		self.screen = pygame.display.set_mode(DISPLAY_SIZE)
		done = False

	def run(self):
			while True:
				t0 = time.time()
				for event in pygame.event.get():
					if event.type == pygame.QUIT:
						break

				draw_arena(self.screen, self.gamestate, self.vision)
				draw_robot(self.screen, self.gamestate, self.vision)

				# Print elapsed time
				msg_font = pygame.font.SysFont("roboto", 36)
				text = msg_font.render("{:.2f}s".format(s.elapsed_time()), True, BLACK)
				screen.blit(text, (25, 25))

				pygame.display.flip() # Swap frame buffers, print current buffer to display
				time.sleep(t0 - (1 / self.framerate)) # Wait remaining frametime


# Run swanky demo when script is run on its own
def demo():
	s = game.GameState(0)
	s.robot_pos = (50, 50)
	s.robot_rot = 45
	s.box_id = 46

	pygame.init()
	screen = pygame.display.set_mode(DISPLAY_SIZE)
	done = False

	msg_font = pygame.font.SysFont("roboto", 36)

	class Spherical():
		def __init__(self, deg): self.rot_y_degrees = deg

	class FakeMarker():
		def __init__(self, position=[600, 600], id=44):
			self.id = id
			self.rot_y_degrees, self.distance_metres = rgslib.trig.to_polar_degrees(position - np.array(s.robot_pos))
			self.spherical = Spherical(s.robot_rot - self.rot_y_degrees)
			self.distance_metres /= rgslib.VISION_DISTANCE_FACTOR

	class FakeVision():
		def __init__(self, markers):
			self.markers = markers
			self.fake = 'yeah not gonna lie'

	def full_update(delay=0.03):
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				done = True

		fv = FakeVision([FakeMarker(*x) for x in markers])
		draw_arena(screen, s, fv)

		# draw_robot(s.robot_pos, s.robot_rot, [FakeMarker(*x) for x in markers])
		draw_robot(screen, s, fv)
		text = msg_font.render("{:.2f}s".format(s.elapsed_time()), True, BLACK)
		screen.blit(text, (25, 25))  # (text.get_width() // 2, text.get_height() // 2))
		pygame.display.flip()
		time.sleep(delay)

	# Manually programmed robot movement
	markers = []
	markers_positions = (np.random.random((2, 5)) * 200) + 18.5
	for rot, ids in zip([0, np.pi/2, np.pi, np.pi*1.5], [range(44, 50), range(50, 56), range(56, 60), range(60, 64)]):
		rot_matrix = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
		print(markers_positions)
		print(rot_matrix)
		positions = np.dot(rot_matrix, markers_positions).T + np.array([400, 400])
		print(positions)
		markers.extend(zip(positions, ids))

	full_update()

	time.sleep(1)

	for i in range(200):
		s.robot_pos = s.robot_pos + rgslib.trig.to_cartesian_degrees(s.robot_rot, 3.5) #(s.robot_pos[0] + 3.5 * np.sin(s.robot_rot / (180 / np.pi)), s.robot_pos[1] + 3.5 * np.cos(s.robot_rot / (180 / np.pi)))
		full_update()

	for i in range(120):
		s.robot_rot -= 1.5
		full_update()

	for i in range(50):
		s.robot_pos = s.robot_pos + rgslib.trig.to_cartesian_degrees(s.robot_rot, 3.5) #(s.robot_pos[0] + 3.5 * np.sin(s.robot_rot / (180 / np.pi)), s.robot_pos[1] + 3.5 * np.cos(s.robot_rot / (180 / np.pi)))
		full_update()

	time.sleep(5)
