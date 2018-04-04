import pygame
import rgslib
import time
import numpy as np

s = rgslib.GameState(0)
s.robot_pos = (50, 50)
s.robot_rot = 45

WHITE = np.array((255, 255, 255))
BLACK = np.array((0, 0, 0))
DISPLAY_SIZE = (1000, 1000)

# Convert rgslib coordinates to pixel position
def transform(c):
	return (c[0] + 100, 900 - c[1])

# Create co-ordinates of a square with top-left corner position and size
def box(c, size):
	return [c, (c[0] + size, c[1]), (c[0] + size, c[1] + size), (c[0], c[1] + size)]

def draw_arena():
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

		text = font.render(str(marker), True, (0, 128, 0))
		screen.blit(text, (x - text.get_width() // 2, y - text.get_height() // 2))

def draw_robot(robot_pos, rot, markers):
	rot /= (180 / np.pi) # Transform to radians
	rot = (np.pi/2) - rot # Transform to clockwise from y direction
	pos = transform(robot_pos) # Transform co-ordinates to pixel values

	# Rotation matrix, when multiplied with a vector (x, y) rotates it by rot degrees about the origin.
	# The origin here is the centre of the robot
	rot_matrix = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])

	# We assume that the centre of rotation is the centre of the robot, actually it's at the camera but shhh
	corners = np.array([[25, -25], [-25, -25], [-25, 25], [25, 25]])
	corners = pos + np.dot(rot_matrix, corners.T).T
	pygame.draw.polygon(screen, (0, 0, 255), corners)

	# Draw a triangle and apply the rotation matrix again
	vertices = np.array([[-10, -25], [0, -35], [10, -25]])
	vertices = pos + np.dot(rot_matrix, vertices.T).T
	# top_middle = np.mean(robot_corners[0:2], axis=0).astype(int)
	pygame.draw.lines(screen, (0, 0, 255), False, vertices, 2)

	for marker in markers:
		print(marker.distance_metres, marker.rot_y_radians)
		# TODO: Completely broken
		marker_pos = s.robot_pos + rgslib.trig.to_cartesian_degrees(marker.rot_y_radians - (90-s.robot_rot), marker.distance_metres * rgslib.VISION_DISTANCE_FACTOR)
		pygame.draw.lines(screen, (255, 0, 0), True, box(transform(marker_pos - [5, 5]), 10), 2)

pygame.init()
screen = pygame.display.set_mode(DISPLAY_SIZE)
done = False

font = pygame.font.SysFont("helvetica", 20)

class FakeMarker():
	def __init__(self, position=[600, 600]):
		# self.distance_metres = np.sqrt(np.sum((np.array(s.robot_pos) - position)**2))
		# self.rot_y_radians = np.arctan2(*(position - np.array(s.robot_pos))[::-1])
		self.rot_y_radians, self.distance_metres = rgslib.trig.to_polar_radians(position - np.array(s.robot_pos))
		self.distance_metres /= rgslib.VISION_DISTANCE_FACTOR

def full_update(delay=0.03):
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			done = True

	draw_arena()
	draw_robot(s.robot_pos, s.robot_rot, [FakeMarker([600, 600])])
	pygame.display.flip()
	time.sleep(delay)

full_update()
time.sleep(1)

for i in range(10):
	s.robot_pos = s.robot_pos + rgslib.trig.to_cartesian_degrees(s.robot_rot, 3.5) #(s.robot_pos[0] + 3.5 * np.sin(s.robot_rot / (180 / np.pi)), s.robot_pos[1] + 3.5 * np.cos(s.robot_rot / (180 / np.pi)))
	full_update()

for i in range(60):
	s.robot_rot -= 1.5
	full_update()

for i in range(50):
	s.robot_pos = s.robot_pos + rgslib.trig.to_cartesian_degrees(s.robot_rot, 3.5) #(s.robot_pos[0] + 3.5 * np.sin(s.robot_rot / (180 / np.pi)), s.robot_pos[1] + 3.5 * np.cos(s.robot_rot / (180 / np.pi)))
	full_update()

time.sleep(10)