import numpy as np
import cv2
import heapq
import time
from math import dist


########## Step 0: Take input from the user ##########

clearance = int(input('Enter the clearance: '))
radius = int(input('Enter the radius of the robot: '))

while True:
    L = int(input('Enter the step size (1-10): '))
    if 1 <= L <= 10:
        break
    else:
        print('Invalid input, re-enter the step size')


########## Step 1: Define the action cost set ##########

# Define the cost of each action
value = L
action_cost_set = {(-60, value), (-30, value), (0, value), (30, value), (60, value)}

########## STEP 2: MATHEMATICAL REPRESENTATION OF FREE SPACE ##########

distance_threshold = 0.5
angular_threshold = 30

# Define the height and width of the canvas
height = 500
width = 1200
color = (0, 255, 255)
radius_color = (254, 254, 254)

# Make an empty canvas
canvas = np.zeros((height, width, 3), dtype=np.uint8)
# Draw a white rectangle from (clearance, clearance) to (width-clearance, height-clearance) on the canvas for wall padding
cv2.rectangle(canvas, (clearance, clearance), (width-clearance, height-clearance), (255, 255, 255), -1)

# OBSTACLE 1
x1, x2 = 100-clearance-radius, 175+clearance+radius
y1, y2 = clearance, 400+clearance+radius
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = radius_color
x1, x2 = 100-clearance, 175+clearance
y1, y2 = clearance, 400+clearance
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = color
x1, x2 = 100, 175
y1, y2 = 0, 400
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = 0

# OBSTACLE 2
x1, x2 = 275-clearance-radius, 350+clearance+radius
y1, y2 = 100-clearance-radius, 500-clearance+1
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = radius_color
x1, x2 = 275-clearance, 350+clearance
y1, y2 = 100-clearance, 500-clearance+1
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = color
x1, x2 = 275, 350
y1, y2 = 100, 500-clearance+1
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = 0

# OBSTACLE 3
# Define the center and side length of the hexagon with clearance
center = (650, height // 2)
side_length = 150 + clearance + radius

# Calculate the coordinates of the vertices
vertices = []
for i in range(6):
    angle_deg = 60 * i + 30
    angle_rad = np.deg2rad(angle_deg)
    x = int(center[0] + side_length * np.cos(angle_rad))
    y = int(center[1] + side_length * np.sin(angle_rad))
    vertices.append((x, y))

# Find the minimum and maximum y-coordinates of the hexagon
min_y = min(vertices, key=lambda vertex: vertex[1])[1]
max_y = max(vertices, key=lambda vertex: vertex[1])[1]
# Find the minimum and maximum x-coordinates of the hexagon
min_x = min(vertices, key=lambda vertex: vertex[0])[0]
max_x = max(vertices, key=lambda vertex: vertex[0])[0]

# Iterate over each scanline (y-coordinate)
for y in range(min_y, max_y + 1):
    for x in range(min_x, max_x + 1):
        # Check if the current point (x, y) is inside the hexagon
        is_inside = False
        # Iterate over each vertex of the hexagon
        for i in range(6):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % 6]
            # Check if the current point is inside the line segment connecting the vertices
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                is_inside = not is_inside
        if is_inside:
            canvas[y, x] = radius_color

center = (650, height // 2)
side_length = 150 + clearance 

# Calculate the coordinates of the vertices
vertices = []
for i in range(6):
    angle_deg = 60 * i + 30
    angle_rad = np.deg2rad(angle_deg)
    x = int(center[0] + side_length * np.cos(angle_rad))
    y = int(center[1] + side_length * np.sin(angle_rad))
    vertices.append((x, y))

# Find the minimum and maximum y-coordinates of the hexagon
min_y = min(vertices, key=lambda vertex: vertex[1])[1]
max_y = max(vertices, key=lambda vertex: vertex[1])[1]
# Find the minimum and maximum x-coordinates of the hexagon
min_x = min(vertices, key=lambda vertex: vertex[0])[0]
max_x = max(vertices, key=lambda vertex: vertex[0])[0]

# Iterate over each scanline (y-coordinate)
for y in range(min_y, max_y + 1):
    for x in range(min_x, max_x + 1):
        # Check if the current point (x, y) is inside the hexagon
        is_inside = False
        # Iterate over each vertex of the hexagon
        for i in range(6):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % 6]
            # Check if the current point is inside the line segment connecting the vertices
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                is_inside = not is_inside
        if is_inside:
            canvas[y, x] = color

center = (650, height // 2)
side_length = 150 

# Calculate the coordinates of the vertices
vertices = []
for i in range(6):
    angle_deg = 60 * i + 30
    angle_rad = np.deg2rad(angle_deg)
    x = int(center[0] + side_length * np.cos(angle_rad))
    y = int(center[1] + side_length * np.sin(angle_rad))
    vertices.append((x, y))

# Find the minimum and maximum y-coordinates of the hexagon
min_y = min(vertices, key=lambda vertex: vertex[1])[1]
max_y = max(vertices, key=lambda vertex: vertex[1])[1]
# Find the minimum and maximum x-coordinates of the hexagon
min_x = min(vertices, key=lambda vertex: vertex[0])[0]
max_x = max(vertices, key=lambda vertex: vertex[0])[0]

# Iterate over each scanline (y-coordinate)
for y in range(min_y, max_y + 1):
    for x in range(min_x, max_x + 1):
        # Check if the current point (x, y) is inside the hexagon
        is_inside = False
        # Iterate over each vertex of the hexagon
        for i in range(6):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % 6]
            # Check if the current point is inside the line segment connecting the vertices
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                is_inside = not is_inside
        if is_inside:
            canvas[y, x] = 0

# OBSTACLE 4
x1, x2 = width-300-clearance-radius, width-100+clearance+radius
y1, y2 = 50-clearance-radius, 125+clearance+radius
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = radius_color
x1, x2 = width-180-clearance-radius, width-100+clearance+radius
y1, y2 = 50-clearance-radius, height-50+clearance+radius
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = radius_color
x1, x2 = width-300-clearance-radius, width-300+120+clearance+radius
y1, y2 = 450-75-clearance-radius, 450+clearance+radius
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = radius_color

x1, x2 = width-300-clearance, width-100+clearance
y1, y2 = 50-clearance, 125+clearance
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = color
x1, x2 = width-180-clearance, width-100+clearance
y1, y2 = 50-clearance, height-50+clearance
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = color
x1, x2 = width-300-clearance, width-300+120+clearance
y1, y2 = 450-75-clearance, 450+clearance
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = color

x1, x2 = width-300, width-100
y1, y2 = 50, 125
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = 0
x1, x2 = width-180, width-100
y1, y2 = 50, height-50
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = 0
x1, x2 = width-300, width-300+120
y1, y2 = 450-75, 450
for x in range(x1, x2):
    for y in range(y1, y2):
        canvas[y, x] = 0

