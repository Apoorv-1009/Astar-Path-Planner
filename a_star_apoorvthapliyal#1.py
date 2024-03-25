# Link to github: https://github.com/Apoorv-1009/Astar-Path-Planner/tree/main

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

# clearance = 10
# L = 10

########## Step 1: Define the action cost set ##########

# Define the cost of each action
value = L
action_cost_set = {(-60, value), (-30, value), (0, value), (30, value), (60, value)}
# action_cost_set = {(60, value), (30, L), (0, L), (-30, L), (-60, L)}

########## STEP 2: MATHEMATICAL REPRESENTATION OF FREE SPACE ##########

distance_threshold = 0.5
angular_threshold = 30

# Define the height and width of the canvas
height = 500
width = 1200
color = (0, 255, 255)

# Please change this color to visualise the robot radius padding 
radius_color = (254, 254, 254)

# Make an empty canvas
canvas = np.zeros((height, width, 3), dtype=np.uint8)
# Draw a white rectangle from (clearance, clearance) to (width-clearance, height-clearance) on the canvas for wall padding
cv2.rectangle(canvas, (clearance+radius, clearance+radius), (width-clearance-radius, height-clearance-radius), (255, 255, 255), -1)

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

########## Step 3: Implement A* search algorithm to search the tree and to find the optimal path ##########
        
# Enter the start and goal nodes with bottom left as origin
# Take input from the user, till its not in the obstacle space
while True:
    x_start = int(input('Enter start x position' + f'({clearance+radius}-{width-clearance-radius-1}): '))
    y_start = int(input('Enter start y position' + f'({clearance+radius}-{height-clearance-radius-1}): '))
    theta_start = int(input('Enter start theta position (+180 to -180): '))

    y_start = height-y_start-1
    try:
        if canvas[y_start, x_start, 0] == 255 and 180 >= theta_start >= -180:
            break
    except:
        print('Invalid input, re-enter the start node position')
    else:
        print('The start node is in the obstacle space, re-enter the goal node position')

while True:
    x_goal = int(input('Enter goal x position' + f'({clearance+radius}-{width-clearance-radius-1}): '))
    y_goal = int(input('Enter goal y position' + f'({clearance+radius}-{height-clearance-radius-1}): '))
    theta_goal = int(input('Enter goal theta position (+180 to -180): '))

    y_goal = height-y_goal-1
    try:
        if canvas[y_goal, x_goal, 0] == 255 and 180 >= theta_goal >= -180:
            break
    except:
        print('Invalid input, re-enter the goal node position')
    else:
        print('The goal node is in the obstacle space, re-enter the goal node position')

print("Positions accepted! Calculating path...")
        
# x_start, y_start, theta_start = clearance+1, height-clearance-1, 0
# x_goal, y_goal, theta_goal = width-clearance-1, height-clearance-1, 0

# Make a lambda function to adjust the value of x to the visited space
adjust = lambda x, threshold: int(round(x*2)/2)/threshold        

q = []
heapq.heappush(q, (0, x_start, y_start, theta_start))

# Dictionary to store visited nodes
visited = {(adjust(x_start, distance_threshold),
            adjust(y_start, distance_threshold),
            adjust(theta_start, angular_threshold)): 1}

# Dictionary to store the parent of each node
parent = {(x_start, y_start, theta_start): (x_start, y_start, theta_start)}
# Dictionary to store the cost to come of each node
cost_to_come = {(adjust(x_start, distance_threshold),
                 adjust(y_start, distance_threshold),
                 adjust(theta_start, angular_threshold)): 0}
# Dictionary to store the cost of each node
cost = {(adjust(x_start, distance_threshold),
         adjust(y_start, distance_threshold),
         adjust(theta_start, angular_threshold)): 0}

reached = False
start = time.time()
while q:
    _, x, y, theta = heapq.heappop(q)

    # Get the cost to come of the current node
    c2c = cost_to_come[(adjust(x, distance_threshold),
                        adjust(y, distance_threshold),
                        adjust(theta, angular_threshold))]

    if dist((x, y), (x_goal, y_goal)) <= L//2 and abs(theta-theta_goal) <= angular_threshold:
        end = time.time()
        # Print time in minutes and seconds
        print("Time taken: ", int((end-start)/60), "minutes", int((end-start)%60), "seconds")
        # print("Goal reached: ", end-start, "seconds")
        reached = True
        x_achieved, y_achieved, theta_achieved = x, y, theta
        break

    for angle, action_cost in action_cost_set:
        steer = theta + angle
        # Keep the steering angle within 180 and -180
        if steer > 180:
            steer -= 360
        elif steer < -180:
            steer += 360

        # Get the new node
        x_new = np.round(x + L*np.cos(np.deg2rad(steer)), 3)
        y_new = np.round(y + L*np.sin(np.deg2rad(steer)), 3)

        # Cap the new node values within the boundaries of the canvas
        x_new = max(clearance, min(width-clearance, x_new))
        y_new = max(clearance, min(height-clearance, y_new))

        # Adjust the values for the canvas
        x_cvs = int(round(x_new*2)/2)
        y_cvs = int(round(y_new*2)/2)
        theta_cvs = int(round(steer*2)/2)

        # Adjust the values for the visited dictionary
        x_vis = adjust(x_new, distance_threshold)
        y_vis = adjust(y_new, distance_threshold)
        theta_vis = adjust(theta_cvs, angular_threshold)

        # Check if the new node is within the boundaries of the canvas
        if 0 <= x_new < width and 0 <= y_new < height and canvas[y_cvs, x_cvs, 0] == 255:

            # Check if the new node is not visited
            if (x_vis, y_vis, theta_vis) not in visited:
                # Store the parent of the node
                parent[(x_new, y_new, steer)] = (x, y, theta)

                cost_to_come[(x_vis, y_vis, theta_vis)] = c2c + action_cost
                cost[(x_vis, y_vis, theta_vis)] = cost_to_come[(x_vis, y_vis, theta_vis)] + dist((x_new, y_new), (x_goal, y_goal))

                heapq.heappush(q, (cost[(x_vis, y_vis, theta_vis)], x_new, y_new, steer))
                visited[(x_vis, y_vis, theta_vis)] = 1

            # If the node is visited, check if the new cost is less than the previous cost
            elif cost_to_come[(x_vis, y_vis, theta_vis)] > c2c + action_cost: 
                parent[(x_new, y_new, steer)] = (x, y, theta)
                cost_to_come[(x_vis, y_vis, theta_vis)] = c2c + action_cost 
                cost[x_vis, y_vis, theta_vis] = cost_to_come[(x_vis, y_vis, theta_vis)] + dist((x_new, y_new), (x_goal, y_goal))
                
if not reached:
    print('Goal could not be reached')
    print("Exiting...")
    exit()

########## STEP 4: OPTIMAL PATH ##########
    
# Get the path from the parent dictionary
path = []
# x, y = x_goal, y_goal   
x, y, theta = x_achieved, y_achieved, theta_achieved
while (x, y, theta) != (x_start, y_start, theta_start):
    # print(x, y)
    path.append((x, y))
    x, y, theta = parent[(x, y, theta)]
path.append((x, y))
path.reverse()

########## STEP 5: REPRESENT THE OPTIMAL PATH ##########

# Start a video writer in mp4 format
astar = cv2.VideoWriter('astar.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 50, (width, height))

# Draw the start and goal nodes on the canvas
cv2.circle(canvas, (x_start, y_start), 10, (0, 255, 0), -1)
cv2.circle(canvas, (x_goal, y_goal), 10, (0, 165, 255), -1)

# Draw on every threshold frame
threshold = 200
counter = 0

# Draw the visited nodes on the canvas as an arrow going from the parent to the child
for x, y, theta in parent:
    counter += 1
    cv2.arrowedLine(canvas, (int(parent[(x, y, theta)][0]), int(parent[(x, y, theta)][1])), (int(x), int(y)), (255, 0, 0), 1)
    if(counter == threshold):
        cv2.imshow('Canvas', canvas)
        astar.write(canvas)
        cv2.waitKey(1)  
        counter = 0

# Draw the start and goal nodes on the canvas
cv2.circle(canvas, (x_start, y_start), 10, (0, 255, 0), -1)
cv2.circle(canvas, (x_goal, y_goal), 10, (0, 165, 255), -1)

# Draw the path on the canvas
for i in range(len(path)-1):
    # Draw a red dot at path points
    cv2.circle(canvas, (int(path[i][0]), int(path[i][1])), 1, (0, 0, 255), 3)
    # cv2.line(canvas, (int(path[i][0]), int(path[i][1])), (int(path[i+1][0]), int(path[i+1][1])), (0, 0, 255), 2)
    cv2.imshow('Canvas', canvas)
    astar.write(canvas)

# Release VideoWriter
astar.release()
cv2.waitKey(0)
cv2.destroyAllWindows()