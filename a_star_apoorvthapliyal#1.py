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

