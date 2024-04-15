# A* Path Planning

![astar new test new](https://github.com/Apoorv-1009/Astar-Path-Planner/assets/57452076/bc0d2384-d7d2-4e84-bde3-01fc841c4c3f)

## Description:
This project implements the A* search algorithm for path planning in a given environment. The program takes into account obstacles, robot clearance, and radius to find an optimal path from a start point to a goal point.

## Dependencies:
- Python 3.x
- OpenCV (cv2)
- Heapq
- math
- time
- NumPy

## How to Run:
1. Run the Python script `a_star_apoorvthapliyal#1.py`.
2. Follow the prompts to enter the clearance, radius of the robot, and step size.
3. Enter the start and goal coordinates along with their respective angles (in degrees).
4. The program will then calculate the optimal path using the A* algorithm and display a visualization of the exploration process and the final path.

---

## Note
- Robot radius is hidden, if youd like to visualise it, please change the radius_color tuple to the color of your choice
- The script generates an output video (`astar.mp4`) showing the progression of the algorithm and the final result.
