import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from heapq import heappop, heappush

# Define area and obstacles
area = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
obstacles = [
    np.array([[2, 3], [4, 3], [4, 5], [2, 5]]),  # Rectangular obstacle
    np.array([[6, 1], [8, 1], [7, 4]]),  # Triangular obstacle
]

start = (1, 1)
goal = (9, 9)

# Function to check if a point is inside an obstacle
def point_in_obstacles(point):
    for obs in obstacles:
        if Polygon(obs).contains_point(point):
            return True
    return False

# A* search algorithm
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def a_star_search(start, goal):
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        # Explore neighbors
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Left, Right, Down, Up
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < area[1][0] and 
                0 <= neighbor[1] < area[2][1] and 
                not point_in_obstacles(neighbor)):  # Valid neighbor
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

# Find path using A*
path = a_star_search(start, goal)

# Visualization
plt.figure(figsize=(10, 10))

# Draw obstacles
for obs in obstacles:
    plt.fill(*zip(*obs), color='red', alpha=0.6)

# Draw the area boundary
plt.plot(area[:, 0], area[:, 1], color='black')

# Draw the path
if path:
    path_x, path_y = zip(*path)
    plt.plot(path_x, path_y, color='blue', linewidth=2, label='Path')

# Mark start and goal points
plt.scatter(*start, color='green', s=100, label='Start')
plt.scatter(*goal, color='purple', s=100, label='Goal')

plt.xlim(-1, 11)
plt.ylim(-1, 11)
plt.gca().set_aspect('equal', adjustable='box')
plt.title('Cell Decomposition Path Planning')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.grid(True)
plt.show()
