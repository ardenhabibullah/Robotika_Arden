import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue

# Define grid size and obstacles
grid_size = (10, 10)
start = (0, 0)
goal = (9, 9)
obstacles = [(3, 3), (3, 4), (3, 5), (5, 5), (6, 5), (7, 5)]

# Create grid
grid = np.zeros(grid_size)
for obs in obstacles:
    grid[obs] = 1  # Mark obstacles as 1

# Directions for movement (right, left, down, up)
directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

def heuristic(a, b):
    """Calculate the heuristic (Manhattan distance) between two points."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal):
    """Perform A* search algorithm to find a path from start to goal."""
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while not open_set.empty():
        current = open_set.get()[1]
        
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Return reversed path
        
        # Explore neighbors
        for d in directions:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if (0 <= neighbor[0] < grid_size[0] and
                0 <= neighbor[1] < grid_size[1] and
                grid[neighbor] == 0):  # Valid neighbor
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))

    return None  # No path found

# Find path using A*
path = a_star(start, goal)

# Plotting the grid, obstacles, and path
plt.figure(figsize=(8, 8))
plt.imshow(grid, cmap='binary')

# Plot obstacles
for obs in obstacles:
    plt.scatter(obs[1], obs[0], color='red', label='Obstacle')

# Plot path
if path:
    path_x, path_y = zip(*path)
    plt.plot(path_y, path_x, color='blue', linewidth=2, label='Path')
else:
    print("No path found")

# Mark start and goal points
plt.scatter(start[1], start[0], color='green', s=100, label='Start')
plt.scatter(goal[1], goal[0], color='purple', s=100, label='Goal')

plt.title('A* Pathfinding Algorithm')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.gca().invert_yaxis()
plt.legend()
plt.grid(True)
plt.show()
