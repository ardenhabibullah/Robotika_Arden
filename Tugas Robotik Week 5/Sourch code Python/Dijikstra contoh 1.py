import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue

# Graph representation using an adjacency matrix
# The graph is represented as a 2D list
graph = [
    [0, 7, 9, 0, 0, 14],
    [7, 0, 10, 15, 0, 0],
    [9, 10, 0, 11, 0, 2],
    [0, 15, 11, 0, 6, 0],
    [0, 0, 0, 6, 0, 9],
    [14, 0, 2, 0, 9, 0]
]

# Number of nodes
num_nodes = len(graph)

def dijkstra(start, goal):
    """Implement Dijkstra's algorithm to find the shortest path."""
    # Priority queue to hold the nodes to explore
    pq = PriorityQueue()
    pq.put((0, start))  # (cost, node)

    # Dictionary to store the shortest cost to reach each node
    shortest_cost = {node: float('inf') for node in range(num_nodes)}
    shortest_cost[start] = 0

    # Dictionary to store the best previous node to reconstruct the path
    previous_node = {node: None for node in range(num_nodes)}

    while not pq.empty():
        current_cost, current_node = pq.get()

        # If we reach the goal, reconstruct the path
        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = previous_node[current_node]
            return path[::-1], shortest_cost[goal]  # Return reversed path and cost

        # Explore neighbors
        for neighbor in range(num_nodes):
            if graph[current_node][neighbor] > 0:  # If there is a connection
                cost = graph[current_node][neighbor]
                new_cost = current_cost + cost

                # If the new cost is lower, update the data structures
                if new_cost < shortest_cost[neighbor]:
                    shortest_cost[neighbor] = new_cost
                    previous_node[neighbor] = current_node
                    pq.put((new_cost, neighbor))

    return None, float('inf')  # No path found

# Define start and goal nodes
start_node = 0  # Starting from node 0
goal_node = 4   # Ending at node 4

# Find the shortest path using Dijkstra's algorithm
path, total_cost = dijkstra(start_node, goal_node)

# Print the results
if path is not None:
    print(f"Shortest path from node {start_node} to node {goal_node}: {path}")
    print(f"Total cost: {total_cost}")
else:
    print("No path found.")

# Plotting the graph and the shortest path
def plot_graph(graph, path):
    """Plot the graph and the shortest path."""
    plt.figure(figsize=(8, 6))
    
    # Node positions for visualization
    positions = {
        0: (0, 0), 1: (1, 2), 2: (2, 1), 3: (3, 2), 4: (4, 0), 5: (5, 1)
    }

    # Plot edges
    for i in range(num_nodes):
        for j in range(num_nodes):
            if graph[i][j] > 0:  # If there is an edge
                plt.plot(
                    [positions[i][0], positions[j][0]],
                    [positions[i][1], positions[j][1]],
                    'k-', alpha=0.5
                )

    # Highlight the shortest path
    if path:
        for i in range(len(path) - 1):
            plt.plot(
                [positions[path[i]][0], positions[path[i + 1]][0]],
                [positions[path[i]][1], positions[path[i + 1]][1]],
                'b-', linewidth=2, label='Shortest Path' if i == 0 else ""
            )

    # Plot nodes
    for node, (x, y) in positions.items():
        plt.scatter(x, y, s=200, label=f'Node {node}')
        plt.text(x, y, str(node), ha='center', va='center')

    plt.title('Dijkstra Algorithm - Shortest Path Visualization')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Call the plot function to display the graph
plot_graph(graph, path)
