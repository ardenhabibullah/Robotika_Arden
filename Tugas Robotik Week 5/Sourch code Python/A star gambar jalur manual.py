import pygame
import math
from queue import PriorityQueue

# Define Colors
GRID_COLOR = (200, 200, 200) 
NODE_COLOR = (255, 255, 255) 
BARRIER_COLOR = (0, 0, 0) 
CLOSED_COLOR = (250, 128, 114) 
END_COLOR = (50, 205, 50) 
PATH_COLOR = (75, 0, 130) 
START_COLOR = (30, 144, 255) 
OPEN_COLOR = (255, 255, 0) 

# Define screen and grid size
WIDTH = 600
ROWS = 20

WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Pathfinding Visualization")

class Node:
    def __init__(self, row, col, width):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.width = width
        self.color = NODE_COLOR
        self.neighbors = []

    def set_color(self, color):
        self.color = color

    def get_pos(self):
        return self.row, self.col

    def draw(self):
        pygame.draw.rect(WIN, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        # Check neighboring cells
        if self.row < ROWS - 1 and grid[self.row + 1][self.col].color != BARRIER_COLOR:  # Down
            self.neighbors.append(grid[self.row + 1][self.col])
        if self.row > 0 and grid[self.row - 1][self.col].color != BARRIER_COLOR:  # Up
            self.neighbors.append(grid[self.row - 1][self.col])
        if self.col < ROWS - 1 and grid[self.row][self.col + 1].color != BARRIER_COLOR:  # Right
            self.neighbors.append(grid[self.row][self.col + 1])
        if self.col > 0 and grid[self.row][self.col - 1].color != BARRIER_COLOR:  # Left
            self.neighbors.append(grid[self.row][self.col - 1])

def heuristic(node1, node2):
    x1, y1 = node1.get_pos()
    x2, y2 = node2.get_pos()
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.set_color(PATH_COLOR)
        draw()

def algorithm(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = heuristic(start, end)
    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                
        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            start.set_color(START_COLOR)
            end.set_color(END_COLOR)
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + heuristic(neighbor, end)
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.set_color(OPEN_COLOR)

        draw()

        if current != start:
            current.set_color(CLOSED_COLOR)
    return False

def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap)
            grid[i].append(node)
    return grid

def draw_grid(rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(WIN, GRID_COLOR, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(WIN, GRID_COLOR, (j * gap, 0), (j * gap, width))

def draw(grid, rows, width):
    WIN.fill(NODE_COLOR)
    for row in grid:
        for node in row:
            node.draw()
    draw_grid(rows, width)
    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    gap = width // rows
    x, y = pos
    row = x // gap
    col = y // gap
    return row, col

def main():
    grid = make_grid(ROWS, WIDTH)
    start = None
    end = None
    running = True

    while running:
        draw(grid, ROWS, WIDTH)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if pygame.mouse.get_pressed()[0]:  # Left Mouse Button
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, WIDTH)
                node = grid[row][col]

                if not start and node != end:
                    start = node
                    start.set_color(START_COLOR)
                elif not end and node != start:
                    end = node
                    end.set_color(END_COLOR)
                elif node != end and node != start:
                    node.set_color(BARRIER_COLOR)

            elif pygame.mouse.get_pressed()[2]:  # Right Mouse Button
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, WIDTH)
                node = grid[row][col]
                node.set_color(NODE_COLOR)
                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                    algorithm(lambda: draw(grid, ROWS, WIDTH), grid, start, end)
                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, WIDTH)

    pygame.quit()

main()
