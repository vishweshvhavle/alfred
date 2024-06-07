
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import random
from heapq import heappop, heappush

# Load the point cloud
file_name = "gazebo_env.pcd"
pc = o3d.io.read_point_cloud(file_name)
points = np.asarray(pc.points)

# Filter points and project onto 2D plane
filtered_points = []
for point in points:
    if 0.10 <= point[2] <= 2.5:  # Keep points within specific height limits
        filtered_points.append([point[0], point[1]])
filtered_points = np.array(filtered_points)

# Determine grid cell size
cell_size = 0.45

# Find the minimum and maximum boundaries
min_bound = np.min(filtered_points, axis=0)
max_bound = np.max(filtered_points, axis=0)

# Compute grid size (number of cells in X and Y directions)
grid_x = int(np.ceil((max_bound[0] - min_bound[0]) / cell_size))
grid_y = int(np.ceil((max_bound[1] - min_bound[1]) / cell_size))

# Initialize an empty grid with color channels
grid_image = np.zeros((grid_y, grid_x, 3), dtype=np.uint8)

# Initialize occupancy grid (binary)
occupancy_grid = np.zeros((grid_y, grid_x), dtype=float)

# Directions for neighboring cells
directions = [
    (-1, 0), (1, 0), (0, -1), (0, 1),  # N, S, W, E
    (-1, -1), (-1, 1), (1, -1), (1, 1)  # NW, NE, SW, SE
]

# Populate the grid with white for occupied cells
for point in filtered_points:
    grid_i = int((point[0] - min_bound[0]) / cell_size)
    grid_j = int((point[1] - min_bound[1]) / cell_size)
    grid_image[grid_j, grid_i] = [255, 255, 255]  # Mark occupied as white
    occupancy_grid[grid_j, grid_i] = 1  # Mark occupied as 1

    # Mark surrounding boxes with 1 unless already marked with 1
    for dx, dy in directions:
        ni, nj = grid_j + dy, grid_i + dx
        if 0 <= ni < grid_y and 0 <= nj < grid_x and occupancy_grid[ni, nj] != 1:
            occupancy_grid[ni, nj] = 1
            grid_image[ni, nj] = [127, 127, 127]  # Mark half-occupied as gray

# Calculate the grid index for the origin (0, 0)
origin_x = int((0 - min_bound[0]) / cell_size)
origin_y = int((0 - min_bound[1]) / cell_size)

# Find a random unoccupied cell for the goal
unoccupied_cells = np.argwhere(occupancy_grid == 0)
goal_y, goal_x = [3, 24]
# goal_y, goal_x = random.choice(unoccupied_cells)

# A* Algorithm
def heuristic(cell, goal):
    return np.sqrt((cell[0] - goal[0]) ** 2 + (cell[1] - goal[1]) ** 2)

def neighbors(cell):
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),   # N, S, W, E
        (-1, -1), (-1, 1), (1, -1), (1, 1)  # NW, NE, SW, SE
    ]
    for dx, dy in directions:
        new_x, new_y = cell[0] + dx, cell[1] + dy
        if 0 <= new_x < grid_y and 0 <= new_y < grid_x and occupancy_grid[new_x, new_y] != 1:
            yield new_x, new_y

def a_star_search(start, goal):
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in neighbors(current):
            tentative_g_score = g_score[current] + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(open_set, (f_score[neighbor], neighbor))
    return []

# Run A* from origin to the goal
path = a_star_search((origin_y, origin_x), (goal_y, goal_x))

# Plot the path
for y, x in path:
    grid_image[y, x] = [0, 0, 255]  # Mark path in blue

# Mark the origin in green
if 0 <= origin_x < grid_x and 0 <= origin_y < grid_y:
    grid_image[origin_y, origin_x] = [0, 255, 0]

# Mark the goal in red
grid_image[goal_y, goal_x] = [255, 0, 0]

# Display the grid with the path
plt.imshow(grid_image, origin='lower')
plt.xlabel('Grid X')
plt.ylabel('Grid Y')
plt.title('Occupancy Grid with A* Path')
plt.show()
