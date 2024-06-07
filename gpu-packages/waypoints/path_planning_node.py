import rospy
import numpy as np
import open3d as o3d
from utils import *
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R

from heapq import heappop, heappush
import os
from matplotlib import pyplot as plt


current_dir = os.path.dirname(os.path.abspath(__file__))
file_name = os.path.join(current_dir, "gazebo_env.pcd")

# Load the point cloud
pc = o3d.io.read_point_cloud(file_name)
points = np.asarray(pc.points)


# Filter points and project onto the 2D plane
filtered_points = []
for point in points:
    if 0.10 <= point[2] <= 2.5:  # Keep points within specific height limits
        filtered_points.append([point[0], point[1]])
filtered_points = np.array(filtered_points)
# 90 rotation about the Z-axis

rotation_matrix = np.array([[0, 1], [-1, 0]])
filtered_points = np.dot(filtered_points, rotation_matrix)

# Determine grid cell size
cell_size = 0.35

# Find the minimum and maximum boundaries
min_bound = np.min(filtered_points, axis=0)
max_bound = np.max(filtered_points, axis=0)

# Compute grid size (number of cells in X and Y directions)
grid_x = int(np.ceil((max_bound[0] - min_bound[0]) / cell_size))
grid_y = int(np.ceil((max_bound[1] - min_bound[1]) / cell_size))
grid_image = np.zeros((grid_y, grid_x, 3), dtype=np.uint8)

# Initialize the occupancy grid (binary)
occupancy_grid = np.zeros((grid_y, grid_x), dtype=float)

# Directions for neighboring cells
directions = [
    (-1, 0), (1, 0), (0, -1), (0, 1),  # N, S, W, E
    (-1, -1), (-1, 1), (1, -1), (1, 1)  # NW, NE, SW, SE
]

# Populate the grid with occupied cells and mark surrounding boxes with 0.5
for point in filtered_points:
    grid_i = int((point[0] - min_bound[0]) / cell_size)
    grid_j = int((point[1] - min_bound[1]) / cell_size)
    occupancy_grid[grid_j, grid_i] = 1  # Mark occupied as 1
    grid_image[grid_j, grid_i] = [255, 255, 255]  # Mark occupied as white

    for dx, dy in directions:
        ni, nj = grid_j + dy, grid_i + dx
        if 0 <= ni < grid_y and 0 <= nj < grid_x and occupancy_grid[ni, nj] != 1:
            occupancy_grid[ni, nj] = 1
            grid_image[ni, nj] = [127, 127, 127]  # Mark half-occupied as gray

# Calculate the grid index for the origin (0, 0)
origin_x = int((0 - min_bound[0]) / cell_size)
origin_y = int((0 - min_bound[1]) / cell_size)

# A* Algorithm
def heuristic(cell, goal):
    return np.sqrt((cell[0] - goal[0]) ** 2 + (cell[1] - goal[1]) ** 2)

def neighbors(cell):
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

# Function to convert grid path to PCD's frame of reference
def grid_to_world(grid_coords):
    return [
        [
            min_bound[0] + coord[1] * cell_size,
            min_bound[1] + coord[0] * cell_size
        ]
        for coord in grid_coords
    ]


def select_corner_points(path):
    if len(path) < 3:
        return path
    
    corner_points = [path[0]]
    for i in range(1, len(path) - 1):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        x3, y3 = path[i + 1]
        if (x3 - x1) * (y2 - y1) - (x2 - x1) * (y3 - y1) != 0:
            corner_points.append(path[i])
    
    corner_points.append(path[-1])

    return corner_points

def subsample_path(path, step_size=4):
    sub_path = []
    for i in range(0, len(path), step_size):
        sub_path.append(path[i])
    return sub_path


def create_beizer_curve(points, samples_per_bezier=10):
    control_points = []
    tangent_points = []
    for point in points:
        control_points.append([point[0], point[1], 0])
        add_tangent_point(control_points, tangent_points)

    bezier_points, yaw_angles = calculate_piecewise_cubic_bezier_with_yaw(control_points, tangent_points, samples_per_bezier)
    return bezier_points, yaw_angles

# Callback to receive the goal and perform path planning
def goal_callback(data):
    rospy.loginfo("Clicked Point: [%f, %f, %f]", data.point.x, data.point.y, data.point.z)

    goal_x = int((data.point.x - min_bound[0]) / cell_size)
    goal_y = int((data.point.y - min_bound[1]) / cell_size)
    rospy.loginfo("Goal in grid coordinates: [%d, %d]", goal_x, goal_y)
    
    if 0 <= goal_x < grid_x and 0 <= goal_y < grid_y:
        rospy.loginfo("Running A* search...")
        path = a_star_search((origin_y, origin_x), (goal_y, goal_x))
        rospy.loginfo("Path found with %d waypoints", len(path))

        if not path:
            rospy.logwarn("No path found!")
            return

        # Subsample the path and select corner points
        path = subsample_path(path, step_size=4)
        path = select_corner_points(path)

        # Convert path to world coordinates
        waypoints = grid_to_world(path)        
        bezier_points, yaw_angles = create_beizer_curve(waypoints, samples_per_bezier=30)
        
        # Create and publish a MarkerArray
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.id = i
            marker_array.markers.append(marker)
        
        waypoints_pub.publish(marker_array)



        if len((bezier_points)) > 4:
            bezier_marker_array = MarkerArray()
            for i, (point, yaw) in enumerate(zip(bezier_points, yaw_angles)):
                r = R.from_euler('xyz', [0, 0, yaw], degrees=False)
                r_quat = r.as_quat()

                marker = Marker()
                marker.header = Header()
                marker.header.frame_id = "odom"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.id = len(bezier_marker_array.markers)
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = point[2]
                marker.pose.orientation.x = r_quat[0]
                marker.pose.orientation.y = r_quat[1]
                marker.pose.orientation.z = r_quat[2]
                marker.pose.orientation.w = r_quat[3]
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                bezier_marker_array.markers.append(marker)

            trajectory_pub.publish(bezier_marker_array)


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
        # non blocking
        # plt.show(block=False)
        # blocking
        plt.show()  



try:
    rospy.init_node('waypoints_node', anonymous=True)
    rospy.Subscriber("/clicked_point", PointStamped, goal_callback)

    # Initialize the ROS node
    waypoints_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=1)
    trajectory_pub = rospy.Publisher("/trajectory", MarkerArray, queue_size=1)

    # Keep the node alive
    rospy.spin()

except rospy.ROSInterruptException:
        pass
except KeyboardInterrupt:
    pass
except Exception as e:
    print(e)
    pass




# import numpy as np
# import open3d as o3d
# import matplotlib.pyplot as plt
# import random
# from heapq import heappop, heappush

# # Load the point cloud
# file_name = "gazebo_env.pcd"
# pc = o3d.io.read_point_cloud(file_name)
# points = np.asarray(pc.points)

# # Filter points and project onto 2D plane
# filtered_points = []
# for point in points:
#     if 0.10 <= point[2] <= 2.5:  # Keep points within specific height limits
#         filtered_points.append([point[0], point[1]])
# filtered_points = np.array(filtered_points)

# # Determine grid cell size
# cell_size = 0.45

# # Find the minimum and maximum boundaries
# min_bound = np.min(filtered_points, axis=0)
# max_bound = np.max(filtered_points, axis=0)

# # Compute grid size (number of cells in X and Y directions)
# grid_x = int(np.ceil((max_bound[0] - min_bound[0]) / cell_size))
# grid_y = int(np.ceil((max_bound[1] - min_bound[1]) / cell_size))

# # Initialize an empty grid with color channels
# grid_image = np.zeros((grid_y, grid_x, 3), dtype=np.uint8)

# # Initialize occupancy grid (binary)
# occupancy_grid = np.zeros((grid_y, grid_x), dtype=float)

# # Directions for neighboring cells
# directions = [
#     (-1, 0), (1, 0), (0, -1), (0, 1),  # N, S, W, E
#     (-1, -1), (-1, 1), (1, -1), (1, 1)  # NW, NE, SW, SE
# ]

# # Populate the grid with white for occupied cells
# for point in filtered_points:
#     grid_i = int((point[0] - min_bound[0]) / cell_size)
#     grid_j = int((point[1] - min_bound[1]) / cell_size)
#     grid_image[grid_j, grid_i] = [255, 255, 255]  # Mark occupied as white
#     occupancy_grid[grid_j, grid_i] = 1  # Mark occupied as 1

#     # Mark surrounding boxes with 1 unless already marked with 1
#     for dx, dy in directions:
#         ni, nj = grid_j + dy, grid_i + dx
#         if 0 <= ni < grid_y and 0 <= nj < grid_x and occupancy_grid[ni, nj] != 1:
#             occupancy_grid[ni, nj] = 1
#             grid_image[ni, nj] = [127, 127, 127]  # Mark half-occupied as gray

# # Calculate the grid index for the origin (0, 0)
# origin_x = int((0 - min_bound[0]) / cell_size)
# origin_y = int((0 - min_bound[1]) / cell_size)

# # Find a random unoccupied cell for the goal
# unoccupied_cells = np.argwhere(occupancy_grid == 0)
# goal_y, goal_x = random.choice(unoccupied_cells)

# # A* Algorithm
# def heuristic(cell, goal):
#     return np.sqrt((cell[0] - goal[0]) ** 2 + (cell[1] - goal[1]) ** 2)

# def neighbors(cell):
#     directions = [
#         (-1, 0), (1, 0), (0, -1), (0, 1),   # N, S, W, E
#         (-1, -1), (-1, 1), (1, -1), (1, 1)  # NW, NE, SW, SE
#     ]
#     for dx, dy in directions:
#         new_x, new_y = cell[0] + dx, cell[1] + dy
#         if 0 <= new_x < grid_y and 0 <= new_y < grid_x and occupancy_grid[new_x, new_y] != 1:
#             yield new_x, new_y

# def a_star_search(start, goal):
#     open_set = []
#     heappush(open_set, (0, start))
#     came_from = {}
#     g_score = {start: 0}
#     f_score = {start: heuristic(start, goal)}

#     while open_set:
#         _, current = heappop(open_set)
#         if current == goal:
#             path = []
#             while current in came_from:
#                 path.append(current)
#                 current = came_from[current]
#             path.append(start)
#             return path[::-1]

#         for neighbor in neighbors(current):
#             tentative_g_score = g_score[current] + heuristic(current, neighbor)
#             if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                 came_from[neighbor] = current
#                 g_score[neighbor] = tentative_g_score
#                 f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
#                 heappush(open_set, (f_score[neighbor], neighbor))
#     return []

# # Run A* from origin to the goal
# path = a_star_search((origin_y, origin_x), (goal_y, goal_x))

# # Plot the path
# for y, x in path:
#     grid_image[y, x] = [0, 0, 255]  # Mark path in blue

# # Mark the origin in green
# if 0 <= origin_x < grid_x and 0 <= origin_y < grid_y:
#     grid_image[origin_y, origin_x] = [0, 255, 0]

# # Mark the goal in red
# grid_image[goal_y, goal_x] = [255, 0, 0]

# # Display the grid with the path
# plt.imshow(grid_image, origin='lower')
# plt.xlabel('Grid X')
# plt.ylabel('Grid Y')
# plt.title('Occupancy Grid with A* Path')
# plt.show()
