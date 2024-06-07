import math
import csv

def add_vectors(v1, v2):
    return [a + b for a, b in zip(v1, v2)]

def sub_vectors(v1, v2):
    return [a - b for a, b in zip(v1, v2)]

def scale_vector(v, scalar):
    return [a * scalar for a in v]

def magnitude(v):
    return math.sqrt(sum(a**2 for a in v))

def normalize_vector(v):
    mag = magnitude(v)
    return [a / mag for a in v]

def dot_product(v1, v2):
    return sum(a * b for a, b in zip(v1, v2))


def calculate_yaw_angle(tangent):
    reference = [1, 0]  # x-axis
    return angle_between_vectors(reference, tangent)

def angle_between_vectors(v1, v2):
    dot = dot_product(v1, v2)
    mag_v1 = magnitude(v1)
    mag_v2 = magnitude(v2)
    return math.degrees(math.acos(dot / (mag_v1 * mag_v2)))

# Tangent point initialization
def init_tangent_point(control_points, tangent_points, cur_idx):
    prev_idx = cur_idx - 1
    next_idx = cur_idx + 1

    p_cur = control_points[cur_idx]
    p_prev = control_points[prev_idx]
    p_next = control_points[next_idx]

    v_left = sub_vectors(p_prev, p_cur)
    v_right = sub_vectors(p_next, p_cur)

    unit_v_left = normalize_vector(v_left)
    unit_v_right = normalize_vector(v_right)

    bisector = add_vectors(unit_v_left, unit_v_right)

    d_product = dot_product(v_left, bisector)
    proj = d_product / magnitude(bisector)

    unit_bisector = normalize_vector(bisector)
    parallel_proj = scale_vector(unit_bisector, proj)

    perpendicular_proj = sub_vectors(v_left, parallel_proj)
    unit_perp_proj = normalize_vector(perpendicular_proj)

    mag_v_left = magnitude(v_left)
    mag_v_right = magnitude(v_right)
    mag_tangent_point = min(mag_v_left, mag_v_right) / 2

    v_left_tangent = add_vectors(p_cur, scale_vector(unit_perp_proj, mag_tangent_point))
    v_right_tangent = sub_vectors(scale_vector(p_cur, 2.0), v_left_tangent)

    tangent_points[cur_idx] = [v_left_tangent, v_right_tangent]

# Adding tangent points
def add_tangent_point(control_points, tangent_points):
    num_control_points = len(control_points)
    cur_tangent_points = [control_points[-1], control_points[-1]]
    tangent_points.append(cur_tangent_points)

    if num_control_points > 2:
        cur_idx = num_control_points - 2
        init_tangent_point(control_points, tangent_points, cur_idx)

# Cubic Bezier calculation
def get_cubic_bezier(p0, p1, p2, p3, t):
    c0 = (1 - t) ** 3
    c1 = 3 * (1 - t) ** 2 * t
    c2 = 3 * (1 - t) * t ** 2
    c3 = t ** 3

    v0 = scale_vector(p0, c0)
    v1 = scale_vector(p1, c1)
    v2 = scale_vector(p2, c2)
    v3 = scale_vector(p3, c3)

    return add_vectors(add_vectors(v0, v1), add_vectors(v2, v3))

def get_cubic_bezier_derivative(p0, p1, p2, p3, t):
    c0 = -3 * (1 - t) ** 2
    c1 = 3 * (1 - t) ** 2 - 6 * t * (1 - t)
    c2 = 6 * t * (1 - t) - 3 * t ** 2
    c3 = 3 * t ** 2

    v0 = scale_vector(p0, c0)
    v1 = scale_vector(p1, c1)
    v2 = scale_vector(p2, c2)
    v3 = scale_vector(p3, c3)

    return add_vectors(add_vectors(v0, v1), add_vectors(v2, v3))

def calculate_yaw_angle(tangent):
    reference = [1, 0]  # x-axis
    return angle_between_vectors(reference, tangent)


def calculate_piecewise_cubic_bezier_with_yaw(control_points, tangent_points, samples_per_bezier):
    cubic_bezier = []
    yaw_angles = []
    num_control_points = len(control_points)

    for i in range(num_control_points - 1):
        p0 = control_points[i]
        p3 = control_points[i + 1]
        p1 = tangent_points[i][1]
        p2 = tangent_points[i + 1][0]

        cubic_bezier.append(p0)
        t = 0.0
        delta_t = 1.0 / (samples_per_bezier - 1.0)
        for j in range(1, samples_per_bezier - 1):
            t += delta_t
            bezier_point = get_cubic_bezier(p0, p1, p2, p3, t)
            tangent = get_cubic_bezier_derivative(p0, p1, p2, p3, t)
            yaw_angle = calculate_yaw_angle(tangent)

            cubic_bezier.append(bezier_point)
            yaw_angles.append(yaw_angle)

    return cubic_bezier, yaw_angles

# def draw_arrow(screen, p, direction, length=20, color=(255, 0, 0)):
#     endpoint = add_vectors(p, scale_vector(normalize_vector(direction), length))
#     pygame.draw.line(screen, color, p, endpoint)
#     pygame.draw.circle(screen, color, endpoint, 3)


def write_bezier_with_yaw_to_csv(bezier_points, yaw_angles, filename='trajectory.csv'):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'yaw'])
        for point, yaw in zip(bezier_points, yaw_angles):
            writer.writerow([int(point[0]), int(point[1]), yaw])
