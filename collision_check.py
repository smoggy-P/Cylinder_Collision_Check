import numpy as np


def distance_between_point_and_line(point, line_point1, line_point2):
    return np.linalg.norm(np.cross(point - line_point1, point - line_point2)) / np.linalg.norm(line_point2 - line_point1)

def check_collision(cylinder1, cylinder2):
    """Checks if two cylinders are colliding.

    Args:
        cylinder1 (Cylinder Direct): The first cylinder.
        cylinder2 (Cylinder Direct): The second cylinder.

    Returns:
        bool: True if the cylinders are colliding, False otherwise.
    """

    d1 = cylinder1['direct'] / np.linalg.norm(cylinder1['direct'])
    d2 = cylinder2['direct'] / np.linalg.norm(cylinder2['direct'])

    
    a = np.dot(d1, d1)
    b = -np.dot(d1, d2)
    c = np.dot(d2, d2)
    d = np.dot(d1, cylinder1['center'] - cylinder2['center'])
    denominator = a * c - b * b

    if denominator == 0:
        # Parallel
        distance = distance_between_point_and_line(cylinder1['center'], cylinder2['center'], cylinder2['center'] + d2)
        if distance >= cylinder1['radius'] + cylinder2['radius']:
            # Not colliding for infinite cylinders
            return False
        elif 

    else:
        # Not parallel
        t1 = (b * np.dot(d2, cylinder2['center'] - cylinder1['center']) + c * d) / denominator
        t2 = (a * np.dot(d2, cylinder2['center'] - cylinder1['center']) - b * d) / denominator
        pA = cylinder1['center'] + t1 * d1
        pB = cylinder2['center'] + t2 * d2

        if np.linalg.norm(pA - pB) >= cylinder1['radius'] + cylinder2['radius']:
            # Not colliding for infinite cylinders