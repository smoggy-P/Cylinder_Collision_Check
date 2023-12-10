import numpy as np
from scipy.linalg import solve
import math

def distance_between_point_and_line(point, line_point1, line_point2):
    return np.linalg.norm(np.cross(point - line_point1, point - line_point2)) / np.linalg.norm(line_point2 - line_point1)

def on_test(TP2, cylinder1, cylinder2, n):
    d1 = cylinder1['direct'] / np.linalg.norm(cylinder1['direct'])
    d2 = cylinder2['direct'] / np.linalg.norm(cylinder2['direct'])
    c1 = cylinder1['center']
    h1 = cylinder1['height']
    r1 = cylinder1['radius']
    r2 = cylinder2['radius']
    distance = distance_between_point_and_line(TP2, c1, c1 + d1)
    if distance > r1 + r2:
        # No collision
        return 0, {}
    else:
        # Find TP2_prime on the edge of cylinder 2, closest to cylinder 1
        TP2_prime = TP2 + (r2 / np.linalg.norm(n)) * n
        # Find t3_prime so that c1 + t3_prime * d1 is the closest point to TP2_prime
        t3_prime = np.dot(TP2_prime - c1, d1) / np.dot(d1, d1)
        # Check if t3_prime is inside cylinder 1
        if abs(t3_prime) <= h1 / 2:
            if np.linalg.norm(TP2_prime - c1 - t3_prime * d1) <= r1:
                # Collision
                return 1, {}
            else:
                # No collision
                return 0, {}
        else:
            # return TP1 for End test
            return 2, {'TP1':c1 + max(-h1 / 2, min(h1 / 2, t3_prime)) * d1}
        
def end_test(TP1, TP2, cylinder1, cylinder2):
    d1 = cylinder1['direct'] / np.linalg.norm(cylinder1['direct'])
    d2 = cylinder2['direct'] / np.linalg.norm(cylinder2['direct'])
    r1 = cylinder1['radius']
    r2 = cylinder2['radius']
    line_dir = np.cross(d1, d2)
    A = np.array([d1, d2, line_dir])
    b = np.array([np.dot(d1, TP1), np.dot(d2, TP2), 0])
    x0 = solve(A, b)

    # Find t4 and t5
    a_t4 = np.dot(line_dir, line_dir)
    b_t4 = 2 * np.dot(line_dir, x0 - TP1)
    c_t4 = np.dot(x0 - TP1, x0 - TP1) - r1 ** 2

    a_t5 = np.dot(line_dir, line_dir)
    b_t5 = 2 * np.dot(line_dir, x0 - TP2)
    c_t5 = np.dot(x0 - TP2, x0 - TP2) - r2 ** 2
    if check_overlap([a_t4, b_t4, c_t4], [a_t5, b_t5, c_t5]):
        return True
    else:
        return False

def solve_quadratic(a, b, c):
    """Solve quadratic equation and return real solutions."""
    discriminant = b**2 - 4*a*c
    if discriminant < 0:
        return []  # No real solutions
    elif discriminant == 0:
        return [-b / (2*a)]  # One real solution
    else:
        sqrt_discriminant = math.sqrt(discriminant)
        return [(-b + sqrt_discriminant) / (2*a), (-b - sqrt_discriminant) / (2*a)]  # Two real solutions

def check_overlap(equation1, equation2):
    """Check if there is an overlap between the real solutions of two quadratic equations."""
    solutions1 = solve_quadratic(*equation1)
    solutions2 = solve_quadratic(*equation2)

    if not solutions1 or not solutions2:
        return False  # No overlap if either has no real solutions

    # Check for overlap
    return max(min(solutions1), min(solutions2)) <= min(max(solutions1), max(solutions2))

def check_collision(cylinder1, cylinder2):
    """Checks if two cylinders are colliding.

    Args:
        cylinder1 (Cylinder Direct): The first cylinder.
        cylinder2 (Cylinder Direct): The second cylinder.

    Returns:
        bool: True if the cylinders are colliding, False otherwise.
    """

    info = {}

    # Extracting the information from the cylinders
    d1 = cylinder1['direct'] / np.linalg.norm(cylinder1['direct'])
    d2 = cylinder2['direct'] / np.linalg.norm(cylinder2['direct'])
    c1 = cylinder1['center']
    c2 = cylinder2['center']
    h1 = cylinder1['height']
    h2 = cylinder2['height']
    r1 = cylinder1['radius']
    r2 = cylinder2['radius']

    # Pre-check the status of the cylinders
    info['parallel'] = np.isclose(np.dot(d1, d2), 1)
    info['coplane'] = np.isclose(np.dot(np.cross(d1, d2), c2 - c1), 0)

    # Infinite cylinder collision check
    # No Collision Possible If the distance between two infinite cylinders is larger than the sum of their radius
    distance = distance_between_point_and_line(c1, c2, c2 + d2) 
    if distance > r1 + r2:
        return False, info
    
    # Finite cylinder collision check
    if info['parallel']:
        if abs(np.dot(c2 - c1, d1)) >= (h1 + h2) / 2:
            return True, info 
        else:
            return False, info
    
    elif info['coplane']:
        #TODO: check the coplane case
        # Find the four corners of the rectangle
        return False, info
    else:
        # Find the common normal vector
        A = np.array([
            [np.dot(d1, d2), -np.dot(d2, d2)],
            [np.dot(d1, d1), -np.dot(d1, d2)]
        ])  
        b = np.array([
            np.dot(d2, c2 - c1),
            np.dot(d1, c2 - c1)
        ])
        t = np.linalg.solve(A, b)
        n = c1 + t[0] * d1 - c2 - t[1] * d2
        if abs(t[0]) <= h1 / 2 and abs(t[1]) <= h2 / 2:
            # Both points are inside the cylinder
            return True, info
        elif abs(t[0]) <= h1 / 2:
            TP2 = c2 + h2 / 2 * d2 if t[1] > 0 else c2 - h2 / 2 * d2
            status, info_on = on_test(TP2, cylinder1, cylinder2, n)
            if status == 1:
                return True, info
            elif status == 0:
                return False, info
            else:
                # End test
                return end_test(info_on['TP1'], TP2, cylinder1, cylinder2), info

        elif abs(t[1]) <= h2 / 2:
            TP1 = c1 + h1 / 2 * d1 if t[0] > 0 else c1 - h1 / 2 * d1
            status, info_on = on_test(TP1, cylinder2, cylinder1, n)
            if status == 1:
                return True, info
            elif status == 0:
                return False, info
            else:
                # End test
                return end_test(TP1, info_on['TP1'], cylinder1, cylinder2), info
        else:
            # Off test
            off_test1 = False
            off_test2 = False

            TP2 = c2 + h2 / 2 * d2
            status, info_on = on_test(TP2, cylinder1, cylinder2, n)
            if status == 1:
                off_test1 = True
            elif status == 0:
                off_test1 = False
            else:
                off_test1 = end_test(info_on['TP1'], TP2, cylinder1, cylinder2)
            
            TP2 = c2 - h2 / 2 * d2
            status, info_on = on_test(TP2, cylinder1, cylinder2, n)
            if status == 1:
                off_test2 = True
            elif status == 0:
                off_test2 = False
            else:
                off_test2 = end_test(info_on['TP1'], TP2, cylinder1, cylinder2)
            
            return off_test1 or off_test2, info
        
cylinder1 = {
    "direct": np.array([1, 1, 1]),
    "center": np.array([1, 0, 0]),
    "radius": 1,
    "height": 1
}
cylinder2 = {
    "direct": np.array([0, 0, 1]),
    "center": np.array([-0.6, -0.6, 0]),
    "radius": 1,
    "height": 1
}
print(check_collision(cylinder1, cylinder2))