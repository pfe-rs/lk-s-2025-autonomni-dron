
import numpy as np
GRID_SIZE = 100
FOV_RADIUS = 30
FOV_DEGREES = 90


br_line = 0

def has_line_of_sight(p1, p2, grid):
    global br_line
    dist = np.linalg.norm(p2 - p1)
    steps = max(int(dist * 2), 1)

    for i in range(steps + 1):
        point = p1 + (p2 - p1) * i / steps
        z, y, x = map(int, np.floor(point))
        if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE and 0 <= z < GRID_SIZE:
            if grid[z, y, x] == 1:
                br_line += 1
                return False
    return True

def is_in_fov(drone_pos, obj_pos, grid):
    vec = obj_pos - drone_pos
    dist = np.linalg.norm(vec)
    if dist > FOV_RADIUS:
        return False

    
    look_dir = vec / dist  # jedinicni vektor pogleda
    vec_norm = vec / dist  # takođe jedinicni vektor do objekta

    # Ugao između pravca gledanja i pravca ka objektu
    angle_cos = np.dot(look_dir, vec_norm)
    angle_deg = np.degrees(np.arccos(np.clip(angle_cos, -1.0, 1.0)))

    return angle_deg <= FOV_DEGREES / 2 and has_line_of_sight(drone_pos, obj_pos, grid)
