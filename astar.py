import numpy as np
import heapq
GRID_SIZE=100
grid = np.zeros((GRID_SIZE, GRID_SIZE, GRID_SIZE), dtype=int)
max_x,max_y,max_z=grid.shape

def astar(start, goal, grid):
    neighbors = [
    ( 1,  0,  0), ( 1,  0,  1), ( 1,  0, -1),
    ( 1,  1,  0), ( 1,  1,  1), ( 1,  1, -1),
    ( 1, -1,  0), ( 1, -1,  1), ( 1, -1, -1),

    ( 0,  1,  0), ( 0,  1,  1), ( 0,  1, -1),
    ( 0, -1,  0), ( 0, -1,  1), ( 0, -1, -1),
    ( 0,  0,  1), ( 0,  0, -1),

    (-1,  0,  0), (-1,  0,  1), (-1,  0, -1),
    (-1,  1,  0), (-1,  1,  1), (-1,  1, -1),
    (-1, -1,  0), (-1, -1,  1), (-1, -1, -1)
]

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: np.linalg.norm(np.array(start) - np.array(goal))}
    oheap = [(fscore[start], start)]

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        close_set.add(current)
        for dx, dy, dz in neighbors:
            neighbor = (current[0]+dx, current[1]+dy, current[2]+dz)
            if 0 <= neighbor[0] < max_x and 0 <= neighbor[1] < max_y and 0<=neighbor[2]< max_z:
                if grid[neighbor[2], neighbor[1], neighbor[0]] == 1:
                    continue
                tentative = gscore[current] + np.linalg.norm(np.array(current) - np.array(neighbor))
                if neighbor in close_set and tentative >= gscore.get(neighbor, 0):
                    continue
                if tentative < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative
                    fscore[neighbor] = tentative + np.linalg.norm(np.array(neighbor) - np.array(goal))
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return []