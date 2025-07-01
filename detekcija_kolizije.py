import numpy as np
GRID_SIZE=100

def detekcija_kolizije(dron_pos, move_vec, grid):
    test_pos = dron_pos + move_vec
    x, y, z = map(int, np.round(test_pos))

    if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE and 0 <= z < GRID_SIZE:
        if grid[z, y, x] == 0:
            return move_vec  # idi napred

    # Ako je blokirano, probaj da ideš blago levo, desno, gore, dole
    fallback_directions = [
        np.array([0, 0, 1]),   # gore
        np.array([0, 0, -1]),  # dole
        np.array([1, 0, 0]),   # desno
        np.array([-1, 0, 0]),  # levo
        np.array([0, 1, 0]),   # napred
        np.array([0, -1, 0])   # nazad
    ]

    for dir_offset in fallback_directions:
        alt_vec = move_vec + dir_offset * 0.5
        alt_pos = dron_pos + alt_vec
        ax, ay, az = map(int, np.round(alt_pos))
        if 0 <= ax < GRID_SIZE and 0 <= ay < GRID_SIZE and 0 <= az < GRID_SIZE:
            if grid[az, ay, ax] == 0:
                return alt_vec  # nađeno alternativno kretanje

    return np.zeros_like(move_vec)  # stani ako ne možeš nigde
