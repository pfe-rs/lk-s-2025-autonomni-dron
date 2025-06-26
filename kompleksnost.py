import numpy as np
GRID_SIZE=100
grid = np.zeros((GRID_SIZE, GRID_SIZE, GRID_SIZE), dtype=int)
max_x,max_y,max_z=grid.shape

# prepreke = [
#     {"x": 40, "y": 40, "r": 5, "h": 40},
#     {"x": 60, "y": 50, "r": 5, "h": 20},
#     {"x": 50, "y": 70, "r": 5, "h": 10},
#     {"x": 10, "y": 70, "r": 5, "h": 50},
#     {"x": 20, "y": 60, "r": 5, "h": 40},
#     {"x": 90, "y": 30, "r": 5, "h": 20},
#     {"x": 85, "y": 20, "r": 5, "h": 30},
#     {"x": 10, "y": 10, "r": 5, "h": 30},
#     {"x": 90, "y": 90, "r": 5, "h": 40},
#     {"x": 20, "y": 90, "r": 5, "h": 40},
# ]

prepreke = [
    {"x": 40, "y": 40, "r": 5, "h": 40},
    {"x": 40, "y": 10, "r": 5, "h": 40},
    {"x": 40, "y": 20, "r": 5, "h": 40},
    {"x": 40, "y": 50, "r": 5, "h": 40},
    {"x": 40, "y": 60, "r": 5, "h": 40},
    {"x": 40, "y": 70, "r": 5, "h": 40},
    {"x": 40, "y": 80, "r": 5, "h": 40},
    
    # {"x": 40, "y": 90, "r": 5, "h": 40},
    # {"x": 10, "y": 40, "r": 5, "h": 40},
    # {"x": 10, "y": 10, "r": 5, "h": 40},
    # {"x": 10, "y": 20, "r": 5, "h": 40},
    # {"x": 10, "y": 50, "r": 5, "h": 40},
    # {"x": 10, "y": 60, "r": 5, "h": 40},
    # {"x": 10, "y": 70, "r": 5, "h": 40},
    # {"x": 10, "y": 80, "r": 5, "h": 40},
    # {"x": 10, "y": 90, "r": 5, "h": 40},
    # {"x": 70, "y": 40, "r": 5, "h": 40},
    # {"x": 70, "y": 10, "r": 5, "h": 40},
    # {"x": 70, "y": 20, "r": 5, "h": 40},
    # {"x": 70, "y": 50, "r": 5, "h": 40},
    # {"x": 70, "y": 60, "r": 5, "h": 40},
    # {"x": 70, "y": 70, "r": 5, "h": 40},
    # {"x": 70, "y": 80, "r": 5, "h": 40},
    # {"x": 70, "y": 90, "r": 5, "h": 40}
]

def oceni_teren(prepreke, grid_shape):
    broj_prepreka = len(prepreke)
    if broj_prepreka == 0:
        return 1

    centri = np.array([[p["x"], p["y"], p["h"]/2] for p in prepreke])

    dists = []
    for i in range(len(centri)):
        for j in range(i+1, len(centri)):
            dists.append(np.linalg.norm(centri[i] - centri[j]))

    if not dists:
        prosek_dist = 0
    else:
        prosek_dist = np.mean(dists)

    max_prepreka = 50 
    max_dist = np.linalg.norm(np.array(grid_shape)) 

    gustina = broj_prepreka / max_prepreka
    razudjenost = 1.0 - min(prosek_dist / max_dist, 1.0)

    score = 1 * gustina + 0.5 * razudjenost
    ocena = int(np.ceil(score * 5))
    return max(1, min(5, ocena))


ocena = oceni_teren(prepreke, grid.shape)
print(f"Ocena kompleksnosti terena: {ocena}/5")