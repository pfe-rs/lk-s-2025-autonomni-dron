import matplotlib.pyplot as plt
import numpy as np
import heapq
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math

from matplotlib.animation import FuncAnimation

from kalman_filter import KalmanFilter
from astar import astar
from crtanje_terena import *
from vidno_polje import *
from numpy import ndarray

import vidno_polje
# from crtanje import update


# Parametri
GRID_SIZE = 100
STEPS = 100
DRONE_SPEED = 1.0
OBJECT_SPEED = 2
FOV_DEGREES = 90
FOV_RADIUS = 30
MIN_DISTANCE = 4.0

# Grid i prepreke
grid = np.zeros((GRID_SIZE, GRID_SIZE, GRID_SIZE), dtype=int)
max_x,max_y,max_z=grid.shape


prepreke = [
    {"x": 50, "y": 50, "r": 5, "h": 100},
    # {"x": 40, "y": 10, "r": 5, "h": 40},
    # {"x": 40, "y": 20, "r": 5, "h": 40},
    # {"x": 40, "y": 50, "r": 5, "h": 40},
    # {"x": 40, "y": 60, "r": 5, "h": 40},
    # {"x": 40, "y": 70, "r": 5, "h": 40},
    # {"x": 40, "y": 80, "r": 5, "h": 40}

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

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(0, GRID_SIZE)
ax.set_ylim(0, GRID_SIZE)
ax.set_zlim(0, GRID_SIZE)

ax.set_title("Simulacija: Dron prati objekat")



dron_dot, = ax.plot([], [], [], 'go', label='Dron')
obj_dot, = ax.plot([], [], [], 'ro', label='Objekat')
linija, = ax.plot([], [], [], 'b-')

kalman_dot, = ax.plot([], [], [], 'x', color='black', label='Kalman predikcija')
# Prepreke
for p in prepreke:
    draw_cylinder(ax, p["x"], p["y"], p["r"], p["h"], grid)

# Pozicije

dron_pos = np.array([5.0, 50.0, 20.0])
objekat_pos = np.array([10.0, 50.0, 10.0])


objekat_directions = [
    np.array([1, 1, 0]), np.array([-1, 1, 0]), np.array([1, -1, 0]), np.array([-1, -1, 0]),
    np.array([1, 1, 1]), np.array([-1, 1, 1]), np.array([1, -1, 1]), np.array([-1, -1, 1]),
    np.array([1, 1, -1]), np.array([-1, 1, -1]), np.array([1, -1, -1]), np.array([-1, -1, -1])
]

putanja = [
    np.array([10.0, 50.0, 10.0]), 

    np.array([25.0, 50.0, 10.0]),
    np.array([30.0, 50.0, 10.0]),

    np.array([60.0, 60.0, 10.0]),

    np.array([60.0, 60.0, 10.0]),
    np.array([60.0, 70.0, 10.0]),
    np.array([70.0, 70.0, 10.0]),
    np.array([90.0, 60.0, 10.0]),
]



putanja_ind=0
direction_index = 0
objekat_vel = objekat_directions[0] / np.linalg.norm(objekat_directions[0]) * OBJECT_SPEED

kf=KalmanFilter(objekat_pos,objekat_vel)

# Pokretanje simulacije
dron_path, objekat_path, colors, fovs = [], [], [], []
dron_dir = np.arctan2(1, 1)


# objekat_visible=[]
# vidi_time=[]

#procenat kad vidi objekat
br=0
br_ukupno=0

angle_log=[]
kalman_predict=[]
# br_line=0
#for step in range(STEPS):
#     if step % 23 == 0 and step > 0:
#         direction_index = (direction_index + 1) % len(objekat_directions)
#         objekat_vel = objekat_directions[direction_index] / np.linalg.norm(objekat_directions[direction_index]) * OBJECT_SPEED

#     next_obj = objekat_pos + objekat_vel
#     gx, gy, gz = int(round(next_obj[0])), int(round(next_obj[1])), int(round(next_obj[2]))
#     if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE and 0 <= gz < GRID_SIZE and grid[gz, gy, gx] == 0:
#         objekat_pos = next_obj
#     
kalman_preds = []
while putanja_ind < len(putanja):
    sledeca_tacka = putanja[putanja_ind]
    smer = sledeca_tacka - objekat_pos
    udaljenost = np.linalg.norm(smer)

    if udaljenost < OBJECT_SPEED:
        objekat_pos = sledeca_tacka
        putanja_ind += 1
        if putanja_ind >= len(putanja):
            objekat_vel = np.zeros(3)
        else:
            sledeca_tacka = putanja[putanja_ind]
            smer = sledeca_tacka - objekat_pos
            udaljenost = np.linalg.norm(smer)
            objekat_vel = smer / udaljenost * OBJECT_SPEED
    else:
        objekat_vel = smer / udaljenost * OBJECT_SPEED
        next_obj = objekat_pos + objekat_vel
        gx, gy, gz = int(round(next_obj[0])), int(round(next_obj[1])), int(round(next_obj[2]))
        if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE and 0 <= gz < GRID_SIZE and grid[gz, gy, gx] == 0:
            objekat_pos = next_obj
        # else:
        #     objekat_vel = np.zeros(3)  # staje kad udari u prepreku

    objekat_path.append(objekat_pos.copy())  #beleži pozicije za animaciju


    vidi_objekat = is_in_fov(dron_pos, objekat_pos, grid)
    predicted = kf.predict()
    kalman_preds.append(predicted.copy())



    # VIDI OBJEKAT
    if vidi_objekat:
        print(f"[STEP {putanja_ind}] UPDATE: vidi objekat.")
        kf.update(objekat_pos)
        target = objekat_pos
        # objekat_visible[step]=1
        # vidi_time[step]+=1
        
        br+=1



    br_ukupno+=1

    vec = target - dron_pos
    dron_dir = np.arctan2(vec[1], vec[0])
    dist = np.linalg.norm(vec)

    if dist > MIN_DISTANCE:
        desired_pos = target - (vec / dist * MIN_DISTANCE)  # pozicija koja drži distancu
        vec_to_desired = desired_pos - dron_pos
        dist_to_desired = np.linalg.norm(vec_to_desired)

    if dist_to_desired > 0:
        step = min(DRONE_SPEED, dist_to_desired)
        move = vec_to_desired / dist_to_desired * step
        dron_pos += move

        vector = objekat_pos - dron_pos
        angle_rad = np.arctan2(vector[1], vector[0])
        angle_deg = np.degrees(angle_rad)
        angle_log.append(angle_deg)

        dron_path.append(dron_pos.copy())
        colors.append("green" if vidi_objekat else "blue")
        fovs.append(dron_dir)
    else:
        print(f"[STEP {putanja_ind}] PREDICT: objekat van FOV.")
        target = predicted


print("Broj puta kada je putanja prekinuta ", vidno_polje.br_line)
# total_visible=sum(objekat_visible)
# procenat_vidi=sum(vidi_time)-STEPS
# br_gubljenja=br

# Vizualizacija

procenat=br/br_ukupno*100
print(procenat)

def update(frame):
    d = dron_path[frame]
    o = objekat_path[frame]
    k = kalman_preds[frame]

    dron_dot.set_data([d[0]], [d[1]])
    dron_dot.set_3d_properties([d[2]])
    obj_dot.set_data([o[0]], [o[1]])
    obj_dot.set_3d_properties([o[2]])

    kalman_dot.set_data([k[0]], [k[1]])
    kalman_dot.set_3d_properties([k[2]])
    linija.set_data([d[0], o[0]], [d[1], o[1]])
    linija.set_3d_properties([d[2], o[2]])
    linija.set_color(colors[frame])
    return dron_dot, obj_dot, linija

ani = FuncAnimation(fig, update, frames=len(objekat_path), interval=100, blit=False)
plt.legend()
plt.show()


###########################################################################################################################################################
vidljivost_signal = [1 if c == "green" else 0 for c in colors]
udaljenost_signal = [np.linalg.norm(np.array(d) - np.array(o)) for d, o in zip(dron_path, objekat_path)]


kalman_greska_signal = []
kf2 = KalmanFilter(objekat_path[0], objekat_vel)
for i, pos in enumerate(objekat_path):
    pred = kf2.predict()
    if colors[i] == "green":
        kf2.update(pos)
    greska = np.linalg.norm(pos - pred)
    kalman_greska_signal.append(greska)

# vector_to_object=objekat_pos-dron_pos
# angle_rad = np.arctan2(vector_to_object[1], vector_to_object[0])
# angle_deg = np.degrees(angle_rad)




#Vizualizacija signala
fig2, axs = plt.subplots(4, 1, figsize=(10, 6), sharex=True)
fig2.suptitle("Analiza: Vidljivost, Udaljenost i Kalman Greška", fontsize=14)

axs[0].plot(vidljivost_signal, label='Vidljivost (1=vidi)', color='green')
axs[0].set_ylabel("Vidljivost")
axs[0].legend(loc="upper right")

axs[1].plot(udaljenost_signal, label='Udaljenost od objekta', color='blue')
axs[1].set_ylabel("Udaljenost")
axs[1].legend(loc="upper right")

axs[2].plot(kalman_greska_signal, label='Greška Kalman predikcije', color='red')
axs[2].set_ylabel("Greška")
axs[2].set_xlabel("Korak")
axs[2].legend(loc="upper right")

axs[3].plot(angle_log, label='ugao do objekta', color='blue')
axs[3].set_ylabel("Ugao")
axs[3].legend(loc="upper right")

plt.tight_layout()
plt.show()

