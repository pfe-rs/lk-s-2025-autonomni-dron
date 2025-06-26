import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
GRID_SIZE=100
def draw_cylinder(ax, x_center, y_center, radius, height, grid ,resolution=20):
    theta = np.linspace(0, 2 * np.pi, resolution)
    r1 = np.linspace(-2, 0, 100)
    t1, R1 = np.meshgrid(theta, r1)

    x = x_center + radius * np.cos(theta)
    y = y_center + radius * np.sin(theta)
    z_bottom = np.zeros_like(x)
    z_top = np.ones_like(x) * height

    verts = []
    verts.append(list(zip(x, y, z_bottom)))
    verts.append(list(zip(x, y, z_top)))
    for i in range(len(x)-1):
        verts.append([
            (x[i], y[i], z_bottom[i]),
            (x[i+1], y[i+1], z_bottom[i+1]),
            (x[i+1], y[i+1], z_top[i+1]),
            (x[i], y[i], z_top[i])
        ])
    ax.add_collection3d(Poly3DCollection(verts, color='gray', alpha=0.5))

    # # Popuni grid
    # for zi in range(height):
    #     for xi in range(x_center - radius, x_center + radius):
    #         for yi in range(y_center - radius, y_center + radius):
    #             if 0 <= xi < GRID_SIZE and 0 <= yi < GRID_SIZE and (xi - x_center)**2 + (yi - y_center)**2 <= radius**2:
    #                 grid[zi, yi, xi] = 1
    # Popuni grid
    x_center = int(round(x_center))
    y_center = int(round(y_center))
    radius = int(np.ceil(radius))
    height = int(np.ceil(height))

    for zi in range(height):
        for xi in range(x_center - radius, x_center + radius + 1):
            for yi in range(y_center - radius, y_center + radius + 1):
                if 0 <= xi < GRID_SIZE and 0 <= yi < GRID_SIZE:
                    if (xi - x_center)**2 + (yi - y_center)**2 <= radius**2:
                        grid[zi, yi, xi] = 1
