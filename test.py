import numpy as np

def intersects_segment_box(P0, P1, box_min, box_max):
    """
    Provera da li segment [P0, P1] seče AABB definisanu sa box_min i box_max.
    Koristi slab metod za efikasnost.
    """
    P0 = np.asarray(P0, dtype=float)
    P1 = np.asarray(P1, dtype=float)
    box_min = np.asarray(box_min, dtype=float)
    box_max = np.asarray(box_max, dtype=float)

    d = P1 - P0
    tmin, tmax = 0.0, 1.0

    for i in range(3):
        if d[i] == 0.0:
            # Paralelno je sa osi i ako tačka nije unutar opsega nema preseka
            if P0[i] < box_min[i] or P0[i] > box_max[i]:
                # print(f"Axis {i}: parallel and outside slab, no intersection")
                return False
        else:
            t1 = (box_min[i] - P0[i]) / d[i]
            t2 = (box_max[i] - P0[i]) / d[i]
            t_near = min(t1, t2)
            t_far = max(t1, t2)

            tmin = max(tmin, t_near)
            tmax = min(tmax, t_far)

            if tmin > tmax:
                # print(f"Axis {i}: no intersection, tmin={tmin}, tmax={tmax}")
                return False

    # print(f"Intersection found between t={tmin} and t={tmax}")
    return True

print(intersects_segment_box([0,0,4], [-2,-2,4], [-1,5,-1,5,0], [-0.5,-0.5,0]))