from pymavlink import mavutil
import time
import math
import random
import numpy as np


def blizu_prepreke(x, y, prepreke, prag=4.0):
    for px, py, pz in prepreke:
        udaljenost = math.sqrt((x - px)**2 + (y - py)**2)
        if udaljenost < prag:
            return True
    return False


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


def has_line_of_sight(dron1, dron2, prepreke_centri, size_x=2.0, size_y=2.0, size_z=10.0):
    """
    Proverava da li postoji slobodan pogled između dron1 i dron2.
    Svaka prepreka je axis-aligned bounding box definisana centrom i dimenzijama.
    """
    print(f"Provera linije vida između {dron1} i {dron2}")
    for cx, cy, cz in prepreke_centri:
        box_min = np.array([cx - size_x / 2, cy - size_y / 2, 0.0])  # donja visina = tlo
        box_max = np.array([cx + size_x / 2, cy + size_y / 2, cz])   # gornja visina = vrh prepreke

        intersects = intersects_segment_box(dron1, dron2, box_min, box_max)
        print(f"Prepreka centar ({cx}, {cy}, {cz}), opseg: {box_min} - {box_max}, preseče: {intersects}")

        if intersects:
            print("Pogled blokiran preprekom!")
            return False

    print("Pogled nije blokiran nijednom preprekoh.")
    return True


# --- MAVLink konekcije i priprema dronova ---

connection_string = "udp:127.0.0.1:14550"
print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

connection_string1 = "udp:127.0.0.1:14560"
print(f"Connecting to {connection_string1}...")
master1 = mavutil.mavlink_connection(connection_string1)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

print("Waiting for heartbeat...")
master1.wait_heartbeat()
print(f"Heartbeat from system (system {master1.target_system} component {master1.target_component})")

print("Arming vehicle 1...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

print("Arming vehicle 2...")
master1.mav.command_long_send(
    master1.target_system,
    master1.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    print(f"Vehicle 1 Armed: {armed}")
    if armed:
        break
    time.sleep(1)

while True:
    msg = master1.recv_match(type='HEARTBEAT', blocking=True)
    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    print(f"Vehicle 2 Armed: {armed}")
    if armed:
        break
    time.sleep(1)

mode_id = master.mode_mapping()['GUIDED']
print("Setting vehicle 1 mode to GUIDED...")
master.set_mode(mode_id)

mode_id = master1.mode_mapping()['GUIDED']
print("Setting vehicle 2 mode to GUIDED...")
master1.set_mode(mode_id)

time.sleep(1)

altitude = 4
print(f"Vehicle 1 takeoff to {altitude} meters...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    altitude)

print(f"Vehicle 2 takeoff to {altitude} meters...")
master1.mav.command_long_send(
    master1.target_system,
    master1.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    altitude)

print("Waiting for vehicle 1 to reach altitude...")
start = time.time()
while time.time() - start < 30:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"Vehicle 1 Altitude: {alt:.2f} m")
        if alt >= altitude * 0.95:
            print("Vehicle 1 reached target altitude")
            break

print("Waiting for vehicle 2 to reach altitude...")
start = time.time()
while time.time() - start < 30:
    msg = master1.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"Vehicle 2 Altitude: {alt:.2f} m")
        if alt >= altitude * 0.95:
            print("Vehicle 2 reached target altitude")
            break

prepreke_centri = [
    (5, 2, 10.0),
    (-3, -3, 10.0),
    (1, -2, 10.0),
    (-1, 2, 10.0),
    (-5, 5, 10.0),
    (8, -4, 10.0),
    (-1, -1, 10.0)
]

prepreke = [(5, 2, 0.5), (-3, -3, 0.5), (-1, 2, 0.5), (1, -2, 0.5), (-1, -1, 10)]

korak = 0.5
brojac = 0

vx, vy = random.choice([
    (1, 0), (0, 1), (-1, 0), (0, -1),
    (1, 1), (-1, 1), (1, -1), (-1, -1)
])
vz = 0

while brojac < 300:
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    msg2 = master1.recv_match(type='LOCAL_POSITION_NED', blocking=True)

    if not msg or not msg2:
        time.sleep(0.1)
        continue

    # Pretpostavka: z invertovan jer NED, gore pozitivno je -z
    p1 = (msg.x, msg.y, msg.z)
    p2 = (msg2.x, msg2.y, msg2.z)

    print(f"[{brojac}] Dron1 pozicija: {p1}, Dron2 pozicija: {p2}")
    print(f"[{brojac}] Visine dronova: {p1[2]:.2f} m, {p2[2]:.2f} m")

    if has_line_of_sight(p1, p2, prepreke_centri):
        print(f"[{brojac}] ✅ Dron 1 vidi drona 2")
    else:
        print(f"[{brojac}] ❌ Pogled je blokiran")

    x, y, z = msg.x, msg.y, msg.z

    if brojac % 5 == 0:
        vx, vy = random.choice([
            (1, 0), (0, 1), (-1, 0), (0, -1),
            (1, 1), (-1, 1), (1, -1), (-1, -1)
        ])
        print(f"[{brojac}] Menjam pravac nasumično: vx={vx}, vy={vy}")

    next_x = x + vx
    next_y = y + vy
    if blizu_prepreke(next_x, next_y, prepreke, prag=2.0):
        print(f"[{brojac}] Prepreka ispred! Tražim novi bezbedan pravac...")
        for _ in range(10):
            novi_vx, novi_vy = random.choice([
                (1, 0), (0, 1), (-1, 0), (0, -1),
                (1, 1), (-1, 1), (1, -1), (-1, -1)
            ])
            test_x = x + novi_vx
            test_y = y + novi_vy
            if not blizu_prepreke(test_x, test_y, prepreke, prag=2.0):
                vx, vy = novi_vx, novi_vy
                print(f"[{brojac}] Novi pravac: vx={vx}, vy={vy}")
                break
        else:
            print(f"[{brojac}] Okružen preprekama! Preskačem korak.")
            time.sleep(0.5)
            brojac += 1
            continue

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000110,
        0, 0, 0,
        vx * korak, vy * korak, vz,
        0, 0, 0,
        0, 0
    )

    print(f"[{brojac}] Pozicija: X={x:.2f}, Y={y:.2f}, Smer: vx={vx}, vy={vy}")
    brojac += 1
    time.sleep(0.2)

print("Landing vehicles...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)

master1.mav.command_long_send(
    master1.target_system,
    master1.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)

print("Done.")
