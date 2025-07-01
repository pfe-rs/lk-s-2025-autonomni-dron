from pymavlink import mavutil
import time
from copy import deepcopy
import math
import random



def frange(start, end, step):
    while start <= end:
        yield start
        start += step
def generisi_prepreku_tacke(cx, cy, cz, size_x=1.0, size_y=1.0, size_z=10.0, rezolucija=0.5):
    prepreka_tacke = set()
    half_x = size_x / 2
    half_y = size_y / 2
    min_z = cz - size_z

    x_range = frange(cx - half_x, cx + half_x, rezolucija)
    y_range = frange(cy - half_y, cy + half_y, rezolucija)
    z_range = frange(min_z, cz, rezolucija)

    for x in x_range:
        for y in y_range:
            for z in z_range:
                prepreka_tacke.add((x, y, z))
    return prepreka_tacke


def generisi_liniju_tacke(p1, p2, korak=0.2):
    dist = math.dist(p1, p2)
    steps = max(int(dist / korak), 1)
    tacke = []

    for i in range(steps + 1):
        t = i / steps
        x = p1[0] + (p2[0] - p1[0]) * t
        y = p1[1] + (p2[1] - p1[1]) * t
        z = p1[2] + (p2[2] - p1[2]) * t
        tacke.append((x, y, z))
    return tacke

def has_line_of_sight(dron1, dron2, prepreke_centri, tolerancija=0.3):
    sve_prepreke_tacke = set()

    for cx, cy, cz in prepreke_centri:
        sve_prepreke_tacke |= generisi_prepreku_tacke(cx, cy, cz)

    linija = generisi_liniju_tacke(dron1, dron2)

    for t in linija:
        for pt in sve_prepreke_tacke:
            # proveravamo da li je tacka na liniji blizu bilo koje tacke prepreke
            dist = math.sqrt((t[0] - pt[0])**2 + (t[1] - pt[1])**2 + (t[2] - pt[2])**2)
            if dist < tolerancija:
                return False  # linija udara u prepreku
    return True


def blizu_prepreke(x, y, prepreke, prag=4.0):
    for px, py, pz in prepreke:
        udaljenost = math.sqrt((x - px)**2 + (y - py)**2)
        if udaljenost < prag:
            return True
    return False

# Connect to MAVProxy's UDP output
connection_string = "udp:127.0.0.1:14550"
print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string)


#drugi dron 
connection_string1 = "udp:127.0.0.1:14560"
print(f"Connecting to {connection_string1}...")
master1 = mavutil.mavlink_connection(connection_string1)

# Wait for the heartbeat message to find the system ID
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")


print("Waiting for heartbeat...")
master1.wait_heartbeat()
print(f"Heartbeat from system (system {master1.target_system} component {master1.target_component})")




#Arm the vehicle
print("Arming vehicle...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0) 
#ovde 1 da kaze da je armed


#Arm the vehicle
print("Arming vehicle...")
master1.mav.command_long_send(
    master1.target_system,
    master1.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0) 
# ovde 1 da kaze da je armed





# Wait until armed
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    print(f"Armed: {armed}")
    if armed:
        break
    time.sleep(1)


# Wait until armed
while True:
    msg = master1.recv_match(type='HEARTBEAT', blocking=True)
    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    print(f"Armed: {armed}")
    if armed:
        break
    time.sleep(1)


mode_id = master.mode_mapping()['GUIDED']
print("Setting mode to GUIDED...")
master.set_mode(mode_id)


# Change mode to GUIDED (4)
mode_id = master1.mode_mapping()['GUIDED']
print("Setting mode to GUIDED...")
master1.set_mode(mode_id)

time.sleep(1)


#poletanje



# Takeoff command (change altitude as needed)

altitude = 4 # meters
print(f"Sending takeoff command to {altitude} meters...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    altitude)


altitude = 4 # meters
print(f"Sending takeoff command to {altitude} meters...")
master1.mav.command_long_send(
    master1.target_system,
    master1.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    altitude)
# stavi ga na tu visinu 

print("Waiting to reach altitude...")
# da ga stavi na visinu 
# Simple wait for altitude message or timeout
start = time.time()
while time.time() - start < 30:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        alt = msg.relative_alt / 1000.0  # in meters
        print(f"Altitude: {alt:.2f} m")
        if alt >= altitude * 0.95:
            print("Target altitude reached")
            break


print("Waiting to reach altitude...")
# da ga stavi na visinu 
# Simple wait for altitude message or timeout
start = time.time()
while time.time() - start < 30:
    msg = master1.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        alt = msg.relative_alt / 1000.0  # in meters
        print(f"Altitude: {alt:.2f} m")
        if alt >= altitude * 0.95:
            print("Target altitude reached")
            break



# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
#     0,
#     0, 0, 0, 0,
#     target_latitude,   
#     target_longitude,
#     target_altitude  
# )

# master.mav.set_position_target_local_ned_send(
#     0,  # time_boot_ms (0 = ignore)
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # lokalni koordinatni sistem
#     0b0000111111000111,  # type_mask (ignoriši brzinu, akceleraciju, yaw)
#     -10,    
#     10,    
#     0,    
#     0, 0, 0,  
#     0, 0, 0,  
#     0, 0      
#     )


prepreke = [(5,2,0.5), (-3, -3, 0.5), (-1, 2, 0.5),(1,-2,0.5)]  # pomeri 10m napred, 10m desno, nazad


print("Letenje napred i desno brzinom 1m/s na visini 10m...")

preskoci = False  # Flag da prekinemo trenutnu brzinu ako je prepreka blizu

# Centri vrhova prepreka
prepreke_centri = [
    (5, 2, 10.0),
    (-3, -3, 10.0),
    (1, -2, 10.0),
    (-1, 2, 10.0),
    (-5, 5, 10.0),
    (8, -4, 10.0),
    (-1, -1, 10.0)
]




current_direction = (1.0, 0.0, 0.0)  

korak = 0.5  # brzina (m/s)
poruke = []
trajanje = 300  # koliko ukupno koraka da ide
brojac = 0

# Pocetni pravac
vx, vy = random.choice([
    (1, 0), (0, 1), (-1, 0), (0, -1),
    (1, 1), (-1, 1), (1, -1), (-1, -1)
])
vz = 0  # nema promene visine

for i in range(trajanje):
# Pozicije dronova
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    msg2 = master1.recv_match(type='LOCAL_POSITION_NED', blocking=True)

    p1 = (msg.x, msg.y, -msg.z)
    p2 = (msg2.x, msg2.y, -msg2.z)
    if has_line_of_sight(p1, p2, prepreke_centri):
        print("✅ Dron 1 vidi drona 2")
    else:
        print("❌ Pogled je blokiran")


    #msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if not msg:
        continue

    x, y, z = msg.x, msg.y, msg.z
    poruke.append(msg)

    # Promeni pravac svakih 18 koraka
    if brojac % 5 == 0:
        vx, vy = random.choice([
            (1, 0), (0, 1), (-1, 0), (0, -1),
            (1, 1), (-1, 1), (1, -1), (-1, -1)
        ])
        print(f"[{i}] Menjam pravac nasumično: vx={vx}, vy={vy}")

    # Provera prepreke u sledećem koraku
    next_x = x + vx
    next_y = y + vy
    if blizu_prepreke(next_x, next_y, prepreke, prag=2.0):
        print(f"[{i}] Prepreka ispred! Tražim novi bezbedan pravac...")
        for _ in range(10):
            novi_vx, novi_vy = random.choice([
                (1, 0), (0, 1), (-1, 0), (0, -1),
                (1, 1), (-1, 1), (1, -1), (-1, -1)
            ])
            test_x = x + novi_vx
            test_y = y + novi_vy
            if not blizu_prepreke(test_x, test_y, prepreke, prag=2.0):
                vx, vy = novi_vx, novi_vy
                print(f"[{i}] Novi pravac: vx={vx}, vy={vy}")
                break
        else:
            print(f"[{i}] Okružen preprekama! Preskačem korak.")
            time.sleep(0.5)
            continue

    # Pošalji komandu za kretanje
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

    print(f"[{i}] Pozicija: X={x:.2f}, Y={y:.2f}, Smer: vx={vx}, vy={vy}")
    brojac += 1
    time.sleep(0.2)





pozicije = [(round(msg.x, 2), round(msg.y, 2), round(msg.z, 2)) for msg in poruke]


# Odvoji X, Y, Z u zasebne liste
xs, ys, zs = zip(*pozicije)
print(pozicije)



print("Landing...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)


print("Landing...")
master1.mav.command_long_send(
    master1.target_system,
    master1.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)

# da ga spusti na zemlju tu gde je 

# nule i jedinice ovi parametri




# mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
# 0,      # confirmation
# 0,      # param1: Minimum pitch (nije obavezno, ovde 0)
# 0,      # param2: Empty, obično 0
# 0,      # param3: Empty, obično 0
# 0,      # param4: Yaw angle (npr. 0 znači da nema promene kursa)
# 0,      # param5: Latitude (ako se koristi ciljna lokacija, ovde nije)
# 0,      # param6: Longitude (nije koristeno)
# altitude # param7: Ciljna visina poletanja u metrima
print("Done.")
