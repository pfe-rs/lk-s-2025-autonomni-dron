from pymavlink import mavutil
import time

# Povezivanje na MAVProxy
connection_string = "udp:127.0.0.1:14550"
print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# Arm
print("Arming vehicle...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# Čekaj dok ne bude armovan
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    print(f"Armed: {armed}")
    if armed:
        break
    time.sleep(1)

# Postavi GUIDED mod
mode_id = master.mode_mapping()['GUIDED']
print("Setting mode to GUIDED...")
master.set_mode(mode_id)
time.sleep(2)

# Početna pozicija u LOCAL_NED
print("Čekam početnu poziciju...")
start_pos = None
while not start_pos:
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    if msg:
        start_pos = (msg.x, msg.y, msg.z)
print(f"Početna pozicija: {start_pos}")

# Poletanje
altitude = 10
print(f"Sending takeoff command to {altitude} meters...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    altitude)

# Čekaj da dostigne visinu
print("Waiting to reach altitude...")
start = time.time()
while time.time() - start < 30:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"Altitude: {alt:.2f} m")
        if alt >= altitude * 0.95:
            print("Target altitude reached")
            break

# Novi waypoints u lokalnim koordinatama
waypoints = [
    (start_pos[0] + 1000, start_pos[1], start_pos[2]),       # 10m napred
    (start_pos[0] + 1000, start_pos[1] + 1000, start_pos[2]),  # pa desno 10m
    (start_pos[0], start_pos[1], start_pos[2])             # nazad na start
]

def goto_position(master, x, y, z, timeout=30):
    print(f"Idem na tačku ({x:.2f}, {y:.2f}, {z:.2f})")
    start_time = time.time()
    while time.time() - start_time < timeout:
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # samo pozicija
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            dx = abs(msg.x - x)
            dy = abs(msg.y - y)
            dz = abs(msg.z - z)
            print(f"Trenutno: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}")
            if dx < 0.5 and dy < 0.5 and dz < 0.5:
                print("Stigao.")
                break
        time.sleep(0.1)

# Prođi kroz sve tačke
for x, y, z in waypoints:
    goto_position(master, x, y, z)

# Sletanje
print("Landing...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0)

print("Done.")
