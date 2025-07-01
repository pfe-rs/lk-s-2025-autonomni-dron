from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
from ignition.msgs.image_pb2 import Image
from gz.transport13 import Node
import cv2
import time
import numpy as np 


def stringmsg_cb(msg):
    print("Received message:")
    print(msg.height, msg.width)
    im = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    cv2.imshow("Image", im)
    cv2.waitKey(25)


def send_body_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration=1):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0,
        0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111,  # type_mask
        0,
        0,
        0,  # x, y, z positions (not used)
        velocity_x,
        velocity_y,
        velocity_z,  # m/s
        0,
        0,
        0,  # x, y, z acceleration
        0,
        0,
    )
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        # time.sleep(1)


def move(vehicle):
    start_time = time.time()
    while True:
        r = 5
        x_speed = -np.sin((time.time() - start_time) / r) * 5
        y_speed = np.cos((time.time() - start_time) / r) * 5
        print("x_speed: ", x_speed)
        print("y_speed: ", y_speed)

        send_body_ned_velocity(vehicle, x_speed, y_speed, 0)
        if time.time() - start_time > 20:
            send_body_ned_velocity(vehicle, 0, 0, 0)
            print("STOPPED")
            break


def land(vehicle):
    vehicle.mode = VehicleMode("LAND")


def takeoff(vehicle, takeoff_alt):
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    vehicle.simple_takeoff(takeoff_alt)  # Take off to target altitude
    while True:
        print("Altitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
            print("REACHED TARGET ALTITUDE")
            break
        time.sleep(0.5)


def main():
    connection_string = "127.0.0.1:14550"
    takeoff_alt = 10
    vehicle = connect(connection_string, wait_ready=True)

    node = Node()
    topic_img = "/default/iris/iris_demo/gimbal_small_2d/tilt_link/camera/image"

    # subscribe to a topic by registering a callback
    if node.subscribe(Image, topic_img, stringmsg_cb):
        print("Subscribing to type {} on topic [{}]".format(Image, topic_img))
    else:
        print("Error subscribing to topic [{}]".format(topic_img))
        return

    while not vehicle.is_armable:
        time.sleep(1)

    takeoff(vehicle, takeoff_alt)
    move(vehicle)
    land(vehicle)


if __name__ == "__main__":
    main()