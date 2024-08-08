from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, mavutil
import time

# Connect to the vehicle
connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.arm()
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_body_ned_velocity(vx, vy, vz, dur):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
    )
    for _ in range(0, dur):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1
    else:
        is_relative = 0

    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, heading, 0, 1, is_relative, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

    time.sleep(3)

def draw_polygon(sides=3, distance=5):
    if sides < 3:
        return print("Cannot draw a polygon with less than 3 sides")
    
    angle = 360 / sides
    speed = 5
    duration = int(distance / speed)

    for i in range(sides):
        print(f"Running side {i + 1}")
        send_body_ned_velocity(speed, 0, 0, duration)
        condition_yaw(angle, relative=True)
        time.sleep(2)

arm_and_takeoff(10)
time.sleep(5)  
draw_polygon(5, 10) 

vehicle.close()
