from pymavlink import mavutil
import time, threading

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("waiting...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system} component {master.target_component}")

def set_mode(mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Set mode to {mode_name}")
    while True:
        ack = master.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"Mode changed to {mode_name}")
            break

def arm_vehicle():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    master.motors_armed_wait()
    print("Armed.")

def takeoff(altitude=10):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 
        0, 0, 0, 0, 0, 0,
        altitude  
    )
    print(f"Takeoff command sent to {altitude} meters")
    
def land():
    print("Initiating landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def disarm():
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors disarmed.")
    
def wait_until_landed(threshold=0):
    print("Waiting for drone to land...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            alt = msg.relative_alt / 1000.0
            if alt < threshold:
                print("Drone Landed.")
                break

def wait_until_altitude_reached(target_alt, threshold=0.3):
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            if abs(current_alt - target_alt) < threshold:
                print(f"Reached altitude: {current_alt:.2f} m")
                break

# TakeOff and Land
set_mode("GUIDED")
arm_vehicle()
takeoff(10)
wait_until_altitude_reached(10)
time.sleep(10)
land()
wait_until_landed()
disarm()
print("Mission completed!")