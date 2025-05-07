# drone_controller.py

from pymavlink import mavutil
import time

class DroneController:
    def __init__(self, connection_str='udp:127.0.0.1:14550'):
        print("Connecting to drone...")
        self.master = mavutil.mavlink_connection(connection_str)
        self.master.wait_heartbeat()
        print(f"Connected to system {self.master.target_system} component {self.master.target_component}")

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Set mode to {mode_name}")
        while True:
            ack = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if ack.custom_mode == mode_id:
                print(f"Mode changed to {mode_name}")
                break

    def arm_vehicle(self):
        print("Arming...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self.master.motors_armed_wait()
        print("Armed.")

    def takeoff(self, altitude=10):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        print(f"Takeoff command sent to {altitude} meters")

    def land(self):
        print("Initiating landing...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def disarm(self):
        print("Disarming motors...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.master.motors_disarmed_wait()
        print("Motors disarmed.")

    def wait_until_altitude_reached(self, target_alt, threshold=0.3):
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_alt = msg.relative_alt / 1000.0
                if abs(current_alt - target_alt) < threshold:
                    print(f"Reached altitude: {current_alt:.2f} m")
                    break

    def wait_until_landed(self, threshold=0.1):
        print("Waiting for drone to land...")
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                alt = msg.relative_alt / 1000.0
                if alt < threshold:
                    print("Drone Landed.")
                    break
