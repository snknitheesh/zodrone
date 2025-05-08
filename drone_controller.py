from pymavlink import mavutil
import time
from math import radians, cos, sin, sqrt, atan2

class DroneController:
    def __init__(self, connection_str='udp:127.0.0.1:14550'):
        self.master = mavutil.mavlink_connection(connection_str)
        self.master.wait_heartbeat()
        print(f"[INFO] Connected to the Drone.")

    def wait_until_initialization(self):
        while True:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if msg:
                base_mode = msg.base_mode
                system_status = msg.system_status
                if system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                    break
            time.sleep(1)

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        while True:
            ack = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if ack.custom_mode == mode_id:
                print(f"[INFO] Mode changed to {mode_name}")
                break

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self.master.motors_armed_wait()
        print("[INFO] Armed.")

    def takeoff(self, altitude):
        print("[INFO] Takeoff started...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        self.wait_until_altitude_reached(altitude)

    def land(self):
        print("[INFO] landing...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.wait_until_landed()

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.master.motors_disarmed_wait()
        print("[INFO] Motors disarmed.")

    def wait_until_altitude_reached(self, target_alt, threshold=0.1):
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_alt = msg.relative_alt / 1000.0
                if abs(current_alt - target_alt) < threshold:
                    break

    def wait_until_landed(self, threshold=0.1):
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                alt = msg.relative_alt / 1000.0
                if alt < threshold:
                    print("[INFO] Drone Landed.")
                    break
                
    def get_distance_meters(self, lat1, lon1, lat2, lon2):
        R = 6371000 
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        return R * c              
                
    def goto_gps_position(self, lat, lon, alt, drop_sensor=False, tag="Position."):
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        self.master.mav.set_position_target_global_int_send(
            0, 
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  
            0b0000111111111000,  
            lat_int, lon_int, alt,
            0, 0, 0,  
            0, 0, 0, 
            0, 0      
        )
        print(f"[INFO] Waiting to reach {tag}...")
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
                current_alt = msg.relative_alt / 1000.0
                distance = self.get_distance_meters(current_lat, current_lon, lat, lon)
                if distance < 1.0 and abs(current_alt - alt) < 0.5:
                    print(f"[INFO] Arrived at {tag}.")
                    break
        if drop_sensor:
            self.place_sensor(lat, lon, alt, drop_sensor, tag)
            
    def place_sensor(self, lat, lon, alt, drop_sensor, tag):
        self.goto_gps_position( lat, lon, 1, False, "Lower Altitude to drop the sensor.")
        self.goto_gps_position( lat, lon, alt, False, "Old Altitude to continue with the mission.")
        
        