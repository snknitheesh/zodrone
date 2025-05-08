from drone_controller import DroneController
import time

drone = DroneController()

# Mission - 1
drone.set_mode("GUIDED")
print("[INFO] Mission started")
# drone.wait_until_initialization()  # rewrite not working...
drone.arm()
drone.takeoff(10)
drone.goto_gps_position(lat=-35.361811240, lon=149.16459099, alt=15, drop_sensor=True, tag='Drop Zone 1')
drone.set_mode("RTL")
drone.wait_until_landed()
drone.disarm()
print("[INFO] Mission completed!")