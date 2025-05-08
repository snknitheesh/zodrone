from drone_controller import DroneController
import time

# Import the controller
drone = DroneController()

# Commands
drone.set_mode("GUIDED")
drone.wait_until_initialization()
drone.arm_vehicle()
drone.takeoff(10)
drone.goto_gps_position(lat=-35.361811240, lon=149.16459099, alt=15, drop_sensor=True, tag='Drop Zone 1')
drone.set_mode("RTL")
drone.wait_until_landed()
drone.disarm()
print("[INFO] Mission completed!")
