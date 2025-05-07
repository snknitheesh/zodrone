from drone_controller import DroneController
import time

# Import the controller
drone = DroneController()

# Commands
drone.set_mode("GUIDED")
drone.arm_vehicle()
drone.takeoff(10)
drone.wait_until_altitude_reached(10)
time.sleep(10) 
drone.land()
drone.wait_until_landed()
drone.disarm()
print("Mission completed!")
