from drone_controller import DroneController
import time

drone = DroneController()

drone.set_mode("GUIDED")
print("[INFO] Mission started")
drone.arm()
drone.takeoff(2)
drone.goto_gps_position(lat=-35.36300267, lon=149.16521565, alt=10, drop_sensor=False, tag='Custom Terrain')
drone.goto_gps_position(lat=-35.36238710, lon=149.16518507, alt=20, drop_sensor=False, tag='Slope Terrain')
drone.goto_gps_position(lat=-35.36157830, lon=149.16500551, alt=10, drop_sensor=False, tag='Volcano Terrain')

# drone.set_mode("RTL")
# drone.wait_until_landed()
# drone.disarm()
# print("[INFO] Mission completed!")

 