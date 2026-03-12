''' Fly over all waypoints '''


from hardware.lidar_connector import LidarConnection
from hardware.mavlink import MavlinkController
from modules.localisation import LocalisationInterface, get_direction, get_distance
from utils.utils import load_waypoints
from utils.structures import Waypoint

import math

import logging

logging.basicConfig(
    filename='log/main.log', 
    level=logging.INFO,      
    format='%(asctime)s [%(levelname)s] %(message)s',
)

# <=== Hardcoded variables: ===>
# point reached distacne
REACH_THRESHOLD = 0.1

# start lidar_porcessing
lidar = LidarConnection()
controller = MavlinkController()
localisation = LocalisationInterface()

lidar.start()

waypoints = load_waypoints("./routes/route.csv")
waypoint_index = 0 

try:
    while True:
       # skip reached points
        while waypoint_index < len(waypoints) and waypoints[waypoint_index].was_reached:
            waypoint_index += 1

        if waypoint_index >= len(waypoints):
            logging.info("All waypoints reached!")
            break

        points = lidar.get_scan()
        if points is None:
            continue

        logging.info(f"Got {len(points)} points")

        curr_pos = localisation.get_current_position(points)
        target = waypoints[waypoint_index]
        target_pos = (target.x, target.y)

        distance = get_distance(curr_pos, target_pos)

        if distance < REACH_THRESHOLD:
            logging.info(f"Waypoint {waypoint_index} reached!")
            waypoints[waypoint_index].was_reached = True
            waypoint_index += 1
            continue
        
        direction_vector = get_direction(curr_pos, target_pos)
        angle = math.degrees(math.atan2(direction_vector[1], direction_vector[0]))

        logging.info(f"Position: {curr_pos} | Target: {target_pos} | Distance: {distance:.2f}m | Angle: {angle:.1f}°")

        controller.move(direction_vector[0], direction_vector[1])
finally:
    lidar.stop()
    