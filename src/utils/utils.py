from structures import Waypoint
import csv
import numpy as np

def load_waypoints(csv_path: str) -> list[Waypoint]:
    waypoints = []
    
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f, delimiter='|')
        for row in reader:
            coords = row['coordinates'].strip().split(',')
            x, y = float(coords[0]), float(coords[1])
            was_reached = row['was_reached'].strip().lower() in ('true', '1', 'yes')
            waypoints.append(Waypoint(x=x, y=y, was_reached=was_reached))
    
    return waypoints    