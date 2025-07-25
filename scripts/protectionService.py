import os
import ydlidar
import time
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import numpy as np
import statistics

import pandas as pd
from datetime import datetime
import math
import cv2
from PIL import Image, ImageDraw

from mavlink import MavlinkController

class OctoSector:
    def __init__(self):
        self.min_dist = None
        self.max_dist = None
        self.median_dist = None
    
    def set_distances(self, min_dist, max_dist, median_dist):
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.median_dist = median_dist

    def get_distances(self):
        return self.min_dist, self.max_dist, self.median_dist

class Octagone:
    '''
        E = (0, 45)
        NE = (45, 90)
        N = (90, 135)
        NW = (135, 180)
        W = (180, 225)
        SW = (225, 270)
        S = (270, 315)
        SE = (315, 360)
    '''
    def __init__(self):
        self.e = OctoSector()
        self.ne = OctoSector()
        self.n = OctoSector()
        self.nw = OctoSector()
        self.w = OctoSector()
        self.s = OctoSector()
        self.sw = OctoSector()
        self.se = OctoSector()

    def set_sphere(self, ranges):
        '''
        ranges in m\n
        angles in degrees
        '''
        sector_data = {
            'e': ranges[338:360] + ranges[0:23],
            'ne':ranges[23:68],                     
            'n': ranges[68:113],                    
            'nw':ranges[113:158],           
            'w': ranges[158:203],            
            'sw':ranges[203:248],      
            's': ranges[248:293],             
            'se':ranges[293:338], 
        }

        # Офк, надо искать красные сектора тут
        # Но кого ебет чужое горе (и скорость??? (переделаем, когда с архитетурой станет яснее))
        for name, data in sector_data.items():
            if data:
                min_dist = min(data)
                max_dist = max(data)
                median_dist = statistics.median(data)
            else:
                min_dist = max_dist = median_dist = None

            getattr(self, name).set_distances(min_dist, max_dist, median_dist)
    
    def get_red_sectors(self, dangerous_distance=1.0):
        sectors = []

        for attr_name, attr_value in self.__dict__.items():
            if isinstance(attr_value, OctoSector):
                min_dist, max_dist, median_dist = attr_value.get_distances()
                
                # тут потом напишем охуенную логику
                if median_dist < dangerous_distance:
                    sectors.append((attr_name, median_dist))

        return sectors


class CollisionProtection:

    def __init__(self):
        
        ports = ydlidar.lidarPortList();
        port = "/dev/ttyUSB0";
        for key, value in ports.items():
            port = value;
            
        self.laser = ydlidar.CYdLidar();
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200);
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 3);
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0);
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08);
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False);

        self.scan = ydlidar.LaserScan()

        self.octagone = Octagone()
        self.controller = MavlinkController()
        
        self.df = pd.DataFrame(columns = ['time', 'angle', 'range', 'intens'])

    def interpolate(self, angles, ranges, resolution = 1.0, max_range=10.0):
        angles, ranges = np.array(np.mod(angles, 2 * np.pi)), np.array(ranges)
        sorted_indices = np.argsort(angles)
        angles_sorted = angles[sorted_indices]
        ranges_sorted = ranges[sorted_indices]

        resolution_rad = np.deg2rad(resolution)
        target_angles_rad = np.arange(0, 2 * np.pi, resolution_rad)

        interpolated_ranges = np.interp(
            target_angles_rad,
            angles_sorted,
            ranges_sorted,
            left=max_range,
            right=max_range
        )
            
        return target_angles_rad, interpolated_ranges

    def simple_count_pwm_values(self, distance):
        '''
        distance in meters
        '''
        max_pwm = 1700
        min_pwm = 1300
        center = 1500

        offset = min(max(distance * 10, -200), 200)  # ±200 максимум

        return center - offset
    
    def count_pwm_values(self, directions):

        DIRECTION_MAP = {
        'N':  (0, 1),
        'NE': (-1, 1),
        'E':  (-1, 0),
        'SE': (-1, -1),
        'S':  (0, -1),
        'SW': (1, -1),
        'W':  (1, 0),
        'NW': (1, 1)
        }

        dx_total, dy_total = 0.0, 0.0

        for direction, distance in directions:
            direction = direction.upper()
            if direction not in DIRECTION_MAP:
                raise ValueError(f"Недопустимое направление: {direction}")
            if distance <= 0:
                continue  # пропускаем нулевые и отрицательные

            dx, dy = DIRECTION_MAP[direction]
            dx_total += dx * distance
            dy_total += dy * distance

        # Если всё компенсировалось
        if dx_total == 0 and dy_total == 0:
            raise ValueError('EBAT ZAZHALO')

        # Нормализация итогового вектора
        length = math.hypot(dx_total, dy_total)
        dx_norm = dx_total / length
        dy_norm = dy_total / length

        # Сила отклонения — по суммарной длине всех векторов (можно иначе)
        max_offset = 400
        total_magnitude = min(length * 10, max_offset)

        roll_pwm  = 1500 + dx_norm * total_magnitude
        pitch_pwm = 1500 + dy_norm * total_magnitude

        # Ограничим PWM
        roll_pwm = int(max(1000, min(2000, roll_pwm)))
        pitch_pwm = int(max(1000, min(2000, pitch_pwm)))

        return roll_pwm, pitch_pwm
    

    def turn_on(self, distance_to_stop, dist_thresh=0.2):
        '''
        Start checking collisions \n
        distance_to_stop: minimal distance of stoping in meters \n
        dist_thresh: threshhold to sort fake data from lidar
        '''
        ret = self.laser.initialize()
        if ret:
            ret = self.laser.turnOn()           

            while ret and ydlidar.os_isOk():
                r = self.laser.doProcessSimple(self.scan)
                min_angle, min_ran = sys.maxsize, sys.maxsize

                if r:
                    angle_range_arr = []
                    angles, ranges = [], []
                    for point in self.scan.points:
                        if point.range > dist_thresh:
                            angle_range_arr.append([point.angle, point.range])
                            angles.append(point.angle)
                            ranges.append(point.range)
                       
                    inter_angles, inter_ranges = self.interpolate(angles, ranges)             
                    print(np.rad2deg(inter_angles))

                    self.octagone.set_sphere(np.rad2deg(inter_ranges))
                    red_sectors = self.octagone.get_red_sectors(distance_to_stop)

                    roll, pitch = self.count_pwm_values(red_sectors)

                    self.controller.stop_drone()
                   
   
    # def check_collision(self, angle_range_arr, dist_thresh):
    #     '''
    #     Returns 
    #     [True, angle, distance] if collision is close 
    #     [False, angle, min distance] if no         
    #     '''
        
    #     min_angle, min_ran = angle_range_arr[0]
        
    #     for angle, ran in angle_range_arr:
    #         # If collision is close
    #         if ran < dist_thresh:
    #             return True, angle, ran
            
    #         if ran < min_ran:
    #             min_angle, min_ran = angle, ran
        
    #     return False, min_angle, min_ran

   

lidar = CollisionProtection()

lidar.turn_on(2.0)

