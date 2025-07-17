import os
import ydlidar
import time
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import numpy as np

import pandas as pd
from datetime import datetime
import math
import cv2
from PIL import Image, ImageDraw

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
        
        self.df = pd.DataFrame(columns = ['time', 'angle', 'range', 'intens'])

    def interpolate(self, angles, ranges, resolution = 1.0, max_range=10.0):
        angles, ranges = np.array(np.mod(angles, 2 * np.pi)), np.array(ranges)
        sorted_indices = np.argsort(angles)
        angles_sorted = angles[sorted_indices]
        ranges_sorted = ranges[sorted_indices]

        resolution_rad = np.deg2rad(resolution)
        target_angles = np.arange(0, 2 * np.pi, resolution_rad)

        interpolated_ranges = np.interp(
            target_angles,
            angles_sorted,
            ranges_sorted,
            left=max_range,
            right=max_range
        )
            
        return target_angles, interpolated_ranges


    def turn_on(self, min_dist, range_min_thresh=0.2):
        '''
        Start checking collisions
        min_dist: minimal distance of stoping in meters
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
                        if point.range > range_min_thresh:
                            angle_range_arr.append([point.angle, point.range])
                            angles.append(point.angle)
                            ranges.append(point.range)
                        # if point.range < min_dist and point.range > 0:
                        #     self.mavlink.stop_drone()
                        #     break
                        # elif point.range < min_ran:
                        #     min_ran = point.range
                        #     min_angle = point.angle 
                    inter_angles, inter_ranges = self.interpolate(angles, ranges)             
                    


   
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

lidar.turn_on(10)

