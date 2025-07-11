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

        
    def turn_on(self, min_dist):
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
                    for point in self.scan.points:
                        angle_range_arr.append([point.angle, point.range])
                        if point.range < min_dist:
                            print('PIZDAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
                        elif point.range < min_ran:
                            min_ran = point.range
                            min_angle = point.angle 
                                          
                print('****', len([]), '\n\n')


   
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

lidar.turn_on()

