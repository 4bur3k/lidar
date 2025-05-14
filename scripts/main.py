import os
import ydlidar
import time
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

import pandas as pd
RMAX = 32.0

class Lidar:

    def __init__(self):
        self.fig = plt.figure()
        self.fig.canvas.set_window_title('YDLidar LIDAR Monitor')

        self.lidar_polar = plt.subplot(polar=True)
        self.lidar_polar.autoscale_view(True,True,True)
        self.lidar_polar.set_rmax(RMAX)
        self.lidar_polar.grid(True)

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

    def save_points(self, angle, range, intens):
        df = pd.DataFrame(columns = {'time', 'angle', 'range', 'intens'})

        df = 

    def _animate(self, num):
        r = self.laser.doProcessSimple(self.scan);
        if r:
            angle = []
            ran = []
            intensity = []
            for point in self.scan.points:
                angle.append(point.angle);
                ran.append(point.range);
                intensity.append(point.intensity);
                print(point.angle, point.range, point.intensity, sep=' | ')
            self.lidar_polar.clear()
            self.lidar_polar.scatter(angle, ran, c=intensity, cmap='hsv', alpha=0.95)

    def draw_points(self):
        ret = self.laser.initialize();
        if ret:
            ret = self.laser.turnOn();
            if ret:
                ani = animation.FuncAnimation(self.fig, self._animate, interval=50)
                plt.show()
            self.laser.turnOff();
        self.laser.disconnecting();
        plt.close();



lidar = Lidar()

lidar.draw_points()
