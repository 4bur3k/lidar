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
RMAX = 32.0

class Position:
    def __init__(self, x_right, x_left, y_up, y_down):
        self.set_pos(x_right, x_left, y_up, y_down)

    def set_pos(self, x_right, x_left, y_up, y_down):
        self.x_right = x_right
        self.x_left = x_left
        self.y_up = y_up
        self.y_down = y_down

    def update_pos(self, x_right, x_left, y_up, y_down, diff_thresh=0.5):
        '''
        x/y: distances from both sides. 
        diff_thersh - thresh value of diffreneces between shifts from right and left side:
         _____________________________
        |     x_l             x_r     |
        | <----------> * <----------> |
        |  left shift   right shift   |
        |_____________________________|  

        return:
        x/y_diff:  
        '''
        x_ok = False
        y_ok = False
        if np.abs(self.x_right - x_right) - np.abs(self.x_left - x_left) <= diff_thresh:
            x_ok = True

        print('****', self.y_up, y_up, self.y_down, y_down)
        if np.abs(self.y_up - y_up) - np.abs(self.y_down - y_down) <= diff_thresh:
            y_ok = True
        
        x_diff = self.x_right - x_right
        y_diff = self.y_up - y_up

        self.set_pos(x_right, x_left, y_up, y_down)

        return x_ok, y_ok, x_diff, y_diff

class MapVisualizer:
    def __init__(self, filename, map_resolution, map_origin):
        """
        :param map_binary: 2D np.array (0 — свободно, 1 — занято)
        :param map_resolution: float — метры на пиксель
        :param map_origin: [x0, y0] — смещение карты в мировой системе координат (в метрах)
        """
        img = cv2.imread(filename+'.pgm', cv2.IMREAD_GRAYSCALE)
        
        self.map = (img < 128).astype(np.uint8)
        self.res = map_resolution
        self.origin = map_origin

        self.trajectory = []  # список позиций в пикселях

        # matplotlib
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.im = self.ax.imshow(self.map, cmap="gray_r", origin="lower")
        self.arrow = None
        self.traj_line, = self.ax.plot([], [], 'g-', linewidth=1, label="Trajectory")

        self.ax.set_title("Drone Pose Visualization")
        self.ax.set_xlabel("X (pixels)")
        self.ax.set_ylabel("Y (pixels)")
        self.ax.legend()
        plt.ion()
        plt.show()

    def update_pose(self, pose):
        """
        Обновить отображение позиции дрона на карте.

        :param pose: [x, y, theta] в метрах
        """
        x, y, theta = pose

        # Перевод из мировых координат в пиксельные
        px = (x - self.origin[0]) / self.res
        py = (y - self.origin[1]) / self.res

        # Сохраняем траекторию
        self.trajectory.append((px, py))

        # Обновляем траекторию
        traj_x, traj_y = zip(*self.trajectory)
        self.traj_line.set_data(traj_x, traj_y)

        # Удаляем старую стрелку
        if self.arrow:
            self.arrow.remove()

        # Рисуем новую стрелку
        arrow_length = 15  # в пикселях
        dx = arrow_length * np.cos(theta)
        dy = arrow_length * np.sin(theta)

        self.arrow = self.ax.arrow(px, py, dx, dy,
                                   head_width=5, head_length=5,
                                   fc='red', ec='red')

        self.ax.set_xlim(0, self.map.shape[1])
        self.ax.set_ylim(0, self.map.shape[0])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

class Scan:
    def __init__(self):
        pass

class Lidar_new:

    def __init__(self):

        self.position = Position(0, 0, 0, 0)

        self.visualizer = MapVisualizer('map/lidar_map', 0.005, [-0.5, -0.4])
        # self.mcl = MonteCarloLocalization('map1', 0.005, [-5.0, -5.0])

        self.fig = plt.figure()
        self.fig.canvas.set_window_title('YDLidar LIDAR Monitor')

        gs = GridSpec(2, 1, height_ratios=[8, 1])

        self.lidar_polar = self.fig.add_subplot(gs[0], polar=True)
        plt.subplots_adjust(bottom=0.25)
        self.lidar_polar.autoscale_view(True,True,True)
        self.lidar_polar.set_rmax(RMAX)
        self.lidar_polar.grid(True)

        self.text_ax = self.fig.add_subplot(gs[1])
        self.text_ax.axis('off')  
        
        self.texts = [
            self.text_ax.text(0.2, 0.5, '', ha='center', va='center', transform=self.text_ax.transAxes),
            self.text_ax.text(0.5, 0.5, '', ha='center', va='center', transform=self.text_ax.transAxes),
            self.text_ax.text(0.8, 0.5, '', ha='center', va='center', transform=self.text_ax.transAxes),
            self.text_ax.text(1.0, 0.5, '', ha='center', va='center', transform=self.text_ax.transAxes)
        ]
       

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

        self.__zero_pos = [0, 0]
        self.cur_pos = [0, 0]

    def add_points(self, angles, ranges, intenses):
        curr_time_arr = [datetime.now()] * len(angles)

        data = np.array([curr_time_arr, angles, ranges, intenses]).T

        self.df = pd.concat([self.df, pd.DataFrame(data, columns = ['time', 'angle', 'range', 'intens'])], ignore_index=True)


    def save_points(self):
        self.df.to_csv('data/data.csv')
    
    def interpolate(self, angles, ranges, resolution = 1.0, max_range=10.0):
        angles, ranges = np.array(angles), np.array(ranges)
        sorted_indices = np.argsort(angles)
        angles_sorted = angles[sorted_indices]
        ranges_sorted = ranges[sorted_indices]

        target_angles = np.arange(0, 360, resolution)
        interpolated = np.interp(
            target_angles,
            angles_sorted,
            ranges_sorted,
            left=max_range,
            right=max_range
        )
        return interpolated


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
                # print(point.angle, point.range, point.intensity, sep=' | ')
            self.lidar_polar.clear()
            self.lidar_polar.scatter(angle, ran, c=intensity, cmap='hsv', alpha=0.95)

            # y_up_range, x_right_range, y_down_range, x_left_range = self.get_ranges(angle, ran)
            # x_ok, y_ok, x_diff, y_diff = self.position.update_pos(x_right_range, x_left_range, y_up_range, y_down_range)
            
            # self.texts[0].set_text(f'Right: {x_diff:.2f}')
            # self.texts[1].set_text(f'Up: {y_diff:.2f}')
            # self.texts[2].set_text(f'x_ok: {x_ok}')
            # self.texts[3].set_text(f'y_ok: {y_ok}')

            self.add_points(angle, ran, intensity)

    def draw_points(self):
        try:
            ret = self.laser.initialize();
            print('One more time')
            if ret:
                ret = self.laser.turnOn();
                if ret:
                    ani = animation.FuncAnimation(self.fig, self._animate, interval=50)
                    plt.show()
                self.save_points()
                self.laser.turnOff();
            self.laser.disconnecting();
            plt.close();
        except KeyboardInterrupt:
            self.save_points() 

    def MCL(self):
        # self.mcl = MonteCarloLocalization('map/lidar_map', 0.005, [-0.5, -0.4])
        
        ret = self.laser.initialize()
        if ret:
            ret = self.laser.turnOn()
            r = r = self.laser.doProcessSimple(self.scan)
            if r:
                    angle = []
                    ran = []
                    for point in self.scan.points:
                        angle.append(point.angle)
                        ran.append(point.range)
                        # print(point.angle, point.range, sep=' | ')
                    self.lidar_polar.clear()
            
            # self.sl = SimpleLocalisator(angle, ran)  
              

            while ret and ydlidar.os_isOk():
                r = self.laser.doProcessSimple(self.scan)
                if r:
                    angle = []
                    ran = []
                    for point in self.scan.points:
                        angle.append(math.degrees(point.angle))
                        ran.append(point.range)
                        # print(point.angle, point.range, sep=' | ')
                    self.lidar_polar.clear()
                    
                print('****', len(angle), '\n\n')


    def save_map(self, resolution=0.005, filename="map/lidar_map.pgm"):
        ret = self.laser.initialize()
        if ret:
            ret = self.laser.turnOn()
            counter = 0
            while ret and ydlidar.os_isOk() and counter < 10:
                r = self.laser.doProcessSimple(self.scan)
                if r:
                    angles = []
                    ranges = []
                    for point in self.scan.points:
                        if point.range > 1e-3:
                            angles.append(point.angle)
                            ranges.append(point.range)
                        
                    self.lidar_polar.clear()
                    print('Other scan')
                    counter += 1
        
       
        # Перевод в XY координаты (в метрах)
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # Границы карты (в метрах)
        min_x, max_x = xs.min(), xs.max()
        min_y, max_y = ys.min(), ys.max()

        # Размеры карты в пикселях
        width_m = max_x - min_x
        height_m = max_y - min_y
        width_px = int(np.ceil(width_m / resolution)) + 1
        height_px = int(np.ceil(height_m / resolution)) + 1

        # Смещение (для привязки 0,0 к нижнему левому краю)
        offset_x = -min_x
        offset_y = -min_y

        # Создание карты (белый фон)
        map_img = Image.new("L", (width_px, height_px), 255)
        draw = ImageDraw.Draw(map_img)

        # Преобразуем точки в пиксельные координаты
        pixel_points = [
            (
                int((x + offset_x) / resolution),
                int((y + offset_y) / resolution)
            )
            for x, y in zip(xs, ys)
        ]

        # Соединяем точки линией
        if len(pixel_points) > 1:
            draw.line(pixel_points, fill=0, width=1)

        # Сохраняем
        map_img.save(filename)
        print(f"Карта сохранена как {filename}")
        print(f"Размер: {width_px} x {height_px} px  ({width_m:.2f}m x {height_m:.2f}m), смещение {float(min_x), float(min_y)}")

    def get_ranges(self, angle_arr, ran_arr):
        '''
                 up
                 ___
        left    /   \    right
                \___/
                down 
        
        '''
        # Getting inds for 0, 90, 180, -90
        y_up_range, x_right_range, y_down_range, x_left_range = -1, -1, -1, -1
        
        for ind, val in enumerate(angle_arr):
            val = math.degrees(val)
            if abs(val - 0) < 2:
                y_up_range = ran_arr[ind]
                continue
            if abs(val - 90) < 2:
                x_right_range = ran_arr[ind]
                continue
            if abs(val - 180) < 2 or abs(val + 180) < 2:
                y_down_range = ran_arr[ind]
                continue
            if abs(val + 90) < 2:
                x_left_range = ran_arr[ind]
        
        return y_up_range, x_right_range, y_down_range, x_left_range

    def _polar_to_cortes(self, angle, ran):
        x, y = ran * math.cos(angle), ran * math.cos(angle)
        return [x, y]

    def _init_position(self, angle, ran):
        self.__zero_pos == self._polar_to_cortes(angle, ran)

    def _get_position(self):
        return self.cur_pos
    
    def _update_position(self, angle, ran):
        self
    
    def get_distances(self):
        pass

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
        
        # self.df = pd.DataFrame(columns = {'time', 'angle', 'range', 'intens'})


    
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

    def _animate(self, num):
        r = self.laser.doProcessSimple(self.scan);
        if r:
            angle = []
            ran = []
            intensity = []
            for point in self.scan.points:
                if point.range > 0.2:
                    angle.append(point.angle);
                    ran.append(point.range);
                    intensity.append(point.intensity);
            intensity = [1008.0] * 360
            angle, ran = self.interpolate(angle, ran) 
            
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

