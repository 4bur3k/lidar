import numpy as np

import math
import cv2

class FingerprintLocalisator:
    def __init__(self):     
        pass
    

    def match_scan_to_fingerprint():
        pass

    def localise(self, angle, ran, gridmap):
        '''
        angle:
        ran:
        '''
        
        min_distance = float("inf")
        best_match = None 

        for position, reference_scan in fingerprint_db.items():
            if len(reference_scan) != len(real_scan):
                continue  # пропустить несовпадающие по размеру сканы

            distance = np.linalg.norm(real_scan - reference_scan)
            if distance < min_distance:
                min_distance = distance
                best_match = position

        return best_match, min_distance



    

class MonteCarloLocalization:
    def __init__(self, map_name, map_resolution, map_origin, num_particles=500):
        
        self.map = self.load_map(map_name)
        self.res = map_resolution # м на пиксель
        self.origin = np.array(map_origin)  # [x0, y0] в метрах
        self.num_particles = num_particles
        self.particles = self._initialize_particles()
    
    def load_map(self, filename):
        img = cv2.imread(filename+'.pgm', cv2.IMREAD_GRAYSCALE)
        occupancy_grid = (img < 128).astype(np.uint8)
        return occupancy_grid

    def _initialize_particles(self):
        h, w = self.map.shape
        particles = []
        while len(particles) < self.num_particles:
            xi = np.random.randint(0, w)
            yi = np.random.randint(0, h)
            if self.map[yi, xi] == 0:
                x = xi * self.res + self.origin[0]
                y = yi * self.res + self.origin[1]
                theta = np.random.uniform(-np.pi, np.pi)
                particles.append([x, y, theta])
        return np.array(particles)

    def _simulate_lidar(self, pose, angles, max_range=10.0):
        x, y, theta = pose
        scan = []
        hit = False
        for a in angles:
            angle = theta + a
            for r in np.linspace(0, max_range, int(max_range * 20)):  # шаг ~5см
                xi = int((x + r * np.cos(angle) - self.origin[0]) / self.res)
                yi = int((y + r * np.sin(angle) - self.origin[1]) / self.res)

                if 0 <= xi < self.map.shape[1] and 0 <= yi < self.map.shape[0]:
                    if self.map[yi, xi] == 1:
                        scan.append(r)
                        hit = True
                        break
                    else:
                        break
            # else:
            #     scan.append(max_range)
            if not hit:
                scan.append(max_range)
        return np.array(scan)

    def _compute_weight(self, particle, observed_ranges, angles):
        expected = self._simulate_lidar(particle, angles)

        if len(expected) != len(observed_ranges):
            return 1e-6

        diff = observed_ranges - expected
        if not np.all(np.isfinite(diff)):
            return 1e-6

        weight = np.exp(-np.sum(diff ** 2) / 1.0)
        if not np.isfinite(weight):
            return 1e-6

        return weight

    def _resample(self, particles, weights):
        weights = np.nan_to_num(weights, nan=0.0, posinf=0.0, neginf=0.0)
        weights_sum = np.sum(weights)

        if weights_sum == 0:
            weights = np.ones_like(weights) / len(weights)
        else:
            weights /= weights_sum

        indices = np.random.choice(len(particles), size=len(particles), p=weights)
        return particles[indices]

    def _motion_model(self, dx=0.0, dy=0.0, dtheta=0.0):
        noise = np.random.normal(0, [0.02, 0.02, 0.01], self.particles.shape)
        self.particles += np.array([dx, dy, dtheta]) + noise
        return self.particles

    def update(self, angles, ranges):
        """
        Основной вызов алгоритма: обновить MCL по новому lidar-скану
        """
        # Обновить движение (можно заменить dx/dy/dtheta на одометрию, если есть)
        # self._motion_model()

        # Вычислить веса
        weights = np.array([
            self._compute_weight(p, ranges, angles)
            for p in self.particles
        ])

        # Ресемплирование
        self.particles = self._resample(self.particles, weights)

        # Оценка позиции — среднее
        estimate = np.mean(self.particles, axis=0)
        return estimate 

class SimpleLocalisator:
    def __init__(self, angle, ran):
        self.x = 0
        self.y = 0

        self.x_right = None
        self.x_left = None
        self.y_up = None
        self.y_down = None
        y_up, x_right, y_down, x_left = self.get_ranges(angle, ran)
        self.set_pos(x_right, x_left, y_up, y_down)

    def set_pos(self, x_right, x_left, y_up, y_down):
        self.x_right = x_right
        self.x_left = x_left
        self.y_up = y_up
        self.y_down = y_down      

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


    def update_pos(self, angle, ran, diff_thresh=0.5):
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
        y_up, x_right, y_down, x_left = self.get_ranges(angle, ran)

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

        self.x += x_diff
        self.y += y_diff
        # return x_ok, y_ok, x_diff, y_diff
        return self.x, self.y, 0
