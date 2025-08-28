import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import cv2

PATH_TO_MAP_FOLDER = '/home/denis/work/lidar/map/'

class Map:
    '''
    Args:
    points: np.ndarray[Point]: all the availeble points(states) on the map
    walls: np.ndarray[Line]: all the walls on the map
    doors: np.ndarray[Line]: all the entrances/exits on the map
    '''
    def __init__(self, map_filename, pixel_size, angle_resolution_deg=1.0, sampling_step_m=0.2, max_range=5.0):
        self.pixel_size_m = pixel_size
        self.max_range_m = max_range
        self.angles_rad = np.deg2rad(np.arange(0, 360, angle_resolution_deg))

        self.grid_img, map_width_m, map_height_m = self._load_occupancy_map(map_filename)

        self.grid_dict = self._generate_grid_map(map_height_m, map_width_m, sampling_step_m)

    @property
    def map(self):
        return self.grid_dict
    
    def visualize(self, show_points=True):
        plt.figure(figsize=(8, 8))
        plt.imshow(self.grid_img, cmap='gray_r', origin='upper')
        plt.title("Occupancy Grid Map")
        plt.xlabel("X (pixels)")
        plt.ylabel("Y (pixels)")

        if show_points:
            xs = [x / self.pixel_size_m for (x, y) in self.grid_dict.keys()]
            ys = [y / self.pixel_size_m for (x, y) in self.grid_dict.keys()]
            plt.scatter(xs, ys, s=5, c='red', alpha=0.6, label='Scan Points')

        plt.legend()
        plt.grid(False)
        plt.axis("equal")
        plt.show()

    def _load_occupancy_map(self, map_filename, threshold=128):
        '''
        Image -> occupancy grid
        map_filename: filename of the map
        '''
        img = Image.open(PATH_TO_MAP_FOLDER+map_filename).convert("L")
        data = np.array(img)
        map_grid = (data < threshold).astype(np.uint8)  # 1 = стена, 0 = свободно

        height_px, width_px = map_grid.shape
        map_width_m = width_px * self.pixel_size_m
        map_height_m = height_px * self.pixel_size_m

        print('Map has been loaded')

        return map_grid, map_width_m, map_height_m
    

    def simulate_lidar_scan(self, position_m):
        """
        Симуляция одного лидара из заданной точки на карте.
        """
        height_px, width_px =  self.grid_img.shape

        scan = np.full(len(self.angles_rad), self.max_range_m)
        x0_m, y0_m = position_m

        for i, angle in enumerate(self.angles_rad):
            for r in np.linspace(0, self.max_range_m, 500):
            
                x_m = x0_m + r * np.cos(angle)
                y_m = y0_m + r * np.sin(angle)
                ix = int(x_m / self.pixel_size_m)
                iy = int(y_m / self.pixel_size_m)

                if 0 <= ix < width_px and 0 <= iy < height_px:
                    if self.grid_img[iy, ix] == 1:
                        scan[i] = r
                        break
                else:
                    scan[i] = r
                    break
        return scan
    

    def _generate_grid_map(self, map_height_m, map_width_m, sampling_step):
        '''
        Генерация 
        '''
        print('Starting generation of grid map...')
        grid_map = {}

        height_px, width_px =  self.grid_img.shape
        y_coords_m = np.arange(0, map_height_m, sampling_step)
        x_coords_m = np.arange(0, map_width_m, sampling_step)
        
        print('x_coords_m size:', x_coords_m.size, '\ny_coords_m size:', y_coords_m.size)

        for y_m in y_coords_m:
            for x_m in x_coords_m:
                ix = int(x_m / self.pixel_size_m)
                iy = int(y_m / self.pixel_size_m)

                if 0 <= ix < width_px and 0 <= iy < height_px:
                    if self.grid_img[iy, ix] == 0:
                        scan = self.simulate_lidar_scan((x_m, y_m))
                        grid_map[(x_m, y_m)] = scan

        return grid_map
    

map = Map('map0.pgm', 0.005)

map.visualize()
cv2.waitKey()