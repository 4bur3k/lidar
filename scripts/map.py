import matplotlib.pyplot as plt
import numpy as np

class Point:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
    
    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_xy(self, x, y):
        self.x = x
        self.y = y

    def get_point(self):
        return self.x, self.y

class Line:
    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2

    def set_point1(self, point):
        self.point1 = point

    def set_point2(self, point):
        self.point2 = point

    def set_points(self, point1, point2):
        self.point1 = point1
        self.point2 = point2

    def get_line(self):
        return self.point1.get_point(), self.point2.get_point()

class Map:
    '''
    Args:
    points: np.ndarray[Point]: all the availeble points(states) on the map
    walls: np.ndarray[Line]: all the walls on the map
    doors: np.ndarray[Line]: all the entrances/exits on the map
    '''
    def __init__(self, points: np.ndarray[Point], walls: np.ndarray[Line], doors: np.ndarray[Line]):
        points = points
        walls = walls
        doors = doors
