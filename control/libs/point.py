import numpy as np


class Point(np.ndarray):
    def __new__(cls, x=0, y=0):
        obj = np.asarray((x, y), dtype=np.float64).view(cls)
        return obj

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def angle(self):
        return np.arctan2(self[1], self[0])

    def translate(self, dx=0, dy=0):
        return self + np.array((dx, dy), dtype=np.float64)

    def rotate(self, angle):
        rotation_matrix = np.array(((np.cos(angle), -np.sin(angle)), (np.sin(angle),  np.cos(angle))))
        return rotation_matrix.dot(self).view(Point)

    def distance(self, other=np.array([0, 0])):
        return np.linalg.norm(self - other)