"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        # 构成四周的边界 ox, oy, w, h
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        # 方形障碍 ox, oy, w, h
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        # 圆形障碍 ox, oy, r
        obs_cir = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir


class Env2:
    def __init__(self):
        self.x_range = (0, 500)
        self.y_range = (0, 300)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        # 构成四周的边界 ox, oy, w, h
        obs_boundary = [
            [0, 0, 1, 300],
            [0, 300, 500, 1],
            [1, 0, 500, 1],
            [500, 1, 1, 300]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        # 方形障碍 ox, oy, w, h
        obs_rectangle = [
            [90, 0, 2, 50],
            [170, 0, 2, 50],
            [250, 0, 2, 50],
            [330, 0, 2, 50],
            [410, 0, 2, 50],
            [490, 0, 2, 50],

            [10, 250, 2, 50],
            [90, 250, 2, 50],
            [170, 250, 2, 50],
            [250, 250, 2, 50],
            [330, 250, 2, 50],
            [410, 250, 2, 50],
            
            [50, 100, 2, 100],
            [130, 100, 2, 100],
            [210, 100, 2, 100],
            [290, 100, 2, 100],
            [370, 100, 2, 100],
            [450, 100, 2, 100],
            [50, 150, 400, 2]

        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        # 圆形障碍 ox, oy, r
        obs_cir = [

        ]

        return obs_cir
