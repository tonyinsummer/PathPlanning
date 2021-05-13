"""
Env 2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = 51  # size of background
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()

        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        for i in range(10, 21):
            obs.add((i, 15))
        for i in range(15):
            obs.add((20, i))

        for i in range(15, 30):
            obs.add((30, i))
        for i in range(16):
            obs.add((40, i))

        return obs


class Env2:
    def __init__(self):
        self.x_range = 501  # size of background
        self.y_range = 301
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()
    #边界
        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

    # 障碍物
        for i in range(50):
            obs.add((90, i))
        for i in range(50):
            obs.add((170, i))
        for i in range(50):
            obs.add((250, i))
        for i in range(50):
            obs.add((330, i))
        for i in range(50):
            obs.add((410, i))
        for i in range(50):
            obs.add((490, i))

        for i in range(250,300):
            obs.add((10, i))
        for i in range(250,300):
            obs.add((90, i))
        for i in range(250,300):
            obs.add((170, i))
        for i in range(250,300):
            obs.add((250, i))
        for i in range(250,300):
            obs.add((330, i))
        for i in range(250,300):
            obs.add((410, i))

        for i in range(100,200):
            obs.add((50, i))
        for i in range(100,200):
            obs.add((130, i))
        for i in range(100,200):
            obs.add((210, i))
        for i in range(100,200):
            obs.add((290, i))
        for i in range(100,200):
            obs.add((370, i))
        for i in range(100,200):
            obs.add((450, i))

        for i in range(50,450):
            obs.add((i, 150))

        return obs