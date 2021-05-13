"""
Fast Marching Trees (FMT*)
@author: huiming zhou
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

# from Sampling_based_Planning.rrt_2D import env, plotting, utils
from rrt_2D import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.cost = np.inf


class FMT:
    def __init__(self, x_start, x_goal, search_radius):
        self.x_init = Node(x_start)
        self.x_goal = Node(x_goal)
        self.search_radius = search_radius

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils_my()

        self.fig, self.ax = plt.subplots()
        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.V = set()
        self.V_unvisited = set()
        self.V_open = set()
        self.V_closed = set()
        self.sample_numbers = 1000
        self.path=[]

    def Init(self):
        samples = self.SampleFree()

        self.x_init.cost = 0.0
        self.V.add(self.x_init)
        self.V.update(samples)
        self.V_unvisited.update(samples)
        self.V_unvisited.add(self.x_goal)
        self.V_open.add(self.x_init)

    def Planning(self):
        self.Init()
        z = self.x_init
        n = self.sample_numbers
        rn = self.search_radius * math.sqrt((math.log(n) / n))
        Visited = []

        while z is not self.x_goal:
            V_open_new = set()
            X_near = self.Near(self.V_unvisited, z, rn)
            Visited.append(z)

            for x in X_near:
                Y_near = self.Near(self.V_open, x, rn)
                cost_list = {y: y.cost + self.Cost(y, x) for y in Y_near}
                y_min = min(cost_list, key=cost_list.get)

                if not self.utils.is_collision(y_min, x):
                    x.parent = y_min
                    V_open_new.add(x)
                    self.V_unvisited.remove(x)
                    x.cost = y_min.cost + self.Cost(y_min, x)

            self.V_open.update(V_open_new)
            self.V_open.remove(z)
            self.V_closed.add(z)

            if not self.V_open:
                print("open set empty!")
                break

            cost_open = {y: y.cost for y in self.V_open}
            z = min(cost_open, key=cost_open.get)

        # node_end = self.ChooseGoalPoint()

        path = self.ExtractPath_my()
        return path, Visited
        # path_x, path_y = self.ExtractPath()
        # self.animation(path_x, path_y, Visited[1: len(Visited)])

    def ChooseGoalPoint(self):
        Near = self.Near(self.V, self.x_goal, 2.0)
        cost = {y: y.cost + self.Cost(y, self.x_goal) for y in Near}

        return min(cost, key=cost.get)

    def ExtractPath(self):
        path_x, path_y = [], []
        node = self.x_goal

        while node.parent:
            path_x.append(node.x)
            path_y.append(node.y)
            node = node.parent

        path_x.append(self.x_init.x)
        path_y.append(self.x_init.y)

        return path_x, path_y

########
    def ExtractPath_my(self):
        path_ = []
        node = self.x_goal

        while node.parent:
            path_.append([node.x, node.y])
            node = node.parent

        path_.append((self.x_init.x, self.x_init.y))

        return path_

    def Cost(self, x_start, x_end):
        if self.utils.is_collision(x_start, x_end):
            return np.inf
        else:
            return self.calc_dist(x_start, x_end)

    @staticmethod
    def calc_dist(x_start, x_end):
        return math.hypot(x_start.x - x_end.x, x_start.y - x_end.y)

    @staticmethod
    def Near(nodelist, z, rn):
        return {nd for nd in nodelist
                if 0 < (nd.x - z.x) ** 2 + (nd.y - z.y) ** 2 <= rn ** 2}

    def SampleFree(self):
        n = self.sample_numbers
        delta = self.utils.delta
        Sample = set()

        ind = 0
        while ind < n:
            node = Node((random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
            if self.utils.is_inside_obs(node):
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample

    def animation(self, path_x, path_y, visited):
        self.plot_grid("Fast Marching Trees (FMT*)")

        for node in self.V:
            plt.plot(node.x, node.y, marker='.', color='lightgrey', markersize=3)

        count = 0
        for node in visited:
            count += 1
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if count % 10 == 0:
                plt.pause(0.001)

        plt.plot(path_x, path_y, linewidth=2, color='red')
        plt.pause(0.01)
        plt.show()


#***********************************************
    def my_animation(self, path_x, path_y, visited):
        self.plot_grid("Fast Marching Trees (FMT*)")

        # for node in self.V:
        #     plt.plot(node.x, node.y, marker='.', color='lightgrey', markersize=3)

        # count = 0
        # for node in visited:
        #     count += 1
        #     plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')
        #     plt.gcf().canvas.mpl_connect(
        #         'key_release_event',
        #         lambda event: [exit(0) if event.key == 'escape' else None])
        #     if count % 10 == 0:
        #         plt.pause(0.001)

        plt.plot(path_x, path_y, linewidth=2, color='red')
        #plt.pause(0.01)
        plt.show()
#************************************************

    def plot_grid(self, name):

        for (ox, oy, w, h) in self.obs_boundary:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.x_init.x, self.x_init.y, "bs", linewidth=3)
        plt.plot(self.x_goal.x, self.x_goal.y, "rs", linewidth=3)

        plt.title(name)
        plt.axis("equal")









#*****************************

class FMT_my:
    def __init__(self, x_start, x_goal, search_radius):
        self.x_init = Node(x_start)
        self.x_goal = Node(x_goal)
        self.search_radius = search_radius

        self.env = env.Env2()
        self.plotting = plotting.Plotting_my(x_start, x_goal)
        self.utils = utils.Utils_my()

        self.fig, self.ax = plt.subplots()
        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.V = set()
        self.V_unvisited = set()
        self.V_open = set()
        self.V_closed = set()
        self.sample_numbers = 1000
        self.path=[]

    def Init(self):
        samples = self.SampleFree()

        self.x_init.cost = 0.0
        self.V.add(self.x_init)
        self.V.update(samples)
        self.V_unvisited.update(samples)
        self.V_unvisited.add(self.x_goal)
        self.V_open.add(self.x_init)

    def Planning(self):
        self.Init()
        z = self.x_init
        n = self.sample_numbers
        rn = self.search_radius * math.sqrt((math.log(n) / n))
        Visited = []

        while z is not self.x_goal:
            V_open_new = set()
            X_near = self.Near(self.V_unvisited, z, rn)
            Visited.append(z)

            for x in X_near:
                Y_near = self.Near(self.V_open, x, rn)
                cost_list = {y: y.cost + self.Cost(y, x) for y in Y_near}
                y_min = min(cost_list, key=cost_list.get)

                if not self.utils.is_collision(y_min, x):
                    x.parent = y_min
                    V_open_new.add(x)
                    self.V_unvisited.remove(x)
                    x.cost = y_min.cost + self.Cost(y_min, x)

            self.V_open.update(V_open_new)
            self.V_open.remove(z)
            self.V_closed.add(z)

            if not self.V_open:
                print("open set empty!")
                break

            cost_open = {y: y.cost for y in self.V_open}
            z = min(cost_open, key=cost_open.get)

        # node_end = self.ChooseGoalPoint()

        path = self.ExtractPath_my()
        return path, Visited
        # path_x, path_y = self.ExtractPath()
        # self.animation(path_x, path_y, Visited[1: len(Visited)])

    def ChooseGoalPoint(self):
        Near = self.Near(self.V, self.x_goal, 2.0)
        cost = {y: y.cost + self.Cost(y, self.x_goal) for y in Near}

        return min(cost, key=cost.get)

    def ExtractPath(self):
        path_x, path_y = [], []
        node = self.x_goal

        while node.parent:
            path_x.append(node.x)
            path_y.append(node.y)
            node = node.parent

        path_x.append(self.x_init.x)
        path_y.append(self.x_init.y)

        return path_x, path_y

########
    def ExtractPath_my(self):
        path_ = []
        node = self.x_goal

        while node.parent:
            path_.append([node.x, node.y])
            node = node.parent

        path_.append((self.x_init.x, self.x_init.y))

        return path_

    def Cost(self, x_start, x_end):
        if self.utils.is_collision(x_start, x_end):
            return np.inf
        else:
            return self.calc_dist(x_start, x_end)

    @staticmethod
    def calc_dist(x_start, x_end):
        return math.hypot(x_start.x - x_end.x, x_start.y - x_end.y)

    @staticmethod
    def Near(nodelist, z, rn):
        return {nd for nd in nodelist
                if 0 < (nd.x - z.x) ** 2 + (nd.y - z.y) ** 2 <= rn ** 2}

    def SampleFree(self):
        n = self.sample_numbers
        delta = self.utils.delta
        Sample = set()

        ind = 0
        while ind < n:
            node = Node((random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
            if self.utils.is_inside_obs(node):
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample

    def animation(self, path_x, path_y, visited):
        self.plot_grid("Fast Marching Trees (FMT*)")

        for node in self.V:
            plt.plot(node.x, node.y, marker='.', color='lightgrey', markersize=3)

        count = 0
        for node in visited:
            count += 1
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if count % 10 == 0:
                plt.pause(0.001)

        plt.plot(path_x, path_y, linewidth=2, color='red')
        plt.pause(0.01)
        plt.show()


#***********************************************
    def my_animation(self, path_x, path_y, visited):
        self.plot_grid("Fast Marching Trees (FMT*)")

        # for node in self.V:
        #     plt.plot(node.x, node.y, marker='.', color='lightgrey', markersize=3)

        # count = 0
        # for node in visited:
        #     count += 1
        #     plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')
        #     plt.gcf().canvas.mpl_connect(
        #         'key_release_event',
        #         lambda event: [exit(0) if event.key == 'escape' else None])
        #     if count % 10 == 0:
        #         plt.pause(0.001)

        plt.plot(path_x, path_y, linewidth=2, color='red')
        #plt.pause(0.01)
        plt.show()
#************************************************

    def plot_grid(self, name):

        for (ox, oy, w, h) in self.obs_boundary:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.x_init.x, self.x_init.y, "bs", linewidth=3)
        plt.plot(self.x_goal.x, self.x_goal.y, "rs", linewidth=3)

        plt.title(name)
        plt.axis("equal")


def main():
    x_start = (18, 8)  # Starting node
    x_goal = (37, 18)  # Goal node

    fmt = FMT(x_start, x_goal, 40)
    fmt.Planning()


def record_time():
    import time
    method_name = 'Fast Matching Trees'
    time_start=time.time()


    x_start = (10, 10)  # Starting node
    x_goal = (490, 290)  # Goal node
    fmt = FMT(x_start, x_goal, 400)
    path, visited = fmt.Planning()


    time_end=time.time()
    time_delta = time_end-time_start
    # print(np.array(path))
    path=np.array(path)
    path_len = path_length(path.T)
    print(method_name, time_delta, path_len)
    # fmt.animation(path[:,0], path[:,1], visited[1: len(visited)])
    # plt.plot(path)
    # plt.show()

    return [method_name, time_delta, path_len]





















def main():
    x_start = (18, 8)  # Starting node
    x_goal = (37, 18)  # Goal node

    fmt = FMT(x_start, x_goal, 40)
    path, visited = fmt.Planning()
    path=np.array(path)
    fmt.animation(path[:,0], path[:,1], visited[1: len(visited)])


def record_time():
    import time
    method_name = 'Fast Matching Trees'
    time_start=time.time()


    x_start = (10, 10)  # Starting node
    x_goal = (490, 290)  # Goal node
    fmt = FMT(x_start, x_goal, 400)
    path, visited = fmt.Planning()


    time_end=time.time()
    time_delta = time_end-time_start
    # print(np.array(path))
    path=np.array(path)
    path_len = path_length(path.T)
    print(method_name, time_delta, path_len)
    # fmt.animation(path[:,0], path[:,1], visited[1: len(visited)])
    # plt.plot(path)
    # plt.show()

    return [method_name, time_delta, path_len]


def path_length(path):
    path_=path
    length = 0
    path_ = np.array(path_)
    for i in range(path_.shape[0]-1):
        d = path_[i+1,:]-path_[i,:]
        length += np.sqrt(np.sum(d**2))
    return length


if __name__ == '__main__':
    # main()
    record_time()
