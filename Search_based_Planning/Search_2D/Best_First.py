"""
Best-First Searching
@author: huiming zhou
"""

import os
import sys
import math
import heapq
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env
from Search_2D.Astar import AStar


class BestFirst(AStar):
    """BestFirst set the heuristics as the priority 
    """
    def searching(self):
        """
        Breadth-first Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.heuristic(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s

                    # best first set the heuristics as the priority 
                    heapq.heappush(self.OPEN, (self.heuristic(s_n), s_n))

        return self.extract_path(self.PARENT), self.CLOSED


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    BF = BestFirst(s_start, s_goal, 'euclidean')
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = BF.searching()
    plot.animation(path, visited, "Best-first Searching")  # animation


def record_time():
    method_name = 'Best First'
    time_start = time.time()

    # s_start = (5, 5)
    # s_goal = (45, 25)
    s_start = (10, 10)
    s_goal = (490, 290)
    BF = BestFirst(s_start, s_goal, 'euclidean')
    path, visited = BF.searching()

    time_end=time.time()
    time_delta = time_end-time_start
    path_len = path_length(path)
    print(method_name, time_delta, path_len)

    # plot = plotting.Plotting_my(s_start, s_goal)
    # plot.animation(path, visited, "Best-first Searching") 

    return [method_name, time_delta, path_len]


def path_length(path):
    import numpy as np
    path_=path
    length = 0
    path_ = np.array(path_)
    for i in range(path_.shape[0]-1):
        d = path_[i+1,:]-path_[i,:]
        length += math.sqrt(np.sum(d**2))
    return length


if __name__ == '__main__':
    # main()
    record_time()