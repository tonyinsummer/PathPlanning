"""
Breadth-first Searching_2D (BFS)
@author: huiming zhou
"""

import os
import sys
from collections import deque

from numpy.core.records import record

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env
from Search_2D.Astar import AStar
import math
import heapq
import time

class BFS(AStar):
    """BFS add the new visited node in the end of the openset
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
                       (0, self.s_start))

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

                    # bfs, add new node to the end of the openset
                    prior = self.OPEN[-1][0]+1 if len(self.OPEN)>0 else 0
                    heapq.heappush(self.OPEN, (prior, s_n))

        return self.extract_path(self.PARENT), self.CLOSED


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    bfs = BFS(s_start, s_goal, 'None')
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = bfs.searching()
    plot.animation(path, visited, "Breadth-first Searching (BFS)")



def record_time():
    method_name = 'Breadth First'
    time_start = time.time()

    # s_start = (5, 5)
    # s_goal = (45, 25)
    s_start = (10, 10)
    s_goal = (490, 290)
    bfs = BFS(s_start, s_goal, 'None')
    path, visited = bfs.searching()

    time_end=time.time()
    time_delta = time_end-time_start
    path_len = path_length(path)
    print(method_name, time_delta, path_len)
    # plot = plotting.Plotting_my(s_start, s_goal)
    # plot.animation(path, visited, "Breadth-first Searching (BFS)")
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