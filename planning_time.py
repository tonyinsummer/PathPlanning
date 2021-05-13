from Search_based_Planning.Search_2D import Astar, Dijkstra, Bidirectional_a_star, ARAstar, RTAAStar, bfs, dfs, LRTAstar
from Sampling_based_Planning.rrt_2D import rrt, fast_marching_trees, rrt_connect, extended_rrt
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    list_all = []
    list_all.append(Astar.record_time())
    list_all.append(Dijkstra.record_time())
    list_all.append(Bidirectional_a_star.record_time())
    list_all.append(ARAstar.record_time())
    list_all.append(RTAAStar.record_time())
    list_all.append(bfs.record_time())
    # list_all.append(dfs.record_time())
    # list_all.append(LRTAstar.record_time())

    for i in range(10):
        list_all.append(rrt.record_time())
        # list_all.append(fast_marching_trees.record_time())
        list_all.append(rrt_connect.record_time())
        list_all.append(extended_rrt.record_time())
    # print(list_all)

    df = pd.DataFrame(list_all,columns=['Method Name','Time Used', 'Path Length'],dtype=float)

    result_sorted = df.groupby('Method Name').mean().sort_values(by='Time Used', ascending=True)

    result_sorted.to_csv('global_planning_time4.csv')

    # Plotting
    plt.figure()
    ax = result_sorted.plot(secondary_y='Path Length',figsize=(15,5))
    ax.set_ylabel('Time Used')
    ax.right_ax.set_ylabel('Path Length')
    ax.set_title('Different Planning Method', fontsize=15)
    plt.grid(axis='both')
    plt.show()
