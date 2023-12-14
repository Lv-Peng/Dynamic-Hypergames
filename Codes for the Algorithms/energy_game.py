from src import MeanPayOffGame, judge, judge_w, reduction
import numpy as np


def energy_game(mpg):
    M = 0
    L = []
    f = [0 for _ in range(mpg.v)]
    count = [0 for _ in range(mpg.v)]
    for node in mpg.G.nodes:
        w_list = []
        for next_node, weight in mpg.G.adj[node].items():
            w_list.append(weight['weight'])
        w_list = np.array(w_list)
        if node in mpg.nodes:
            if w_list.max() < 0:
                L.append(node)
        else:
            if w_list.min() < 0:
                L.append(node)
        w_list_op = [-1*i for i in w_list] + [0]
        M += np.array(w_list_op).max()

    for node in mpg.nodes:
        if node not in L:
            for next_node, weight in mpg.G.adj[node].items():
                if judge(reduction(f[next_node], weight['weight'], M), f[node], M):
                    count[node] += 1

    while len(L) != 0:
        node = L[0]
        L.remove(node)
        old = f[node]
        num_list = []

        for next_node, weight in mpg.G.adj[node].items():
            num_list.append(reduction(f[next_node], weight['weight'], M))

        num_list = np.array(num_list)
        if node in mpg.nodes:
            count[node] = 0
            f[node] = num_list.min()
            for next_node, weight in mpg.G.adj[node].items():
                if judge(reduction(f[next_node], weight['weight'], M), f[node], M):
                    count[node] += 1
        else:
            f[node] = num_list.max()

        for pre_node in mpg.G.predecessors(node):
            weight = mpg.G[pre_node][node]['weight']
            if judge_w(f[pre_node], reduction(f[node], weight, M), M):
                if pre_node in mpg.nodes:
                    if judge(reduction(old, weight, M), f[pre_node], M):
                        count[pre_node] -= 1
                    if count[pre_node] <= 0:
                        if pre_node not in L:
                            L.append(pre_node)
                else:
                    if pre_node not in L:
                        L.append(pre_node)

    return f, M


def energy_strategy(mpg, f, M):
    strategies = []
    for node in mpg.nodes:
        temp = 2*M + 1
        for next_node, weight in mpg.G.adj[node].items():
            value = reduction(f[next_node], weight['weight'], M)
            if value <= temp:
                temp = value
                strategy = next_node
        strategies.append((node, strategy))

    return strategies
