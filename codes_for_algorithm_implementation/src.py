import networkx as nx
import numpy as np
import math

class MeanPayOffGame:
    def __init__(self, node_p, edge_list, w, v):
        self.v = v
        self.w = w
        self.e = len(edge_list)
        self.nodes = node_p

        self.G = nx.DiGraph()
        # not_p is a list of ver for player0
        # for example: n=10, [1, 3, 7, 9], then [0, 2, 4, 5, 6, 8] is for player1.
        self.G.add_nodes_from([(i, {'player': int(i not in node_p)}) for i in range(v)])
        self.G.add_weighted_edges_from(edge_list)
        
        
def judge(x, y, m):
    if y == 2*m:
        return True
    if (x <= y) and (y <= m):
        return True
    return False


# To slove the situation without equal, only "less than"
def judge_w(x, y, m):
    if (y == 2*m) and (x != 2*m):
        return True
    if (x < y) and (y <= m):
        return True
    return False


def reduction(x, y, m):
    if (x != 2*m) and ((x - y) <= m):
        return max(0, x - y)
    else:
        return 2*m
    
    

# used for ratio.py



def mpg_decision(mpg, point, threshold=0):
    k = 4 * mpg.v * mpg.v * mpg.w
    flag = 1
    value = np.zeros((k + 1, mpg.v))
    for i in range(1, k + 1):
        for p in range(mpg.v):
            edge_dict = mpg.G.adj[p]
            value_list = []
            for p_next, weight in edge_dict.items():
                value_list.append(weight['weight'] + value[i - 1][p_next])

            value_list = np.array(value_list)
            if p in mpg.nodes:
                value[i][p] = value_list.min()
            else:
                value[i][p] = value_list.max()

    inf = value[k][point] / k - 1 / (2 * mpg.v * (mpg.v - 1))
    sup = value[k][point] / k + 1 / (2 * mpg.v * (mpg.v - 1))
    if inf > threshold:
        flag = 2
    elif sup < threshold:
        flag = 0

    return flag


def game_value_old(k, v, node_p, edge_list, w, point):
    mpg_new = MeanPayOffGame(node_p, edge_list, w, v)
    value = np.zeros((k, v))
    for i in range(1, k):
        for p in range(v):
            edge_dict = mpg_new.G.adj[p]
            value_list = []
            for p_next, weight in edge_dict.items():
                value_list.append(mpg_new.G[p][p_next]['weight'] + value[i - 1][p_next])

            value_list = np.array(value_list)
            if p in node_p:
                value[i][p] = value_list.min()
            else:
                value[i][p] = value_list.max()

    inf = value[k - 1][point] / k - 1 / (2 * v * (v - 1))
    sup = value[k - 1][point] / k + 1 / (2 * v * (v - 1))
    for b in range(1, v + 1):
        inf_b = b * inf
        sup_b = b * sup
        if math.ceil(inf_b) == math.floor(sup_b):
            a = math.ceil(inf_b)
            if (a / b > inf) and (a / b < sup):
                break

    return a / b
