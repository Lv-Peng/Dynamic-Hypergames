import DESops as d
import numpy as np
import random
import networkx as nx
import math
import time



def read_data():
    G = d.DFA()
    E = d.Event("e")
    S = d.Event("s")
    W = d.Event("w")
    N = d.Event("n")
    N_bar = d.Event("nuc")
    B = d.Event("b")
    
    G.Euc = set([N_bar, B])
    data_set = [(0, 1, S),
       (1, 10, E),
       (1, 14, S),
       (2, 3, S),
       (3, 11, E),
       (3, 15, S),
       (4, 5, S),
       (5, 12, E),
       (5, 16, S),
       (6, 13, E),
       (6, 17, S),
       (7, 6, S),
       (8, 9, S),
       (9, 12, E),
       (9, 17, S),
       (10, 1, W),
       (10, 18, E),
       (10, 21, S),
       (11, 3, W),
       (11, 19, E),
       (11, 22, S),
       (12, 6, W),
       (12, 20, E),
       (12, 24, S),
       (13, 6, W),
       (13, 20, E),
       (13, 24, S),
       (14, 1, N),
       (14, 21, E),
       (15, 3, N),
       (15, 22, E),
       (16, 5, N),
       (16, 23, E),
       (17, 9, N),
       (17, 25, E),
       (18, 26, S),
       (19, 27, S),
       (20, 29, S),
       (21, 14, W),
       (21, 10, N_bar),
       (21, 26, E),
       (22, 15, W),
       (22, 11, N_bar),
       (22, 27, E),
       (23, 16, W),
       (23, 12, N_bar),
       (23, 28, E),
       (24, 17, W),
       (24, 13, N_bar),
       (24, 29, E),
       (25, 17, W),
       (25, 12, N_bar),
       (25, 30, E),
       (26, 21, W),
       (26, 31, E),
       (26, 36, S),
       (27, 22, W),
       (27, 32, E),
       (27, 36, S),
       (28, 23, W),
       (28, 33, E),
       (28, 37, S),
       (29, 24, W),
       (29, 34, E),
       (29, 38, S),
       (30, 25, W),
       (30, 35, E),
       (30, 39, S),
       (31, 40, N_bar),
       (31, 26, W),
       (31, 0, E),
       (32, 41, N_bar),
       (32, 27, W),
       (32, 2, E),
       (33, 42, N_bar),
       (33, 28, W),
       (33, 4, E),
       (34, 43, N_bar),
       (34, 29, W),
       (34, 7, E),
       (35, 44, N_bar),
       (35, 30, W),
       (35, 8, E),
       (36, 45, W),
       (36, 48, E),
       (36, 27, N),
       (37, 45, W),
       (37, 49, E),
       (37, 28, N),
       (38, 46, W),
       (38, 50, E),
       (38, 29, N),
       (39, 47, W),
       (39, 51, E),
       (39, 30, N),
       (40, 40, B),
       (40, 31, S),
       (41, 41, B),
       (41, 32, S),
       (42, 42, B),
       (42, 33, S),
       (43, 43, B),
       (43, 34, S),
       (44, 44, B),
       (44, 35, S),
       (45, 37, E),
       (46, 38, E),
       (47, 39, E),
       (48, 32, N),
       (49, 33, N),
       (50, 34, N),
       (51, 35, N)]
    
    marks = [False for _ in range(52)]
    marks[12] = True
    G.add_vertices(52, marked=marks)
    for (u, v, sigma) in data_set:
        G.add_edge(u, v, sigma)
    return G

# t1 = time.time()
# G = read_data()
# event = set()
# event_uc = set()
# for i in G.events:
#     event.add(str(i))
# for i in G.Euc:
#     event_uc.add(str(i))
    

# t2 = time.time()
# print(t2-t1)
# num_q = {(1, 2): [3]}
# wt = list(num_q[(1,2)])[0]
# g_des = nx.DiGraph()
# g_des.add_nodes_from([1, (0, 1)])
# g_des.add_edge(1, (0, 1))
# g_des.add_edge((0, 1), 1, weight=wt)
# for (u, v, wt) in g_des.edges.data('weight'):
#     print(u, v, wt)
# G = des.random_automata.generate(20, 5, num_marked=2, num_uc=2, max_trans_per_state=3)
# print(G)

