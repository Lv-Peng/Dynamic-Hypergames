import DESops as des
import numpy as np
import networkx as nx
import random
from energy_game import energy_game, energy_strategy
from src import MeanPayOffGame
# from ratio import ratio_game
import time
import math
from test import read_data
## Generate a new discrete event system.
num_states_origin = 20
num_events = 6
num_marked = int(num_states_origin/10)
num_uncontroable_event = 2

num_ce_max = 1
num_cd_max = 3
epochs = 10
singal_weight = 1

time_count = 0
node_count = 0
edge_count = 0
time_r_count1 = 0
time_r_count2 = 0
t_count = 0
max_good_aver = 0
print('------------------')
print('------------------')
print('------------------')
# print("The expriment begins with the following parameters:")
# print("num_states = {}, num_events = {}, num_marked = {}, num_uncontroable_event = {}".format(num_states_origin, num_events, num_marked, num_uncontroable_event))
# print("ce is randomly selected in [1, {}], and cd is randomly selected in [0, {}]".format(num_ce_max, num_cd_max))
for epoch in range(epochs):
    ce = {}
    cd = {}
    
    ## The loop begins.  
    # G = des.random_automata.generate(num_states_origin, num_events, num_marked=num_marked, num_uc=num_uncontroable_event, max_trans_per_state=math.ceil(num_events/4), det=False)
    # flag = True
    
    ## Things used for test.
    G = read_data()
    flag = False
    num_states_origin = 52
    num_events = 6
    num_marked = 1
    num_states = num_states_origin
    num_uncontroable_event = 2
    cd['w'] = 1
    cd['e'] = 1
    cd['n'] = 2
    cd['s'] = 3
    ce['w'] = 1
    ce['s'] = 1
    ce['n'] = 1
    ce['e'] = 1
    ce['b'] = 1
    ce['nuc'] = 1 
    
    num_states = num_states_origin
    event = set()
    event_uc = set()
    for i in G.events:
        event.add(str(i))
    for i in G.Euc:
        event_uc.add(str(i))
    event_c = event - event_uc
    
    if flag:
        for i in event:
            ce[i] = random.randint(1, num_ce_max)
        for i in event_c:
            cd[i] = random.randint(0, num_cd_max)
    

    marked = set([i for i,e in enumerate(G.vs['marked']) if e])
    Q = set([i for i in range(num_states)])
    description = {}
    ### To deal with the multiple edges.
    pre_q = {}
    num_q = {}
    #### pre_q: { v: {u1, u2, ...} such that u1, u2... can reach v}
    #### num_q: { (q_bar, q): {sigma1, sigma2 ...} such that q_bar reach q by event sigma1, sigma2...}
    for i, v_out in enumerate(G.vs['out']):
        for pair in v_out:
            if pair.target not in pre_q:
                pre_q[pair.target] = set([i])
            else:
                pre_q[pair.target].add(i)
                
            if (i, pair.target) not in num_q:
                num_q[(i, pair.target)] = set([str(pair.label)])
            else:
                num_q[(i, pair.target)].add(str(pair.label))

    act_q = {}          
    #### act_q: { v: {sigma1, sigma2...} such that len(num_q[(u, v)]) > 1 and sigma1, sigma2 ... in num_q[(u, v)]} 
    for ((u, v), label) in num_q.items():
        if len(label) > 1:
            for sigma in label:
                if v not in act_q:
                    act_q[v] = set([sigma])
                else:
                    act_q[v].add(sigma)
                    
    ### Build a new DES structure.
    Q_bar = set()
    g_des = nx.DiGraph()
    for node in Q:
        if node not in act_q:
            Q_bar.add(node)
        else:
            for sigma in act_q[node]:
                Q_bar.add((node, sigma))
    g_des.add_nodes_from(Q_bar)

    for v in Q_bar:
        if (v in Q) and (v in pre_q):
            for u in pre_q[v]:
                wt = list(num_q[(u, v)])[0]
                if u in (Q & Q_bar):               
                    g_des.add_edge(u, v, weight=wt)
                else:
                    for sigma in act_q[u]:
                        g_des.add_edge((u, sigma), v, weight=wt)
        if v not in Q:
            sigma = v[1]
            v = v[0]
            for u in pre_q[v]:
                if len(num_q[(u, v)]) == 1:
                    wt = list(num_q[(u,v)])[0]
                    sigma_bar = list(act_q[v])[0]
                    if u in (Q & Q_bar):                
                        g_des.add_edge(u, (v, sigma_bar), weight=wt)
                    else:
                        for sigma_bar_bar in act_q[u]:
                            g_des.add_edge((u, sigma_bar_bar), (v, sigma_bar), weight=wt)
                else:
                    if u in (Q & Q_bar):
                        g_des.add_edge(u, (v, sigma), weight=sigma)
                    else:
                        for sigma_bar in act_q[u]:
                            g_des.add_edge((u, sigma_bar), (v, sigma), weight=sigma)
                    
    if 0 in act_q:
        g_des.add_node((0, 'intial')) 
        description[(0, 'intial')] = 0 
        det_q = [str(pair.label) for pair in G.vs['out'][0]]
        for sigma_bar in act_q[0]:
            description[(0, sigma_bar)] = num_states
            num_states += 1
            for v, wt in g_des.adj[(0, sigma_bar)].items():
                wt = wt['weight']
                for sigma in det_q:             
                    if wt == sigma:
                        g_des.add_edge((0, 'intial'), v, weight=wt) 
    else:
        description[0] = 0

    for node in Q:
        if node != 0:
            if node in Q_bar:
                description[node] = node
            else:
                for count, sigma in enumerate(act_q[node]):
                    if count == 0:
                        description[(node, sigma)] = node
                    else:
                        description[(node, sigma)] = num_states
                        num_states += 1
                        
    for node in Q:
        if (node not in Q_bar) and (node in marked):
            if node != 0:
                marked.remove(node)
            for sigma in act_q[node]:
                marked.add(description[(node, sigma)])     
    
    # Next, transform the DES into a SC game.
    dic = {}
    for (u, v, wt) in g_des.edges.data('weight'):
        u = description[u]
        v = description[v]
        dic[(u, v)] = [wt]

    ## Define all gamma.
    ### another example used for test.
    # dic = {(0,1): ['a'], (0,2): ['e'], (1,0): ['d'], (1,2):['e'], (1,3):['a'],
    #          (2,0): ['a'], (2,1): ['e'], (3,3): ['e'], (3,4):['d'], (4,4):['d']}
    # event_c = {'a'}
    # event = {'a', 'd', 'e'}
    # event_uc = {'d', 'e'}
    # marked = [0, 1, 4]
    # num_states = 5
    # num_events = 3   
    # num_marked = 3  
    # num_ce_max = 2
    # num_cd_max = 1
    # ce = {'a':1, 'd':2, 'e':2}
    # cd = {'a':1}
    # singal_weight = 1

    ### Now begins.
    gamma = {}
    event_c_list = list(event_c)
    description1 = {}

    ## Build the map.
    nodes = [i for i in range(num_states)]
    num = len(nodes)

    g = nx.DiGraph()
    num_states = num_states + 2
    vd_0, vd_1 = num_states - 2, num_states - 1
    description1[num_states - 2] = 'vd_0'
    description1[num_states - 1] = 'vd_1'
    nodes.append(vd_0)

    g.add_nodes_from(range(num_states))
    g.add_edge(vd_0, vd_1, weight=singal_weight)
    g.add_edge(vd_1, vd_0, weight=singal_weight)
    w = 1
    w_bar = num_events * (num_cd_max + num_ce_max) + 1
    for i in range(num):
        det_q = set()
        for ((u, v), voca) in dic.items():
            if u==i:
                det_q.add(voca[0])
        det_q_c = det_q & event_c
        det_q_c_bu = event - det_q_c
        det_q_c = list(det_q_c)
        gamma[i] = []
        num_det_q = len(det_q_c)
        for j in range(2 ** num_det_q): 
            temp = set()
            for k in range(num_det_q): 
                if(j>>k)%2:
                    temp.add(det_q_c[k])
            gamma[i].append(temp)
        gamma[i] = [gamma[i][_].union(det_q_c_bu) for _ in range(len(gamma[i]))]
    
        for gamma_set in gamma[i]:
            sigma_set = (det_q & event_c) - gamma_set
            weight = sum([cd[s] for s in sigma_set]) 
            if weight > w:
                w = weight
            if weight < w_bar:
                w_bar = weight
            g.add_node(num_states)
            g.add_edge(i, num_states, weight=weight)
            description1[num_states] = (i, gamma_set)
                       
            if len(det_q & gamma_set) == 0:
                g.add_edge(num_states, vd_0, weight=singal_weight)
            else:
                sigma_new = det_q & gamma_set
                for ((u, v), voca) in dic.items(): 
                    if (u==i) and (voca[0] in sigma_new):
                        weight = ce[voca[0]]
                        if weight > w:
                            w = weight
                        if weight < w_bar:
                            w_bar = weight
                        g.add_edge(num_states, v, weight=weight)
            num_states += 1
    # Now we have a SC-game
    edges = [] 
    # edges_ratio = []             
    for (u, v, wt) in g.edges.data('weight'):
        edges.append((u, v, wt))
        # edges_ratio.append((u, v, wt, 1))
    
    edge_count += len(g.edges)
    node_count += len(g.nodes)
    # print("the {} epoch have {} states and {} edges.".format(epoch, int(len(g.nodes)), int(len(g.edges))))
    
    # time_r_1, time_r_2, t_c = ratio_game(edges_ratio, nodes, w, len(g.nodes), len(marked), w_bar)  
    # print("{} epoch ratio game takes {} seconds and {} seconds, with single point {} seconds.".format(epoch, time_r_1, time_r_2, t_c))
    # time_r_count1 += time_r_1
    # time_r_count2 += time_r_2
    # t_count += t_c
    
    # start = time.time()
    ## note w and w_bar has been defined above.
    ksi_list = []
    ksi_value = []
    for d in range(1, len(marked)+1):
        for n in range(2*d*w_bar, len(g.nodes)*w+1):
            if n/d not in ksi_value:
                ksi_value.append(n/d)
                ksi_list.append((n, d, n/d))

    ksi_list = np.array(ksi_list)
    ksi_list = ksi_list[np.lexsort(ksi_list.T)]
    # print("{} epoch SC game : {}.".format(epoch, w))
    max_good_aver += w
    # The Algorithm 1: Optimal Strategy Search begins.
    i = 0
    i_min = 1
    i_max = len(ksi_list)
    max_w_sc = 0
    flag = False
    # Here we use a way to slove mpg, which has differnet meaning with player0 and player1.
    # So we use p - wt to represent the new weight.
    while i_min < i_max:
        i = int((i_min+i_max)/2)
        p = ksi_list[i-1][2]
        a = ksi_list[i-1][0]
        b = ksi_list[i-1][1]
        edges_new = []
        for (node, node_next, weight) in edges:
            if node_next not in marked:
                weight += p
            if (-1*weight+p)*b > max_w_sc:
                max_w_sc = (-1*weight+p)*b
            edges_new.append((node, node_next, (-1*weight+p)*b))

        mpg = MeanPayOffGame(nodes, edges_new, w*b, num_states)
        f, M = energy_game(mpg)
        if f[0] != 2*M:
            i_max = i
            edges_good = edges_new.copy()
            f_good = f.copy()
            M_good = M.copy()
            flag = True
        else:
            i_min = i + 1

    if flag:
        print("p = {}".format(ksi_list[i_max-1][2]))
        mpg = MeanPayOffGame(nodes, edges_good, w*b, num_states)
        best_strategy = energy_strategy(mpg, f_good, M_good)
        print("Note if you see description with all elements in the form of a:a, it means the DES has no multiple edges.")
        print("description:\n", description)
        print("\n")
        print("description1:\n", description1)
        print('-----------------------')
        print('-----------------------')
        print("The strategy is:\n", best_strategy)
    else:
        print("None!")
    # end = time.time()
    # print("{} epoch SC game takes {} seconds.".format(epoch, end-start))
    # time_count += end - start
        
# print("all the {} epochs SC game takes {} seconds in average.".format(epochs, time_count/epochs))
# print("all the {} epochs ratio game takes {} seconds in average to determine value.".format(epochs, time_r_count1/epochs))
# print("with single point takes {} seconds in average.".format(t_count/epochs))
# print("all the {} epochs ratio game takes {} seconds in average to determine strategies.".format(epochs, time_r_count2/epochs))
# print("all the {} epochs have {} states and {} edges in average.".format(epochs, int(node_count/epochs), int(edge_count/epochs)))
# print("all the {} epochs have {} max weight in average.".format(epochs, int(max_good_aver/epochs)))
    


    