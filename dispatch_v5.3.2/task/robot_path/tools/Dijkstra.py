# ------------------------------------
# 路径规划-迪杰斯特拉算法计算任意两点间的最短距离和路径
# Right© of MOONPAC
# Written By hd on 2022.05.16
# Updated By hd on 2022.06.10
# DEV: 在最短路径的基础上计算最优路径。
# Envs: python 3.6.9
# ------------------------------------
import copy
import numpy as np

def findAllPath(graph, start, end):
    path = []
    stack = []
    stack.append(start)
    visited = set()
    visited.add(start)
    seen_path = {}
    while (len(stack) > 0):
        start = stack[-1]
        nodes = graph[start]
        if start not in seen_path.keys():
            seen_path[start] = []
        g = 0
        for w in nodes:
            if w not in visited and w not in seen_path[start]:
                g = g + 1
                stack.append(w)
                visited.add(w)
                seen_path[start].append(w)
                if w == end:
                    path.append(list(stack))
                    old_pop = stack.pop()
                    visited.remove(old_pop)
                break
        if g == 0:
            old_pop = stack.pop()
            del seen_path[old_pop]
            visited.remove(old_pop)
    return path

def cal_dis(graph, paths):
    dis_list = []
    for i in paths:
        dis = 0
        for j in range(0, len(i) - 1):
            dis += graph[i[j]][i[j + 1]]
        dis_list.append(dis)
    index = [i for i, x in enumerate(dis_list) if x == min(dis_list)]
    roads = paths[index[0]]
    for i in range(0, len(index)):
        if len(paths[index[i]]) < len(roads):
            roads = paths[index[i]]
    return min(dis_list), roads

def dijkstra(graph, start, end):
    """
    :param graph: 邻接矩阵
    :param start: 起点
    :param end: 终点
    :return:
    """
    B = copy.deepcopy(graph)
    for i in range(0, len(B)):
        for j in range(0, len(B[i])):
            if B[i][j] != float('inf'):
                B[i][j] = 1
            else:
                B[i][j] = 0

    A_dict = {i: np.nonzero(row)[0].tolist() for i, row in enumerate(B)}
    path = findAllPath(A_dict, start, end)
    dis, roads = cal_dis(graph, path)

    return dis, roads