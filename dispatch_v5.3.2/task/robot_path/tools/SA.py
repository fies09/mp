import itertools
import math
import random
import numpy as np
from task.robot_path.tools.Dijkstra import dijkstra

# SA算法设置
tInitial = 100  # initial temperature
tFinal = 10  # stop temperature
nMarkov = 1000  # Markov链长度，也即内循环运行次数
alfa = 0.98  # 设定降温参数，T(k)=alfa*T(k-1)

def calTourMileage(tourGiven, nCities, distMat):
    """
    根据路径序列和距离矩阵计算路径长度
    :param tourGiven: 停靠点路径
    :param nCities: 停靠点数目
    :param distMat: 停靠点矩阵
    :return:
    """
    mileageTour = 0  # dist((n-1),0)
    for i in range(nCities - 1):  # dist(0,1),...dist((n-2)(n-1))
        mileageTour += distMat[tourGiven[i], tourGiven[i + 1]]

    return mileageTour  # 路径总长度


def mutateSwap(tourGiven, nCities):
    """
    交换操作算子，使用2-Swap操作符生成随机变换,在给定的顺序中交换两个停靠点的位置
    :param tourGiven: 停靠点路径
    :param nCities: 停靠点数量
    :return: 新的停靠点路径
    """
    # 在 [0,n) 产生 2个不相等的随机整数 i,j
    i = np.random.randint(nCities)  # 产生第一个 [0,n) 区间内的随机整数
    while True:
        j = np.random.randint(nCities)  # 产生一个 [0,n) 区间内的随机整数
        if i != j:
            break  # 保证 i, j 不相等
    tourSwap = tourGiven.copy()  # 将给定路径复制给新路径 tourSwap
    tourSwap[i], tourSwap[j] = tourGiven[j], tourGiven[i]  # 交换 城市 i 和 j 的位置————简洁的实现方法

    return tourSwap


def get_st_end_distance(path_list, order_sequence, stop_dis_path_mutual):
    '''
    :param path_list: 传入的所有路径点
    :param order_sequence: 停靠点顺序
    :param stop_dis_path_mutual: 所有路径点的两两距离和路径矩阵
    :return: 两端点路径距离长度之和
    '''

    ll = []
    for i in range(0, len(order_sequence)):
        ll.append(path_list[1:-1][order_sequence[i]])

    path_len = 0
    distance1 = stop_dis_path_mutual[str(path_list[0]) + '_' + str(ll[0])]['dis']
    distancen = stop_dis_path_mutual[str(ll[-1]) + '_' + str(path_list[-1])]['dis']

    path_len = distance1 + distancen

    return path_len


def get_path_sequence(path_list, order_sequence, stop_dis_path_mutual):
    '''
    :param path_list: 传入的所有路径点
    :param order_sequence: 停靠点顺序
    :param stop_dis_path_mutual: 所有路径点的两两距离和路径矩阵
    :return: 总路径序列
    '''
    path_list = [i + 1 for i in path_list]  # 数据点从1开始

    ll = []
    for i in range(0, len(order_sequence)):
        ll.append(path_list[1:-1][order_sequence[i]])

    ll = [path_list[0]] + ll + [path_list[-1]]  # 总巡检点数据
    path_sequence = []
    j = 0
    ll = [i - 1 for i in ll]  # 数据点从1开始
    while j < (len(ll) - 1):
        path_sequence += stop_dis_path_mutual[str(ll[j]) + '_' + str(ll[j + 1])]['path'][:-1]
        j += 1
    path_sequence += [ll[-1]]  # 数据点从1开始

    return path_sequence


def getDistMat(path_list, graph):
    '''
    :param path_list: 路径序列
    :param path_list: 邻接矩阵
    :return:distMat   停靠点距离
            stop_dis_path_mutual 所有点的两两距离 路径序列字典
    '''

    stop_dis_path_mutual = {

    }
    # 计算上三角距离 全部序列
    lst = path_list
    for x in itertools.combinations(lst, 2):
        distance, path_sequence = dijkstra(graph, x[0], x[1])
        temp = {

        }
        temp['dis'] = distance
        temp['path'] = path_sequence
        stop_dis_path_mutual[str(x[0]) + '_' + str(x[1])] = temp

    # # 构建停靠点的距离矩阵
    # 构建停靠点的空的初始距离矩阵
    distMat_len = len(path_list)
    distMat = np.mat(np.zeros((distMat_len, distMat_len)))

    dis = []
    for k, v in stop_dis_path_mutual.items():
        for k1, v1 in v.items():
            if k1 == 'dis':
                dis.append(v1)

    # # 更新停靠点的距离矩阵
    # 取上三角索引
    iu2 = np.triu_indices(distMat_len, 1)
    # 给矩阵上三角赋值
    for i in range(0, len(iu2[0])):
        distMat[iu2[0][i], iu2[1][i]] = dis[i]
    # 根据上三角给下三角赋值
    distMat += distMat.T - np.diag(np.diag(distMat))

    stop_distMat = distMat[1:-1, 1:-1]  # 停靠点矩阵

    # 更新全部路径点距离，路径存储字典
    dict2 = {

    }
    for k, v in stop_dis_path_mutual.items():
        temp = {}
        temp['dis'] = v['dis']
        temp['path'] = v['path'][::-1]
        dict2[k.split('_')[1] + '_' + k.split('_')[0]] = temp
    stop_dis_path_mutual.update(dict2)

    return stop_distMat, stop_dis_path_mutual, distMat


def sa(path_list, matrix):
    """
    SA算法计算停靠点的最佳顺序
    :param path_list: 路径序列
    :param matrix: 邻接矩阵
    :return: 停靠点顺序，路径距离和路径序列
    """
    stop_distMat, stop_dis_path_mutual, distMat = getDistMat(path_list, matrix)

    nMarkov = len(stop_distMat)  # Markov链 的初值设置
    tNow = tInitial  # 初始化 当前温度(current temperature)

    # 初始化准备
    tourNow = np.arange(nMarkov)  # 产生初始停靠点路径
    valueNow = 0
    for i in range(0, distMat.shape[0] - 1):
        valueNow += distMat[i, i + 1]

    tourBest = tourNow.copy()  # 初始化最优路径，复制 tourNow
    valueBest = valueNow  # recordPath，复制 valueNow

    recordNow = []  # 初始化 最优路径记录表
    recordPath = []  # 初始化 最优路径路径记录表

    # 开始模拟退火优化过程
    iter = 0  # 外循环迭代次数计数器
    while tNow >= tFinal:  # 外循环，直到当前温度达到终止温度时结束
        tem_best_value = 1000000
        tem_best_path = []
        # 在当前温度下，进行充分次数(nMarkov)的状态转移以达到热平衡
        for k in range(nMarkov):  # 内循环，循环次数为Markov链长度
            # 产生新解：
            tourNew = mutateSwap(tourNow, nMarkov)  # 通过 交换操作 产生新路径
            valueNew = calTourMileage(tourNew, nMarkov, stop_distMat)  # 计算当前路径的总长度

            # 寻求每次温度下的最短路径
            if tem_best_value > valueNew:
                tem_best_value = valueNew
                tem_best_path = tourNew

            deltaE = valueNew - valueNow
            # 接受判别：按照 Metropolis 准则决定是否接受新解
            if deltaE < 0:  # 更优解：如果新解的目标函数好于当前解，则接受新解
                accept = True
                if valueNew < valueBest:  # 如果新解的目标函数好于最优解，则将新解保存为 最优解
                    tourBest[:] = tourNew[:]
                    valueBest = valueNew
            else:  # 容忍解：如果新解的目标函数比当前解差，则以一定概率接受新解
                pAccept = math.exp(-deltaE / tNow)  # 计算容忍解的状态迁移概率
                if pAccept > random.random():
                    accept = True
                else:
                    accept = False

            # 保存新解
            if accept == True:  # 如果接受新解，则将新解保存为当前解
                tourNow[:] = tourNew[:]
                valueNow = valueNew

        # 平移当前路径，以解决变换操作避开 0,（n-1）所带来的问题，并未实质性改变当前路径。
        tourNow = np.roll(tourNow, 2)  # 循环移位函数，沿指定轴滚动数组元素
        # 完成当前温度的搜索，保存数据和输出
        # 每个温度下得到的解
        recordPath.append(tem_best_path.tolist())  # 将本次温度下的最优路径长度追加到 最优路径记录表   停靠点路径
        recordNow.append(tem_best_value)  # 将当前路径长度追加到 当前路径记录表  当前路径

        # 缓慢降温至新的温度
        iter = iter + 1
        tNow = tNow * alfa  # 指数降温曲线：T(k)=alfa*T(k-1)
    # 结束模拟退火过程

    # 去重每次温度判断得到的路径和路径长度
    dedup_recordPath = []
    dedup_index = []
    for i in range(0, len(recordPath)):
        if recordPath[i] not in dedup_recordPath:
            dedup_recordPath.append(recordPath[i])
            dedup_index.append(i)

    dedup_recordNow = []
    for i in dedup_index:
        dedup_recordNow.append(recordNow[i])


    # 计算温度得到的最短路径和长度
    tem_order_best = []
    tem_distance = 1000000
    for i in range(0, len(dedup_recordPath)):
        tem_distance0 = get_st_end_distance(path_list, dedup_recordPath[i], stop_dis_path_mutual)
        tem_distance0 += dedup_recordNow[i]  # 叠加停靠点距离
        if tem_distance > tem_distance0:
            tem_distance = tem_distance0
            tem_order_best = dedup_recordPath[i]

    best_distance = get_st_end_distance(path_list, tourBest, stop_dis_path_mutual)  # 最优解
    best_distance += valueBest

    if best_distance > tem_distance:
        distance = tem_distance
        path_sequence = get_path_sequence(path_list, tem_order_best, stop_dis_path_mutual)
        order = tem_order_best
    else:
        distance = best_distance
        path_sequence = get_path_sequence(path_list, tourBest, stop_dis_path_mutual)
        order = tourBest

    return order, distance, path_sequence