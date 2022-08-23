#!/usr/bin/env python
#  -*- coding: utf-8 -*-

# ------------------------------------
# Right© of MOONPAC
# Written By hd on 2022.05.16
# DEV: path-planning
# Envs: python 3.6.9
# ------------------------------------
import numpy as np
import pandas as pd
import pymysql
import time
import os
from task.robot_path.tools import Dijkstra
from task.robot_path.tools import SA
from configs.log import logger
from schema.db_robot_position import DBRobotPosition
from task.robot_path.utils.load_config import get_yaml_data


class PathPlan:
    def __init__(self, config):
        try:
            # 数据库设置
            self.host = config["mysql"]["host"]
            self.database = config["mysql"]["database"]
            self.user = config["mysql"]["user"]
            self.password = config["mysql"]["password"]
            self.port = config["mysql"]["port"]
            self.charset = config["mysql"]["charset"]

            # 所需数据库表设置-路由表
            self.route_table = config["table"]["route_table"]
            self.BeginId = config["table"]["BeginId"]
            self.StopId = config["table"]["StopId"]
            self.Distance = config["table"]["Distance"]

        except Exception as e:
            logger.error(
                f'Wrong with the config paramters, FaceConfig-error: {e}')
            os._exit(0)

    def con_mysql(self):
        '''
        数据库连接
        :return:
        '''
        conn = pymysql.connect(host=self.host, database=self.database, user=self.user, password=self.password,
                               port=self.port,
                               charset=self.charset)
        # engine = create_engine('mysql+pymysql://root:mengpa@..@10.173.27.120:3306/v5.3_robot?charset=utf8')

        return conn

    def recovery_adjacency_matrix(self, con):
        '''
        还原邻接矩阵
        :param data: 数据库点坐标和连通关系
        :return: 邻接矩阵
        '''

        # 读取表
        get_sql = "select {begin_id}, {stop_id}, {distance} from {table};".format(begin_id=self.BeginId,
                                                                                  stop_id=self.StopId,
                                                                                  distance=self.Distance,
                                                                                  table=self.route_table)
        # 利用pandas 模块导入MysqL数据
        data = pd.read_sql(get_sql, con)

        if data.empty == True:
            logger.error(f"empty route database!")

        # 巡检点id
        points = list(set(data[self.BeginId].tolist() + data[self.StopId].tolist()))
        # 巡检点数量
        points_num = len(points)

        fields = [[float('inf')] * points_num for i in range(points_num)]
        # 还原矩阵
        for i in range(0, data.shape[0]):
            begin = data[self.BeginId].copy().tolist()[i]
            stop = data[self.StopId].copy().tolist()[i]
            dis = data[self.Distance].copy().tolist()[i]
            fields[points.index(begin)][points.index(stop)] = fields[points.index(stop)][points.index(begin)] = dis

        return points, fields

    def check_field(self, matrix, points):
        """
        孤立点检测
        :param matrix:
        :return:
        """
        status = 1
        outliers = []
        for i in range(0, len(matrix)):
            if len(set(matrix[i])) == 1:
                logger.warning(f"Inspection point {points[i]} is an Outliers point!")
                outliers.append(i)
                status = 0
        return status

    def check_point(self, point_num, point):
        """
        巡检点检测
        :param point_num:
        :param point:
        :return:
        """
        status = 1
        if point not in point_num:
            logger.warning(f"No exist point {point} is inspection points!")
            status = 0
        return status

    def run_two_points_plan(self, pathlist):
        """
        两点路径规划主流程
        :param pathlist: 多点路径序列
        :return:  状态，巡检点顺序，路径序列
        """
        status = 0
        path_sequence = []

        try:
            con = self.con_mysql()
            logger.info("Successfully connect database! ")

        except Exception as e:
            logger.error(f"Failed to connect database!  Connection error：{e}")
            status = -2
            return status, pathlist, path_sequence

        points, graph = self.recovery_adjacency_matrix(con)

        # 参数校验
        for i in pathlist:
            res = self.check_point(points, i)
            if res == 0:
                status = -1
                return status, pathlist, path_sequence
        # 孤立点校验
        res0 = self.check_field(graph, points)
        if res0 == 0:
            return status, path_sequence

        else:
            # 获取起点和终点在巡检点中的索引,计算得到距离和巡检点索引路径
            distance, path_index = Dijkstra.dijkstra(graph, points.index(pathlist[0]), points.index(pathlist[1]))
            # 通过索引找巡检点id
            path_sequence = [points[i] for i in path_index]
            status = 1
            return status, pathlist, path_sequence

    def run_multi_points_plan(self, pathlist):
        """
        多点路径规划主流程
        :param pathlist: 多点路径序列
        :return:  状态，巡检点顺序，路径序列
        """
        status = 0
        path_sequence = []

        try:
            con = self.con_mysql()
            logger.info("Successfully connect database! ")
        except Exception as e:
            logger.error(f"Failed to connect database!  Connection error：{e}")
            status = -2
            return status, pathlist, path_sequence

        points, graph = self.recovery_adjacency_matrix(con)
        # 参数校验
        for i in pathlist:
            res = self.check_point(points, i)
            if res == 0:
                status = -1
                return status, pathlist, path_sequence

        # 孤立点校验
        res0 = self.check_field(graph, points)
        if res0 == 0:
            return status, pathlist, path_sequence

        # 获取路径序列对应索引
        pathindex = [points.index(i) for i in pathlist]

        if len(pathlist) == 3:
            distance1, path_index1 = Dijkstra.dijkstra(graph, pathindex[0], pathindex[1])
            distance2, path_index2 = Dijkstra.dijkstra(graph, pathindex[1], pathindex[2])
            path_sequence = [points[i] for i in path_index1][:-1] + [points[i] for i in path_index2]
            distance = distance1 + distance2
            pointsorder = pathlist
        else:
            orderlist, distance, path_index = SA.sa(pathindex, graph)
            path_sequence = [points[i] for i in path_index]

            new = [pathlist[1:-1][int(index)] for index in orderlist]  # 停靠点顺序
            pointsorder = [[pathlist[0]] + new + [pathlist[-1]]]  # 所有巡检点顺序

        status = 1
        return status, pointsorder, path_sequence

    def run_avoid_plan(self, pathlist, pre, last):
        """
        避障后两点路径规划主流程
        :param pathlist: 两点路径序列
        :param pre: 机器人到达的上一个点
        :param last: 机器人应到达的下一个点
        :return:  状态，巡检点顺序，路径序列
        """
        status = 0
        path_sequence = []

        try:
            con = self.con_mysql()
            logger.info("Successfully connect database! ")
        except Exception as e:
            logger.error(f"Failed to connect database!  Connection error：{e}")
            status = -2
            return status, pathlist, path_sequence

        points, graph = self.recovery_adjacency_matrix(con)

        # 参数校验
        for i in pathlist:
            res = self.check_point(points, i)
            if res == 0:
                status = -1
                return status, pathlist, path_sequence
        # 孤立点校验
        res0 = self.check_field(graph, points)
        if res0 == 0:
            return status, pathlist, path_sequence

        pathindex = [points.index(i) for i in pathlist]

        graph[points.index(pre)][points.index(last)] = float('inf')
        graph[points.index(last)][points.index(pre)] = float('inf')

        # 对避障拓扑图进行孤立点校验
        res0 = self.check_field(graph, points)
        if res0 == 0:
            return status, pathlist, path_sequence

        distance, path_index = Dijkstra.dijkstra(graph, points.index(pre), pathindex[1])
        path_sequence = [points[i] for i in path_index]
        status = 1

        return status, pathlist, path_sequence


class RobotPathQuery():
    def __int__(self):
        # self.config = get_yaml_data('configs/pathplan.yaml')
        pass

    def robot_path(self, position_id_list: list) -> list:
        path_list = []
        logger.info("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        logger.info("算法入参 {}".format(position_id_list))
        logger.info("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        result = PathPlan(get_yaml_data('./task/robot_path/configs/pathplan.yaml')).run_two_points_plan(
            position_id_list)
        if len(result) != 3:
            return path_list

        if result[0] != 1:
            return path_list

        if result[1][0] != position_id_list[0] and result[1][1] != position_id_list[1]:
            return path_list

        logger.info("################################")
        logger.info("算法结果 {}".format(result))
        logger.info("算法规划路径 {}".format(result[2]))
        logger.info("################################")
        # 根据算法返回的路径顺序组装路径数据
        path_list = self.query_path(result[2])
        logger.info("路径序列 {}".format(path_list))
        return path_list

    def query_path(self, position_id_list_result: list) -> list:
        new_path_list = []
        path_list, path_list_total = DBRobotPosition().get_list(position_id_list_result)
        if len(path_list) < 0:
            return []

        # 按照算法返回的路径顺序排序
        for result_id in position_id_list_result:
            for path in path_list:
                if result_id == path["robot_position_id"]:
                    new_path_list.append(path)

        return new_path_list


if __name__ == '__main__':
    ts = time.time()
    config = get_yaml_data('configs/pathplan.yaml')
    print(config)
    pathplan = PathPlan(config)
    te = time.time()
    print(f'Inited, runtime={int(1000 * (te - ts))}ms')

    # ## 场景一test 两点路径规划--------------------------------------------
    ts2 = time.time()
    path_list = [4, 19]
    result = pathplan.run_two_points_plan(path_list)
    te = time.time()
    print(f'Results: {result}, runtime={int(1000 * (te - ts2))}ms')

    # ## 场景二test 多点路径规划-----------------------------------------------
    ts2 = time.time()
    # path_list = [11, 555, 6]
    path_list = [12, 4, 19, 5, 7]
    # path_list = [12,4,19]
    path_list = list(dict.fromkeys(path_list))  # 去重
    result = pathplan.run_multi_points_plan(path_list)
    te = time.time()
    print(f'Results: {result}, runtime={int(1000 * (te - ts2))}ms')
    #
    #
    # # ## 场景三test  两点间避障后路径规划---------------------------------------------
    ts2 = time.time()
    path_list = [10, 12]
    result = pathplan.run_avoid_plan(path_list, 11, 12)
    te = time.time()
    print(f'Results: {result}, runtime={int(1000 * (te - ts2))}ms')




