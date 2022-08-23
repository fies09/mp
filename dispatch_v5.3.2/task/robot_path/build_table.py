#!/usr/bin/env python
#  -*- coding: utf-8 -*-

# ------------------------------------
# Right© of MOONPAC
# Written By han on 2022.05.12
# DEV: path-planning
# Envs: python 3.6.9
# ------------------------------------

import pandas as pd
import pymysql
import json
import time
import os
from decimal import Decimal
import re
from configs.log import logger


class BuildTable:
    def __init__(self, config):
        try:
            # 数据库设置
            self.host = config["mysql"]["host"]
            self.database = config["mysql"]["database"]
            self.user = config["mysql"]["user"]
            self.password = config["mysql"]["password"]
            self.port = config["mysql"]["port"]
            self.charset = config["mysql"]["charset"]

            # 所需数据库表设置-机器人点位置表
            self.position_table = config["table"]["position_table"]
            self.begin_id = config["table"]["begin_id"]
            self.position = config["table"]["position"]
            self.stop_id = config["table"]["stop_id"]
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
        conn = pymysql.connect(host=self.host, database=self.database, user=self.user, password=self.password, port=self.port,
                              charset=self.charset)

        return conn


    def create_adjacency_matrix(self, con):
        '''
        构建邻接矩阵
        :param data: 数据库点坐标和连通关系
        :return: 邻接矩阵
        '''
        # 读取表
        get_sql = "select {begin_id}, {position}, {stop_id} from {table}".format(begin_id=self.begin_id, position=self.position,stop_id=self.stop_id,table=self.position_table)

        # 利用pandas 模块导入MysqL数据
        data = pd.read_sql(get_sql, con)

        if data.empty == True:
            logger.error(f"Empty database!")

        # 根据巡检点数量创建空的邻接矩阵
        fields = [[float(-1)] * len(data) for i in range(len(data))]

        # 根据巡检点连通关系和巡检点坐标更新邻接矩阵，权重利用切尔霍夫距离
        for i in range(0, data.shape[0]):
            stop_id = str(data.loc[i][self.stop_id]).split(",") #各起始点的终点集合1

           # 起始点坐标
            begin_position = json.loads(data.loc[i][self.position])
            begin_position_x = begin_position['x']
            begin_position_y = begin_position['y']

            if stop_id == ['None']:
                pass
            else:
                for j in range(0, len(stop_id)):
                    #起始点和终点关系
                    end_position = json.loads(data.loc[data[self.begin_id] == int(stop_id[j]), self.position].iloc[0])
                    end_position_x = end_position['x']
                    end_position_y = end_position['y']

                    # 更新邻接矩阵
                    # 鉴于点编号从1开始，故邻接矩阵的起始点都需-1
                    fields[int(data.loc[i][self.begin_id]) - 1][int(stop_id[j]) - 1] = \
                        float(abs(Decimal(begin_position_x) - Decimal(end_position_x)) if (abs(Decimal(begin_position_x) - Decimal(end_position_x))) > (abs(Decimal(begin_position_y) - Decimal(end_position_y))) else abs((Decimal(begin_position_y) - Decimal(end_position_y))))

        return fields

    def check_field(self, matrix):
        for i in range(0, len(matrix)):
            if len(set(matrix[i])) == 1:
                logger.warning(f"Inspection point {i} is an Outliers point!")


    def build_table(self, con, matrix):
        '''
        在数据库中创建路由表
        '''
        res = 0
        cur = con.cursor()
        # 判断路由表是否存在
        query_sql = "show tables"
        cur.execute(query_sql)
        tables = cur.fetchall()
        tables_list = re.findall('(\'.*?\')', str(tables))
        tables_list = [re.sub("'", '', each) for each in tables_list]

        # 若路由表存在，则删除。
        if self.route_table in tables_list:
            logger.warning(f"Routing table '{self.route_table}' existed and will be replaced!")
            drop_sql = "DROP TABLE {route_table}".format(route_table=self.route_table)
            cur.execute(drop_sql)
        # 新建路由表
        try:
            create_tb_sql = "CREATE TABLE {route_table} ({BeginId} INT(11) NOT NULL, {StopId} INT(11) NOT NULL, {Distance} Float NOT NULL)".format(BeginId=self.BeginId,StopId=self.StopId,Distance=self.Distance,route_table=self.route_table)
            cur.execute(create_tb_sql)
            logger.info(f'Successed to create route table {self.route_table}! ')
            logger.info(f'Inserting data into the route table {self.route_table}, Please wait about a minute!')
            # 向路由表中插入数据
            for i in range(0, len(matrix)):
                for j in range(0, len(matrix)):
                    values = (i+1, j+1, matrix[i][j])
                    if matrix[i][j] != -1:
                        insert_sql = '''INSERT INTO {route_table} VALUES (%s, %s, %s);'''.format(route_table=self.route_table)
                        cur.execute(insert_sql, values)
            con.commit()
            logger.info(f'Successed to update route table {self.route_table}! ')
            # 关闭游标对象
            cur.close()
            res = 1

        except Exception as e:
            logger.error(f'Failed to create route route_table: {e}')

        return res

    def run(self):
        result = 0
        try:
            con = self.con_mysql()
            logger.info("Successfully connect database! ")
        except Exception as e:
            logger.error(f"Failed to connect database!  Connection error：{e}")

        graph = self.create_adjacency_matrix(con)
        self.check_field(graph)
        result = self.build_table(con, graph)

        # # 断开数据库连接
        con.close()

        return result



if __name__ == '__main__':
    from utils.load_config import get_yaml_data
    ts = time.time()
    config = get_yaml_data('configs/pathplan.yaml')
    bulid_table = BuildTable(config)
    te = time.time()
    print(f'Inited, runtime={int(1000*(te-ts))}ms')

    # ## test------------------------------------------------
    ts2 = time.time()
    result = bulid_table.run()
    te = time.time()
    print(f'Results: {result}, runtime={int(1000*(te-ts2))}ms')

