#!/usr/bin/env python3
#coding:utf-8


import importlib,sys
importlib.reload(sys)
import MySQLdb, MySQLdb.cursors
#import modbus_tk.modbus_tcp
from pymodbus.client.sync import ModbusTcpClient
import time, threading
import rospy
from configs import db_mysql

from core import dict_config
from modules import moving_ring_data
from configs.log import logger

class hmiOperation():

    def __init__(self,robot_id,db_mysql):
        self.team_robot_id = robot_id
        self.mysql_conf = db_mysql
        self.dict_config = dict_config
        self.env = moving_ring_data
        self.connectMysql()
        self.hmi_info = self.getHMIInfo(robot_id)
        self.connectHmi()
    # 创建数据库连接
    def connectMysql(self):
        self.conn = MySQLdb.connect(host=self.mysql_conf['host'], user=self.mysql_conf['user'], passwd=self.mysql_conf['pwd'], db=self.mysql_conf['database'], cursorclass=MySQLdb.cursors.DictCursor, charset='utf8')
        self.cur = self.conn.cursor()
    # 从数据库获取显示屏参数，连接显示屏
    def connectHmi(self):
        self.UNIT = 1
        self.client = ModbusTcpClient(host=self.hmi_info['hmi_ip'], port=self.hmi_info['hmi_port'])
        result = self.client.connect()
        if not result:
            print(self.result(1, 'connect hmi error, address or port error'))
            return self.result(1, 'connect hmi error, address or port error')
    # 获取显示屏参数
    def getHMIInfo(self, robot_id):
        sql = ('select * from robot_config where \t\t\trobot_config_id = {0}').format(robot_id)
        self.cur.execute(sql)
        team_robot = self.cur.fetchone()
        if not team_robot:
            return self.result(1, 'search db robot error')
        sql = ("select * from robot_device where \t\t\trobot_id = {0} and \t\t\tname = '{1}'").format(team_robot['robot_id'], '显示屏')
        self.cur.execute(sql)
        device = self.cur.fetchone()
        if not device:
            return self.result(1, 'search db robot_device error')
        sql = ('select * from robot_path where \t\t\trobot_id = {0} and \t\t\tstatus = 0').format(robot_id)
        self.cur.execute(sql)
        path_type_id = self.cur.fetchone()['robot_path_id']
        if not path_type_id:
            return self.result(1, 'search db robot_path_type error')
        hmi_info = {'hmi_ip': device['address'], 
           'hmi_port': device['port'], 
           'hmi_path': path_type_id}
        return hmi_info

    # 更新显示屏值
    def updateHMIValue(self):
        # 确认目前机器人有哪些传感器值
        for name in dict_config.algorithm_dic.keys():
            try:
                # 遍历现有传感器值，写入到寄存器中
                if name in dict_config.screen_list:
                    value = int(float(self.env.Dynamic_environment(name)))
                    self.client.write_registers(201+2*(dict_config.screen_list.index(name)), value, unit=1)
                    continue
                else:
                    continue
                return True
            except Exception as e:
                logger.error(e)
                return False

    # def result(self, code=0, msg='', data=''):
    #     result = {'code': code,
    #        'msg': msg,
    #        'time': int(time.time()),
    #        'data:': data}
    #     return result


if __name__ == '__main__':
    rospy.init_node('hmiScreen', anonymous=True)
    robot_id =1
    print(db_mysql)
    def repeat():
        hmi = hmiOperation(robot_id, db_mysql)
        hmi.updateHMIValue()
        cycle = 1
        t = threading.Timer(cycle, repeat)
        t.start()

    repeat()

