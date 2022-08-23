#! /usr/bin/env python3
# -*-coding: UTF-8 -*-
import logging
import os
import sys
import time
import rospy
import json
from ptz_service.srv import Greeting, GreetingRequest
from mprobot_msgs.srv import GeneralService
import ast

class PtzClient:
    def __init__(self, tty="/dev/ttyS7", vt=20, vp=20, lsb=15, msb=20):
        rospy.init_node('test_ptz_client')
        rospy.wait_for_service('greetings', 5.0)
        self.tty = "/dev/ttyS7"
        self.vt = vt
        self.vp = vp
        self.lsb = lsb
        self.msb = msb

    def ptz_method(self, common=None, position=None):
        try:
            person_client = rospy.ServiceProxy('greetings', Greeting)
            if common:
                cmd_name = ord(common)
                response = person_client(self.tty, cmd_name, self.vt, self.vp, position, self.lsb, self.msb)
                return True
            else:
                return False
        except Exception as e:
            return False


def ptz_client(postion):
    # ROS节点初始化
    # rospy.init_node('ptz_client')

    # 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('greetings', 5.0)
    try:
        vt = 20
        vp = 20
        lsb = 15
        msb = 20
        person_client = rospy.ServiceProxy('greetings', Greeting)
        # tty为端口号,cmdName为动作号 postion为预置点号
        tty = "/dev/ttyS7"
        cmdName = ord("G")
        # 请求服务调用，输入请求数据
        response = person_client(tty, cmdName, vt, vp, postion, lsb, msb)
        return True
    except:
        return False
    # rospy.signal_shutdown('clear')


def hk_hotcamera_client(req_type):
    """
    param: req_type temp:获取最高温最低温; pic:抓图
    海康热成像ros客户端
    """
    # 创建节点
    print("开始调取海康热成像")
    try:
        # 创建Service Client
        serviceName = 'hk_server_'
        client = rospy.ServiceProxy(serviceName, GeneralService)

        # 等待服务开启
        rospy.wait_for_service(serviceName)

        if req_type == "temp":  # 获取最高温最低温
            # logger.info("海康热成像开始获取温度")
            res = client('{\"command\":\"get_temperature\"}')
        elif req_type == "pic":  # 抓图
            # logger.info("海康热成像开始抓图")
            res = client('{\"command\":\"capture_picpure\"}')
        else:
            # logger.error("海康热成像调用传值错误")
            return None

        if "success" not in str(res):
            return None

        data = ast.literal_eval(str(res)[10:])
        return json.loads(data)
    except Exception as e:
        # logger.info("海康热成像调用ros错误: ", e)
        print("海康热成像调用ros错误: ", e)
        return None


if __name__ == '__main__':
    '''
    rospy.init_node('test_ptz_client')
    list_a = [1,2,3,4,5]
    for i in list_a:
        a = ptz_client(i)
        time.sleep(3)
        print(a)
    '''
    ptz = PtzClient()
    ptz_res = ptz.ptz_method(common="L")  # 向左移动
    print(ptz_res)
    time.sleep(2)
    ptz_res = ptz.ptz_method(common="R")  # 向右移动
    time.sleep(2)
    print(ptz_res)
    ptz_res = ptz.ptz_method(common="U")  # 向上移动
    time.sleep(2)
    print(ptz_res)
    ptz_res = ptz.ptz_method(common="D")  # 向下移动
    time.sleep(2)
    print(ptz_res)
    ptz_res = ptz.ptz_method(common="S")  # 停止云台
    time.sleep(2)
    print(ptz_res)
    ptz_res = ptz.ptz_method(common="P", position=1)  # 设置当前位置为1号预置位
    time.sleep(2)
    print(ptz_res)
    ptz_res = ptz.ptz_method(common="C", position=1)  # 清除1号预置位
    time.sleep(2)
    print(ptz_res)
    ptz_res = ptz.ptz_method(common="G", position=1)  # 移动到1号预置位
    time.sleep(2)
    print(ptz_res)

    # 初始化client
    nodeName = "hk_hotcamera_client_node"
    rospy.init_node(nodeName)

    data = hk_hotcamera_client("pic")
    print("data_ex: ", data)
    print("data_type: ", type(data))
    print("temperature: ", data["temperature"])
    print("picture_file: ", data["picture_file"])

