#!/usr/bin/env python3
# coding:utf-8
# 此脚本用于页面自检
import sys
import time
import urllib
import os
from cv2 import log
sys.path.append(os.getcwd())
#from joblib import PrintTime
import rospy
import socket
import requests
import logging
from requests.auth import HTTPDigestAuth
from std_msgs.msg import Float64MultiArray, String, Bool, UInt8, Bool, Int8
from ptz_service.srv import Greeting, GreetingRequest
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from mprobot_msgs.msg import commonAction, commonGoal
from modules.robot_hardware_server import RobotDeviceStatus
from configs.log import logger
import actionlib
from task.light_al import post_voice_str
import json
from modules.robot_hardware_server import robot_ptz
import random

# 导入mysql告警管理信息
from schema.db_alarm_manage import DBAlarmManage
from datetime import datetime
from task.hardware_exception import sensor_test, server_tests, hart_inspect_test
from sensor_msgs.msg import BatteryState

inspect_res = {
    "sensor": {
        "temhum": "正常",
        "noise": "正常",
        "battery": "正常",
        "mp_all": "正常",
        "sidoxide": "正常",
        "hsulfide": "正常",
        # "wind_velocity": "正常",
    },
    "camera": {
        "ptz": "正常",
        "fourfold": "正常",
        "lift": "正常",
        "thermal_imagery": "正常",
        "industrial": "正常"
    },
    "state": {
        "scan": "正常",
        "IMU": "正常",
        "fall": "正常",
        "avoidance": "正常"
    },
    "server": {
        "agx": "正常",
        "ips": "正常",
        "arithmetic": "正常",
    }
}

temp_ip = {
    'host': '192.168.10.230'
}
optical_ip = {
    'host': '192.168.9.120',
    'port': '1080',
    'username': 'admin',
    'passwd': 'Admin123',
    'id': "1"
}
industry_api = {
    'host': '192.168.10.100',
    'port': "8088",
}

fourfold_ip = {
    "lower_fourfold": {
        'host': '192.168.10.211',
        'port': '80',
        'username': 'admin',
        'password': 'Admin123',
    },
    "upper_fourfold": {
        'host': '192.168.10.212',
        'port': '80',
        'username': 'admin',
        'password': 'Admin123',
    }
}

# 传感器
sensor_list = [
    ["/temhum", Float64MultiArray, "温湿度"],
    ["/noise", String, "噪音"],
    ["/battery", BatteryState, "电池"],
    ["/mp_all", Float64MultiArray, "七合一"],
    ["/sdioxide", String, "二氧化硫"],
    ["/hsulfide", String, "硫化氢"],
    # ["/wind_velocity", Float64MultiArray, "风速"],
]

# 服务
server_data = {
    "ip": "10.173.27.120",
    "all_port" : [[8086, "agx"], [8088, "ips"]],
    "arithmetic_ip" : "192.168.10.134",
    "arithmetic_port" : [8087, "arithmetic"]
}


# 动环检测
def sensor_test():
    res_list = []
    for i in sensor_list:
        key=i[0][1:]
        try:
            data = rospy.wait_for_message(i[0], i[1], timeout=5)
            if not data:
                RobotDeviceStatus().set_device_status(i[2], False)
                sensor_test()
            else:
                logger.info("{}检测正常".format(i[2]))
        except Exception as e:
            logger.error(e)
            RobotDeviceStatus().set_device_status(i[2], False)
            sensor_test()

    if False in res_list:
        return False
    else:
        return True

def robot_voice(name):
    post_voice_str(name, 1)

# 摄像头检测
def camera_test():
    pd_list = []
    # 云台检测
    # 控制云台，传递两个参数，第一个为控制命令，第二个为开关命令(0表示开始，1表示停止),第三个为运行速度
    try:
        num = random.randint(1, 3)
        result = robot_ptz("G", num)
        if result == False:
            logger.info("云台检测正常")
        else:
            logger.info("云台检测异常")
            inspect_res["camera"]["ptz"] = "异常"
            RobotDeviceStatus().set_device_status('ptz',False)
            data_value = "云台"
            hart_inspect_test(data_value)
    except Exception as e:
        logger.error(e)
        logger.error("云台检测异常")
        inspect_res["camera"]["ptz"] = "异常"
        RobotDeviceStatus().set_device_status('ptz',False)
        # 将告警信息插入alarm_manage数据表
        data_value = "云台"
        hart_inspect_test(data_value)

    # 四倍相机检测
    try:
        for port in ["2007", "2009"]:
            if port == '2007':
                fourfold_preview = fourfold_ip.get("lower_fourfold")
                ip = fourfold_preview.get("host")
                port = fourfold_preview.get("port")
                username = fourfold_preview.get("username")
                password = fourfold_preview.get("password")
            else:
                fourfold_preview = fourfold_ip.get("upper_fourfold")
                ip = fourfold_preview.get("host")
                port = fourfold_preview.get("port")
                username = fourfold_preview.get("username")
                password = fourfold_preview.get("password")
            StreamingUrl = "http://" + ip + ":" + str(port) + "/ISAPI/Streaming/channels/1"
            auth = HTTPDigestAuth(username, password)
            url = StreamingUrl + "/picture"
            xml_data = requests.get(url, auth=auth)
            if xml_data.status_code != 200:
                inspect_res["camera"]["fourfold"] = "异常"
                pd_list.append(False)
                dic1 = {}
                dic1['inspect_res["camera"]["fourfold"]'] = '异常'
                if port == "2007":
                    logger.error("上四倍相机检测异常")
                    RobotDeviceStatus().set_device_status('fourfold_up',False)
                    # 将告警信息插入alarm_manage数据表
                    data_value = "四倍相机上"
                    hart_inspect_test(data_value)
                else:
                    logger.error("下四倍相机检测异常")
                    RobotDeviceStatus().set_device_status('fourfold_down',False)
                    # 将告警信息插入alarm_manage数据表
                    data_value = "四倍相机下"
                    hart_inspect_test(data_value)
            else:
                if port == "2007":
                    logger.info("上四倍相机检测正常")
                else:
                    logger.info("下四倍相机检测正常")
    except Exception as e:
        logger.error(e)
        inspect_res["camera"]["fourfold"] = "异常"
        logger.error("四倍相机检测异常")
        RobotDeviceStatus().set_device_status('fourfold',False)
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "四倍相机"
        hart_inspect_test(data_value)

    # 升降杆检测
    try:
        client = actionlib.SimpleActionClient('elevator_ctl', commonAction)
        client.wait_for_server(rospy.Duration(3.0))
        goal = commonGoal()
        goal.actionId = 0
        action_params = {}
        # 0:return 0  1:move to the position  2:move to to distance
        inspect_list = [0, 2, 0]
        for i in inspect_list:
            action_params['type'] = i
            action_params['position'] = 300
            action_params['distance'] = 300
            goal.actionArg = json.dumps(action_params)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(15.0))
            state = client.get_result().actionStatus
            if state:
                rospy.loginfo("Goal succeeded!")
            else:
                rospy.logerr("Goal failed with error code:升降杆:")
                inspect_res["camera"]["lift"] = "异常"
                RobotDeviceStatus().set_device_status('lifter',False)
                logger.info("升降杆检测异常")
                pd_list.append(False)
                # 将告警信息插入alarm_manage数据表
                data_value = "升降杆"
                hart_inspect_test(data_value)
        logger.info("升降杆检测正常")
    except Exception as e:
        logger.error(e)
        inspect_res["camera"]["lift"] = "异常"
        logger.info("升降杆检测异常")
        # robot_voice("升降杆检测异常")
        RobotDeviceStatus().set_device_status('lifter',False)
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "升降杆"
        hart_inspect_test(data_value)

    # 热成像检测
    try:
        thermal_ip = "http://{}".format(temp_ip.get("host"))
        # 登录Post数据
        loginurl = thermal_ip + "/cgi-bin/system?Module=DMTtl"
        # 构造header
        headers = {
            "User-Agent": "Mozilla/5.0 (Linux; Android 6.0; Nexus 5 Build/MRA58N) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/66.0.3359.139 Mobile Safari/537.36",
            "Cookie": "Language=zh-cn; DevType=DM20; OEMINFO=OEM; Name=admin; Pass=Admin123; UserType=Admin; IPCamera=DM20"}
        response = requests.get(loginurl, headers=headers)
        temp_info = urllib.parse.unquote(response.text)
        Area = temp_info.split('&')[1]
        Area_list = Area.split('|')
        temp_list = Area_list[1].split(',')
        temp_dic = {"high_temp": temp_list[0], "mini_temp": temp_list[1], "aver_temp": temp_list[2]}
        logger.info("热成像检测正常")
    except Exception as e:
        logger.error(e)
        inspect_res["camera"]["thermal_imagery"] = "异常"
        logger.info("热成像检测异常")
        RobotDeviceStatus().set_device_status('thermal_imagery',False)
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "热成像"
        hart_inspect_test(data_value)

    # 工业相机检测
    try:
        for i in range(2):
            url = "http://{}:{}/get_pic?name=test_path&id={}".format(str(industry_api.get("host")),
                                                                     str(industry_api.get("port")), str(i))
            response = requests.get(url)
            if response.status_code != 200:
                if i == 0:
                    inspect_res["camera"]["industrial"] = "异常"
                    logger.error("上工业相机检测异常")
                    # robot_voice("上工业相机检测异常")
                    RobotDeviceStatus().set_device_status('industrial_camera_up',False)
                    pd_list.append(False)
                    # 将告警信息插入alarm_manage数据表
                    data_value = "工业相机上"
                    hart_inspect_test(data_value)
                else:
                    inspect_res["camera"]["industrial"] = "异常"
                    logger.error("下工业相机检测异常")
                    RobotDeviceStatus().set_device_status('industry_camera_up',False)
                    pd_list.append(False)
                    # 将告警信息插入alarm_manage数据表
                    data_value = "工业相机下"
                    hart_inspect_test(data_value)
            else:
                logger.info("工业相机检测正常")
    except Exception as e:
        logger.error(e)
        inspect_res["camera"]["industrial"] = "异常"
        logger.info("工业相机检测异常")
        RobotDeviceStatus().set_device_status('industrial_camera', False)
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "工业相机"
        hart_inspect_test(data_value)

    if False in pd_list:
        return False
    else:
        return True

# 状态检测
def state_test():
    pd_list = []
    # 激光
    try:
        scan = rospy.wait_for_message("/scan", LaserScan, timeout=10)
        logger.info("激光检测正常")
    except Exception as e:
        logger.error(e)
        inspect_res["state"]["scan"] = "异常"
        logger.error("激光检测异常")
        RobotDeviceStatus().set_device_status('scan',False)
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "激光"
        hart_inspect_test(data_value)
    # IMU
    try:
        imua = rospy.wait_for_message("/imu/data", Imu, timeout=10)
        logger.info("IMU检测正常")
    except Exception as e:
        logger.error(e)
        inspect_res["state"]["IMU"] = "异常"
        logger.error("IMU检测异常")
        # robot_voice("IMU检测异常")
        RobotDeviceStatus().set_device_status('IMU',False)
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "IMU"
        hart_inspect_test(data_value)

    # 防跌落检测
    try:
        fall = rospy.wait_for_message("/fall", Int8, timeout=5)
        if not fall:
            inspect_res["state"]["fall"] = "异常"
            logger.error("防跌落检测异常")
            RobotDeviceStatus().set_device_status('fall',False)
            # robot_voice("防跌落检测异常")
            pd_list.append(False)
            # 将告警信息插入alarm_manage数据表
            data_value = "防跌落"
            hart_inspect_test(data_value)
        else:
            logger.info("防跌落检测正常")
    except Exception as e:
        logger.error(e)
        inspect_res["state"]["fall"] = "异常"
        logger.error("防跌落检测异常")
        RobotDeviceStatus().set_device_status('fall',False)
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "防跌落"
        hart_inspect_test(data_value)
    # 超声波
    try:
        avoidance = rospy.wait_for_message("/avoidance", Int8, timeout=5)
        if not avoidance:
            inspect_res["state"]["avoidance"] = "异常"
            logger.error("超声波检测异常")
            RobotDeviceStatus().set_device_status('avoidance',False)
            pd_list.append(False)
            # 将告警信息插入alarm_manage数据表
            data_value = "超声波"
            hart_inspect_test(data_value)
        else:
            logger.info("超声波检测正常")
    except Exception as e:
        logger.error(e)
        inspect_res["state"]["avoidance"] = "异常"
        logger.error("超声波检测异常")
        RobotDeviceStatus().set_device_status('avoidance',False)
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "超声波"
        hart_inspect_test(data_value)
    if False in pd_list:
        return False
    else:
        return True

# 服务检测函数
def is_open(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((ip, int(port)))
        s.shutdown(2)
        return True
    except Exception as e:
        logger.error(e)
        return False

# 服务检测
def server_test():
    pd_list = list()
    ip = server_data.get("ip")
    all_port = server_data.get("all_port")
    for i_port in all_port:
        res = is_open(ip, i_port[0])
        if not res:
            # 将告警信息插入alarm_manage数据表
            server_tests()
        else:
            logger.info("{}服务检测正常".format("算法" if i_port[1] == "arithmetic" else i_port[1]))
    arithmetic_ip = server_data.get("arithmetic_ip")
    arithmetic_port = server_data.get("arithmetic_port")
    res2 = is_open(arithmetic_ip, arithmetic_port[0])
    if not res2:
        inspect_res["server"][arithmetic_port[1]] = "异常"
        logger.error("算法服务检测异常")
        pd_list.append(False)
        # 将告警信息插入alarm_manage数据表
        data_value = "算法服务"
        hart_inspect_test(data_value)
    else:
        logger.info("算法服务检测正常")

    if False in pd_list:
        return False
    else:
        return True

def inspect_main(type):
    logger.info("======================初始化硬件状态======================")
    RobotDeviceStatus()
    if type == 1:
        # API检测
        logger.info("======================执行机器人服务检测======================")
        res=server_test()
        if res:
            logger.info("======================机器人服务检测正常======================")
        else:
            logger.info("======================机器人服务检测异常======================")
        return_data=inspect_res["server"]
        # print("return_data", return_data)
        return return_data
    elif type == 0:
        logger.info("======================执行传感器检测======================")
        res=sensor_test()
        if res:
            logger.info("======================传感器检测正常======================")
        else:
            logger.info("======================传感器检测异常======================")
        logger.info("======================执行摄像头检测======================")
        res=camera_test()
        if res:
            logger.info("======================摄像头检测正常======================")
        else:
            logger.error("======================摄像头检测异常======================")
        logger.info("======================执行状态检测======================")
        res=state_test()
        if res:
            logger.info("======================状态检测正常======================")
        else:
            logger.info("======================状态检测异常======================")
        dic = {}
        dic.update(inspect_res["sensor"])
        dic.update(inspect_res["camera"])
        dic.update(inspect_res["state"])
        # print("dic", dic)
        return dic
    else:
        return None
if __name__ == '__main__':
    # 自检接口：http://主控IP:8088/robot_inspect
    rospy.init_node('self_test')
    # 接收type参数
    try:
        inspect_type=sys.argv[1]
    except:
        inspect_type=False

    if inspect_type:
        logger.info(inspect_type)
        result=list()
        result.append(inspect_main(int(inspect_type)))
        # print(json.dumps(result, ensure_ascii=False))
        # print(result)
    else:
        for i in [0, 1]:
            inspect_main(i)
            print("inspect_main(i)", inspect_main(i))
# 非必要情况，勿动此程序！！！
# 非必要情况，勿动此程序！！！
# 非必要情况，勿动此程序！！！

