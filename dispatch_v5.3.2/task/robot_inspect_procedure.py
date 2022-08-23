#!/usr/bin/env python3
# coding:utf-8
# 此脚本用于测试人员检测使用
import os
import sys
import psutil
import time
import urllib
import rospy
import socket
import requests
import logging
from requests.auth import HTTPDigestAuth
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64MultiArray, String, Bool, UInt8, Bool, Int8, Int32MultiArray
from ptz_service.srv import Greeting, GreetingRequest
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

# from configs.log import logger
import actionlib
import json
from mprobot_msgs.msg import commonAction, commonGoal
from light_al import post_voice_str

logger = logging.getLogger(__name__)
logger.setLevel(level=logging.INFO)
handler = logging.FileHandler("/var/log/dispatch_logs/robot_inspect_procedure.log")
handler.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s | line-%(lineno)s')

handler.setFormatter(formatter)
logger.addHandler(handler)

# 网络服务信息列表
server_data = [
        ["192.168.10.134", 8086, "AGX"],
        ["192.168.10.100", 8088, "IPS"],
        ["192.168.10.134", 8087, "算法"],
        ["192.168.10.210", 1005, "云台"],
        ["192.168.10.212", 1009, "四倍相机上"],
        ["192.168.10.211", 1007, "四倍相机下"],
        ["192.168.10.230", 554, "热成像"],
        ["192.168.10.136", 80, "RFID发射器"],
]
# 热成像网络配置
temp_ip = {
    'host': '192.168.10.230'
}
# 工业相机网络配置
industry_api = {
    'host': '192.168.10.100',
    'port': "8088",
}
# 四倍相机网络配置
four_camera = {
    "top_ip": "192.168.10.212",
    "button_ip": "192.168.10.211",
    "port": 80
}


def digest(url):
    """
    http权限函数
    """
    time.sleep(2)
    auth = HTTPDigestAuth("admin", "Admin123")
    return auth


# 传感器检测
def sensor_test():
    """
    判断所有传感器是否正常
    """
    # 定义传感器参数列表
    sensor_list = [
        ["/temhum", Float64MultiArray, "温湿度"],
        ["/noise", String, "噪音"],
        ["/battery", BatteryState, "电池"],
        ["/mp_all", Float64MultiArray, "七合一"],
        ["/sdioxide", String, "二氧化硫"],
        ["/hsulfide", String, "硫化氢"],
    ]
    # 遍历传感器列表
    for i in sensor_list:
        try:
            # 订阅传感器数据
            data = rospy.wait_for_message(i[0], i[1], timeout=5)
            # 订阅成功则说明传感器正常，否则为异常
            if not data:
                logger.error("==={}检测异常===".format(i[2]))
                print("==={}检测异常===".format(i[2]))
            else:
                logger.info("==={}检测正常===".format(i[2]))
                print("==={}检测正常===".format(i[2]))
        except Exception as e:
            logger.error(e)
            logger.error("==={}检测异常===".format(i[2]))
            print("==={}检测异常===".format(i[2]))


# 摄像头检测
def camera_test():
    """
    摄像头检测函数
    """
    pd_list = []
    # 云台检测
    try:
        rospy.wait_for_service('greetings', timeout=10)
        vt = 20
        vp = 20
        lsb = 15
        msb = 20
        person_client = rospy.ServiceProxy('greetings', Greeting)
        # tty为端口号,cmdName为动作号 postion为预置点号
        tty = "/dev/ttyS2"
        cmdName = ord("G")
        # 请求服务调用，输入请求数据
        person_client(tty, cmdName, vt, vp, 2, lsb, msb)
        logger.info("===云台检测正常===")
        print("===云台检测正常===")
    except Exception as e:
        logger.error(e)
        logger.error("===云台检测异常===")
        print("===云台检测异常===")

    # 四倍相机检测
    try:
        # 调用四倍相机拍照接口
        sbxj_top = "http://{}:{}/ISAPI/Streaming/channels/1/picture".format(four_camera["top_ip"], four_camera["port"])
        # 获取权限
        auth_top = digest(sbxj_top)
        response_top = requests.get(sbxj_top, auth=auth_top)
        sbxj_button = "http://{}:{}/ISAPI/Streaming/channels/1/picture".format(four_camera["button_ip"], four_camera["port"])
        # 获取权限
        auth_button = digest(sbxj_button)
        response_button = requests.get(sbxj_top, auth=auth_button)
        # 如果可以正常访问，则代表相机正常，否则异常
        if response_top.status_code == 200:
            logger.info("===四倍相机上检测正常===")
            print("===四倍相机上检测正常===")
        else:
            logger.error("===四倍相机上检测异常===")
            print("===四倍相机上检测异常===")
        if response_button.status_code == 200:
            logger.info("===四倍相机下检测正常===")
            print("===四倍相机下检测正常===")
        else:
            logger.error("===四倍相机下检测异常===")
            print("===四倍相机下检测异常===")
    except Exception as e:
        logger.error(e)
        logger.error("===四倍相机检测异常===")
        print("===四倍相机检测异常===")

    # 升降杆检测
    try:
        client = actionlib.SimpleActionClient('elevator_ctl', commonAction)
        client.wait_for_server(rospy.Duration(3.0))
        goal = commonGoal()
        goal.actionId = 0
        action_params = {}
        # 0:return 0  1:move to the position  2:move to to distance
        inspect_list = [0, 2, 0]  # 定义运动类型，使其可上下移动检测
        # 遍历类型，使得升降杆上下移动
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
                logger.error("===升降杆检测异常===")
                print("===升降杆检测异常===")
        logger.info("===升降杆检测正常===")
        print("===升降杆检测正常===")
    except Exception as e:
        logger.error(e)
        logger.error("===升降杆检测异常===")
        print("===升降杆检测异常===")

    # 热成像检测
    try:
        thermal_ip = "http://{}".format(temp_ip.get("host"))
        # 登录Post数据
        loginurl = thermal_ip + "/cgi-bin/system?Module=DMTtl"
        # 构造header
        headers = {
            "User-Agent": "Mozilla/5.0 (Linux; Android 6.0; Nexus 5 Build/MRA58N) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/66.0.3359.139 Mobile Safari/537.36",
            "Cookie": "Language=zh-cn; DevType=DM20; OEMINFO=OEM; Name=admin; Pass=Admin123; UserType=Admin; IPCamera=DM20"}
        # 访问热成像接口确保它可以正常使用
        response = requests.get(loginurl, headers=headers)
        temp_info = urllib.parse.unquote(response.text)
        Area = temp_info.split('&')[1]
        Area_list = Area.split('|')
        temp_list = Area_list[1].split(',')
        temp_dic = {"high_temp": temp_list[0], "mini_temp": temp_list[1], "aver_temp": temp_list[2]}
        logger.info("===热成像检测正常===")
        print("===热成像检测正常===")
    except Exception as e:
        logger.error(e)
        logger.error("===热成像检测异常===")
        print("===热成像检测异常===")

    # 工业相机检测
    try:
        # 遍历上下工业相机参数
        for i in range(2):
            # 分别访问各个工业相机拍照接口
            url = "http://{}:{}/get_pic?name=test_path&id={}".format(str(industry_api.get("host")),
                                                                     str(industry_api.get("port")), str(i))
            response = requests.get(url)
            # 如果可以正常拍照，则表示工业相机正常
            if response.status_code == 200:
                if i == 0:
                    logger.info("===工业相机下检测正常===")
                    print("===工业相机下检测正常===")
                else:
                    logger.info("===工业相机上检测正常===")
                    print("===工业相机上检测正常===")
            else:
                if i == 0:
                    logger.error("===工业相机下检测异常===")
                    print("===工业相机下检测异常===")
                else:
                    logger.error("===工业相机上检测异常===")
                    print("===工业相机上检测异常===")
    except Exception as e:
        logger.error(e)
        logger.error("工业相机检测异常")


# 状态检测
def state_test():
    """
    状态检测函数
    """
    pd_list = []
    # 激光
    try:
        # 订阅激光消息，可以正常订阅则表示激光正常
        scan = rospy.wait_for_message("/scan", LaserScan, timeout=10)
        logger.info("===激光检测正常===")
        print("===激光检测正常===")
    except Exception as e:
        logger.error(e)
        logger.error("===激光检测异常===")
        print("===激光检测异常===")
        pd_list.append(False)

    # IMU
    try:
        # 订阅IMU消息，可以正常订阅则表示IMU正常
        imua = rospy.wait_for_message("/imu/data", Imu, timeout=10)
        logger.info("===IMU检测正常===")
        print("===IMU检测正常===")
    except Exception as e:
        logger.error(e)
        logger.error("===IMU检测异常===")
        print("===IMU检测异常===")

    # 防跌落检测
    try:
        # 订阅防跌落消息，可以正常订阅则表示防跌落正常
        fall = rospy.wait_for_message("/fall", Int8, timeout=5)
        if not fall:
            logger.error("===防跌落检测异常===")
            print("===防跌落检测异常===")
        else:
            logger.info("===防跌落检测正常===")
            print("===防跌落检测正常===")
    except Exception as e:
        logger.error(e)
        logger.error("===防跌落检测异常===")
        print("===防跌落检测异常===")

    # 超声波
    try:
        # 订阅超声波消息，可以正常订阅则表示超声波正常
        avoidance = rospy.wait_for_message("/avoidance", Int8, timeout=5)
        if not avoidance:
            logger.error("===超声波检测异常===")
            print("===超声波检测异常===")
        else:
            logger.info("===超声波检测正常===")
            print("===超声波检测正常===")
    except Exception as e:
        logger.error(e)
        logger.error("===超声波检测异常===")
        print("===超声波检测异常===")
    if False in pd_list:
        return False
    else:
        return True


# 灯带判断
def light_status_listen():
    """
    灯带判断函数
    """
    light_node = [
        ["/battery", BatteryState],
        ['/avoidance', Int8],
        # ['/obstacle', UInt8],
        ['/isStop', Bool]
    ]  # 定义灯带需检查的节点
    pd_list = list()
    # 遍历各个节点
    for s in light_node:
        try:
            # 如果有一个节点订阅失败，则表示灯带异常
            data = rospy.wait_for_message(s[0], s[1], timeout=5)
            if not data:
                pd_list.append(False)
            else:
                pd_list.append(True)
        except Exception as e:
            logger.error(e)
            pd_list.append(False)
    if False in pd_list:
        return False
    return True


# 其他程序检测
def else_procedure():
    """
    机器人程序判断函数
    """
    process_list = [
        ["battery_driver.py", "电池驱动程序"],
        ["elevator_driver.py", "升降杆驱动程序"],
        ["motor_driver.py", "电机驱动程序"],
        # ["status_light.py", "灯带驱动程序"],
        ["robot_hmiScreen.py", "机器人主屏幕程序"],
        ["robot_wait.py", "机器人主程序"],
    ]  # 定义需要判断的程序列表
    # 遍历程序列表
    for process in process_list:
        cmd = "ps -ef | grep '{}' | grep 'python'".format(process[0])
        data = [i for i in os.popen(cmd)]
        # 如果进程存在则表示程序正常
        if len(data) > 1:
            logger.info("==={}检测正常===".format(process[1]))
            print("==={}检测正常===".format(process[1]))
        else:
            logger.error("==={}检测异常===".format(process[1]))
            print("==={}检测异常===".format(process[1]))
    # 灯带检测
    light_status = light_status_listen()
    if light_status:
        logger.info("===灯带驱动程序检测正常===")
        print("===灯带驱动程序检测正常===")
    else:
        logger.error("===灯带驱动程序检测异常===")
        print("===灯带驱动程序检测异常===")


# 服务检测函数
def is_open(ip, port):
    """
    根据socket，判断此ip和端口是否可以访问
    """
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
    # 遍历所有网络服务，并判断
    for server in server_data:
        pd = is_open(server[0], server[1])
        if pd:
            logger.info("==={}网络正常===".format(server[2]))
            print("==={}网络正常===".format(server[2]))
        else:
            logger.error("==={}网络异常===".format(server[2]))
            print("==={}网络异常===".format(server[2]))


if __name__ == '__main__':
    rospy.init_node('self_test')
    logger.info("=======================机器人网络检测开始=======================")
    print("=======================机器人网络检测开始=======================")
    server_test()
    logger.info("=======================机器人网络检测结束=======================")
    print("=======================机器人网络检测结束=======================")
    logger.info("=======================机器人程序检测开始=======================")
    print("=======================机器人程序检测开始=======================")
    sensor_test()
    camera_test()
    state_test()
    else_procedure()
    logger.info("=======================机器人程序检测结束=======================")
    print("=======================机器人程序检测结束=======================")




