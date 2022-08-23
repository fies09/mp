#!/usr/bin/env python3
# -*-coding: UTF-8 -*-
import json
import os
import sys
import time
from std_msgs.msg import String
from mprobot_msgs.srv import GeneralService, GeneralServiceRequest
sys.path.append("/opt/moss_robot/lib/dispatch_ips_web")
import rospy
import traceback
from task import hardware_exception
# 导入mysql告警管理信息
from schema.db_alarm_manage import DBAlarmManage
from configs import db_rabbit_mq
from core.rabbitmq_db import RabbitPublisher
from modules.robot_hardware_server import RobotDeviceStatus

# def voice_client(data):
#     pub = rospy.Publisher('/mp_voice/xf_tts', String, queue_size=10)  # 发布消息到话题 chatter 中,队列长度10
#     # rate.sleep()  # 配合发布频率的休眠
#     time.sleep(0.5)
#     rate = rospy.Rate(1)  # 1hz
#     pub.publish(data)
#     rate.sleep()
from configs.log import logger


def voice_client(data):
    try:
        rospy.wait_for_service('/voice_tts_node/voice_tts')
        voice_start = rospy.ServiceProxy('/voice_tts_node/voice_tts', GeneralService)
        req = GeneralServiceRequest()
    except Exception as e:
        logger.error(e)
        logger.error("ROS语音调用错误")
        # 更新占用状态
        use_status_obj=RobotDeviceStatus()
        use_status_obj.set_device_use_status("voice",False)
        return False


    params = {}
    params['text'] = data
    req.request = json.dumps(params)
    res = voice_start.call(req)
    logger.info(res.response)
    # print(json.loads(res.response))


# def voice_start():
#     rospy.wait_for_service('/voice_tts_node/voice_tts')
#     voice_start = rospy.ServiceProxy('/voice_tts_node/voice_tts', GeneralService)
#     req = GeneralServiceRequest()
#
#     params = {}
#     params['text'] = '在工业机器人领域，我国约占全球市场份额的三分之一，是全球第一大工业机器人应用市场。2017年，我国工业机器人保持高速增长，销量同比增长30%。按照应用类型分，2017年国内市场的搬运上下料机器人占比最高，达65%'
#     req.request = json.dumps(params)
#     res = voice_start.call(req)
#     print(json.loads(res.response))


def voice_start():
    try:
        rospy.wait_for_service('/voice_manager/voiceIntercom_start')
        voice_start = rospy.ServiceProxy('/voice_manager/voiceIntercom_start', GeneralService)
        req = GeneralServiceRequest()
    except Exception as e:
        logger.error(e)
        logger.error("ROS语音调用错误")
        # 更新占用状态
        use_status_obj=RobotDeviceStatus()
        use_status_obj.set_device_use_status("voice",False)
       
    params = {}
    params['serverIp'] = '192.168.10.100'
    params['userName'] = 'robotlinux'
    params['roomId'] =2
    # print(params)
    logger.info(params)

    req.request = json.dumps(params)
    res = voice_start.call(req)
    logger.info(res.response)

    # print(json.loads(res.response))


def voice_stop():
    try:
        rospy.wait_for_service('/voice_manager/voiceIntercom_stop')
        voice_stop = rospy.ServiceProxy('/voice_manager/voiceIntercom_stop', GeneralService)
        req = GeneralServiceRequest()
    except Exception as e:
        logger.error(e)
        logger.error("ROS语音调用错误")
        # 更新占用状态
        use_status_obj=RobotDeviceStatus()
        use_status_obj.set_device_use_status("voice",False)
    params = {}
    req.request = json.dumps(params)
    res = voice_stop.call(req)
    logger.info(res.response)

    # print(json.loads(res.response))


if __name__ == '__main__':
    rospy.init_node('mp_voice11111', anonymous=True)
    voice_stop()
    voice_client(
        "在工业机器人领域，我国约占全球市场份额的三分之一，是全球第一大工业机器人应用市场。2017年，我国工业机器人保持高速增长，销量同比增长30%。按照应用类型分，2017年国内市场的搬运上下料机器人占比最高，达65%")

    # data_dic = {"robotId": 1, "voiceState": 1, "voiceAction": 0}
    # rout = db_rabbit_mq.get("rout_ex")
    # routing_ey = db_rabbit_mq.get("rout_send_robotVoice")
    # warn_queue = db_rabbit_mq.get("queue_send_robotVoice")
    # logger.info("发送打断信息MQ数据！")
    # RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
    # time.sleep(10)
    # voice_start()
    # data_dic = {"robotId": 1, "voiceState": 0, "voiceAction": 1}
    # rout = db_rabbit_mq.get("rout_ex")
    # routing_ey = db_rabbit_mq.get("rout_send_robotVoice")
    # warn_queue = db_rabbit_mq.get("queue_send_robotVoice")
    # logger.info("推送结束语音信息MQ数据！")
    # RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
    #

