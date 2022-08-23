#!/usr/bin/env python3
# coding:utf-8
import json

import rospy
import time
from std_msgs.msg import Int8
from mprobot_msgs.srv import GeneralService, GeneralServiceRequest

from configs.log import logger


def voice_now():
    try:
        voice_status_data = rospy.wait_for_message('/voice_status', Int8, timeout=5)
        print(voice_status_data)
    except:
        return False
    if voice_status_data.data != 0:
        return True
    else:
        return False


def voice_start():
    rospy.wait_for_service('/voice_manager/voiceIntercom_start')
    voice_start = rospy.ServiceProxy('/voice_manager/voiceIntercom_start', GeneralService)
    req = GeneralServiceRequest()

    params = {}
    params['serverIp'] = '192.168.10.100'
    params['userName'] = 'robotlinux'
    params['roomId'] = 2
    # print(params)
    logger.info(params)

    req.request = json.dumps(params)
    res = voice_start.call(req)
    logger.info(res.response)

    # print(json.loads(res.response))


def voice_stop():
    rospy.wait_for_service('/voice_manager/voiceIntercom_stop')
    voice_stop = rospy.ServiceProxy('/voice_manager/voiceIntercom_stop', GeneralService)
    req = GeneralServiceRequest()

    params = {}
    req.request = json.dumps(params)
    res = voice_stop.call(req)
    logger.info(res.response)


if __name__ == '__main__':
    rospy.init_node('kjdsnvlk')
    voice_now()
    while 1:
        time.sleep(0.3)

    # while 1:
    #     print(obstacle())
    #     time.sleep(3)
