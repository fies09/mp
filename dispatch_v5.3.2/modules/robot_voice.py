#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/7 9:35
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : robot_voice.py
# @description: "机器人语音服务"
import sys
import json
import rospy
from std_msgs.msg import String
from mprobot_msgs.srv import GeneralService, GeneralServiceRequest
sys.path.append("/opt/moss_robot/lib/dispatch_ips_web")
from configs.log import logger


class RobotVoice:
    def __init__(self, text):
        self.text = text
        pass

    def start(self):
        try:
            rospy.wait_for_service('/voice_tts_node/voice_tts')
            voice_start = rospy.ServiceProxy('/voice_tts_node/voice_tts', GeneralService)
            req = GeneralServiceRequest()
            params = {}
            params['text'] = self.text
            req.request = json.dumps(params)
            res = voice_start.call(req)
            logger.info(res.response)
            if json.loads(res.response).get("status"):
                return True
        except Exception as e:
            logger.error(e)
            logger.error("语音错误")
        return False

    def voice_stop(self):
        rospy.wait_for_service('/voice_manager/voiceIntercom_stop')
        voice_stop = rospy.ServiceProxy('/voice_manager/voiceIntercom_stop', GeneralService)
        req = GeneralServiceRequest()

        params = {}
        req.request = json.dumps(params)
        res = voice_stop.call(req)
        logger.info(res.response)


if __name__ == '__main__':
    rospy.init_node('mp_voice11111', anonymous=True)
    robot_voice = RobotVoice("123456")
    robot_voice.voice_stop()
    robot_voice.start()

