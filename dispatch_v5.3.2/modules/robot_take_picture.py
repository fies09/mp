#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/6 16:39
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : robot_take_picture.py
# @description: "机器人拍照服务"
import os
import sys
import uuid
import urllib
import requests
from requests.auth import HTTPDigestAuth

sys.path.append("/opt/moss_robot/lib/dispatch_ips_web")
from configs.log import logger
from configs import hot_camera
from modules.ptz_client import hk_hotcamera_client


class ThermalImagery:
    """
    热成像拍照服务
    """

    def __init__(self):
        pass

    def take_picture(self):
        """
        热成像抓图
        """
        if hot_camera.get("hot_camera_type") == "1":  # 大力热成像
            return DaliHotCamera().dali_hotcamera_take_picture()

        elif hot_camera.get("hot_camera_type") == "2":  # 海康热成像
            return HKHotCamera().hk_hotcamera_take_picture()

        else:
            logger.error("热成像配置错误")
            return None

    def get_results(self):
        """
        获取热成像结果
        """
        if hot_camera.get("hot_camera_type") == "1":  # 大力热成像
            return DaliHotCamera().dali_hotcamera_get_results()

        elif hot_camera.get("hot_camera_type") == "2":  # 海康热成像
            return HKHotCamera().hk_hotcamera_get_results()

        else:
            logger.error("热成像配置错误")
            return None


class DaliHotCamera(object):
    def __init__(self):

        self.photo_path = "./data/ther_path"
        self.headers = {
            "User-Agent": "Mozilla/5.0 (Linux; Android 6.0; Nexus 5 Build/MRA58N) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/66.0.3359.139 Mobile Safari/537.36",
            "Cookie": "Language=zh-cn; DevType=DM20; OEMINFO=OEM; Name=admin; Pass=Admin123; UserType=Admin; IPCamera=DM20"
        }

    def dali_hotcamera_take_picture(self):
        """
        热成像拍照(大力热成像)
        """
        try:
            url = "http://127.0.0.1:5000/stream?Type=JPEG&Source=Jpeg&Mode=TCP&Heart-beat=No&Frames=1&Snap=Yes&Channel=0"
            response = requests.get(url, self.headers)
            if not os.path.exists(self.photo_path):
                os.makedirs(self.photo_path)
            if response.status_code == 200:
                file_path = self.photo_path + str(uuid.uuid1()) + '.jpg'
                with open(file_path, 'wb') as f:
                    f.write(response.content)
                    return file_path

        except Exception as e:
            logger.error(e)
            logger.error("热成像拍照错误")
        return None

    def dali_hotcamera_get_results(self):
        """
        获取热成像结果(大力热成像)
        """
        try:
            url = "http://127.0.0.1/cgi-bin/system?Module=DMTtl"
            response = requests.get(url, headers=self.headers)
            temp_info = urllib.parse.unquote(response.text)
            Area = temp_info.split('&')[1]
            Area_list = Area.split('|')
            temp_list = Area_list[1].split(',')
            res = {"high_temp": temp_list[0], "mini_temp": temp_list[1], "aver_temp": temp_list[2]}
            return res
        except Exception as e:
            logger.error(e)
            logger.error("热成像结果识别错误")
        return {}


class HKHotCamera(object):
    def hk_hotcamera_take_picture(self):
        """
        海康热成像抓图
        """
        result = hk_hotcamera_client(req_type="pic")
        logger.info("海康热成像结果: {}".format(result))
        if result == None:
            return None

        return result["picture_file"]

    def hk_hotcamera_get_results(self):
        """
        海康热成像测温
        """
        result = hk_hotcamera_client(req_type="temp")
        logger.info("海康热成像结果: {}".format(result))
        if result == None:
            return None

        if len(result) != 4:
            logger.error("海康热成像测温失败")

        res = {
            "high_temp": round(float(result["temperature"][0]), 2),
            "mini_temp": round(float(result["temperature"][1]), 2),
            "aver_temp": round(float(result["temperature"][2]), 2),
        }

        return res


class FourCamera:
    def __init__(self, ip, photo_path="./data/four_photo", username='admin', password='Admin123'):
        self.ip = ip
        self.photo_path = photo_path
        self.username = username
        self.password = password

    def start(self):
        try:
            url = "http://{}:80/ISAPI/Streaming/channels/1/picture".format(self.ip)
            auth = HTTPDigestAuth(self.username, self.password)
            print(auth)
            response = requests.get(url, auth=auth)
            print(response)
            if not os.path.exists(self.photo_path):
                os.makedirs(self.photo_path)
            if response.status_code == 200:
                file_path = self.photo_path + str(uuid.uuid1()) + '.jpg'
                with open(file_path, 'wb') as f:
                    f.write(response.content)
                    return file_path
        except Exception as e:
            logger.error(e)
            logger.error("相机拍照错误")
        return ""


if __name__ == '__main__':
    # 热成像测试
    ThermalImagery("192.168.10.230", "/opt/moss_robot/lib/dispatch_v5.3/test/test001.jpg").start()
