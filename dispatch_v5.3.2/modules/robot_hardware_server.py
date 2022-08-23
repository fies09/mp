#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/7 14:49
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : robot_hardware_server.py
# @description: "机器人硬件服务调用模块"
import sys
import json
import time
import requests
from configs.log import logger
RobotMoveUrl = "http://127.0.0.1:8088/robot_move"
RobotPtzUrl = "http://127.0.0.1:8088/ptz"
RobotThermalImageryUrl = "http://127.0.0.1:8088/robot_thermal_imagery"
RobotLifterUrl = "http://127.0.0.1:8088/elevator"
from modules.robot_move import RobotMove
from modules.robot_lifter import RobotLifer
from modules.robot_take_picture import FourCamera
from modules.robot_transducer import RobotTransducer
from lib.hikSDK import NET_DVR_PTZPreset_Other
from lib.hikSDK import NET_DVR_Login_V30
from modules.robot_take_picture import ThermalImagery
from core.redis_interactive import RedisPipe
from task import hardware_exception
from schema.db_alarm_manage import DBAlarmManage
from configs import db_rabbit_mq
from core.rabbitmq_db import RabbitPublisher

class RobotDeviceStatus:
    """
    机器人硬件状态
    """
    def __init__(self):
        # logger.info("初始化硬件状态......")
        # 初始化硬件可用状态（True为可用，False为不可用）
        self.redis_device_status = RedisPipe("device_status")
        now_status_data = self.redis_device_status.get_data()
        if now_status_data:
            self.device_status = eval(now_status_data)
            # logger.info(self.device_status)
        else:
            self.device_status = {
                "temhum": True,
                "noise": True,
                "battery": True,
                "mp_all": True,
                "sdioxide": True,
                "hsulfide": True,
                "ptz_camera": True,
                "fourfold_up": True,
                "fourfold_down": True,
                "scan": True,
                "IMU": True,
                "fall": True,
                "avoidance": True,
                "move": True,
                "ptz": True,
                "thermal_imagery": True,
                "industry_camera_up": True,
                "industry_camera_down": True,
                "lifter": True,
                "voice": True,
            }
            self.redis_device_status.set_data(self.device_status)
        # 初始化设备占用状态（True为被占用，False为未被占用）
        self.redis_device_use_status = RedisPipe("device_use_status")
        now_use_status_data = self.redis_device_use_status.get_data()
        if now_use_status_data:
            self.device_use_status = eval(now_use_status_data)
        else:
            self.device_use_status = {
                "move": False,
                "ptz": False,
                "thermal_imagery": False,
                "camera": False,
                "industry_camera_up": False,
                "industry_camera_down": False,
                "lifter": False,
                "voice": False
            }
            self.redis_device_use_status.set_data(self.device_use_status)

    def init_device(self):
        """
        状态值初始化
        """
        device_status = {
            "temhum": True,
            "noise": True,
            "battery": True,
            "mp_all": True,
            "sdioxide": True,
            "hsulfide": True,
            "ptz_camera": True,
            "fourfold_up": True,
            "fourfold_down": True,
            "scan": True,
            "IMU": True,
            "fall": True,
            "avoidance": True,
            "move": True,
            "ptz": True,
            "thermal_imagery": True,
            "industry_camera_up": True,
            "industry_camera_down": True,
            "lifter": True,
            "voice": True,
        }
        self.redis_device_status.set_data(device_status)
        device_use_status = {
            "move": False,
            "ptz": False,
            "thermal_imagery": False,
            "camera": False,
            "industry_camera_up": False,
            "industry_camera_down": False,
            "lifter": False,
            "voice": False
        }
        self.redis_device_use_status.set_data(device_use_status)

    def set_device_status(self, device=None, value=None, all_device=False):
        """
        设置设备可用状态
        """
        try:
            self.device_status = eval(self.redis_device_status.get_data())
            if all_device:
                for device_key in self.device_status.keys():
                    self.device_status[device_key] = value
            else:
                self.device_status[device] = value
            self.redis_device_status.set_data(self.device_status)
            return True
        except Exception as e:
            logger.error(e)
            logger.error("设备可用状态设置错误")

    def get_device_status(self, device=None, all_device=False):
        """
        获取设备可用状态
        """
        try:
            self.device_status = eval(self.redis_device_status.get_data())
            if all_device:
                return self.device_status
            else:
                return self.device_status.get(device)
        except Exception as e:
            logger.error(e)
            logger.error("设备可用状态获取错误")
            return {}

    def set_device_use_status(self, device=None, value=None, all_device=False):
        """
        设置设备使用状态
        """
        try:
            self.device_use_status = eval(self.redis_device_use_status.get_data())
            if all_device:
                for device_key in self.device_use_status.keys():
                    self.device_use_status[device_key] = value
            else:
                self.device_use_status[device] = value
            self.redis_device_use_status.set_data(self.device_use_status)
            return True
        except Exception as e:
            logger.error(e)
            logger.error("设备使用状态设置错误")

    def get_device_use_status(self, device=None, all_device=False):
        """
        获取设备使用状态
        """
        try:
            self.device_use_status = eval(self.redis_device_use_status.get_data())
            if all_device:
                return self.device_use_status
            else:
                return self.device_use_status.get(device)
        except Exception as e:
            logger.error(e)
            logger.error("设备使用状态获取错误")
            return {}


def robot_move(position, move_type):
    """
    机器人移动服务
    """
    # 创建移动对象
    try:
        while True:
            # 获取占用状态
            use_status_obj = RobotDeviceStatus()
            device_use_status = use_status_obj.get_device_use_status("move")
            # logger.info(device_use_status)
            # 设备被占用
            if device_use_status:
                # logger.info("移动服务被占用")
                time.sleep(1)
                continue
            else:
                # 更新占用状态
                use_status_obj.set_device_use_status("move", True)
                # 移动
                move_obj = RobotMove(position, move_type)
                move_status = move_obj.start()
                # 更新占用状态
                use_status_obj.set_device_use_status("move", False)
                return move_status
    except Exception as e:
        logger.error(e)
        logger.error("机器人移动服务错误")
        # 更新占用状态
        use_status_obj = RobotDeviceStatus()
        use_status_obj.set_device_use_status("move", False)
        return False


def robot_ptz(method, ptz_val):
    """
    机器人云台服务
    """
    try:
        while True:
            # 获取占用状态
            use_status_obj = RobotDeviceStatus()
            device_use_status = use_status_obj.get_device_use_status("ptz")
            # 设备被占用
            if device_use_status:
                logger.info("云台服务被占用")
                time.sleep(1)
                continue
            else:
                # 更新占用状态
                use_status_obj.set_device_use_status("ptz", True)
                NET_DVR_Login_V30("192.168.10.210", 8000, "admin", "Admin123")
                if method == "G":
                    result = NET_DVR_PTZPreset_Other().get_PTZPreset(39, int(ptz_val))
                elif method == "C":
                    result = NET_DVR_PTZPreset_Other().get_PTZPreset(9, int(ptz_val))
                else:
                    result = NET_DVR_PTZPreset_Other().get_PTZPreset(8, int(ptz_val))
                # 更新占用状态
                use_status_obj.set_device_use_status("ptz", False)
                if result:
                    return True
                else:
                    return False
    except Exception as e:
        logger.error(e)
        logger.error("机器人云台服务错误")
        # 更新占用状态
        use_status_obj = RobotDeviceStatus()
        use_status_obj.set_device_use_status("ptz", False)
        return False


def robot_thermal_imagery(ip):
    """
    机器人热成像服务
    """
    try:
        while True:
            # 获取占用状态
            use_status_obj = RobotDeviceStatus()
            device_use_status = use_status_obj.get_device_use_status("thermal_imagery")
            # 设备被占用
            if device_use_status:
                logger.info("热成像服务被占用")
                time.sleep(1)
                continue
            else:
                # 更新占用状态
                use_status_obj.set_device_use_status("thermal_imagery", True)
                thermal_imagery = ThermalImagery(ip)
                file_path = thermal_imagery.take_picture()
                result = thermal_imagery.get_results()
                # 更新占用状态
                use_status_obj.set_device_use_status("thermal_imagery", False)
                return file_path, result
    except Exception as e:
        logger.error(e)
        logger.error("机器人热成像服务错误")
        # 更新占用状态
        use_status_obj = RobotDeviceStatus()
        use_status_obj.set_device_use_status("thermal_imagery", False)
        return "", {}


def robot_lifter(move_type:int, position:int):
    """
    机器人升降杆服务
    move_type: 1:绝对;2:相对
    position: 高度
    """
    try:
        while True:
            # 获取占用状态
            use_status_obj = RobotDeviceStatus()
            device_use_status = use_status_obj.get_device_use_status("lifter")
            # 设备被占用
            if device_use_status:
                logger.info("升降杆服务被占用")
                time.sleep(1)
                continue
            else:
                # 更新占用状态
                use_status_obj.set_device_use_status("lifter", True)
                status, now_position = RobotLifer(move_type, position).start()
                use_status_obj.set_device_use_status("lifter", False)
                return status, now_position
    except Exception as e:
        logger.error(e)
        logger.error("机器人升降杆服务错误")
        # 更新占用状态
        use_status_obj = RobotDeviceStatus()
        use_status_obj.set_device_use_status("lifter", False)
        return False, 0


def robot_four_camera(ip):
    """
        四倍相机，云台25倍拍照服务
    """
    try:
        while True:
            if ip == "192.168.10.212":
                camera = "fourfold_up"
            else:
                camera = "fourfold_down"
            # 获取占用状态
            use_status_obj = RobotDeviceStatus()
            device_use_status = use_status_obj.get_device_use_status(camera)
            # 设备被占用
            if device_use_status:
                logger.info("相机服务被占用")
                time.sleep(1)
                continue
            else:
                # 更新占用状态
                use_status_obj.set_device_use_status("camera", True)
                file_path = FourCamera(ip).start()
                use_status_obj.set_device_use_status("camera", False)
                return file_path
    except Exception as e:
        logger.error(e)
        logger.error("机器人相机服务错误")
        # 更新占用状态
        use_status_obj = RobotDeviceStatus()
        use_status_obj.set_device_use_status("fourfold_up", False)
        use_status_obj.set_device_use_status("fourfold_down", False)
        return ""


if __name__ == '__main__':
    # robot_move({"x": "-0.15", "y": "0.02", "yaw": "-0.08"},"back_charging")
    # robot_ptz("G", 1)
    # robot_thermal_imagery("192.168.10.230", "/opt/moss_robot/lib/dispatch_v5.3/test/")
    # robot_lifter(2, -50)

    pass
