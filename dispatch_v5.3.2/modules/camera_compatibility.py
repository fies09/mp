# -*- coding:utf-8 -*-
# @Author: WangLiang
# @file:  camera_compatibility.py
# @Time: 2021/04/19 16:14
# @Description: 工业相机兼容模块（海康、大恒）
import os
import sys
import time
sys.path.append(os.path.abspath("./modules/industry_daheng/"))
from configs import industrial_daheng_id
from configs.log import logger
from configs import industrial_camera_position
from modules.Industrial_camera.MvImport.MvCameraControl_class import *
from modules.robot_hardware_server import RobotDeviceStatus
try:
    from modules.industry_daheng.Industry_cam import DH_CAMERA
    import gxipy as gx
except:
    pass
try:
    from modules.Industrial_camera.industry_camera import steparm
except:
    pass


class Hikvision:
    def __init__(self):
        """
        初始化海康工业相机
        """
        try:
            self.image_upper = steparm(stDevice_position=1)
            self.image_lower = steparm(stDevice_position=0)
            upper_state = self.image_upper.set_acquisition_mode()
            lower_state = self.image_lower.set_acquisition_mode()
            # logger.info("上工业相机设置单触发模式")
            # logger.info(str(upper_state))
            # logger.info("下工业相机设置单触发模式")
            # logger.info(str(lower_state))
            # logger.info("上工业相机设置X镜像")
            upper_x_state = self.image_upper.setparm_Boolean("ReverseX", "true")
            # logger.info(str(upper_x_state))
            # logger.info("上工业相机设置Y镜像")
            upper_y_state = self.image_upper.setparm_Boolean("ReverseY", "true")
            # logger.info(str(upper_y_state))
            # logger.info("下工业相机设置X镜像")
            lower_x_state = self.image_lower.setparm_Boolean("ReverseX", "true")
            # logger.info(str(lower_x_state))
            # logger.info("下工业相机设置Y镜像")
            lower_y_state = self.image_lower.setparm_Boolean("ReverseY", "true")
            # logger.info(str(lower_y_state))
            self.modify_num = 0
            self.queue_flag = True
            self.image_upper_state = 0
            self.image_lower_state = 0
            # 初始化工业相机参数前置条件
            self.image_upper.setparm_Enum("ExposureAuto", 0)
            self.image_upper.setparm_Enum("GainAuto", 0)
            self.image_upper.setparm_Enum("BalanceWhiteAuto", 0)
            self.image_lower.setparm_Enum("ExposureAuto", 0)
            self.image_lower.setparm_Enum("GainAuto", 0)
            self.image_lower.setparm_Enum("BalanceWhiteAuto", 0)
        except Exception as e:
            logger.error(e)
            logger.error("海康工业相机初始化错误！")

    def get_camera_parameter(self, position):
        """
        获取海康相机参数
        :return:
        """
        use_status_obj = RobotDeviceStatus()
        # 初始化工业相机参数
        balance_ratio_range = [1, 4095]
        gain_range = [0, 20]
        exposure_time_range = [23, 2000000]
        dict_parm = {
            "ExposureTime": 50000,
            "Gain": 0,
            "BalanceRatio": 0,
            "Width": 4032,
            "Height": 3036,
            "range": {
                "ExposureTime": exposure_time_range,
                "Gain": gain_range,
                "BalanceRatio": balance_ratio_range
            },
            "default": {
                "ExposureTime": 100000,
                "Gain": 0,
                "BalanceRatio": 2000,
            }
        }
        try:
            if int(position) == industrial_camera_position.get("down"):
                # 开启下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", True)
                # ExposureAuto = self.image_lower.getparm_Enum("ExposureAuto")
                ExposureTime = self.image_lower.getparm_Float("ExposureTime")
                # GainAuto = self.image_lower.getparm_Enum("GainAuto")
                # GammaSelector = self.image_lower.getparm_Enum("GammaSelector")
                Gain = self.image_lower.getparm_Float("Gain")
                # Brightness = self.image_lower.getparm_int("Brightness")
                # GammaEnable = self.image_lower.getparm_Boolean("GammaEnable")
                # Gamma = self.image_lower.getparm_Float("Gamma")
                # BalanceWhiteAuto = self.image_lower.getparm_Enum("BalanceWhiteAuto")
                BalanceRatio = self.image_lower.getparm_int("BalanceRatio")
                Width = self.image_lower.getparm_int("Width")
                Height = self.image_lower.getparm_int("Height")
                # OffsetX = self.image_lower.getparm_int("OffsetX")
                # OffsetY = self.image_lower.getparm_int("OffsetY")
                # 关闭下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", False)
            else:
                # 开启上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", True)
                # ExposureAuto = self.image_upper.getparm_Enum("ExposureAuto")
                ExposureTime = self.image_upper.getparm_Float("ExposureTime")
                # GainAuto = self.image_upper.getparm_Enum("GainAuto")
                # GammaSelector = self.image_upper.getparm_Enum("GammaSelector")
                Gain = self.image_upper.getparm_Float("Gain")
                # Brightness = self.image_upper.getparm_int("Brightness")
                # GammaEnable = self.image_upper.getparm_Boolean("GammaEnable")
                # Gamma = self.image_upper.getparm_Float("Gamma")
                # BalanceWhiteAuto = self.image_upper.getparm_Enum("BalanceWhiteAuto")
                BalanceRatio = self.image_upper.getparm_int("BalanceRatio")
                Width = self.image_upper.getparm_int("Width")
                Height = self.image_upper.getparm_int("Height")
                # OffsetX = self.image_upper.getparm_int("OffsetX")
                # OffsetY = self.image_upper.getparm_int("OffsetY")
                # 关闭上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", False)
            dict_parm["ExposureTime"] = ExposureTime
            dict_parm["Gain"] = int(Gain) if Gain else 0
            dict_parm["BalanceRatio"] = BalanceRatio
            dict_parm["Width"] = Width
            dict_parm["Height"] = Height
        except Exception as e:
            logger.error(e)
            logger.error("海康工业相机参数获取错误！")
            # 关闭下工业相机占用
            use_status_obj.set_device_use_status("industry_camera_down", False)
            # 关闭上工业相机占用
            use_status_obj.set_device_use_status("industry_camera_up", False)
        return dict_parm

    def industry_photo(self, file_path, position):
        """
        海康工业相机拍照
        :param file_path:
        :param position:
        :return:
        """
        use_status_obj = RobotDeviceStatus()
        try:
            if int(position) == industrial_camera_position.get("down"):
                # 开启下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", True)
                logger.info("开启下工业相机流连接")
                self.image_lower.open_quliu()
                logger.info("下工业相机开始抓图")
                time.sleep(0.1)
                if not self.image_lower.grap_img(file_path):
                    time.sleep(0.1)
                    logger.info("开始重启下工业相机")
                    self.image_lower.setparm_Command()
                    time.sleep(20)
                    self.image_lower = steparm(stDevice_position=0)
                    self.modify_num = 0
                    return False
                logger.info("下工业相机抓图结束")
                logger.info("关闭下工业相机流连接")
                time.sleep(0.1)
                self.image_lower.close_quliu()
                # 关闭下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", False)
                return True
            else:
                # 开启上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", True)
                logger.info("开启上工业相机流连接")
                self.image_upper.open_quliu()
                logger.info("上工业相机开始抓图")
                logger.info(file_path)
                logger.info(type(file_path))
                time.sleep(0.1)
                if not self.image_upper.grap_img(file_path):
                    time.sleep(0.1)
                    logger.info("开始重启上工业相机")
                    self.image_upper.setparm_Command()
                    time.sleep(20)
                    self.image_upper = steparm(stDevice_position=1)
                    self.modify_num = 0
                    return False
                logger.info("上工业相机抓图结束")
                logger.info("关闭上工业相机流连接")
                time.sleep(0.1)
                self.image_upper.close_quliu()
                # 关闭上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", False)
                return True
        except Exception as e:
            logger.error(e)
            logger.error("海康工业相机拍照错误！")
            # 关闭下工业相机占用
            use_status_obj.set_device_use_status("industry_camera_down", False)
            # 关闭上工业相机占用
            use_status_obj.set_device_use_status("industry_camera_up", False)
            return False

    def modify_industry_parameter(self, position, parma, parma_value, parma_dict):
        """
        海康相机参数设置函数
        ExposureTime		前置条件：ExposureAuto = 0 成立 否则无效
        Gain                前置条件：GainAuto = 0 成立 否则无效
        Brightness          前置条件 ExposureAuto = 2 or GainAuto = 2
        Gamma               前置条件：Gamma = 0 成立 否则无效
        SensorShutterMode   前置条件 TriggerMode = 1 否则不显示
        BalanceRatio        前置条件：BalanceWhiteAuto = 0 成立 否则无效
        """
        use_status_obj = RobotDeviceStatus()
        try:
            setparm_Enum_list = ["ExposureAuto", "GainAuto", "BalanceWhiteAuto"]
            setparm_Float_list = ["ExposureTime", "Gain"]
            setparm_int_list = ["Width", "Height", "OffsetX", "OffsetY", "BalanceRatio"]
            logger.info(parma_dict)
            if int(position) == industrial_camera_position.get("down"):
                # 开启下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", True)
                if parma_dict:
                    for parma, parma_value in parma_dict.items():
                        logger.info("设置参数key--{}，value--{}".format(parma, parma_value))
                        try:
                            if parma in setparm_Enum_list:
                                logger.info("设置Enum类型")
                                self.image_lower.setparm_Enum(parma, int(parma_value))
                            elif parma in setparm_Float_list:
                                logger.info("设置float类型")
                                self.image_lower.setparm_Float(parma, float(parma_value))
                            elif parma in setparm_int_list:
                                self.image_lower.setparm_int(parma, int(parma_value))
                            elif parma == 'GammaEnable':
                                self.image_lower.setparm_Boolean(parma, int(parma_value))
                        except Exception as e:
                            logger.error(e)
                            # 关闭下工业相机占用
                            use_status_obj.set_device_use_status("industry_camera_down", False)
                            return False
                else:
                    try:
                        logger.info("设置参数key--{}，value--{}".format(parma, parma_value))
                        if parma in setparm_Enum_list:
                            logger.info("设置Enum类型")
                            self.image_lower.setparm_Enum(parma, int(parma_value))
                        elif parma in setparm_Float_list:
                            logger.info("设置float类型")
                            self.image_lower.setparm_Float(parma, float(parma_value))
                        elif parma in setparm_int_list:
                            self.image_lower.setparm_int(parma, int(parma_value))
                        elif parma == 'GammaEnable':
                            self.image_lower.setparm_Boolean(parma, int(parma_value))
                    except Exception as e:
                        logger.error(e)
                        # 关闭下工业相机占用
                        use_status_obj.set_device_use_status("industry_camera_down", False)
                        return False
                # 关闭下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", False)
            else:
                # 开启上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", True)
                if parma_dict:
                    for parma, parma_value in parma_dict.items():
                        logger.info("设置参数key--{}，value--{}".format(parma, parma_value))
                        try:
                            if parma in setparm_Enum_list:
                                self.image_upper.setparm_Enum(parma, int(parma_value))
                            elif parma in setparm_Float_list:
                                logger.info("设置float类型")
                                self.image_upper.setparm_Float(parma, float(parma_value))
                            elif parma in setparm_int_list:
                                self.image_upper.setparm_int(parma, int(parma_value))
                            elif parma == 'GammaEnable':
                                self.image_upper.setparm_Boolean(parma, int(parma_value))
                        except Exception as e:
                            logger.error(e)
                            # 关闭上工业相机占用
                            use_status_obj.set_device_use_status("industry_camera_up", False)
                            return False
                else:
                    try:
                        logger.info("设置参数key--{}，value--{}".format(parma, parma_value))
                        if parma in setparm_Enum_list:
                            self.image_upper.setparm_Enum(parma, int(parma_value))
                        elif parma in setparm_Float_list:
                            logger.info("设置float类型")
                            self.image_upper.setparm_Float(parma, float(parma_value))
                        elif parma in setparm_int_list:
                            self.image_upper.setparm_int(parma, int(parma_value))
                        elif parma == 'GammaEnable':
                            self.image_upper.setparm_Boolean(parma, int(parma_value))
                    except Exception as e:
                        logger.error(e)
                        # 关闭上工业相机占用
                        use_status_obj.set_device_use_status("industry_camera_up", False)
                        return False
                # 关闭上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", False)
        except Exception as e:
            logger.error(e)
            logger.error("海康工业相机参数设置错误！")
            # 关闭下工业相机占用
            use_status_obj.set_device_use_status("industry_camera_down", False)
            # 关闭上工业相机占用
            use_status_obj.set_device_use_status("industry_camera_up", False)
        return True


class Daheng:
    def __init__(self):
        """
        初始化大恒工业相机
        """
        try:
            self.cam_lower = self.cam_upper = DH_CAMERA()
            self.BalanceRatio = 0
            self.ExposureTime = 0
            self.Gain = 0
            self.id_list = []
        except Exception as e:
            logger.error(e)
            logger.error("大恒工业相机初始化错误！")

    def get_camera_parameter(self, position):
        """
        获取大恒相机参数
        :return:
        """
        use_status_obj = RobotDeviceStatus()
        # 初始化工业相机参数
        balance_ratio_range = [1, 7.9961]
        gain_range = [0, 24]
        exposure_time_range = [20, 1000000]
        dict_parm = {
            "ExposureTime": 40000,
            "Gain": 0,
            "BalanceRatio": 0,
            "Width": 3840,
            "Height": 2160,
            "range": {
                "ExposureTime": exposure_time_range,
                "Gain": gain_range,
                "BalanceRatio": balance_ratio_range
            },
            "default": {
                "ExposureTime": 100000,
                "Gain": 0,
                "BalanceRatio": 1,
            }
        }
        try:
            if int(position) == industrial_camera_position.get("down"):
                # 开启下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", True)
                self.cam_lower.open_camera('sn', industrial_daheng_id.get("lower_sn_id"))
                exposure_time, gain, balance = self.cam_lower.get_camera_parameter()
                self.cam_lower.close_camera()
                self.cam_lower = DH_CAMERA()
                # 关闭下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", False)
            else:
                # 开启上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", True)
                self.cam_upper.open_camera('sn', industrial_daheng_id.get("upper_sn_id"))
                exposure_time, gain, balance = self.cam_upper.get_camera_parameter()
                self.cam_upper.close_camera()
                self.cam_upper = DH_CAMERA()
                # 关闭上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", False)
            dict_parm["ExposureTime"] = exposure_time
            dict_parm["Gain"] = gain
            dict_parm["BalanceRatio"] = balance
        except Exception as e:
            if int(position) == industrial_camera_position.get("down"):
                # self.cam_lower.close_camera()
                self.cam_lower = DH_CAMERA()
                # 关闭下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", False)
            else:
                # self.cam_upper.close_camera()
                self.cam_lower = DH_CAMERA()
                # 关闭上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", False)
            logger.error(e)
        return dict_parm

    def industry_photo(self, file_path, position):
        """
        大恒工业相机拍照
        :param file_path:
        :param position:
        :return:
        """
        use_status_obj = RobotDeviceStatus()
        try:
            if int(position) == industrial_camera_position.get("down"):
                # 开启下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", True)
                logger.info("开启下工业相机句柄")
                self.cam_lower.open_camera('sn', industrial_daheng_id.get("lower_sn_id"))
                logger.info("开启下工业相机流连接")
                self.cam_lower.get_stream()
                logger.info("下工业相机开始抓图")
                logger.info(file_path)
                time.sleep(0.3)
                image = self.cam_lower.take_a_picture(0)
                if image:
                    image.save(file_path)
                else:
                    logger.error("下工业相机抓图{}失败".format(file_path))
                logger.info("关闭下工业相机流连接")
                self.cam_lower.close_stream()
                self.cam_lower.close_camera()
                logger.info("下工业相机抓图结束")
                # 关闭下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", False)
                return True
            else:
                # 开启上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", True)
                logger.info("开启上工业相机句柄")
                self.cam_upper.open_camera('sn', industrial_daheng_id.get("upper_sn_id"))
                logger.info("开启上工业相机流连接")
                self.cam_upper.get_stream()
                logger.info("上工业相机开始抓图")
                logger.info(file_path)
                time.sleep(0.3)
                image = self.cam_upper.take_a_picture(0)
                if image:
                    image.save(file_path)
                else:
                    logger.error("上工业相机抓图{}失败".format(file_path))
                logger.info("关闭上工业相机流连接")
                self.cam_upper.close_stream()
                self.cam_upper.close_camera()
                logger.info("上工业相机抓图结束")
                # 关闭上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", False)
                return True
        except Exception as e:
            if int(position) == industrial_camera_position.get("down"):
                # self.cam_lower.close_camera()
                self.cam_lower = DH_CAMERA()
                # 关闭下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", False)
            else:
                # self.cam_upper.close_camera()
                self.cam_upper = DH_CAMERA()
                # 关闭上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", False)
            logger.error(e)
            return False

    def modify_industry_parameter(self, position, parma, parma_value, parma_dict):
        """
        大恒相机参数设置函数
        ExposureTime		前置条件：ExposureAuto = 0 成立 否则无效
        Gain                前置条件：GainAuto = 0 成立 否则无效
        Brightness          前置条件 ExposureAuto = 2 or GainAuto = 2
        Gamma               前置条件：Gamma = 0 成立 否则无效
        SensorShutterMode   前置条件 TriggerMode = 1 否则不显示
        BalanceRatio        前置条件：BalanceWhiteAuto = 0 成立 否则无效
        """
        use_status_obj = RobotDeviceStatus()
        try:
            setparm_Enum_list = ["BalanceRatio"]
            setparm_Float_list = ["ExposureTime"]
            setparm_int_list = ["Gain"]
            logger.info(parma_dict)
            if int(position) == industrial_camera_position.get("down"):
                # 开启下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", True)
                logger.info("开启下工业相机句柄")
                self.cam_lower.open_camera('sn', industrial_daheng_id.get("lower_sn_id"))
                if parma_dict:
                    for parma, parma_value in parma_dict.items():
                        logger.info("设置参数key--{}，value--{}".format(parma, parma_value))
                        try:
                            if parma in setparm_Enum_list:
                                self.BalanceRatio = parma_value
                                self.cam_lower.set_white_balance(float(parma_value))
                            elif parma in setparm_Float_list:
                                self.ExposureTime = parma_value
                                self.cam_lower.set_exposure_time(float(parma_value))
                            elif parma in setparm_int_list:
                                self.Gain = parma_value
                                self.cam_lower.set_gain(float(parma_value))
                        except Exception as e:
                            self.cam_lower.close_camera()
                            logger.error(e)
                            # 关闭下工业相机占用
                            use_status_obj.set_device_use_status("industry_camera_down", False)
                            return False
                else:
                    try:
                        logger.info("设置参数key--{}，value--{}".format(parma, parma_value))
                        if parma in setparm_Enum_list:
                            self.cam_lower.set_white_balance(float(parma_value))
                        elif parma in setparm_Float_list:
                            self.cam_lower.set_exposure_time(float(parma_value))
                        elif parma in setparm_int_list:
                            self.cam_lower.set_gain(float(parma_value))
                    except Exception as e:
                        self.cam_lower.close_camera()
                        logger.error(e)
                        # 关闭下工业相机占用
                        use_status_obj.set_device_use_status("industry_camera_down", False)
                        return False
                self.cam_lower.close_camera()
                # 关闭下工业相机占用
                use_status_obj.set_device_use_status("industry_camera_down", False)
            else:
                # 开启上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", True)
                logger.info("开启上工业相机句柄")
                self.cam_upper.open_camera('sn', industrial_daheng_id.get("upper_sn_id"))
                if parma_dict:
                    for parma, parma_value in parma_dict.items():
                        logger.info("设置参数key--{}，value--{}".format(parma, parma_value))
                        try:
                            if parma in setparm_Enum_list:
                                self.BalanceRatio = parma_value
                                self.cam_upper.set_white_balance(float(parma_value))
                            elif parma in setparm_Float_list:
                                self.ExposureTime = parma_value
                                self.cam_upper.set_exposure_time(float(parma_value))
                            elif parma in setparm_int_list:
                                self.Gain = parma_value
                                self.cam_upper.set_gain(float(parma_value))
                        except Exception as e:
                            self.cam_upper.close_camera()
                            logger.error(e)
                            # 关闭上工业相机占用
                            use_status_obj.set_device_use_status("industry_camera_up", False)
                            return False
                else:
                    try:
                        logger.info("设置参数key--{}，value--{}".format(parma, parma_value))
                        if parma in setparm_Enum_list:
                            self.cam_upper.set_white_balance(float(parma_value))
                        elif parma in setparm_Float_list:
                            self.cam_upper.set_exposure_time(float(parma_value))
                        elif parma in setparm_int_list:
                            self.cam_upper.set_gain(float(parma_value))
                    except Exception as e:
                        self.cam_upper.close_camera()
                        logger.error(e)
                        # 关闭上工业相机占用
                        use_status_obj.set_device_use_status("industry_camera_up", False)
                        return False
                self.cam_upper.close_camera()
                # 关闭上工业相机占用
                use_status_obj.set_device_use_status("industry_camera_up", False)
        except Exception as e:
            logger.error(e)
            # 关闭上下工业相机占用
            use_status_obj.set_device_use_status("industry_camera_up", False)
            use_status_obj.set_device_use_status("industry_camera_down", False)
            logger.error("大恒工业相机参数设置错误！")
        return True


class FindDevice:
    def __init__(self):
        logger.info("========== 开始寻找工业相机设备 ==========")
        # ==================== 查找海康设备 ====================
        try:
            cam = MvCamera()
            deviceList = MV_CC_DEVICE_INFO_LIST()
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
            cam.MV_CC_EnumDevices(tlayerType, deviceList)
            self.hk_device_num = deviceList.nDeviceNum
        except:
            logger.error("未安装海康相机驱动！")
        cam = MvCamera()
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        cam.MV_CC_EnumDevices(tlayerType, deviceList)
        self.hk_device_num = deviceList.nDeviceNum
        # ==================== 查找大恒设备 ====================
        try:
            device_manager = gx.DeviceManager()
            self.dh_device_num, self.dev_info_list = device_manager.update_device_list()
        except:
            logger.error("未安装大恒相机驱动！")
            self.dh_device_num = False
        device_manager = gx.DeviceManager()
        self.dh_device_num, self.dev_info_list = device_manager.update_device_list()

    def start(self):
        logger.info("扫描结束......")
        if self.hk_device_num:
            logger.info("共找到 {} 个海康工业相机设备".format(self.hk_device_num))
            return {"Hikvision": True}
        elif self.dh_device_num:
            logger.info("共找到 {} 个大恒工业相机设备".format(self.dh_device_num))
            return {"Daheng": True}
        else:
            logger.info("未找到工业相机设备！")
            return {}


if __name__ == '__main__':
    FindDevice().start()
