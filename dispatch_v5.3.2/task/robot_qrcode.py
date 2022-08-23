# -*- coding: utf-8 -*-
import os
import time
from core.queue_config import err_qrcocde_quene, check_qrcode_quene
from datetime import datetime
from configs.log import logger
from configs import db_rabbit_mq
from utils.file_path import Path
from task.blend import lightBlend
from schema.db_robot import DBRobot
from core.rabbitmq_db import RabbitPublisher
from task.robot_algorithm import QrcodeIdentify
from schema.db_robot_device import DBRobotDevice
from schema.db_alarm_manage import DBAlarmManage
from schema.db_robot_device_param import DBRobotDeviceParam
from schema.db_core_inventory_type import DBCoreInventoryType
from schema.db_core_inventory_data import DBCoreInventoryDatum
from schema.db_robot_elevator_param import DBRobotElevatorParam
from schema.db_core_indcamera_param import DBCoreIndcameraParam
from modules.robot_hardware_server import robot_lifter, robot_four_camera
from schema.db_alarm_manage import DBAlarmManage
from task.hardware_exception import schedule_test
from task.light_al import post_voice_str
from configs import db_rabbit_mq
from core.rabbitmq_db import RabbitPublisher
from modules.robot_hardware_server import RobotDeviceStatus

class QRcodeControl(object):
    def __init__(self):
        self.identify_device_ids = []  # 识别到的设备id列表
        self.check_device_ids = []  # 已有的设备id

    def qrcode_start(self, check_device, qrcode_items, dlcw_l, task_data, robot_position_id, inventory_record_id, robot_room_id):
        """
        二维码盘点入口
        """
        for check_device_id in check_device:
            self.check_device_ids.append(check_device_id.core_device_id)
            check_qrcode_quene.put(check_device_id.core_device_id)
        for item in qrcode_items:
            # 查询设备参数
            device_param = DBRobotDeviceParam().get_device_param(id=item["robot_device_param_id"])
            if device_param == None:
                continue

            file_path = ""
            # 判断是4倍相机还是工业相机
            camera = DBRobotDevice().get_device_id(robot_device_id=device_param["robot_device_id"])
            if camera == None:
                continue

            if camera["name"] == "升降机相机上" or camera["name"] == "升降机相机下":
                # 4倍相机
                file_path = self.qrcode_four_camera(item, device_param)
                if file_path == "":
                    logger.error("未获取到四倍相机拍照结果")
                    continue

            elif camera["name"] == "工业相机上" or camera["name"] == "工业相机下":
                # 工业相机
                file_path = self.qrcode_industrial_camera(item, device_param, dlcw_l)
                if file_path == "":
                    logger.error("未获取到工业相机拍照结果")
                    continue

            # 调用算法服务识别二维码
            qrcode_device_id_list = self.qrcode_identify(file_path)
            if len(qrcode_device_id_list) > 0:
                for qrcode_device_result in qrcode_device_id_list:
                    self.identify_device_ids.append(qrcode_device_result)
            else:
                continue

        # 资产盘点
        self.qrcode_inventory(task_data, robot_position_id, inventory_record_id, robot_room_id)

    def qrcode_four_camera(self, check_item, device_param) ->str:
        """
        四倍相机
        """
        robot_device_status = RobotDeviceStatus()
        # 查询升降机参数
        print("robot_elevator_param_id: ", check_item["robot_elevator_param_id"])
        elevator_detail = DBRobotElevatorParam().get_robot_elevator_detail(id=check_item["robot_elevator_param_id"])
        if elevator_detail == None:
            return ""

        print("elevator_detail: ", elevator_detail)
        # 调用升降机函数,调整升降机
        main_state, position = robot_lifter(move_type=1, position=elevator_detail["height"])
        lifter_status = robot_device_status.get_device_status("lifter")
        if lifter_status == False:
            logger.error("二维码盘点四倍相机升降杆调用失败！")
            # 开始向告警信息表插入数据
            data_value = "升降杆"
            schedule_test(data_value)
            return ""
        elif not main_state:
            return ""
        else:
            pass

        # 查询四倍相机参数
        camera_param = DBRobotDevice().get_device_id(robot_device_id=device_param["robot_device_id"])
        if camera_param == None:
            logger.error("二维码盘点四倍相机未查询到相机参数！")
            return ""

        fourfold_up_status=robot_device_status.get_device_status("fourfold_up")
        fourfold_down_status=robot_device_status.get_device_status("fourfold_down")
        if fourfold_up_status==False or fourfold_down_status==False:
            logger.info("四倍相机不可用，跳过此检测项")
            # 开始向告警信息表插入数据
            data_value = "四倍相机"
            schedule_test(data_value)
            return ""
        else:
            # FIXME 相机参数
            file_path = robot_four_camera(ip=camera_param["address"])
        return file_path

    def qrcode_industrial_camera(self, check_item, device_param, dlcw_l) ->str:
        """
        工业相机
        """
        robot_device_status=RobotDeviceStatus()
        # 查询设备参数
        camera_param = DBCoreIndcameraParam().get_robot_device_id(robot_device_id=device_param["robot_device_id"])
        if camera_param == None:
            return ""

        # 调用升降机函数,调整升降机
        main_state,position=robot_lifter(move_type=1,position=check_item["height"])
        lifter_status = robot_device_status.get_device_status("lifter")
        if lifter_status == False:
            logger.error("二维码盘点四倍相机升降杆调用失败！")
            # 开始向告警信息表插入数据
            data_value="升降杆"
            schedule_test(data_value)
            return ""
        elif not main_state:
            return ""
        else:
            pass

        # 查询相机参数
        device_check_param = DBRobotDevice().get_device_id(robot_device_id=device_param["robot_device_id"])
        if device_check_param == None:
            logger.error("二维码盘点未查询到相机参数！")
            return ""

        os_path = Path().industry + datetime.datetime.now().strftime("%Y_%m_%d")
        photo_name = os_path + "/" + datetime.datetime.now().strftime("%H_%M_%S.%f") + ".jpg"
        if not os.path.exists(os_path):
            os.makedirs(os_path)

        # 摄像头参数
        device_check_param = camera_param.get("device_param")
        # 设置工业相机参数
        camera_params = dict()
        camera_params["ExposureTime"] = device_check_param.get("ExposureTime")
        camera_params["BalanceRatio"] = device_check_param.get("BalanceRatio")
        camera_params["Gain"] = device_check_param.get("Gain")
        camera_params["PhotoNum"] = device_check_param.get("PhotoNum")
        camera_params["PhotoIntervalTime"] = device_check_param.get("PhotoIntervalTime")

        try:
            if not dlcw_l.modify_industry_parameter(device_param["robot_device_id"], None, None, camera_params):
                logger.error("工业相机参数设置失败")
                return ""
        except Exception as e:
            logger.error(e)
            logger.error("工业相机参数设置失败")
            return ""

        photo_num = device_check_param.get("PhotoNum")
        photo_interval_time = device_check_param.get("PhotoIntervalTime")
        try:
            if int(photo_num) == 1:
                try:
                    industry_camera_up = robot_device_status.get_device_status('industry_camera_up')
                    industry_camera_down = robot_device_status.get_device_status('industry_camera_down')
                    if industry_camera_up == False or industry_camera_down == False:
                        logger.error("工业相机拍照异常")
                        # 开始向告警信息表插入数据
                        data_value = "工业相机"
                        schedule_test(data_value)
                        return ""
                    elif not dlcw_l.industry_photo(photo_name,device_check_param.get("address")):
                        return ""
                except Exception as e:
                    logger.error(e)
                    logger.error("工业相机拍照异常")
                    # 初始化工业相机状态
                    use_status_obj=RobotDeviceStatus()
                    use_status_obj.set_device_status("industry_camera_up",False)
                    use_status_obj.set_device_status("industry_camera_down",False)
                    # 开始向告警信息表插入数据
                    data_value="工业相机"
                    schedule_test(data_value)
                    return ""
                # P机柜 U设备名称 D起始U-结束U
                logger.info('开始识别')

                return photo_name
            else:
                file_name_base = photo_name.split(".jpg")[0]
                logger.info('开始合成照片')
                for num in range(photo_num):
                    file_name = file_name_base + "_{}.jpg".format(num)
                    try:
                        industry_camera_up=robot_device_status.get_device_status('industry_camera_up')
                        industry_camera_down=robot_device_status.get_device_status('industry_camera_down')
                        if industry_camera_up == False or industry_camera_down == False:
                            # 开始向告警信息表插入数据
                            data_value="工业相机"
                            schedule_test(data_value)
                            return ""
                        elif not dlcw_l.industry_photo(photo_name,device_param.get("address")):
                            logger.error("工业相机拍照异常")
                            return ""
                        else:
                            pass

                    except Exception as e:
                        logger.error(e)
                        logger.error("工业相机拍照异常")
                        # 初始化工业相机状态
                        use_status_obj=RobotDeviceStatus()
                        use_status_obj.set_device_status("industry_camera_up",False)
                        use_status_obj.set_device_status("industry_camera_down",False)
                        # 开始向告警信息表插入数据
                        data_value="工业相机"
                        schedule_test(data_value)
                        return ""
                    time.sleep(photo_interval_time)
                blend_state, photo_name = lightBlend(photo_name, photo_num)
                return photo_name
        except Exception as e:
            logger.error(e)
            # 初始化工业相机状态
            use_status_obj=RobotDeviceStatus()
            use_status_obj.set_device_status("industry_camera_up",False)
            use_status_obj.set_device_status("industry_camera_down",False)
            # 开始向告警信息表插入数据
            data_value="工业相机"
            schedule_test(data_value)
            return ""

    def qrcode_identify(self, file_path) ->str:
        """
        调用算法服务识别二维码
        """
        device_ok, result_list, img_data = QrcodeIdentify().qrcode_req(file_path=file_path, pic_type=1)
        return result_list

    def qrcode_inventory(self, task_data, robot_position_id, inventory_record_id, robot_room_id):
        """
        二维码设备盘点
        """
        data = {
            "robot_path_id": task_data.get("robot_path_id", 0),
            "robot_id": task_data.get("robot_id", 0),
            "inventory_status": 1,
            "inspect_project_detail_id": task_data.get("inspect_project_detail_id", 0),
            "inspect_project_task_id": task_data.get("inspect_project_task_id"),
            "robot_position_id": robot_position_id,
            "inventory_record_id": inventory_record_id,
            "core_room_id": robot_room_id,
            }

        # 异常数据
        print("check_device_ids: ", self.check_device_ids)
        print("identify_device_ids: ", self.identify_device_ids)
        unusual_device_id_list = list(set(self.check_device_ids) - set(self.identify_device_ids))
        print("unusual_device_id_list: ", unusual_device_id_list)
        if len(unusual_device_id_list) > 0:
            logger.info("二维码异常设备数量: " + str(len(unusual_device_id_list)))

            for unusual_check_id in unusual_device_id_list:
                err_qrcocde_quene.put(unusual_check_id)

                unusual_entity = DBCoreInventoryType().get_by_device_id(core_device_id=unusual_check_id)
                if unusual_entity != None:
                    data["inventory_id"] = unusual_entity["inventory_id"]
                    data["status"] = 1
                    DBCoreInventoryDatum().insert_data(data)

                # 写入告警数据, 发送告警消息
                info = {}
                robot_id = task_data.get("robot_id")
                core_room_id = DBRobot().get_core_room_id(int(robot_id))
                
                info["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
                info["robot_id"] = robot_id
                info["core_room_id"] = core_room_id
                info["core_device_id"] = 0
                info["core_cabinet_id"] = 0
                info["alarm_type"] = 9
                info["robot_item_id"] = 31
                info["level"] = 5
                info["num"] = 1
                info["value"] = "没有检测到该设备"
                info["alarm_desc"] = "识别到的设备id {}没有在资产表中".format(unusual_check_id)
                alarm_manage_id = DBAlarmManage().insert_data(info)

                data_dic = {"alarm_manage_id": alarm_manage_id, "type": 9}
                rout = db_rabbit_mq.get("robot_warn")
                routing_ey = db_rabbit_mq.get("warn_rout_key")
                warn_queue = db_rabbit_mq.get("warn_queue")
                RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
                
                alarm_data = {
                    "sourceAlarmId": alarm_manage_id,
                    "alarmContent": "机器人二维码盘点告警",
                    "alarmLevel": 5,
                    "alarmNum": 1,
                    "alarmStatus": 0,
                    "alarmAsset": "机器人二维码",
                    "alarmStartTime": str(datetime.now()).split(".")[0],
                    "alarmEndTime": str(datetime.now()).split(".")[0],
                    "alarmIsProcess": 0,
                    "alarmIsRecovery": 0,
                    "createTime": str(datetime.now()).split(".")[0],
                    "updateTime": str(datetime.now()).split(".")[0],
                }
                # 将告警推送至 ×××平台
                RabbitPublisher.run("alarm_exchange_99099", "alarm.moss5_0010", "alarm.moss5_0010", alarm_data)

        # 正常数据
        normal_device_id_list = list(set(self.check_device_ids) & set(self.identify_device_ids))
        if len(normal_device_id_list) > 0:
            for normal_device_id in normal_device_id_list:
                normal_entity = DBCoreInventoryType().get_by_device_id(core_device_id=normal_device_id)
                if normal_entity != None:
                    data["inventory_id"] = normal_entity["inventory_id"]
                    data["status"] = 0
                    DBCoreInventoryDatum().insert_data(data)

        logger.info("二维码盘点动作结束")


if __name__ == '__main__':
    pass
