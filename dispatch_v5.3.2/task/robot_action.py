import os
import urllib
import requests
from datetime import datetime
from configs import temp_ip,db_rabbit_mq
from configs.log import logger
from core.dict_config import temp_dic
from core.rabbitmq_db import RabbitPublisher
from schema.db_alarm_manage import DBAlarmManage
from task.hardware_exception import schedule_test
from task.robot_alarm import EnvironmentAlarm
from task.robot_algorithm import RobotAlgorithm
from schema.db_inspect_position_item_data import DBInspectPositionItemDatum
from schema.db_robot_position import DBRobotPosition
from schema.db_robot_ptz_param import DBRobotPtzParam
from schema.db_robot_device_param import DBRobotDeviceParam
from schema.db_robot_device import DBRobotDevice
from utils.file_path import Path
from modules.robot_hardware_server import robot_ptz
from modules.robot_hardware_server import robot_thermal_imagery
from modules.robot_hardware_server import RobotDeviceStatus

class RobotAction(object):
    def __init__(self):
        self.environment_alarm = EnvironmentAlarm()

    # 调取云台动作
    def robot_yuntai_action(self, robot_ptz_param_id, yuntai_state):
        robot_device_status=RobotDeviceStatus()
        if yuntai_state == 0:
            state = robot_ptz("G", 1)
            return state
        if yuntai_state == 1:
            state = robot_ptz("G", 2)
            return state
        if not robot_ptz_param_id:
            return False
        ret = DBRobotPtzParam().get_yuntai_preset_num(robot_ptz_param_id)
        try:
            location_num = ret[0].get("preset_num")
            logger.info("云台位置")
            logger.info(location_num)
            robot_device_status=RobotDeviceStatus()
            ptz_status = robot_device_status.get_device_status("ptz")
            if ptz_status == False:
                # 开始向告警信息表插入数据
                data_value = "云台"
                schedule_test(data_value)
            else:
                state = robot_ptz("G", int(location_num))
            return state
        except Exception as e:
            logger.error(e)
            # 开始向告警信息表插入数据
            data_value="云台"
            schedule_test(data_value)
            return False

    # 抓取热成像工作
    def robot_temp_action(self, res_device_param_id, ptz_list, position_res, task_data):
        task_data["robot_position_id"] = position_res.get("robot_position_id")
        inspect_project_detail_id = task_data.get("inspect_project_detail_id")
        ret = DBRobotDeviceParam().get_device_param(res_device_param_id)
        res = DBRobotDevice().get_device_data(ret.get("robot_device_id"))
        ip = res.get("address")
        file_path, result = robot_thermal_imagery(ip)
        logger.info("热成像照片的解析" + str(result))
        task_data["img_path"] = file_path
        for elevator in ptz_list:
            # 插入数据
            temp_name = elevator.get("name")
            temp_name_en = temp_dic.get(temp_name)
            temp_num = result.get(temp_name_en)
            if float(temp_num) <= 0:
                if temp_name_en == "high_temp":
                    temp_num = "25.0"
                elif temp_name_en == "aver_temp":
                    temp_num = "20.0"
                else:
                    temp_num = "15.0"
            ret["inspect_project_detail_id"] = inspect_project_detail_id
            ret["core_cabinet_id"] = position_res.get("core_cabinet_id")
            ret["robot_position_id"] = position_res.get("robot_position_id")
            ret["robot_path_id"] = position_res.get("robot_path_id")
            ret["core_cabinet_id"] = position_res.get("core_cabinet_id")
            ret["power_cabinet_id"] = position_res.get("power_cabinet_id")
            ret["robot_item_id"] = elevator.get("robot_item_id")
            ret["create_time"] = datetime.now()
            ret['value'] = temp_num
            ret["img_path"] = file_path
            ret['type'] = 2
            try:
                DBInspectPositionItemDatum().insert(info=ret)
            except Exception as e:
                logger.error(e)
        return True

    def get_temp_photo(self):
        path = Path()
        # 抓取图片
        ip = "http://{}".format(temp_ip.get("host"))
        headers = {
            "User-Agent": "Mozilla/5.0 (Linux; Android 6.0; Nexus 5 Build/MRA58N) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/66.0.3359.139 Mobile Safari/537.36",
            "Cookie": "Language=zh-cn; DevType=DM20; OEMINFO=OEM; Name=admin; Pass=Admin123; UserType=Admin; IPCamera=DM20"}
        temp_path = path.thermal + datetime.now().strftime("%Y_%m_%d")
        if not os.path.exists(temp_path):
            os.makedirs(temp_path)
        photo_path = temp_path + "/" + datetime.now().strftime("%H_%M_%S")
        # 图片地址
        url = ip + ':5000/stream?Type=JPEG&Source=Jpeg&Mode=TCP&Heart-beat=No&Frames=1&Snap=Yes&Channel=0'
        html = requests.get(url, headers)
        # 将图片保存到本地
        with open(photo_path + ".jpg", "wb")as f:
            f.write(html.content)
        return photo_path + ".jpg"

    def get_temp_data(self):
        ip = "http://{}".format(temp_ip.get("host"))
        # 登录Post数据
        loginurl = ip + "/cgi-bin/system?Module=DMTtl"
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
        return temp_dic


# 返回到充电点
def move_state(robot_id, robot_path_id, move_state=None):
    robot_device_status=RobotDeviceStatus()
    logger.info("改点不在充电点，先返回到充电点")
    robot_algorithm = RobotAlgorithm()
    robot_location = DBRobotPosition().get_back_status(robot_id)

    if len(robot_location) != 1:
        if robot_location[0].get("order_num") > robot_location[1].get("order_num"):
            move_type_name = "back_move"
        else:
            move_type_name = "forward_move"
    else:
        move_type_name = "forward_move"
    for res in robot_location:
        if res.get("type") == 1:
            # 调取move算法
            logger.info("到达返回点 -----》" + str(res))
            move_status = robot_device_status.get_device_status("move")
            if not robot_algorithm.robot_back_al(res, move_type_name) or move_status == False:
                # 开始向告警信息表插入数据
                data_value = "移动服务"
                schedule_test(data_value)
                return False

        if res.get("type") == 3:
            # 调取move算法
            logger.info("返回充电预备点 -----》" + str(res))
            move_status = robot_device_status.get_device_status("move")
            if not robot_algorithm.robot_back_al(res, "back_move") or move_status == False:
                # 开始向告警信息表插入数据
                data_value = "移动服务"
                schedule_test(data_value)
                return False

        elif res.get("type") == 4:
            # 调取充电算法
            logger.info("返回充电点 -----》" + str(res))
            move_status = robot_device_status.get_device_status("move")
            if not robot_algorithm.robot_back_al(res, "back_charging") or move_status == False:
                # 开始向告警信息表插入数据
                data_value="移动服务"
                schedule_test(data_value)
                return False
    return True

