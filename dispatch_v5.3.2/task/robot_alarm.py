from datetime import datetime

from configs import db_rabbit_mq
from configs.log import logger
from core.rabbitmq_db import RabbitPublisher
# from schema.db_alarm_device_check import DBAlarmDeviceCheck
# from schema.db_alarm_indicator_light import DBAlarmIndicatorLight
from schema.db_alarm_manage import DBAlarmManage
# from schema.db_alarm_moving_ring import DBAlarmMovingRing
from schema.db_alarm_rule import DBAlarmRule
from schema.db_alarm_rule_indcamera import DBAlarmRuleIndcamera
from schema.db_alarm_rule_indcamera_light import DBAlarmRuleIndcameraLight
from schema.db_alarm_rule_light import DBAlarmRuleLight
from schema.db_alarm_rule_ring import DBAlarmRuleRing
from schema.db_core_device import DBCoreDevice
from schema.db_core_device_location import DBCoreDeviceLocation
from schema.db_core_device_part import DBCoreDevicePart
from schema.db_core_inventory_type import DBCoreInventoryType
from schema.db_power_alarm_rule import DBPowerAlarmRule
from schema.db_robot import DBRobot
from schema.db_robot_algorithm_param import DBRobotAlgorithmParam
from schema.db_robot_item import DBRobotItem
# from schema.db_power_alarm import DBPowerAlarm
from schema.db_power_alarm_rule_detail import DBPowerAlarmRuleDetail


# 动环告警规则
class EnvironmentAlarm(object):
    def __init__(self):
        self.db_alarm_rule_ring = DBAlarmRuleRing()
        self.db_alarm_manage = DBAlarmManage()
        self.db_robot_item = DBRobotItem()
        self.db_alarm_rule = DBAlarmRule()
        self.alarm_data_rule_id = 1

    # 获取动环和表面温度告警比对参数
    def get_alarm_type(self, item_name):
        alarm_data = self.db_alarm_rule_ring.get_alarm_data(item_name)
        robot_item = self.db_robot_item.get_item_name(item_name)
        logger.info("检测项名字为{}".format(item_name))
        logger.info("检测项数据为{}".format(robot_item))
        logger.info("告警规则数据为{}".format(alarm_data))
        if robot_item:
            item_id = robot_item.get("robot_item_id")
        else:
            return False, None, None, None, None
        if alarm_data:
            return item_id, int(alarm_data.get("value")), alarm_data.get("conditions"), \
                   alarm_data.get("level"), alarm_data.get("rule_id")
        return False, None, None, None, None

    # 动环对比规则(入口)
    def contrast_alarm(self, task_data, env_name, env_values):
        logger.info(env_name)
        if env_name in ["平均温度", "最低温度", "最高温度"]:
            task_data["alarm_type"] = 2
        else:
            task_data["alarm_type"] = 1
            task_data["img_path"] = ""
        if "温度" in str(env_name):
            task_data["alarm_type"] = 2
        else:
            task_data["alarm_type"] = 1
            task_data["img_path"] = ""
        logger.info(task_data["alarm_type"])
        item_id, alarm_data_value, alarm_data_conditions, \
        alarm_data_level, alarm_data_rule_id = self.get_alarm_type(env_name)
        self.alarm_data_rule_id = alarm_data_rule_id
        env_value = eval(str(env_values))
        if not item_id:
            return False
        robot_position_id = task_data.get("robot_position_id")
        alive_data = DBAlarmManage().get_alarm_data(robot_position_id, item_id)
        if alive_data:
            alive_data["img_path"] = task_data.get("img_path")
            alive_data["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
            alive_data["value"] = env_value

        task_data["value"] = env_value

        logger.info("当前告警符号为")
        logger.info(alarm_data_conditions)
        logger.info("当前告警规则数值为")
        logger.info(alarm_data_value)
        logger.info("当前动环显示数值为")
        logger.info(env_value)
        logger.info("动环告警表告警信息")
        logger.info(alive_data)
        logger.info("动环告警表告警描述")
        alarm_desc = env_name + ":" + str(env_value)
        logger.info(alarm_desc)
        logger.info(task_data["alarm_type"])
        if alarm_data_conditions == ">":
            if env_value > alarm_data_value:
                if alive_data:
                    alive_data["alarm_desc"] = alarm_desc
                    # 更新告警
                    logger.info(">更新")
                    self.update_alarm_num(alive_data)
                else:
                    # 写入新的告警
                    logger.info(">插入")
                    logger.info(alarm_desc)
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)

            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

        elif alarm_data_conditions == ">=":
            if env_value >= alarm_data_value:
                if alive_data:
                    # 更新告警
                    alive_data["alarm_desc"] = alarm_desc
                    logger.info(">=更新")
                    self.update_alarm_num(alive_data)
                else:
                    # 写入新的告警
                    logger.info(alarm_desc)
                    logger.info(">=插入")
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

        elif alarm_data_conditions == "<=":
            if env_value <= alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    alive_data["alarm_desc"] = alarm_desc
                    # 更新告警
                    logger.info("<=更新")
                    self.update_alarm_num(alive_data)
                else:
                    # 写入新的告警
                    logger.info("<=插入")
                    logger.info(alarm_desc)
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

        elif alarm_data_conditions == "==":
            if env_value == alarm_data_value:
                if alive_data:
                    alive_data["alarm_desc"] = alarm_desc
                    # 更新告警
                    logger.info("==更新")
                    self.update_alarm_num(alive_data)
                else:
                    # 写入新的告警
                    logger.info("==插入")
                    logger.info(alarm_desc)
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

        else:
            if env_value < alarm_data_value:
                if alive_data:
                    alive_data["alarm_desc"] = alarm_desc
                    # 更新告警
                    logger.info("<更新")
                    self.update_alarm_num(alive_data)
                else:
                    # 写入新的告警
                    logger.info("<插入")
                    logger.info(alarm_desc)
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

    # 更新告警次数
    def update_alarm_num(self, alive_data):
        alarm_manage_id = alive_data.get("alarm_manage_id")
        num = alive_data.get("num")
        num += 1
        alive_data["num"] = num
        self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    # 更新正常次数
    def update_alarm_common_num(self, alive_data, rule_id):
        alarm_rule_data = self.db_alarm_rule.get_by_rule_id(rule_id)
        alarm_manage_id = alive_data.get("alarm_manage_id")
        common_num = alive_data.get("common_num")
        common_num += 1
        if common_num > alarm_rule_data.get("undo_alarm"):
            # info = {"undo_status": 1}
            alive_data["undo_status"] = 1
            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)
        else:
            # info = {"common_num": common_num}
            alive_data["common_num"] = common_num

            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    def insert_alarm(self, task_data, num, robot_item_id, level, alarm_desc):
        logger.info("插入动环告警的数据")
        logger.info(task_data)
        ret = {}
        robot_id = task_data.get("robot_id")
        core_room_id = DBRobot().get_core_room_id(int(robot_id))
        ret["robot_id"] = robot_id
        ret["core_room_id"] = core_room_id
        ret["robot_position_id"] = task_data.get("robot_position_id")
        ret["robot_item_id"] = robot_item_id
        ret["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
        ret["inspect_project_task_id"] = task_data.get("inspect_project_task_id")

        ret["level"] = level
        ret["num"] = num
        ret["assign"] = 0
        ret["alarm_desc"] = alarm_desc
        ret["value"] = task_data.get("value")
        ret["alarm_type"] = task_data.get("alarm_type")
        ret["user_id"] = task_data.get("user_id")
        ret["alarm_type"] = task_data.get("alarm_type")
        ret["power_cabinet_id"] = task_data.get("power_cabinet_id", None)
        ret["core_cabinet_id"] = task_data.get("core_cabinet_id", None)
        ret["img_path"] = ""
        try:
            if task_data.get("img_path"):
                logger.info("666666666666")
                alarm_manage_id = DBAlarmManage().insert_data(ret)
                logger.info("插入完成")
                data_dic = {"alarm_manage_id": alarm_manage_id, "notice_rule_id": self.alarm_data_rule_id, "type": 2}
                rout = db_rabbit_mq.get("robot_warn")
                routing_ey = db_rabbit_mq.get("warn_rout_key")
                warn_queue = db_rabbit_mq.get("warn_queue")
                RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
            else:
                logger.info("666666666666")
                alarm_manage_id = DBAlarmManage().insert_data(ret)
                logger.info("插入完成")
                data_dic = {"alarm_manage_id": alarm_manage_id, "notice_rule_id": self.alarm_data_rule_id, "type": 1}
                rout = db_rabbit_mq.get("robot_warn")
                routing_ey = db_rabbit_mq.get("warn_rout_key")
                warn_queue = db_rabbit_mq.get("warn_queue")
                RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
        except Exception as e:
            logger.error(e)
        logger.info("666666666666")


# 四倍告警规则
class FourFoldAlarm(object):
    def __init__(self):
        self.db_robot_algorithm_param = DBRobotAlgorithmParam()
        self.db_core_device_part = DBCoreDevicePart()
        self.db_core_device = DBCoreDevice()
        self.db_alarm_rule = DBAlarmRule()
        self.db_alarm_rule_light = DBAlarmRuleLight()
        self.db_robot_item = DBRobotItem()
        self.db_alarm_manage = DBAlarmManage()
        self.core_device_id = 1
        self.alarm_data_rule_id = 1

    # 获取告警比对参数
    def get_alarm_type(self, robot_algorithm_param_id, robot_position_id, item_name):
        logger.info(robot_algorithm_param_id)
        algorithm_data = self.db_robot_algorithm_param.get_by_id(robot_algorithm_param_id)
        logger.info(algorithm_data)
        start_u = algorithm_data["parameters"]["u_params"]["start_u"]
        logger.info(robot_position_id)
        logger.info(start_u)

        core_device_part_data = self.db_core_device_part.get_core_device_data(robot_position_id, start_u)
        if not core_device_part_data:
            logger.error("没有点{}与U数{}确定的唯一资产ID值".format(robot_position_id, start_u))
            logger.error("没有点{}与U数{}确定的算法ID值为{}".format(robot_position_id, start_u, robot_algorithm_param_id))
            logger.info(algorithm_data)

            return False, None, None, None, None, None
        core_device_id = core_device_part_data.get("core_device_id")
        self.core_device_id = core_device_id
        name = core_device_part_data.get("name")

        core_device_data = self.db_core_device.get_core_device_data(core_device_id)
        device_number = core_device_data.get("device_number")
        try:
            alarm_rule_data = self.db_alarm_rule.get_by_device_num(device_number)
            rule_id = alarm_rule_data.get("rule_id")

            alarm_light_data = self.db_alarm_rule_light.get_light_rule_data(rule_id, name, item_name)
            robot_item = self.db_robot_item.get_item_name(item_name)
        except Exception as e:
            logger.info("core_device_id={}".format(core_device_id))
            logger.info("device_number={}".format(device_number))
            logger.error(e)
            return False, None, None, None, None, None

        if robot_item:
            item_id = robot_item.get("robot_item_id")
        else:
            return False, None, None, None, None, None
        if alarm_light_data:
            return item_id, int(alarm_light_data.get("value")), alarm_light_data.get("conditions"), \
                   alarm_light_data.get("level"), alarm_light_data.get("rule_id"), start_u
        return False, None, None, None, None, None

    # 四倍对比规则(入口)
    def contrast_alarm(self, task_data, env_name, env_value, robot_algorithm_param_id):
        robot_position_id = task_data.get("robot_position_id")
        img_path = task_data.get("img_path")
        item_id, alarm_data_value, alarm_data_conditions, \
        alarm_data_level, alarm_data_rule_id, alarm_data_rule_start_u = self.get_alarm_type(robot_algorithm_param_id,
                                                                                            robot_position_id, env_name)
        self.alarm_data_rule_id = alarm_data_rule_id
        task_data["value"] = env_value
        task_data["alarm_u"] = alarm_data_rule_start_u
        if not item_id:
            return False
        alive_data = self.db_alarm_manage.get_alarm_light_data(robot_position_id, item_id, alarm_data_rule_start_u)
        if alive_data:
            alive_data["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
            alive_data["value"] = env_value
            alive_data["img_path"] = img_path

        if alarm_data_conditions == ">":
            if env_value > alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)

            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

        elif alarm_data_conditions == ">=":
            if env_value >= alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

        elif alarm_data_conditions == "<=":
            if env_value <= alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

        elif alarm_data_conditions == "=":
            if env_value <= alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

        else:
            if env_value < alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_rule_id)

    # 更新告警次数
    def update_alarm_num(self, alive_data, img_path):
        alarm_manage_id = alive_data.get("alarm_manage_id")
        num = alive_data.get("num")
        num += 1
        alive_data["num"] = num
        alive_data["img_path"] = img_path

        # info = {"num": num, "img_path": img_path}
        self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    # 更新正常次数
    def update_alarm_common_num(self, alive_data, rule_id):
        alarm_rule_data = self.db_alarm_rule.get_by_rule_id(rule_id)
        alarm_manage_id = alive_data.get("alarm_manage_id")
        common_num = alive_data.get("common_num")
        common_num += 1
        if common_num > alarm_rule_data.get("undo_alarm"):
            # info = {"undo_status": 1}
            alive_data["undo_status"] = 1
            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)
        else:
            # info = {"common_num": common_num}
            alive_data["common_num"] = common_num
            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    def insert_alarm(self, task_data, num, robot_item_id, level, alarm_desc):
        logger.info("插入四倍告警的数据")
        logger.info(task_data)
        ret = {}
        ret["robot_id"] = task_data.get("robot_id")
        ret["core_room_id"] = task_data.get("core_room_id", 1)
        ret["robot_position_id"] = task_data.get("robot_position_id")
        ret["robot_item_id"] = robot_item_id
        ret["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
        ret["level"] = level
        ret["type"] = 0
        ret["num"] = num
        ret["alarm_u"] = task_data.get("alarm_u")
        ret["assign"] = 0
        ret["img_path"] = task_data.get("img_path")
        ret["core_device_id"] = self.core_device_id
        ret["alarm_desc"] = alarm_desc
        ret["value"] = task_data.get("value", 0)
        ret["alarm_desc"] = task_data.get("alarm_desc")
        ret["user_id"] = task_data.get("user_id")
        ret["power_cabinet_id"] = task_data.get("power_cabinet_id", None)
        ret["core_cabinet_id"] = task_data.get("core_cabinet_id", None)
        alarm_indicator_light_id = DBAlarmManage().insert_data(ret)

        data_dic = {"alarm_manage_id": alarm_indicator_light_id, "notice_rule_id": self.alarm_data_rule_id, "type": 3}

        rout = db_rabbit_mq.get("robot_warn")
        routing_ey = db_rabbit_mq.get("warn_rout_key")
        warn_queue = db_rabbit_mq.get("warn_queue")
        RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)


# 工业告警规则
class IndustryAlarm(object):
    def __init__(self):
        self.db_robot_item = DBRobotItem()
        self.db_alarm_manage = DBAlarmManage()
        self.db_core_device_location = DBCoreDeviceLocation()
        self.db_alarm_rule_indcamera = DBAlarmRuleIndcamera()
        self.db_alarm_rule_indcamera_light = DBAlarmRuleIndcameraLight()
        self.core_device_id = 1
        self.env_value = 0
        self.alarm_rule_indcamera_id = 1

    # 获取告警比对参数
    def get_alarm_type(self, core_device_id, item_name):
        algorithm_data = self.db_core_device_location.get_start_u(core_device_id)
        if not algorithm_data:
            logger.info("没有告警比对参数{}".format(str(core_device_id)))
            return False, None, None, None, None, None
        start_u = algorithm_data.get("start_u")
        end_u = algorithm_data.get("end_u")
        alarm_u = str(start_u) + "-" + str(end_u)

        alarm_rule_indcamera_data = self.db_alarm_rule_indcamera.get_light_rule_data(core_device_id)
        alarm_rule_indcamera_id = alarm_rule_indcamera_data.get("alarm_rule_indcamera_id")
        self.alarm_rule_indcamera_id = alarm_rule_indcamera_id
        undo_alarm = alarm_rule_indcamera_data.get("undo_alarm")
        alarm_light_data = self.db_alarm_rule_indcamera_light.get_light_rule_data(alarm_rule_indcamera_id, item_name)
        robot_item = self.db_robot_item.get_item_name(item_name)

        if robot_item:
            item_id = robot_item.get("robot_item_id")
        else:
            return False, None, None, None, None, None
        if alarm_light_data:
            return item_id, int(alarm_light_data.get("value")), alarm_light_data.get("conditions"), \
                   alarm_light_data.get("level"), undo_alarm, alarm_u
        return False, None, None, None, None, None

    # 工业对比规则(入口)
    def contrast_alarm(self, task_data, env_name, env_value):
        logger.info("开始工业对比规则")
        logger.info(env_name)
        logger.info(env_value)

        try:
            env_value = eval(env_value)
            self.env_value = env_value
        except:
            env_value = env_value
            self.env_value = env_value

        robot_position_id = task_data.get("robot_position_id")
        img_path = task_data.get("img_path")
        self.core_device_id = task_data.get("core_device_id")

        item_id, alarm_data_value, alarm_data_conditions, \
        alarm_data_level, alarm_data_undo_alarm, alarm_data_rule_start_u = self.get_alarm_type(self.core_device_id,
                                                                                               env_name)
        task_data["alarm_u"] = alarm_data_rule_start_u
        task_data["value"] = env_value

        if not item_id:
            logger.error("工业相机告警对比错误，无item_id数据")
            return False
        alive_data = DBAlarmIndicatorLight().get_alarm_data(robot_position_id, item_id, alarm_data_rule_start_u)
        if alive_data:
            alive_data["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
            alive_data["value"] = env_value
            alive_data["img_path"] = img_path

        logger.info("当前告警符号为")
        logger.info(alarm_data_conditions)
        logger.info("当前告警规则数值为")
        logger.info(alarm_data_value)
        logger.info("当前指示灯显示数值为")
        logger.info(env_value)
        logger.info("是否存在当前设备的告警信息")
        logger.info(env_value)
        if alarm_data_conditions == ">":
            if env_value > alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)

            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

        elif alarm_data_conditions == ">=":
            if env_value >= alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

        elif alarm_data_conditions == "<=":
            if env_value <= alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

        elif alarm_data_conditions == "=":
            if env_value == alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

        else:
            if env_value < alarm_data_value:
                alarm_desc = env_name + ":" + str(env_value)
                if alive_data:
                    # 更新告警
                    self.update_alarm_num(alive_data, img_path)
                else:
                    # 写入新的告警
                    self.insert_alarm(task_data, 1, item_id, alarm_data_level, alarm_desc)
            else:
                if alive_data:
                    self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

    # 更新告警次数
    def update_alarm_num(self, alive_data, img_path):
        alarm_manage_id = alive_data.get("alarm_manage_id")
        num = alive_data.get("num")
        num += 1
        alive_data["num"] = num
        alive_data["img_path"] = img_path
        alive_data["value"] = self.env_value

        # info = {"num": num, "img_path": img_path, "value": self.env_value}
        self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    # 更新正常次数
    def update_alarm_common_num(self, alive_data, alarm_data_undo_alarm):
        alarm_manage_id = alive_data.get("alarm_manage_id")
        common_num = alive_data.get("common_num")
        common_num += 1
        if common_num > alarm_data_undo_alarm:
            alive_data["undo_status"] = 1
            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)
        else:
            alive_data["common_num"] = common_num

            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    def insert_alarm(self, task_data, num, robot_item_id, level, alarm_desc):
        logger.info("开始插入工业相机数据")
        logger.info(task_data)
        ret = {}
        ret["robot_id"] = task_data.get("robot_id")
        ret["core_room_id"] = task_data.get("core_room_id", "1")
        ret["robot_position_id"] = task_data.get("robot_position_id")
        ret["robot_item_id"] = robot_item_id
        ret["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
        ret["level"] = level
        ret["num"] = num
        ret["alarm_type"] = 7
        ret["assign"] = 0
        ret["alarm_u"] = task_data.get("alarm_u")
        ret["img_path"] = task_data.get("img_path")
        ret["core_device_id"] = self.core_device_id
        ret["alarm_desc"] = alarm_desc
        ret["value"] = task_data.get("value")
        # ret["alarm_desc"] = task_data.get("alarm_desc")
        ret["user_id"] = task_data.get("user_id")
        ret["power_cabinet_id"] = task_data.get("power_cabinet_id", None)
        ret["core_cabinet_id"] = task_data.get("core_cabinet_id", None)
        alarm_manage_id = DBAlarmManage().insert_data(ret)

        data_dic = {"alarm_manage_id": alarm_manage_id, "notice_rule_id": self.alarm_rule_indcamera_id, "type": 7}

        rout = db_rabbit_mq.get("robot_warn")
        routing_ey = db_rabbit_mq.get("warn_rout_key")
        warn_queue = db_rabbit_mq.get("warn_queue")
        RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)


# 资产告警规则
class AssetsAlarm(object):
    def __init__(self):
        self.alarm_manage = DBAlarmManage()
        self.core_room_id = 4

    # 资产对比规则(入口)
    def contrast_alarm(self, task_data, rf_id):
        info = {}
        alive_data = DBCoreInventoryType().get_by_rfid(rf_id)
        logger.info("RFID为{}".format(rf_id))
        logger.info("查询RFID结果为{}".format(alive_data))
        if alive_data:
            alive_alarm_data = self.alarm_manage.get_by_core_device_id(alive_data.get("core_device_id"))
            logger.info("查询是否在表中插入过RFID{}".format(alive_alarm_data))
            if alive_alarm_data:
                self.update_alarm_num(alive_alarm_data)
            else:
                self.core_room_id = alive_data.get("core_room_id")

                info["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
                info["inspect_project_task_id"] = task_data.get("inspect_project_task_id")

                info["robot_id"] = task_data.get("robot_id")
                info["core_room_id"] = alive_data.get("core_room_id")
                info["core_device_id"] = alive_data.get("core_device_id")
                # robot_position_data = DBRobotPosition(). \
                #     get_by_core_cabinet(task_data.get("robot_path_id"), alive_data.get("core_cabinet_id"))
                # if robot_position_data:
                #     info["robot_position_id"] = robot_position_data.get("robot_position_id")
                robot_device = DBCoreDeviceLocation().get_by_cabinet_id(alive_data.get("core_device_id"))
                if robot_device:
                    info["core_cabinet_id"] = robot_device.get("core_cabinet_id")

                info["robot_item_id"] = 31
                info["level"] = 5
                info["num"] = 1
                info["alarm_type"] = 9
                info["value"] = "没有检测到该设备"
                info["core_room_id"] = alive_data.get("core_room_id")
                info["alarm_desc"] = "资产表中存在的RFID{}没有被检测到".format(rf_id)

                alarm_manage_id = self.insert_alarm(info)
                data_dic = {"alarm_manage_id": alarm_manage_id, "type": 9}
                rout = db_rabbit_mq.get("robot_warn")
                routing_ey = db_rabbit_mq.get("warn_rout_key")
                warn_queue = db_rabbit_mq.get("warn_queue")
                RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
                alarm_data = {
                    "sourceAlarmId": alarm_manage_id,
                    "alarmContent": "机器人RFID告警",
                    "alarmLevel": 5,
                    "alarmNum": 1,
                    "alarmStatus": 0,
                    "alarmAsset": "机器人RFID",
                    "alarmStartTime": str(datetime.now()).split(".")[0],
                    "alarmEndTime": str(datetime.now()).split(".")[0],
                    "alarmIsProcess": 0,
                    "alarmIsRecovery": 0,
                    "createTime": str(datetime.now()).split(".")[0],
                    "updateTime": str(datetime.now()).split(".")[0],
                }
                # 将告警推送至 ×××平台
                RabbitPublisher.run("alarm_exchange_99099", "alarm.mp_0010", "alarm.mp_0010", alarm_data)
        else:
            info["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
            robot_id = task_data.get("robot_id")
            core_room_id = DBRobot().get_core_room_id(int(robot_id))
            info["robot_id"] = robot_id
            info["core_room_id"] = core_room_id
            info["core_device_id"] = 0

            info["core_cabinet_id"] = 0
            info["alarm_type"] = 9
            info["robot_item_id"] = 31
            info["level"] = 5
            info["num"] = 1
            info["value"] = "没有检测到该设备"

            info["alarm_desc"] = "识别到的RFID{}没有在资产表中".format(rf_id)
            alarm_manage_id = self.insert_alarm(info)
            data_dic = {"alarm_manage_id": alarm_manage_id, "type": 9}
            rout = db_rabbit_mq.get("robot_warn")
            routing_ey = db_rabbit_mq.get("warn_rout_key")
            warn_queue = db_rabbit_mq.get("warn_queue")
            RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
            alarm_data = {
                "sourceAlarmId": alarm_manage_id,
                "alarmContent": "机器人RFID告警",
                "alarmLevel": 5,
                "alarmNum": 1,
                "alarmStatus": 0,
                "alarmAsset": "机器人RFID",
                "alarmStartTime": str(datetime.now()).split(".")[0],
                "alarmEndTime": str(datetime.now()).split(".")[0],
                "alarmIsProcess": 0,
                "alarmIsRecovery": 0,
                "createTime": str(datetime.now()).split(".")[0],
                "updateTime": str(datetime.now()).split(".")[0],
            }
            # 将告警推送至 ×××平台
            RabbitPublisher.run("alarm_exchange_99099", "alarm.moss5_0010", "alarm.moss5_0010", alarm_data)

    # 更新告警次数
    def update_alarm_num(self, alive_data):
        alarm_manage_id = alive_data.get("device_check_id")
        num = alive_data.get("num")
        num += 1
        alive_data["num"] = num
        self.alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    def insert_alarm(self, info):
        logger.info("插入资产告警的数据")
        logger.info(info)
        device_check_id = DBAlarmManage().insert_data(info)
        return device_check_id


# 数字仪表盘告警
class PowerAlarmAL(object):
    def __init__(self):
        self.db_power_alarm_rule_data = DBPowerAlarmRuleDetail()
        self.db_alarm_manage = DBAlarmManage()
        self.db_power_alarm_rule = DBPowerAlarmRule()
        self.power_device_id = 1
        self.env_value = 0
        self.alarm_rule_indcamera_id = 1

    # 获取告警比对参数
    def get_alarm_type(self, camera_data, item_name):
        res_list = []
        power_alarm_rule_id = camera_data.get("power_alarm_rule_id")
        algorithm_data_list = self.db_power_alarm_rule_data.get_power_alarm_data(power_alarm_rule_id, item_name)
        if not algorithm_data_list:
            logger.info("没有告警比对参数{}".format(str(power_alarm_rule_id)))
            return False, None

        power_robot_item_id = camera_data.get("power_robot_item_id")

        for algorithm_data in algorithm_data_list:
            data_dic = {}
            data_dic["power_robot_item_id"] = power_robot_item_id
            data_dic["value"] = algorithm_data.get("value")
            data_dic["conditions"] = algorithm_data.get("conditions")
            data_dic["level"] = algorithm_data.get("level")
            res_list.append(data_dic)

        if res_list:
            return True, res_list
        return False, None,

    # 数字仪表盘告警对比规则(入口)
    def contrast_alarm(self, task_data, env_name, env_value):
        logger.info("开始数字仪表盘对比规则")
        logger.info(env_name)
        logger.info(env_value)
        self.alarm_desc = env_name + ":" + str(env_value)

        try:
            env_value = eval(env_value)
            self.env_value = env_value
        except:
            env_value = env_value
            self.env_value = env_value

        robot_position_id = task_data.get("robot_position_id")
        img_path = task_data.get("img_path")
        camera_data = task_data.get("camera_data")
        self.power_device_id = camera_data.get("power_device_id")
        logger.info(camera_data)
        logger.info(type(camera_data))
        power_robot_item_id = camera_data.get("power_robot_item_id")
        power_device_id = camera_data.get("power_device_id")

        data_state, alarm_data_list = self.get_alarm_type(camera_data, env_name)

        task_data["value"] = env_value

        if not data_state:
            logger.error("数字仪表盘告警对比错误，无item_id数据")
            return False
        for alarm_data_data in alarm_data_list:
            alive_data = self.db_alarm_manage.get_power_alarm_data(robot_position_id, power_robot_item_id,
                                                                   power_device_id)
            if alive_data:
                alive_data["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
                alive_data["value"] = env_value
                alive_data["img_path"] = img_path
            alarm_data_conditions = alarm_data_data.get("conditions")
            alarm_data_value = float(alarm_data_data.get("value"))
            alarm_data_level = alarm_data_data.get("level")

            alarm_data_undo_alarm_data = self.db_power_alarm_rule.get_power_alarm_rule(
                alarm_data_data.get("power_alarm_rule_id"))
            if alarm_data_undo_alarm_data:
                alarm_data_undo_alarm = alarm_data_undo_alarm_data.get("undo_alarm")
            else:
                alarm_data_undo_alarm = 5
            logger.info("当前告警符号为")
            logger.info(alarm_data_conditions)
            logger.info("当前告警规则数值为")
            logger.info(alarm_data_value)
            logger.info("当前数据显示数值为")
            logger.info(env_value)

            if alarm_data_conditions == ">":
                if env_value > alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)

                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

            elif alarm_data_conditions == ">=":
                if env_value >= alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)
                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

            elif alarm_data_conditions == "<=":
                if env_value <= alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)
                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

            elif alarm_data_conditions == "=":
                if env_value == alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)
                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

            else:
                if env_value < alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)
                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

    # 更新告警次数
    def update_alarm_num(self, alive_data, img_path):
        alarm_manage_id = alive_data.get("alarm_manage_id")
        num = alive_data.get("num")
        num += 1
        alive_data["num"] = num
        alive_data["img_path"] = img_path
        alive_data["value"] = self.env_value
        alive_data["alarm_desc"] = self.alarm_desc

        self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    # 更新正常次数
    def update_alarm_common_num(self, alive_data, alarm_data_undo_alarm):
        alarm_manage_id = alive_data.get("alarm_manage_id")
        common_num = alive_data.get("common_num")
        common_num += 1
        if common_num > alarm_data_undo_alarm:
            alive_data["undo_status"] = 1
            alive_data["alarm_desc"] = self.alarm_desc
            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)
        else:
            alive_data["common_num"] = common_num
            alive_data["alarm_desc"] = self.alarm_desc
            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    def insert_alarm(self, task_data, num, power_robot_item_id, level, alarm_desc):
        logger.info("开始插入数字仪表盘数据")
        logger.info(task_data)
        ret = {}
        ret["robot_id"] = task_data.get("robot_id")
        ret["core_room_id"] = task_data.get("core_room_id", "1")
        ret["robot_position_id"] = task_data.get("robot_position_id")
        ret["power_robot_item_id"] = power_robot_item_id
        ret["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
        ret["power_device_id"] = self.power_device_id
        ret["level"] = level
        ret["num"] = num
        ret["value"] = task_data.get("value")
        ret["assign"] = 0
        ret["img_path"] = task_data.get("img_path")
        ret["alarm_desc"] = alarm_desc
        ret["power_cabinet_id"] = task_data.get("power_cabinet_id", None)
        ret["core_cabinet_id"] = task_data.get("core_cabinet_id", None)
        alarm_manage_id = DBAlarmManage().insert_data(ret)

        data_dic = {"alarm_manage_id": alarm_manage_id, "noticeRuleId": self.alarm_rule_indcamera_id, "type": 10}
        rout = db_rabbit_mq.get("robot_warn")
        routing_ey = db_rabbit_mq.get("warn_rout_key")
        warn_queue = db_rabbit_mq.get("warn_queue")
        logger.info("发送MQ告数据！")
        RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)


# 指针(开关)仪表盘告警
class PointerAlarmAL(object):
    def __init__(self):
        self.db_power_alarm_rule_data = DBPowerAlarmRuleDetail()
        self.db_alarm_manage = DBAlarmManage()
        self.db_power_alarm_rule = DBPowerAlarmRule()
        self.power_device_id = 1
        self.env_value = 0
        self.alarm_rule_indcamera_id = 1
        self.alarm_desc = ""

    # 获取告警比对参数
    def get_alarm_type(self, camera_data, item_name):
        res_list = []
        power_alarm_rule_id = camera_data.get("power_alarm_rule_id")
        logger.info(power_alarm_rule_id)
        logger.info(item_name)
        algorithm_data_list = self.db_power_alarm_rule_data.get_power_alarm_data(power_alarm_rule_id, item_name)
        if not algorithm_data_list:
            logger.info("没有告警比对参数{}".format(str(power_alarm_rule_id)))
            return False, None

        power_robot_item_id = camera_data.get("power_robot_item_id")

        for algorithm_data in algorithm_data_list:
            data_dic = {}
            data_dic["power_robot_item_id"] = power_robot_item_id
            data_dic["value"] = algorithm_data.get("value")
            data_dic["conditions"] = algorithm_data.get("conditions")
            data_dic["level"] = algorithm_data.get("level")
            res_list.append(data_dic)

        if res_list:
            return True, res_list
        return False, None,

    # 指针(开关)仪表盘告警对比规则(入口)
    def contrast_alarm(self, task_data, env_name, env_value):
        logger.info("指针(开关)仪表盘告警")
        logger.info(env_name)
        logger.info(env_value)
        self.alarm_desc = env_name + ":" + str(env_value)

        try:
            env_value = eval(env_value)
            self.env_value = env_value
        except:
            env_value = env_value
            self.env_value = env_value

        robot_position_id = task_data.get("robot_position_id")
        img_path = task_data.get("img_path")

        camera_data = task_data.get("camera_data")
        self.power_device_id = camera_data.get("power_device_id")
        logger.info(camera_data)
        logger.info(type(camera_data))
        power_robot_item_id = camera_data.get("power_robot_item_id")
        power_device_id = camera_data.get("power_device_id")

        data_state, alarm_data_list = self.get_alarm_type(camera_data, env_name)

        task_data["value"] = env_value

        if not data_state:
            logger.error("指针表盘告警对比错误，无item_id数据")
            return False
        for alarm_data_data in alarm_data_list:
            alive_data = DBPowerAlarm().get_power_alarm_data(robot_position_id, power_robot_item_id, power_device_id)
            if alive_data:
                alive_data["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
                alive_data["value"] = env_value
                alive_data["img_path"] = img_path
            alarm_data_conditions = alarm_data_data.get("conditions")
            alarm_data_value = float(alarm_data_data.get("value"))
            alarm_data_level = alarm_data_data.get("level")

            alarm_data_undo_alarm_data = self.db_power_alarm_rule.get_power_alarm_rule(
                alarm_data_data.get("power_alarm_rule_id"))
            if alarm_data_undo_alarm_data:
                alarm_data_undo_alarm = alarm_data_undo_alarm_data.get("undo_alarm")
            else:
                alarm_data_undo_alarm = 5
            logger.info("当前告警符号为")
            logger.info(alarm_data_conditions)
            logger.info("当前告警规则数值为")
            logger.info(alarm_data_value)
            logger.info("当前数据显示数值为")
            logger.info(env_value)
            logger.info(alive_data)

            if alarm_data_conditions == ">":
                if env_value > alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)

                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

            elif alarm_data_conditions == ">=":
                if env_value >= alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)
                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

            elif alarm_data_conditions == "<=":
                if env_value <= alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)
                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

            elif alarm_data_conditions == "=":
                if env_value == alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)
                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

            else:
                if env_value < alarm_data_value:
                    alarm_desc = env_name + ":" + str(env_value)
                    if alive_data:
                        # 更新告警
                        self.update_alarm_num(alive_data, img_path)
                    else:
                        # 写入新的告警
                        self.insert_alarm(task_data, 1, power_robot_item_id, alarm_data_level, alarm_desc)
                else:
                    if alive_data:
                        self.update_alarm_common_num(alive_data, alarm_data_undo_alarm)

    # 更新告警次数
    def update_alarm_num(self, alive_data, img_path):

        alarm_manage_id = alive_data.get("alarm_manage_id")
        num = alive_data.get("num")
        num += 1
        alive_data["num"] = num
        alive_data["img_path"] = img_path
        alive_data["value"] = self.env_value
        alive_data["alarm_desc"] = self.alarm_desc

        self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    # 更新正常次数
    def update_alarm_common_num(self, alive_data, alarm_data_undo_alarm):
        alarm_manage_id = alive_data.get("alarm_manage_id")
        common_num = alive_data.get("common_num")
        common_num += 1
        if common_num > alarm_data_undo_alarm:
            alive_data["undo_status"] = 1
            alive_data["alarm_desc"] = self.alarm_desc
            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)
        else:
            alive_data["alarm_desc"] = self.alarm_desc
            alive_data["common_num"] = common_num

            self.db_alarm_manage.update({"alarm_manage_id": alarm_manage_id}, alive_data)

    def insert_alarm(self, task_data, num, power_robot_item_id, level, alarm_desc):
        logger.info("开始插入数字仪表盘数据")
        logger.info(task_data)
        ret = {}
        ret["robot_id"] = task_data.get("robot_id")
        ret["core_room_id"] = task_data.get("core_room_id", "1")
        ret["robot_position_id"] = task_data.get("robot_position_id")
        ret["power_robot_item_id"] = power_robot_item_id
        ret["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
        ret["power_device_id"] = self.power_device_id
        ret["level"] = level
        ret["num"] = num
        ret["value"] = task_data.get("value")
        ret["assign"] = 0
        ret["img_path"] = task_data.get("img_path")
        ret["alarm_desc"] = alarm_desc
        ret["power_cabinet_id"] = task_data.get("power_cabinet_id", None)
        ret["core_cabinet_id"] = task_data.get("core_cabinet_id", None)
        alarm_manage_id = DBAlarmManage().insert_data(ret)

        data_dic = {"alarm_manage_id": alarm_manage_id, "notice_rule_id": self.alarm_rule_indcamera_id, "type": 10}
        rout = db_rabbit_mq.get("robot_warn")
        routing_ey = db_rabbit_mq.get("warn_rout_key")
        warn_queue = db_rabbit_mq.get("warn_queue")
        logger.info("发送MQ告数据！")
        RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
