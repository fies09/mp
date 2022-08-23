# 给升降杆中摄像头传参
import re
import numpy as np
from datetime import datetime
from configs.log import logger
from analysis.robot_alarm import PointerAlarmAL, PowerAlarmAL
from analysis.robot_request import post_agx_digital, post_agx_pointer, post_agx_switch
from schema.db_robot_device import DBRobotDevice
from schema.db_config_algorithm_model import DBConfigAlgorithmModel
from schema.db_power_camera_param import DBPowerCameraParam
from schema.db_power_inspect_data import DBPowerInspectDatum
from schema.db_power_robot_item import DBPowerRobotItem
from schema.db_robot_algorithm import DBRobotAlgorithm


# 算法结果验证函数
def is_value_correct(condition, *value):
    """
    检测值是否正确
    :param condition:
    :param value:
    :return:
    """
    if len(condition) > 0:
        mu = np.mean(condition)
        std = np.std(condition)
        if value[0] > mu - value[1] * std and value[0] < mu + value[1] * std:
            return True
        else:
            return False


def remove_additional_dot(result, index):
    """
    该函数用于移除额外识别的点，用于移除第一个数字前，和最后一个数字之后的点。
    :param result:
    :param index:
    :return:
    """
    res = result
    flag = 1
    while flag:
        if res[index] == '.':
            if index == -1:
                res = res[:index - 1]
            else:
                res = res[index + 1:]
        else:
            flag = 0
    return res


def preprocess(res_str):
    """
    1. 去除前后的点 '.' 2. 如果有点，输出结果
    :param res_str:
    :return:
    """
    res = remove_additional_dot(res_str, 0)  # 去除前面的点
    res = remove_additional_dot(res, -1)  # 去除后面的点
    return res


def postprocess(history_data, result, sigma):
    """
    拿以前结果计算是否在3sigma范围内。如果在，则输出结果。如果不在，
    则依次从第一个数字后添加点，依次循环。如果在3sigma范围内，则输出处理后的结果。如果不在，则输出异常。
    :param history_data:
    :param result:
    :param sigma:
    :return:
    """
    if result is None:
        return -1, ''
    if bool(re.search(r'\d', result)) is False:
        return -1, result
    result = preprocess(result)
    dots = re.findall(r'', result)
    if len(dots) == 1:
        if is_value_correct(history_data, float(result), 3):
            return 1, result
        else:
            return 0, result
    elif len(dots) > 1:
        return -1, result
    elif len(dots) == 0:
        for i in range(len(result)):
            pre_str = result[:i + 1]
            post_str = result[i + 1:]
            # comb = f"{pre_str}.{post_str}"
            comb = "{}.{}".format(pre_str, post_str)
            if is_value_correct(history_data, float(comb), sigma):
                return 1, str(comb)
        return 0, result


def section_judge(val, left_val, right_val):
    """
    判断val的值是否在left_val与right_val之间
    """
    if left_val <= val <= right_val or right_val >= val >= left_val:
        return True
    else:
        return False


class PointerDistinguishAL(object):
    def __init__(self):
        self.parameter_type = 2
        self.parameters = {}
        self.db_power_camera_param_obj = DBPowerCameraParam()
        self.db_power_inspect_data = DBPowerInspectDatum()
        self.db_config_algorithm_model = DBConfigAlgorithmModel()
        self.db_robot_algorithm = DBRobotAlgorithm()
        self.task_data = {}

    # 数字仪表盘算法参数转换
    def digital_camera_transform(self, camera_data):
        logger.info("仪表盘算法参数---->{}".format(str(camera_data)))
        parma_list = []
        for camera in camera_data:
            power_robot_item_id = camera.get("power_robot_item_id")
            item_name = DBPowerRobotItem().get_item_data(power_robot_item_id)
            logger.info("仪表盘算法检测项---->{}".format(str(item_name)))
            # 框的范围
            algorithm_param = camera.get("algorithm_param")

            algorithm_param["legend"] = item_name.get("name", "meter")
            algorithm_param["id"] = str(algorithm_param["id"])
            parma_list.append(algorithm_param)
        return parma_list

    # 指针仪表盘算法参数转换
    def pointer_camera_transform(self, camera_data):
        parma_list = []
        for camera in camera_data:
            power_robot_item_id = camera.get("power_robot_item_id")
            logger.info(power_robot_item_id)
            item_name = DBPowerRobotItem().get_item_data(power_robot_item_id)
            logger.info(item_name)
            # 框的范围
            algorithm_param = camera.get("algorithm_param")
            algorithm_param["legend"] = item_name.get("name")
            parma_list.append(algorithm_param)
        return parma_list

        # 指针仪表盘算法参数转换

    def switch_camera_transform(self, camera_data):

        parma_list = []
        for camera in camera_data:
            append_dict = {}

            power_robot_item_id = camera.get("power_robot_item_id")
            item_name = DBPowerRobotItem().get_item_data(power_robot_item_id)

            # 框的范围
            algorithm_param = camera.get("algorithm_param")
            template_data = algorithm_param.get("template", {})
            template = template_data.get("switch_type", "c")
            # 外框
            append_dict["id"] = str(algorithm_param.get("id"))
            append_dict["startCol"] = int(algorithm_param.get("startCol", 0))
            append_dict["startRow"] = int(algorithm_param.get("startRow", 0))
            append_dict["endCol"] = int(algorithm_param.get("endCol", 0))
            append_dict["endRow"] = int(algorithm_param.get("endRow", 0))
            append_dict["legend"] = item_name.get("name", "开关状态")
            append_dict["template"] = template

            parma_list.append(append_dict)

        return parma_list

    def get_al_item_data(self, robot_algorithm_id, lengend_en_name):
        data = DBPowerRobotItem().get_item_id(robot_algorithm_id, lengend_en_name)

        if data:
            item_id = data.get("power_robot_item_id")
            type_num = data.get("type")
        else:
            item_id = 1
            type_num = 1
        return item_id, type_num

    def get_switch_item_data(self, robot_algorithm_id, config_algorithm_model_id):
        data = DBPowerRobotItem().get_model_item_id(robot_algorithm_id, config_algorithm_model_id)
        if data:
            item_id = data.get("power_robot_item_id")
            type_num = data.get("type")
        else:
            item_id = 1
            type_num = 1
        return item_id, type_num

    # 指针仪表盘进行检测
    def pointer_dashboard_al(self, camera_data, power_device_id, power_cabinet_id, robot_algorithm_id, photo_name,
                             task_data):
        logger.info("开始进行指针仪表盘解析识别")
        self.task_data = task_data
        pointer_alarm = PointerAlarmAL()
        try:
            camera_al_data = self.pointer_camera_transform(camera_data)
        except Exception as e:
            logger.error(e)
            logger.error("参数错误，请查看数据库中配置参数是否正确")
            return False
        logger.info("指针仪表盘进行检测传入算法所需参数-------》{}".format(str(camera_al_data)))

        logger.info("合成后的图片-------》{}".format(photo_name))
        robot_device_id = camera_data[0].get("robot_device_id")
        db_robot_device = DBRobotDevice()
        robot_device_data = db_robot_device.get_device_id(robot_device_id)
        if "工业" in robot_device_data["name"]:
            parameter_type = 2
        else:
            parameter_type = 1
        light_detect_result, light_path, plot_data = post_agx_pointer(photo_name, camera_al_data, parameter_type)
        self.task_data["img_path"] = light_path
        logger.info("照片的解析" + str(light_detect_result))
        logger.info("请求power_indcamera_param------》id{}数据的结果为{}".format(power_device_id, str(light_detect_result)))
        power_robot_item_id = camera_data[0].get("power_robot_item_id")
        power_inspect_data_id_list = list()
        if not light_detect_result:
            logger.info("没有解析指针仪表盘结果{}".format(power_device_id))
            logger.info("检测结果{}".format(str(light_detect_result)))
            ret = {}
            try:
                lengend_en_name = camera_al_data[0].get("legend")
            except Exception as e:
                logger.error(e)
                return False
            # lengend_en_name = lengend_en.get("name")
            logger.info("检测结果项{}".format(lengend_en_name))
            value = "未识别"
            logger.info("检测结果值{}".format(value))

            item_id, type_num = self.get_al_item_data(robot_algorithm_id, lengend_en_name)
            # # 添加告警信息
            logger.info("告警算法ID为{}".format(robot_algorithm_id))
            logger.info("告警项ID为{}".format(power_robot_item_id))
            camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id, power_robot_item_id)
            logger.info("告警参数为{}".format(str(camera_data)))
            if camera_data:
                self.task_data["camera_data"] = camera_data
                try:
                    pointer_alarm.contrast_alarm(self.task_data, lengend_en_name, value)
                except Exception as e:
                    logger.error(e)
            else:
                logger.info("动力告警参数错误，不执行告警动作")

            ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
            ret["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")

            ret['user_id'] = self.task_data.get("user_id")
            ret['robot_id'] = self.task_data.get("robot_id")
            ret['power_robot_item_id'] = power_robot_item_id
            ret["robot_position_id"] = self.task_data.get("robot_position_id")
            ret["power_cabinet_id"] = power_cabinet_id
            ret["power_device_id"] = power_device_id
            ret["value"] = value
            # 检测项
            ret["img_path"] = light_path
            # 临时写死，7被占用
            ret["type"] = 8
            ret["create_time"] = datetime.now()

            power_inspect_data_id = DBPowerInspectDatum().insert(ret)
            logger.info("插入ID数据为{}".format(str(power_inspect_data_id)))
            logger.info("插入数据为{}".format(str(value)))
            power_inspect_data_id_list.append(power_inspect_data_id)
            return plot_data, power_inspect_data_id_list
        self.task_data["power_device_id"] = power_device_id
        self.task_data["img_path"] = light_path
        logger.info(light_detect_result)
        for detect_result in light_detect_result:
            logger.info("检测结果{}".format(str(detect_result)))
            ret = {}
            lengend_en_name = detect_result.get("legend")
            logger.info("检测结果项{}".format(lengend_en_name))
            value = detect_result.get("res", "未识别")
            if not value:
                value = "未识别"
            else:
                value_num = re.findall(r"-?\d+\.?\d*", str(value))
                if value_num:
                    value = float(value_num[0])
                else:
                    value = "未识别"
            logger.info("检测结果值{}".format(value))

            item_id, type_num = self.get_al_item_data(robot_algorithm_id, lengend_en_name)
            # # 添加告警信息
            logger.info("告警算法ID为{}".format(robot_algorithm_id))
            logger.info("告警项ID为{}".format(power_robot_item_id))
            camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id, power_robot_item_id)
            logger.info("告警参数为{}".format(str(camera_data)))
            if camera_data:
                self.task_data["camera_data"] = camera_data
                try:
                    if value != "未识别":
                        pointer_alarm.contrast_alarm(self.task_data, lengend_en_name, value)
                except Exception as e:
                    logger.error(e)
            else:
                logger.info("动力告警参数错误，不执行告警动作")
            ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
            ret["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")

            ret['user_id'] = self.task_data.get("user_id")
            ret['robot_id'] = self.task_data.get("robot_id")
            ret['power_robot_item_id'] = power_robot_item_id
            ret["robot_position_id"] = self.task_data.get("robot_position_id")
            ret["power_cabinet_id"] = power_cabinet_id
            ret["power_device_id"] = power_device_id
            ret["value"] = value
            # 检测项
            ret["img_path"] = light_path
            # ret["type"] = type_num
            # 临时写死，7被占用
            ret["type"] = 8
            ret["create_time"] = datetime.now()
            power_inspect_data_id = DBPowerInspectDatum().insert(ret)
            logger.info("插入ID数据为{}".format(str(power_inspect_data_id)))
            logger.info("插入数据为{}".format(str(value)))
            power_inspect_data_id_list.append(power_inspect_data_id)
            logger.info("{}动力巡检指针检测成功".format(light_path))
            logger.info("{}指针仪表盘识别结果".format(light_detect_result))
        return plot_data, power_inspect_data_id_list

    # 开关进行检测
    def switch_dashboard_al(self, all_camera_data, power_device_id, power_cabinet_id, robot_algorithm_id, photo_name,
                            task_data):
        logger.info("开始进行开关解析识别")
        self.task_data = task_data
        logger.info(all_camera_data)
        pointer_alarm = PointerAlarmAL()
        for camera_data in all_camera_data:
            camera_data = [camera_data]
            try:
                camera_al_data = self.switch_camera_transform(camera_data)
            except Exception as e:
                logger.error(e)
                logger.error("参数错误，请查看数据库中配置参数是否正确")
                return False
            logger.info("传入算法所需参数图片-------》{}".format(str(camera_al_data)))

            logger.info("合成后的图片-------》{}".format(photo_name))
            robot_device_id = camera_data[0].get("robot_device_id")
            db_robot_device = DBRobotDevice()
            robot_device_data = db_robot_device.get_device_id(robot_device_id)
            if "工业" in robot_device_data["name"]:
                parameter_type = 2
            else:
                parameter_type = 1

            light_detect_result, light_path, plot_data = post_agx_switch(photo_name, camera_al_data, parameter_type)
            self.task_data["img_path"] = light_path
            logger.info("照片的解析" + str(light_detect_result))
            logger.info("请求power_indcamera_param------》id{}数据的结果为{}".format(power_device_id, str(light_detect_result)))
            # 获取动力检测名称
            power_robot_item_id = camera_data[0].get("power_robot_item_id")
            item_name = DBPowerRobotItem().get_item_data(power_robot_item_id)
            logger.info(item_name)
            lengend_en_name = item_name.get("name", "开关状态")
            power_inspect_data_id_list = list()
            if not light_detect_result:
                logger.info("没有解析开关结果{}".format(power_device_id))
                logger.info("检测结果{}".format(str(light_detect_result)))
                ret = {}
                logger.info("检测结果项{}".format(lengend_en_name))
                value = "未识别"
                logger.info("检测结果值{}".format(value))
                item_id, type_num = self.get_al_item_data(robot_algorithm_id, lengend_en_name)

                # # 添加告警信息
                logger.info("告警算法ID为{}".format(robot_algorithm_id))
                logger.info("告警项ID为{}".format(power_robot_item_id))
                camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id,
                                                                                  power_robot_item_id)

                logger.info("告警参数为{}".format(str(camera_data)))
                if camera_data:
                    self.task_data["camera_data"] = camera_data
                    try:
                        if value != "未识别":
                            pointer_alarm.contrast_alarm(self.task_data, lengend_en_name, value)
                    except Exception as e:
                        logger.error(e)
                else:
                    logger.info("动力告警参数错误，不执行告警动作")

                ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id", "0000000")
                ret["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")

                ret['user_id'] = self.task_data.get("user_id", 1)
                ret['robot_id'] = self.task_data.get("robot_id", 1)
                ret['power_robot_item_id'] = power_robot_item_id
                ret["robot_position_id"] = self.task_data.get("robot_position_id")
                ret["power_cabinet_id"] = power_cabinet_id
                ret["power_device_id"] = power_device_id

                ret["value"] = value
                # 检测项
                ret["img_path"] = light_path
                # 临时写死，7被占用
                ret["type"] = 9
                ret["create_time"] = datetime.now()

                power_inspect_data_id = DBPowerInspectDatum().insert(ret)
                power_inspect_data_id_list.append(power_inspect_data_id)
                return plot_data, power_inspect_data_id_list

            self.task_data["power_device_id"] = power_device_id
            self.task_data["img_path"] = light_path
            logger.info(light_detect_result)
            for detect_result in light_detect_result:

                logger.info("检测结果{}".format(str(detect_result)))
                ret = {}
                config_algorithm_model_id = camera_data[0].get("config_algorithm_model_id")
                logger.info("检测结果model项{}".format(config_algorithm_model_id))
                value = str(detect_result.get("res", "未识别"))
                logger.info("检测结果值{}".format(value))
                logger.info(robot_algorithm_id)
                logger.info(config_algorithm_model_id)
                item_id, type_num = self.get_switch_item_data(robot_algorithm_id, config_algorithm_model_id)
                # # 添加告警信息
                logger.info("告警算法ID为{}".format(robot_algorithm_id))
                logger.info("告警项ID为{}".format(power_robot_item_id))
                camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id,
                                                                                  power_robot_item_id)
                logger.info("告警参数为{}".format(str(camera_data)))
                # 映射开关算法对应的值
                try:
                    if value != "未识别":
                        new_value = "未识别"
                        # value = value.lstrip().rstrip().split(":")[-1]
                        logger.info("当前的值======================================{}".format(value))
                        mapping_rule = camera_data.get("algorithm_param")
                        # switch = mapping_rule.get("switch")
                        # angle_scope = switch.get("angle_scope", None)  # 开关判定范围
                        # angle_name = switch.get("angle_name", None)  # 开关映射
                        logger.info(mapping_rule)
                        template = mapping_rule.get("template", {})
                        angle_scope = template["gear"]["gearAngels"]
                        logger.info(angle_scope)

                        # 如果算法识别的值为负值，则给这个负值结果加360，得到最终值。
                        if int(value) < 0:
                            value = int(value) + 360
                        # 判断范围
                        for angle in angle_scope:
                            jump_out_val = False
                            angle_list = angle["interval"]
                            scope_value = angle["value"]
                            for scope in angle_list:
                                scope_list = scope.split(",")
                                left_val = scope_list[0]
                                right_val = scope_list[1]
                                judge_res = section_judge(int(value), int(left_val), int(right_val))
                                perigon_value = 360 - int(value)
                                # 如果匹配不到数据,则根据当前角度的周角再做一次判断。
                                judge_res_perigon = section_judge(int(perigon_value), int(left_val), int(right_val))
                                if judge_res or judge_res_perigon:  # 如果匹配到对应范围，则映射出此角度的真实值
                                    new_value = scope_value
                                    jump_out_val = True
                                    break
                            if jump_out_val:
                                break
                        logger.info("映射后的值======================================{}".format(new_value))
                    else:
                        new_value = "未识别"
                except Exception as e:
                    logger.error("开关映射值获取错误")
                    logger.error(e)
                    new_value = "未识别"
                logger.info(new_value)
                if camera_data:
                    self.task_data["camera_data"] = camera_data
                    try:
                        if value != "未识别":
                            pointer_alarm.contrast_alarm(self.task_data, lengend_en_name, new_value)
                    except Exception as e:
                        logger.error(e)
                else:
                    logger.info("动力告警参数错误，不执行告警动作")

                ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
                ret["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")

                ret['user_id'] = self.task_data.get("user_id")
                ret['robot_id'] = self.task_data.get("robot_id")
                ret['power_robot_item_id'] = power_robot_item_id
                ret["robot_position_id"] = self.task_data.get("robot_position_id")
                ret["power_cabinet_id"] = power_cabinet_id
                ret["power_device_id"] = power_device_id
                ret["value"] = new_value
                # 检测项
                ret["img_path"] = light_path
                # ret["type"] = type_num
                # 临时写死，7被占用
                ret["type"] = 9
                ret["create_time"] = datetime.now()
                power_inspect_data_id = DBPowerInspectDatum().insert(ret)
                logger.info("插入ID数据为{}".format(str(power_inspect_data_id)))
                logger.info("插入数据为{}".format(str(value)))
                power_inspect_data_id_list.append(power_inspect_data_id)
            logger.info("{}动力巡检开关进行检测成功".format(light_path))
            logger.info("{}开关进行检测识别结果".format(light_detect_result))
            return plot_data, power_inspect_data_id_list

    # 数字仪表盘进行检测
    def digital_dashboard_al(self, camera_data, power_device_id, power_cabinet_id, robot_algorithm_id, photo_name,
                             task_data):
        # 进行识别
        logger.info("开始进行数字仪表盘解析识别")
        self.task_data = task_data
        # power_alarm = PowerAlarmAL()
        try:
            camera_al_data = self.digital_camera_transform(camera_data)
        except Exception as e:
            logger.error(e)
            logger.error("参数错误，请查看数据库中配置参数是否正确")
            return False
        logger.info("传入算法所需参数图片-------》{}".format(str(camera_al_data)))
        logger.info("合成后的图片-------》{}".format(photo_name))
        robot_device_id = camera_data[0].get("robot_device_id")
        db_robot_device = DBRobotDevice()
        robot_device_data = db_robot_device.get_device_id(robot_device_id)
        if "工业" in robot_device_data["name"]:
            parameter_type = 2
        else:
            parameter_type = 1
        light_detect_result, light_path, plot_data = post_agx_digital(photo_name, camera_al_data, parameter_type)
        power_robot_item_id = camera_data[0].get("power_robot_item_id")
        self.task_data["img_path"] = light_path
        power_inspect_data_id_list = list()
        if not light_detect_result:
            for camera in camera_al_data:
                logger.info("没有解析数字仪表盘结果{}".format(power_device_id))
                ret = {}
                lengend_en_name = camera.get("legend")
                if lengend_en_name == "meter":
                    continue
                value = "未识别"
                item_id, type_num = self.get_al_item_data(robot_algorithm_id, lengend_en_name)
                # # 添加告警信息
                logger.info("告警算法ID为{}".format(robot_algorithm_id))
                logger.info("告警项ID为{}".format(power_robot_item_id))
                camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id,
                                                                                  power_robot_item_id)
                logger.info("告警参数为{}".format(str(camera_data)))

                ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
                ret["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")

                ret['user_id'] = self.task_data.get("user_id")
                ret['robot_id'] = self.task_data.get("robot_id")
                ret['power_robot_item_id'] = power_robot_item_id
                ret["robot_position_id"] = self.task_data.get("robot_position_id")
                ret["power_cabinet_id"] = power_cabinet_id
                ret["power_device_id"] = power_device_id
                ret["value"] = value
                # 检测项
                ret["img_path"] = light_path
                # 临时写死，7被占用
                ret["type"] = 10
                ret["create_time"] = datetime.now()
                ret["robot_device_id"] = robot_device_id
                power_inspect_data_id = DBPowerInspectDatum().insert(ret)
                logger.info("插入ID数据为{}".format(str(power_inspect_data_id)))
                logger.info("插入数据为{}".format(str(value)))
                power_inspect_data_id_list.append(power_inspect_data_id)
            return plot_data, power_inspect_data_id_list

        logger.info("照片的解析" + str(light_detect_result))
        logger.info("请求power_indcamera_param------》id{}数据的结果为{}".format(power_device_id, str(light_detect_result)))

        self.task_data["power_device_id"] = power_device_id
        self.task_data["img_path"] = light_path
        # relative_path = str(light_path).split(Path().APP_PATH)[-1]
        for detect_result in [light_detect_result[0]]:
            ret = {}
            # lengend_en_name = detect_result.get("legend")
            for result in detect_result["items"]:
                logger.info("识别结果分析{}".format(str(result)))
                value = result.get("res", "未识别")
                lengend_en_name = result.get("label")
                item_id, type_num = self.get_al_item_data(robot_algorithm_id, lengend_en_name)
                if not value:
                    value = "未识别"
                else:
                    try:
                        value_num = re.findall(r"-?\d+\.?\d*", str(value))
                        value = value_num[0]
                        # 这里已获取到数据，根据已获取的数据判断最终结果
                        # 先获取历史数据
                        history_data = DBPowerInspectDatum().get_item_data(power_robot_item_id=power_robot_item_id)[
                                       0:30]
                        if history_data:
                            history_list = list()
                            for hd in history_data:
                                if hd["value"] not in ["未识别", "与历史数据不匹配"]:
                                    history_list.append(float(hd["value"]))
                            history_list = list(set(history_list))
                            logger.info(history_list)
                            # 对比历史数据，获取最终的值
                            if len(history_list) > 10:
                                judge_status, judge_res = postprocess(history_list, str(value), 3)
                                logger.info("与历史数据匹配后的状态>>> {}".format(judge_status))
                                if int(judge_status) == 1:
                                    value = judge_res
                                # 多次识别之后变为未识别
                                # else:
                                #     value = "未识别"
                                else:
                                    value = judge_res
                    except Exception as e:
                        logger.error(e)
                        logger.error("数据类比错误！")
                        value = "未识别"

                # # 添加告警信息
                logger.info("告警算法ID为{}".format(robot_algorithm_id))
                logger.info("告警项ID为{}".format(power_robot_item_id))
                camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id,
                                                                                  power_robot_item_id)
                logger.info("告警参数为{}".format(str(camera_data)))


                ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
                ret["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")

                ret['user_id'] = self.task_data.get("user_id")
                ret['robot_id'] = self.task_data.get("robot_id")
                ret['power_robot_item_id'] = power_robot_item_id
                ret["robot_position_id"] = self.task_data.get("robot_position_id")
                ret["power_cabinet_id"] = power_cabinet_id
                ret["power_device_id"] = power_device_id
                ret["value"] = value
                # 检测项
                ret["img_path"] = light_path
                # ret["type"] = type_num
                # 临时写死，7被占用
                ret["type"] = 10
                ret["create_time"] = datetime.now()
                power_inspect_data_id = DBPowerInspectDatum().insert(ret)
                logger.info("插入ID数据为{}".format(str(power_inspect_data_id)))
                logger.info("插入数据为{}".format(str(value)))

                power_inspect_data_id_list.append(power_inspect_data_id)
        logger.info("{}动力巡检检测成功".format(light_path))
        logger.info("{}数字仪表盘识别结果".format(light_detect_result))
        return plot_data, power_inspect_data_id_list


if __name__ == '__main__':
    import rospy

    rospy.init_node("Mian_test")
    # result = {'task_data': {'img_path': '/home/robot/Dev/dispatch_agx_web/data_pkg/ther_path/2021_10_24/17_03_50.jpg',
    #                         'now_battery_power': 50, 'robot_task_type_id': None, 'robot_path_id': 1,
    #                         'robot_position_id': 8,
    #                         'is_break': 0, 'task_status': 2, 'obstacle': 0, 'task_type_name': '动力巡检', 'value': 24.5,
    #                         'robot_id': 1,
    #                         'status': 1, 'minimum_battery': 20, 'inspect_project_detail_id': 1448370089},
    #           'power_cabinet_id': 1,
    #           'ret': {'distance': 600, 'robot': None, 'del_flag': 0, 'core_cabinet_id': 0, 'update_time': None,
    #                   'type': 1,
    #                   'create_by': 1, 'remark': '8', 'robot_id': 1, 'status': 0, 'name': 'c1前方',
    #                   'create_time': '2021-10-24T11:56:18', 'current_state': 0, 'robot_path_id': 1,
    #                   'position': {'x': '1.12', 'yaw': '3.12', 'y': '2.00'}, 'direction': None, 'update_by': None,
    #                   'position_num': 8,
    #                   'robot_path': None, 'camera_config': None, 'device_id': 0, 'power_cabinet_id': 1,
    #                   'robot_position_id': 8}}
    # {"Width": 4032, "Height": 3036, "OffsetX": 0, "OffsetY": 0, "Brightness": 0,
    #  "gainParams": {"Gain": 0, "GainAuto": 0}, "catchParams": {"repeate_num": 1, "interval_time": 0},
    #  "gammaParams": {"Gamma": 0, "GammaEnable": 0}, "GammaSelector": 1,
    #  "exposureParams": {"ExposureAuto": "0", "ExposureTime": 30000}}
    """
    {'task_data': {'is_break': 0, 'task_type_name': '动力巡检', 'core_room_id': 1, 'robot_id': 1, 'now_battery_power': 100, 'robot_task_type_id': None, 'obstacle': 0, 'robot_path_id': 3, 'inspect_project_detail_id': 1336190473, 'task_status': 2, 'status': 1, 'minimum_battery': 20, 'robot_position_id': 28}, 'power_cabinet_id': 1, 'ret': {'name': '动力前方', 'robot_id': 1, 'direction': None, 'position': {'y': '-2.76', 'yaw': '1.62', 'x': '2.37'}, 'current_state': 0, 'position_num': 11, 'update_time': '2021-12-01T16:51:44', 'robot_path': None, 'robot_path_id': 3, 'power_cabinet_id': 1, 'create_time': '2021-10-28T11:50:08', 'del_flag': 0, 'camera_config': None, 'status': 0, 'device_id': 0, 'robot_position_id': 28, 'robot': None, 'update_by': 1, 'type': 1, 'distance': 0, 'remark': '', 'create_by': 2, 'core_cabinet_id': 0}}
    """

    # res = {
    #     'ret': {'position_num': 8, 'direction': None, 'camera_config': None, 'core_cabinet_id': 0, 'robot_path_id': 1,
    #             'robot_position_id': 8, 'del_flag': 0, 'robot': None, 'current_state': 0, 'power_cabinet_id': 1,
    #             'name': 'c1前方', 'create_time': '2021-10-24T11:56:18', 'robot_path': None, 'status': 0, 'create_by': 1,
    #             'distance': 600, 'position': {'yaw': '3.12', 'y': '2.00', 'x': '1.12'}, 'robot_id': 1, 'type': 1,
    #             'remark': '8', 'update_time': None, 'device_id': 0, 'update_by': None}, 'power_cabinet_id': 1,
    #     'task_data': {'task_status': 2,
    #                   'img_path': '/home/robot/Dev/dispatch_agx_web/data_pkg/ther_path/2021_10_09/00_38_14.jpg',
    #                   'now_battery_power': 81, 'value': 26.1, 'obstacle': 0, 'robot_id': 1,
    #                   'inspect_project_detail_id': 1437132816, 'robot_path_id': 1, 'robot_position_id': 8,
    #                   'robot_task_type_id': None, 'minimum_battery': 20, 'status': 1, 'is_break': 0,
    #                   'task_type_name': '动力巡检'}}
    res = {'task_data': {'task_type_name': '动力巡检', 'robot_id': 1, 'robot_task_type_id': None, 'robot_position_id': 28,
                         'robot_path_id': 3, 'core_room_id': 1, 'task_status': 2,
                         'inspect_project_detail_id': 1911274248, 'obstacle': 0, 'now_battery_power': 22, 'is_break': 0,
                         'status': 1, 'minimum_battery': 20}, 'power_cabinet_id': 1,
           'ret': {'robot_path': None, 'position_num': 11, 'robot_position_id': 28, 'robot_path_id': 3,
                   'core_cabinet_id': 0, 'robot': None, 'update_time': '2021-12-01T16:51:44', 'camera_config': None,
                   'current_state': 0, 'update_by': 1, 'remark': '', 'status': 0,
                   'position': {'y': '-2.76', 'yaw': '1.62', 'x': '2.37'}, 'power_cabinet_id': 1, 'create_by': 2,
                   'name': '动力前方', 'del_flag': 0, 'device_id': 0, 'create_time': '2021-10-28T11:50:08',
                   'direction': None, 'type': 1, 'robot_id': 1, 'distance': 0}}

    PointerDistinguishAL().power_photo_action(res)
