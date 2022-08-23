# 给升降杆中摄像头传参
import os
import re
import time
import numpy as np
from datetime import datetime
from configs import analysis_api
import requests
from requests.auth import HTTPDigestAuth
from threading import Thread
from task.blend import lightBlend
from configs.log import logger
from core.dict_config import camera_type
from task.light_al import post_agx_pointer, post_agx_digital, post_agx_switch, get_industry_photo, set_industry_parameter
from modules.mainManage import set_devicePrama, loging
from modules.robot_hardware_server import robot_lifter
from modules.robot_lifter import RobotLifer
from task.robot_alarm import PowerAlarmAL, PointerAlarmAL

from schema.db_config_algorithm_model import DBConfigAlgorithmModel
from schema.db_power_camera_param import DBPowerCameraParam
from schema.db_power_device import DBPowerDevice
# from schema.db_power_device_location import DBPowerDeviceLocation
from schema.db_power_inspect_data import DBPowerInspectDatum
from schema.db_power_robot_item import DBPowerRobotItem
from schema.db_robot_algorithm import DBRobotAlgorithm
from schema.db_robot_device import DBRobotDevice
from schema.db_power_panel import DBPowerPanel
from analysis.robot_analysis import task_analysis
from utils.file_path import Path
from modules.robot_hardware_server import RobotDeviceStatus
from schema.db_alarm_manage import DBAlarmManage
from task import hardware_exception
from configs import db_rabbit_mq
from core.rabbitmq_db import RabbitPublisher

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
    dots = re.findall(r'..', result)
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


class PointerDistinguishAL(object):
    def __init__(self):
        self.parameter_type = 2
        self.parameters = {}
        self.db_power_camera_param_obj = DBPowerCameraParam()
        self.db_power_inspect_data = DBPowerInspectDatum()
        self.db_config_algorithm_model = DBConfigAlgorithmModel()
        self.db_robot_algorithm = DBRobotAlgorithm()

    # 调取工业相机拍照动作
    def action_industry_photo(self, id, photo_path, parameters):
        photo_num = parameters.get("PhotoNum", 0)
        photo_interval_time = parameters.get("PhotoIntervalTime", 0)
        logger.info("拍照数量{}".format(photo_num))
        logger.info("拍照间隔时间{}s".format(photo_interval_time))
        if not os.path.exists(photo_path):
            os.makedirs(photo_path)
        file_name_time = photo_path + datetime.now().strftime("%H_%M_%S")
        file_name_base = photo_path + datetime.now().strftime("%H_%M_%S") + ".jpg"
        logger.info(str(photo_path))
        if int(photo_num) == 1:
            photo_state = get_industry_photo(id, file_name_base)
            if not photo_state:
                return False, None
            logger.info("拍照完成")
            return True, file_name_base

        for num in range(int(photo_num)):
            file_name = file_name_time + "_{}.jpg".format(num)
            photo_state = get_industry_photo(id, file_name)
            time.sleep(photo_interval_time)
            if not photo_state:
                return False, None
        blend_state, photo_name = lightBlend(file_name_base, int(photo_num))
        if not blend_state:
            return False, None
        return True, photo_name

    # 登录并设置四倍相机参数
    def set_fourfold_parameter(self, camera_datas, res):
        # logger.info(camera_datas)
        address, port, username, passwd = res.get("address"), res.get("port"), \
                                          res.get("username"), res.get("passwd")

        if address == "0":
            return False
        # 摄像头传参
        # logger.info(camera_datas.get("device_param"))
        try:
            self.parameters = camera_datas.get("device_param")
            self.set_parameter(address, port, username, passwd)
            login_state = self.camera_login(res)
            if not login_state:
                logger.error(address + "摄像头login参数错误")
                return False
            return True
        except Exception as e:
            self.parameters = camera_datas.get("device_param")
            logger.error(e)

    # # 调取拍照动作
    # def action_fourfold_photo(self):
    #     photo_path = Path().photo + datetime.now().strftime("%Y_%m_%d") + "/"
    #     photo_state, file_path = self.photo_action(photo_path=photo_path)
    #
    #     if not photo_state:
    #         logger.error("摄像头拍照错误")
    #         return False
    #     logger.info("给算法接口中发送图片")
    #     # 解析照片
    #     return file_path

    # 登录摄像头
    def camera_login(self, res):
        address, port, username, passwd = res.get("address"), res.get("port"), \
                                          res.get("username"), res.get("passwd")

        logger.info('当前ip：' + address)

        # ISAPI方式登录 返回值为请求后返回的所有数据 其中就包括拍照后照片的数据
        self.login_state = self.loging(address, port, username, passwd)
        if self.login_state == False:
            logger.error(address + "摄像头login参数错误")
            return False
        return True

    # 四倍调取抓图动作
    def fourfold_photo_action(self, photo_path):
        catchParams = self.parameters.get("catchParams")
        if catchParams:
            nums = catchParams.get("repeate_num")
            interval_time = catchParams.get("interval_time")
            logger.info("拍照数量{}".format(nums))
            logger.info("间隔时间{}".format(interval_time))
        else:
            nums = 1
            interval_time = 0
        logger.info("拍照数量{}".format(nums))
        # logger.info("间隔时间{}".format(interval_time))

        if not os.path.exists(photo_path):
            os.makedirs(photo_path)
        file_name_time = photo_path + datetime.now().strftime("%H_%M_%S")
        file_name_base = photo_path + datetime.now().strftime("%H_%M_%S") + ".jpg"

        if nums == 1:
            photo_state = self.photo_get(file_name_base)
            if not photo_state:
                return False, None
            return True, file_name_base

        for num in range(nums):
            file_name = file_name_time + "_{}.jpg".format(num)
            photo_state = self.photo_get(file_name)
            time.sleep(interval_time)
            if not photo_state:
                return False, None
        blend_state, photo_name = lightBlend(file_name_base, nums)
        if not blend_state:
            return False, None
        return True, photo_name

    # 执行登录操作（/picture为抓图动作）
    def loging(self, address, port, username, passwd):
        url = "http://" + address + "/ISAPI/Streaming/channels/1" + "/picture"
        try:
            resp = requests.get(url, auth=HTTPDigestAuth(username, passwd))
            return resp.content
        except:
            return False

    # 调取拍照动作 此方法只是把登录后以获取好的数据截取后保存为图片
    def photo_get(self, file_name):
        try:
            with open(file_name, 'wb') as f:
                f.write(self.login_state)
            return True
        except:
            return False

    # 数字仪表盘算法参数转换
    def digital_camera_transform(self, camera_data):
        logger.info("仪表盘算法参数---->{}".format(str(camera_data)))
        parma_list = []
        for camera in camera_data:

            append_dict = {}

            power_robot_item_id = camera.get("power_robot_item_id")
            item_name = DBPowerRobotItem().get_item_data(power_robot_item_id)
            logger.info("仪表盘算法检测项---->{}".format(str(item_name)))
            # 框的范围
            algorithm_param = camera.get("algorithm_param")
            # 这是临时测试数据
            algorithm_param = {"b_params": {"start_imgCols": 12,"start_imgRows": 160,"end_imgCols": 193,"end_imgRows": 309,},"b_id": 1}
            logger.info("---->{}".format(str(item_name.get("name"))))
            b_params = algorithm_param.get("b_params")

            if not b_params:
                b_params = {}
                logger.error("数据库算法参数错误")
            # 获取外框的值
            b_id = algorithm_param.get("b_id")
            if not b_id and b_id != 0:
                continue
            try:
                append_dict["id"] = int(b_id)
                append_dict["startCol"] = int(b_params.get("start_imgCols", 0))
                append_dict["startRow"] = int(b_params.get("start_imgRows", 0))
                append_dict["endCol"] = int(b_params.get("end_imgCols", 0))
                append_dict["endRow"] = int(b_params.get("end_imgRows", 0))
                append_dict["legend"] = "meter"
            except Exception as e:
                logger.error(e)
                logger.error("算法参数获取错误")
                continue

            # 在算法模型配置表中获取获取内框的值
            config_algorithm_model_id = camera.get("config_algorithm_model_id")
            model_data = self.db_config_algorithm_model.get_model_data(config_algorithm_model_id)
            parameters = model_data.get("parameters")
            template = parameters.get("template", None)
            if template:
                append_dict["template"] = template
            else:
                logger.error("算法模型数据获取错误！")
            parma_list.append(append_dict)

        return parma_list

    # 指针仪表盘算法参数转换
    def pointer_camera_transform(self, camera_data):
        parma_list = []
        for camera in camera_data:
            append_dict = {}

            power_robot_item_id = camera.get("power_robot_item_id")
            item_name = DBPowerRobotItem().get_item_data(power_robot_item_id)

            # 框的范围
            algorithm_param = camera.get("algorithm_param")
            b_params = algorithm_param.get("b_params")
            # 外框
            if not b_params:
                b_params = {}
            append_dict["id"] = camera.get("b_id")
            append_dict["startCol"] = int(b_params.get("start_imgCols", 0))
            append_dict["startRow"] = int(b_params.get("start_imgRows", 0))
            append_dict["endCol"] = int(b_params.get("end_imgCols", 0))
            append_dict["endRow"] = int(b_params.get("end_imgRows", 0))
            append_dict["legend"] = item_name.get("name")
            parma_list.append(append_dict)

        return parma_list

        # 指针仪表盘算法参数转换

    def switch_camera_transform(self, camera_data):
        """
        {"b_id": 0,
        "b_params": {"end_imgCols": 913, "end_imgRows": 1058, "start_imgCols": 610, "start_imgRows": 879}, "
        u_params": {}, "configAlgorithmModelId": 1}
        {"b_id": 1,
        "u_id": 2,
        "b_params": {"end_imgCols": 2539, "end_imgRows": 521, "start_imgCols": 2097, "start_imgRows": 73},
        "u_params": {"end_imgCols": 2397, "end_imgRows": 211, "start_imgCols": 2261, "start_imgRows": 161},
        "configAlgorithmModelId": 3}

        :param camera_data:
        :return:
        """
        parma_list = []
        for camera in camera_data:
            append_dict = {}

            power_robot_item_id = camera.get("power_robot_item_id")
            item_name = DBPowerRobotItem().get_item_data(power_robot_item_id)

            # 框的范围
            algorithm_param = camera.get("algorithm_param")
            b_params = algorithm_param.get("b_params")
            if not b_params:
                b_params = {}
            # 外框
            append_dict["rswitchKind"] = algorithm_param.get("b_id")
            append_dict["startCol"] = int(b_params.get("start_imgCols", 0))
            append_dict["startRow"] = int(b_params.get("start_imgRows", 0))
            append_dict["endCol"] = int(b_params.get("end_imgCols", 0))
            append_dict["endRow"] = int(b_params.get("end_imgRows", 0))
            append_dict["legend"] = item_name.get("name")
            parma_list.append(append_dict)

        return parma_list

    def set_camera(self, url, params, tag):
        """
        发送请求设置相机参数
        """
        response = requests.get(url=url, params=params)
        if response.status_code == 200:
            logger.info("{}设置成功！".format(tag))
        else:
            logger.info("{}设置失败！".format(tag))

    # 给四倍摄像头传参
    def set_parameter(self,address, port, username, passwd):
        focusParams = self.parameters.get("focusParams")
        mode = focusParams.get("focusMode")
        distance = focusParams.get("minFocusDistance")
        opticalZoomLevel = focusParams.get("opticalZoomLevel")

        exposureParams = self.parameters.get("exposureParams")
        gain = exposureParams.get("gain")
        iris = exposureParams.get("iris")
        shutter = exposureParams.get("shutter")
        exposureMode = exposureParams.get("exposureMode")
        logger.info("摄像头传入参数{}".format(str(self.parameters)))
        try:
            loging(address, port, username, passwd)
            set_devicePrama(mode=int(mode), distance=int(distance), opticalZoomLevel=opticalZoomLevel,
                            gain=int(gain), iris=int(iris), shutter=int(shutter), exposureMode=int(exposureMode))
            logger.info("=====摄像头设置完成=====")
        except Exception as e:
            logger.error(e)
            logger.error("摄像头参数设置错误！")

    def action_elevator_height(self, position):
        logger.info("升降杆升起高度为{}".format(position))
        state, position = robot_lifter(1, position)
        robot_device_status = RobotDeviceStatus()
        lift_status = robot_device_status.get_device_status("lifter")
        if lift_status==False:
            # 开始向告警信息表插入数据
            res=DBAlarmManage()
            result=res.get_alarm_data(0,0)
            if result:
                if result["value"]=="升降杆服务":
                    num=result["num"]
                    robot_position_id=result["robot_position_id"]
                    dic=hardware_exception.lift_update
                    logger.info("升降杆服务异常, 更新告警信息")
                    res.update({"num":num},dic)
                    logger.info("告警信息更新成功")
                    # 将升降杆服务异常信息推送到mq
                    alarm_manage_id=result["alarm_manage_id"]
                    alarm_desc=result["alarm_desc"]
                    dic_data={"alarm_manage_id":alarm_manage_id,"alarm_desc":alarm_desc}
                    rout=db_rabbit_mq.get("robot_warn")
                    routing_ey=db_rabbit_mq.get("warn_rout_key")
                    warn_queue=db_rabbit_mq.get("warn_queue")
                    RabbitPublisher.run(rout,routing_ey,warn_queue,dic_data)
                    logger.info("升降杆服务异常推送成功")
                else:
                    dic=hardware_exception.lift_insert
                    logger.info("升降杆服务异常, 添加告警信息")
                    res.insert_data(dic)
                    logger.info("告警信息添加成功")
                    # 将升降杆服务异常信息推送到mq
                    result=res.get_alarm_data(0,0)
                    alarm_manage_id=result["alarm_manage_id"]
                    alarm_desc=result["alarm_desc"]
                    dic_data={'alarm_manage_id':alarm_manage_id,'alarm_desc':alarm_desc}
                    rout=db_rabbit_mq.get("robot_warn")
                    routing_ey=db_rabbit_mq.get("warn_rout_key")
                    warn_queue=db_rabbit_mq.get("warn_queue")
                    RabbitPublisher.run(rout,routing_ey,warn_queue,dic_data)
                    logger.info("升降杆服务异常推送成功")
            else:
                dic=hardware_exception.lift_insert
                logger.info("升降杆服务异常, 添加告警信息")
                res.insert_data(dic)
                logger.info("告警信息添加成功")
                # 将热成像服务调用异常信息推送到mq
                result=res.get_alarm_data(0,0)
                alarm_manage_id=result["alarm_manage_id"]
                alarm_desc=result["alarm_desc"]
                dic_data={'alarm_manage_id':alarm_manage_id,'alarm_desc':alarm_desc}
                logger.info("升降杆服务异常推送成功")
        elif state:
            return True
        else:
            pass

    def power_photo_action(self, data):
        logger.info("==================================任务数据==================================")
        # logger.info(data)
        os_path = Path().pointer + datetime.now().strftime("%Y_%m_%d")
        task_data = data.get("task_data")
        self.task_data = task_data
        power_cabinet_id = data.get("power_cabinet_id")

        # 需要robot_cmd_id
        panel_data = DBPowerPanel().get_panel_data(power_cabinet_id)
        db_power_device = DBPowerDevice()
        # logger.info(panel_data)
        if not panel_data:
            logger.info("此机柜无面板信息！")
            return False
        for panel in panel_data:
            power_id_list = []
            post_data_list = []
            power_panel_id = panel.get("power_panel_id")
            # logger.info(power_panel_id)
            power_device_data = db_power_device.get_panel_device_data(power_panel_id)
            # logger.info(power_device_data)
            for device in power_device_data:
                power_device_id = device.get("power_device_id")
                power_id_list.append(power_device_id)


            # power_id_list = []
            # res = DBPowerDeviceLocation().get_device_data(power_cabinet_id)
            # db_power_device = DBPowerDevice()
            # for device_data in res:
            #     power_device_id = device_data.get("power_device_id")
            #     if db_power_device.get_device_data(power_device_id):
            #         power_id_list.append(power_device_id)

            if not power_id_list:
                logger.info("此面板没有power_indcamera_param配置检测信息")
            # logger.info(str(power_id_list))
            for power_device_id in power_id_list:
                camera_data = self.db_power_camera_param_obj.get_power_data(power_device_id)
                if not camera_data:
                    logger.info("该设备id--{}无数据".format(power_device_id))
                    continue
                logger.info("该设备id--{}有数据".format(power_device_id))

                elevator_height = camera_data[0].get("elevator_height")
                if not self.action_elevator_height(elevator_height):
                    logger.info("该设备id{}控制升降杆异常".format(power_device_id))
                    continue
                logger.info("调用升降杆成功！")
                # 连接并设置相机参数
                # logger.info(str(camera_data))
                # logger.info(str(camera_data[0]))
                # 确定是使用哪个算法
                robot_algorithm_id = camera_data[0].get("robot_algorithm_id")
                # logger.info(robot_algorithm_id)
                # 判断相机类型
                if camera_data[0].get("robot_device_id") == 0:
                    continue
                try:
                    res = DBRobotDevice().get_device_id(camera_data[0].get("robot_device_id"))
                    camera_name = res.get("name")
                    # logger.info(camera_name)
                except Exception as e:
                    logger.error(e)
                    continue
                # 使用工业
                if camera_name in camera_type.get("industry_camera"):
                    parameters = camera_data[0].get("device_param")
                    # 设置工业相机参数
                    set_industry_state = set_industry_parameter(res.get("address"), parameters)
                    if not set_industry_state:
                        logger.info("该设备id{}工业相机参数设置错误")
                        continue
                    # 使用工业相机进行拍照
                    action_state, photo_name = self.action_industry_photo(res.get("address"), os_path, parameters)
                    if not action_state:
                        logger.info("工业相机进行拍照出现问题")
                    self.parameter_type = 2
                    logger.info("工业相机进行拍照完成")

                # 使用四倍
                elif camera_name in camera_type.get("fourfold_camera"):
                    set_fourfold_state = self.set_fourfold_parameter(camera_data[0], res)
                    if not set_fourfold_state:
                        logger.info("该设备id{}四倍相机登录出现问题")
                        continue

                    # 使用四倍相机进行拍照
                    logger.info("使用四倍相机进行拍照")
                    action_state, photo_name = self.fourfold_photo_action(os_path)
                    if not action_state:
                        logger.info("四倍相机进行拍照出现问题")
                    self.parameter_type = 1
                else:
                    continue

                # 数字仪表盘进行检测
                # camera_data, power_device_id, power_cabinet_id, robot_algorithm_id,photo_name

                post_data = {
                    "data": {
                        "camera_data": camera_data,
                        "power_device_id": power_device_id,
                        "power_cabinet_id": power_cabinet_id,
                        "robot_algorithm_id": robot_algorithm_id,
                        "photo_name": photo_name,
                        "task_data": self.task_data
                    }
                }
                if robot_algorithm_id == self.db_robot_algorithm.get_name_data("数字仪表盘算法"):
                    post_data["type"] = 2
                if robot_algorithm_id == self.db_robot_algorithm.get_name_data("指针仪表盘算法"):
                    post_data["type"] = 3
                if robot_algorithm_id == self.db_robot_algorithm.get_name_data("开关算法"):
                    logger.info(post_data)
                    post_data["type"] = 4
                post_data_list.append(post_data)
            try:
                power_work = Thread(target=task_analysis, args=(post_data_list,))
                power_work.start()
                robot_position_id = task_data.get("robot_position_id")
                data["work_list"][robot_position_id].append(power_work)
            except Exception as e:
                logger.error("异步解析接口错误！")
                logger.error(e)
        robot_lifter(1, 0)

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
    def pointer_dashboard_al(self, camera_data, power_device_id, power_cabinet_id, robot_algorithm_id, photo_name):
        # pointer_alarm = PointerAlarmAL()
        try:
            camera_al_data = self.pointer_camera_transform(camera_data)
        except Exception as e:
            logger.error(e)
            logger.error("参数错误，请查看数据库中配置参数是否正确")
            return False
        logger.info("指针仪表盘进行检测传入算法所需参数-------》{}".format(str(camera_al_data)))

        logger.info("合成后的图片-------》{}".format(photo_name))
        light_detect_result, light_path = post_agx_pointer(photo_name, camera_al_data, self.parameter_type)
        logger.info("照片的解析" + str(light_detect_result))
        logger.info("请求power_indcamera_param------》id{}数据的结果为{}".format(power_device_id, str(light_detect_result)))
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
            logger.info("告警项ID为{}".format(item_id))
            camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id, item_id)
            logger.info("告警参数为{}".format(str(camera_data)))
            ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
            ret['user_id'] = self.task_data.get("user_id")
            ret['robot_id'] = self.task_data.get("robot_id")
            ret['power_robot_item_id'] = item_id
            ret["robot_position_id"] = self.task_data.get("robot_position_id")
            ret["power_cabinet_id"] = power_cabinet_id
            ret["power_device_id"] = power_device_id
            ret["value"] = value
            # 检测项
            ret["img_path"] = light_path
            ret["type"] = type_num
            ret["create_time"] = datetime.now()

            self.db_power_inspect_data.insert(ret)
            return True

        self.task_data["power_device_id"] = power_device_id
        self.task_data["img_path"] = light_path
        logger.info(light_detect_result)
        for detect_result in light_detect_result:
            logger.info("检测结果{}".format(str(detect_result)))
            ret = {}
            lengend_en_name = detect_result.get("legend")
            logger.info("检测结果项{}".format(lengend_en_name))
            value = detect_result.get("value", "未识别")
            if not value:
                value = "未识别"
            else:
                value_num = re.findall(r"\d+\.?\d*", str(value))
                if value_num:
                    value = float(value_num[0])
                else:
                    value = "未识别"
            logger.info("检测结果值{}".format(value))

            item_id, type_num = self.get_al_item_data(robot_algorithm_id, lengend_en_name)
            # # 添加告警信息
            logger.info("告警算法ID为{}".format(robot_algorithm_id))
            logger.info("告警项ID为{}".format(item_id))
            camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id, item_id)
            logger.info("告警参数为{}".format(str(camera_data)))

            ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
            ret['user_id'] = self.task_data.get("user_id")
            ret['robot_id'] = self.task_data.get("robot_id")
            ret['power_robot_item_id'] = item_id
            ret["robot_position_id"] = self.task_data.get("robot_position_id")
            ret["power_cabinet_id"] = power_cabinet_id
            ret["power_device_id"] = power_device_id
            ret["value"] = value
            # 检测项
            ret["img_path"] = light_path
            ret["type"] = type_num
            ret["create_time"] = datetime.now()

            self.db_power_inspect_data.insert(ret)
        logger.info("{}动力巡检指针检测成功".format(light_path))
        logger.info("{}指针仪表盘识别结果".format(light_detect_result))

    # 开关进行检测
    def switch_dashboard_al(self, all_camera_data, power_device_id, power_cabinet_id, robot_algorithm_id, photo_name):
        # pointer_alarm = PointerAlarmAL()
        logger.info("一共有{}个开关识别".format(len(all_camera_data)))
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

            light_detect_result, light_path = post_agx_switch(photo_name, camera_al_data, self.parameter_type)
            logger.info("照片的解析" + str(light_detect_result))
            logger.info("请求power_indcamera_param------》id{}数据的结果为{}".format(power_device_id, str(light_detect_result)))
            # 获取动力检测名称
            power_robot_item_id = camera_data[0].get("power_robot_item_id")
            item_name = DBPowerRobotItem().get_item_data(power_robot_item_id)
            lengend_en_name = item_name.get("name", "开关状态")
            if not light_detect_result:
                logger.info("没有解析指针仪表盘结果{}".format(power_device_id))
                logger.info("检测结果{}".format(str(light_detect_result)))
                ret = {}
                logger.info("检测结果项{}".format(lengend_en_name))
                value = "未识别"
                logger.info("检测结果值{}".format(value))
                item_id, type_num = self.get_al_item_data(robot_algorithm_id, lengend_en_name)

                # # 添加告警信息
                logger.info("告警算法ID为{}".format(robot_algorithm_id))
                logger.info("告警项ID为{}".format(item_id))
                camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id, item_id)
                logger.info("告警参数为{}".format(str(camera_data)))
                ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
                ret['user_id'] = self.task_data.get("user_id")
                ret['robot_id'] = self.task_data.get("robot_id")
                ret['power_robot_item_id'] = item_id
                ret["robot_position_id"] = self.task_data.get("robot_position_id")
                ret["power_cabinet_id"] = power_cabinet_id
                ret["power_device_id"] = power_device_id

                ret["value"] = value
                # 检测项
                ret["img_path"] = light_path
                ret["type"] = type_num
                ret["create_time"] = datetime.now()

                self.db_power_inspect_data.insert(ret)
                return True

            self.task_data["power_device_id"] = power_device_id
            self.task_data["img_path"] = light_path
            logger.info(light_detect_result)
            for detect_result in light_detect_result:
                logger.info("检测结果{}".format(str(detect_result)))
                ret = {}
                config_algorithm_model_id = camera_data[0].get("config_algorithm_model_id")
                logger.info("检测结果model项{}".format(config_algorithm_model_id))
                value = detect_result.get("step", "未识别")
                if not value:
                    value = "未识别"
                else:
                    value_num = re.findall(r"\d+\.?\d*", str(value))
                    if value_num:
                        value = float(value_num[0])
                    else:
                        value = "未识别"
                logger.info("检测结果值{}".format(value))
                # get_switch_item_data
                logger.info(robot_algorithm_id)
                logger.info(config_algorithm_model_id)
                item_id, type_num = self.get_switch_item_data(robot_algorithm_id, config_algorithm_model_id)
                # # 添加告警信息
                logger.info("告警算法ID为{}".format(robot_algorithm_id))
                logger.info("告警项ID为{}".format(item_id))
                camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id, item_id)
                logger.info("告警参数为{}".format(str(camera_data)))
                # 映射开关算法对应的值
                try:
                    logger.info("当前的值======================================{}".format(value))
                    mapping_rule = camera_data.get("algorithm_param").get("switch")
                    logger.info(mapping_rule)
                    # 临时指定数据
                    # mapping_rule = {"1": "2", "2": "0", "3": "1"}

                    mapping_value = mapping_rule.get(str(int(value)), None)
                    if mapping_value:
                        new_value = mapping_value
                        logger.info("映射后的值======================================{}".format(new_value))
                    else:
                        logger.error("开关值映射错误！")
                        new_value = value
                except Exception as e:
                    new_value = value
                    logger.error("开关映射值获取错误")
                    logger.error(e)
                ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
                ret['user_id'] = self.task_data.get("user_id")
                ret['robot_id'] = self.task_data.get("robot_id")
                ret['power_robot_item_id'] = item_id
                ret["robot_position_id"] = self.task_data.get("robot_position_id")
                ret["power_cabinet_id"] = power_cabinet_id
                ret["power_device_id"] = power_device_id
                ret["value"] = new_value
                # 检测项
                ret["img_path"] = light_path
                ret["type"] = type_num
                ret["create_time"] = datetime.now()

                self.db_power_inspect_data.insert(ret)
            logger.info("{}动力巡检开关进行检测成功".format(light_path))
            logger.info("{}开关进行检测识别结果".format(light_detect_result))

    # 数字仪表盘进行检测
    def digital_dashboard_al(self, camera_data, power_device_id, power_cabinet_id, robot_algorithm_id, photo_name):
        # 进行识别
        # power_alarm = PowerAlarmAL()

        try:
            camera_al_data = self.digital_camera_transform(camera_data)
        except Exception as e:
            logger.error(e)
            logger.error("参数错误，请查看数据库中配置参数是否正确")
            return False
        logger.info("传入算法所需参数图片-------》{}".format(str(camera_al_data)))
        logger.info("合成后的图片-------》{}".format(photo_name))
        light_detect_result, light_path = post_agx_digital(photo_name, camera_al_data, self.parameter_type)

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
                logger.info("告警项ID为{}".format(item_id))
                camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id, item_id)
                logger.info("告警参数为{}".format(str(camera_data)))
                ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
                ret['user_id'] = self.task_data.get("user_id")
                ret['robot_id'] = self.task_data.get("robot_id")
                ret['power_robot_item_id'] = item_id
                ret["robot_position_id"] = self.task_data.get("robot_position_id")
                ret["power_cabinet_id"] = power_cabinet_id
                ret["power_device_id"] = power_device_id
                ret["value"] = value
                # 检测项
                ret["img_path"] = light_path
                ret["type"] = type_num
                ret["create_time"] = datetime.now()

                self.db_power_inspect_data.insert(ret)
            return True

        logger.info("照片的解析" + str(light_detect_result))
        logger.info("请求power_indcamera_param------》id{}数据的结果为{}".format(power_device_id, str(light_detect_result)))

        self.task_data["power_device_id"] = power_device_id
        self.task_data["img_path"] = light_path

        # relative_path = str(light_path).split(Path().APP_PATH)[-1]
        for detect_result in light_detect_result:
            ret = {}
            # lengend_en = detect_result.get("legend")
            lengend_en_name = detect_result.get("legend")
            value = detect_result.get("res", "未识别")
            item_id, type_num = self.get_al_item_data(robot_algorithm_id, lengend_en_name)
            if not value:
                value = "未识别"
            else:
                try:
                    value_num = re.findall(r"\d+\.?\d*", str(value))
                    value = float(value_num[0])
                    # 这里已获取到数据，根据已获取的数据判断最终结果
                    # 先获取历史数据
                    history_data = self.db_power_inspect_data.get_item_data(power_robot_item_id=item_id)[0:30]
                    if history_data:
                        history_list = list()
                        for hd in history_data:
                            if hd["value"] not in ["未识别", "与历史数据不匹配"]:
                                history_list.append(float(hd["value"]))
                        # 对比历史数据，获取最终的值
                        judge_status, judge_res = postprocess(history_list, str(value), 3)

                        if int(judge_status) == -1:
                            value = "未识别"
                        elif int(judge_status) == 0:
                            value = "与历史数据不匹配"
                        else:
                            value = judge_res
                except Exception as e:
                    logger.error(e)
                    logger.error("数据类比错误！")
                    value = "未识别"

            # # 添加告警信息
            logger.info("告警算法ID为{}".format(robot_algorithm_id))
            logger.info("告警项ID为{}".format(item_id))
            camera_data = self.db_power_camera_param_obj.get_power_param_data(robot_algorithm_id, item_id)
            logger.info("告警参数为{}".format(str(camera_data)))
            ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
            ret['user_id'] = self.task_data.get("user_id")
            ret['robot_id'] = self.task_data.get("robot_id")
            ret['power_robot_item_id'] = item_id
            ret["robot_position_id"] = self.task_data.get("robot_position_id")
            ret["power_cabinet_id"] = power_cabinet_id
            ret["power_device_id"] = power_device_id
            ret["value"] = value
            # 检测项
            ret["img_path"] = light_path
            ret["type"] = type_num
            ret["create_time"] = datetime.now()

            self.db_power_inspect_data.insert(ret)
        logger.info("{}动力巡检检测成功".format(light_path))
        logger.info("{}数字仪表盘识别结果".format(light_detect_result))


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
    res = {
        "power_cabinet_id": 1,
        "ret": {
            "current_state": 0,
            "power_cabinet_id": 1,
            "del_flag": 0,
            "status": 0,
            "robot_path_id": 2,
            "remark": "",
            "update_by": 1,
            "distance": 600,
            "robot_id": 1,
            "core_cabinet_id": 0,
            "robot": None,
            "update_time": "2022-02-10T18:54:12",
            "direction": None,
            "device_id": 0,
            "type": 1,
            "is_around": 0,
            "position_num": 11,
            "camera_config": None,
            "create_by": 1,
            "create_time": "2022-01-19T21:13:07",
            "name": "动力前方",
            "robot_position_id": 29,
            "robot_path": None,
            "is_obstacle": 0,
            "position": {
                "yaw": "1.61",
                "x": "2.37",
                "y": "-2.76"
            }
        },
        "task_data": {
            "now_battery_power": 94,
            "robot_path_id": 2,
            "inspect_project_detail_id": 1687348045,
            "status": 1,
            "img_path": "",
            "obstacle": 0,
            "core_room_id": 1,
            "core_cabinet_id": 0,
            "is_break": 0,
            "value": 64.4,
            "task_type_name": "自动巡检",
            "power_cabinet_id": 1,
            "robot_task_type_id": None,
            "robot_position_id": 29,
            "task_status": 2,
            "robot_id": 1,
            "alarm_type": 1,
            "minimum_battery": 20
        }
    }

    PointerDistinguishAL().power_photo_action(res)
