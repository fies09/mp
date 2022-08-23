# 给升降杆中摄像头传参
import os
import time
from datetime import datetime

import requests
from threading import Thread
from requests.auth import HTTPDigestAuth
from configs import analysis_api
from task.blend import lightBlend
from configs.log import logger
from core.dict_config import lamp_dic
from core.thread_pool import Pool
from task.light_al import post_agx_light
from modules.mainManage import set_devicePrama, loging
from modules.robot_hardware_server import robot_lifter
from schema.db_inspect_position_item_data import DBInspectPositionItemDatum
from schema.db_robot_algorithm_param import DBRobotAlgorithmParam
from schema.db_robot_device import DBRobotDevice
from schema.db_robot_device_param import DBRobotDeviceParam
from schema.db_robot_elevator_param import DBRobotElevatorParam
from utils.file_path import Path
from analysis.robot_analysis import task_analysis
from schema.db_alarm_manage import DBAlarmManage
from task.hardware_exception import schedule_test
from modules.robot_hardware_server import RobotDeviceStatus
from configs import db_rabbit_mq
from core.rabbitmq_db import RabbitPublisher



class FourFoldLight(object):
    def __init__(self):
        self.pool = Pool
        self.core_cabinet_id = 0

    # 传入参数
    def elevator_action(self, elevator_ret, task_data, core_cabinet_id, work_list):
        self.core_cabinet_id = core_cabinet_id
        logger.info("提取出升降杆相机需要的参数")
        logger.info(elevator_ret)
        logger.info(task_data)
        logger.info(core_cabinet_id)
        alive_elevator, elevator_type_list, algorithm_type_list, res_device_type_list = \
            self.screen_elevator_data(elevator_ret)

        for robot_elevator_param_id, robot_algorithm_param_id, res_device_param_id in \
                zip(elevator_type_list, algorithm_type_list, res_device_type_list):

            # 控制升降杆动作，1为有表中存在参数
            lift_state = self.robot_lift_action(robot_elevator_param_id, 1)
            logger.info(lift_state)
            if lift_state:
                elevator_list = self.screen_elevator_dict(alive_elevator, robot_elevator_param_id)

                self.robot_photo_state(robot_algorithm_param_id,
                                       res_device_param_id,
                                       elevator_list, task_data, core_cabinet_id, work_list)

        self.robot_lift_action(0, 0)

    def robot_photo_state(self, robot_algorithm_param_id,
                          res_device_param_id, elevator_list, task_data, core_cabinet_id, work_list):
        rets = DBRobotDeviceParam().get_device_param(res_device_param_id)
        res = DBRobotDevice().get_device_data(rets.get("robot_device_id"))
        address, port, username, passwd = res.get("address"), res.get("port"), \
                                          res.get("username"), res.get("passwd")

        if address == "0":
            return False
        login_state = loging(address, port, username, passwd)
        if not login_state:
            logger.error(address + "摄像头login参数错误")
            return False

        # 摄像头传参
        try:
            parameters = rets.get("parameters")
            self.robot_photo_parameter(parameters=parameters)
        except Exception as e:
            logger.error(e)

        ret = DBRobotDeviceParam().get_device_param(res_device_param_id)
        self.camera_login(ret)
        parameters = ret.get("parameters")

        photo_path = Path().photo + datetime.now().strftime("%Y_%m_%d") + "/"
        # 调取拍照动作
        photo_state, file_path = self.photo_action(photo_path=photo_path, parameters=parameters)

        if not photo_state:
            logger.error("摄像头拍照错误")
            return False
        logger.info("给IPS中发送信息")

        post_data = {
            "data": {
                "ret": ret,
                "elevator_list": elevator_list,
                "task_data": task_data,
                "robot_algorithm_param_id": robot_algorithm_param_id,
                "file_path": file_path,
                "core_cabinet_id": core_cabinet_id
            }
        }

        post_data["type"] = 1
        logger.info("111111111111111")
        logger.info(str(post_data))
        try:
            four_work = Thread(target=task_analysis, args=([post_data],))
            four_work.start()
            robot_position_id = task_data.get("robot_position_id")
            work_list[robot_position_id].append(four_work)
        except Exception as e:
            logger.error(e)

        # self.pool.submit(self.img_discernment, ret, elevator_list, task_data, robot_algorithm_param_id,
        #                  file_path)

    # 登录摄像头
    def camera_login(self, ret):
        res = DBRobotDevice().get_device_data(ret.get("robot_device_id"))
        address, port, username, passwd = res.get("address"), res.get("port"), \
                                          res.get("username"), res.get("passwd")

        logger.info('当前ip：' + address)

        # ISAPI方式登录 返回值为请求后返回的所有数据 其中就包括拍照后照片的数据
        self.login_state = self.loging(address, port, username, passwd)
        if self.login_state == False:
            logger.error(address + "摄像头login参数错误")
            return False
        return True

    # 调取抓图动作
    def photo_action(self, photo_path, parameters):
        robot_device_status=RobotDeviceStatus()
        catchParams = parameters.get("catchParams")
        nums = catchParams.get("repeate_num")
        interval_time = catchParams.get("interval_time")
        logger.info("拍照数量{}".format(nums))
        logger.info("间隔时间{}".format(interval_time))

        if not os.path.exists(photo_path):
            os.makedirs(photo_path)
        file_name_time = photo_path + datetime.now().strftime("%H_%M_%S")
        file_name_base = photo_path + datetime.now().strftime("%H_%M_%S") + ".jpg"

        if nums == 1:
            photo_state = self.photo_get(file_name_base)
            fourfold_up_status = robot_device_status.get_device_status("fourfold_up")
            fourfold_down_status=robot_device_status.get_device_status("fourfold_down")
            if fourfold_up_status == False or fourfold_down_status == False:
                # 开始向告警信息表插入数据
                data_value = "四倍相机"
                schedule_test(data_value)
                return False
            elif not photo_state:
                return False,None
            else:
                pass
            return True, file_name_base

        for num in range(nums):
            file_name = file_name_time + "_{}.jpg".format(num)
            photo_state = self.photo_get(file_name)
            time.sleep(interval_time)
            fourfold_up_status = robot_device_status.get_device_status("fourfold_up")
            fourfold_down_status=robot_device_status.get_device_status("fourfold_down")
            if fourfold_up_status == False or fourfold_down_status == False:
                # 开始向告警信息表插入数据
                data_value="四倍相机"
                schedule_test(data_value)
                return False
            elif not photo_state:
                return False,None
            else:
                pass
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

    # 解析照片
    def img_discernment(self, ret, elevator_list, task_data, robot_algorithm_param_id, file_path):
        param_ret = DBRobotAlgorithmParam().get_by_id(robot_algorithm_param_id)
        logger.info(param_ret)
        parameters = param_ret.get("parameters")
        u_params = parameters.get("u_params")
        start_u = u_params.get("start_u")
        start_imgRows = int(u_params.get("start_imgRows"))
        end_imgRows = int(u_params.get("end_imgRows"))
        start_imgCols = int(u_params.get("start_imgCols"))
        end_imgCols = int(u_params.get("end_imgCols"))

        logger.info("合成后的图片-------》{}".format(file_path))
        light_detect_result, light_path, narrow_path = post_agx_light(file_path, start_imgRows, end_imgRows,
                                                                      start_imgCols, end_imgCols, "", 1)
        if not narrow_path:
            narrow_path = light_path
        logger.info("照片的解析" + str(light_detect_result))
        logger.info("所属机柜id{}".format(self.core_cabinet_id))
        if not light_path:
            return False
        else:
            task_data["img_path"] = light_path
            for elevator in elevator_list:
                # 插入数据
                lamp_name = elevator.get("name")
                lamp_name_en = lamp_dic.get(lamp_name)
                lamp_num = light_detect_result.get(lamp_name_en)
                # 添加告警信息
                logger.info("四倍相机关联信息信息")
                logger.info(elevator)
                logger.info(lamp_name)
                logger.info(lamp_name_en)
                logger.info(lamp_num)
                ret["server_u"] = str(start_u)
                ret["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
                ret["robot_position_id"] = elevator.get("robot_position_id")
                ret["robot_path_id"] = task_data.get("robot_path_id")
                ret["core_cabinet_id"] = self.core_cabinet_id
                ret["robot_item_id"] = elevator.get("robot_item_id")
                ret["create_time"] = datetime.now()
                ret['value'] = lamp_num
                ret["img_path"] = narrow_path
                ret['type'] = 3
                ret['crop_path'] = ""

                DBInspectPositionItemDatum().insert(info=ret)

            return True

    # 取出对应升降杆设置的值
    def screen_elevator_dict(self, dicts, keys):
        elevator_list = []
        for a_dict in dicts:
            if a_dict.get("robot_elevator_param_id") == keys:
                elevator_list.append(a_dict)
        return elevator_list

    # 整合数据库调动升降杆算法的数据
    def screen_elevator_data(self, res_dic):
        res_list = []
        res_elevator_param = []
        res_algorithm_param = []
        res_device_param = []
        for res in res_dic:
            # 没有升降杆动作的时候 不调取升降杆
            if res.get("robot_elevator_param_id") != 0:
                res_list.append(res)
                res_elevator_param.append(res.get("robot_elevator_param_id"))
                res_algorithm_param.append(res.get("robot_algorithm_param_id"))
                res_device_param.append(res.get("robot_device_param_id"))

        res_elevator_type_list = sorted(set(res_elevator_param), key=res_elevator_param.index)
        res_algorithm_type_list = sorted(set(res_algorithm_param), key=res_algorithm_param.index)
        res_device_type_list = sorted(set(res_device_param), key=res_device_param.index)

        return res_list, res_elevator_type_list, res_algorithm_type_list, res_device_type_list

    # 调取升降杆动作
    def robot_lift_action(self, robot_elevator_param_id, lift_state):
        robot_device_status=RobotDeviceStatus()
        if lift_state == 0:
            state, position = robot_lifter(1, 0)
            return state
        ret = DBRobotElevatorParam().get_robot_elevator_high(robot_elevator_param_id)
        try:
            distance = ret[0].get("height")
            logger.info("升降杆高度{}".format(str(distance)))
            lift_status = robot_device_status.get_device_status("lifter")
            if not lift_state or lift_status == False:
                # 开始向告警信息表插入数据
                data_value = "升降杆"
                schedule_test(data_value)
            else:
                state, position = robot_lifter(1, int(distance))
        except Exception as e:
            logger.error(e)
            # 开始向告警信息表插入数据
            data_value="升降杆"
            schedule_test(data_value)
            return False
        return state

    def set_camera(self, url, params, tag):
        """
        发送请求设置相机参数
        """
        response = requests.get(url=url, params=params)
        if response.status_code == 200:
            logger.info("{}设置成功！".format(tag))
        else:
            logger.info("{}设置失败！".format(tag))

    # 给摄像头传参
    def robot_photo_parameter(self, parameters):
        focusParams = parameters.get("focusParams")
        mode = focusParams.get("focusMode")
        distance = focusParams.get("minFocusDistance")
        opticalZoomLevel = focusParams.get("opticalZoomLevel")

        exposureParams = parameters.get("exposureParams")
        gain = exposureParams.get("gain")
        iris = exposureParams.get("iris")
        shutter = exposureParams.get("shutter")
        exposureMode = exposureParams.get("exposureMode")
        logger.info("摄像头传入参数{}".format(str(parameters)))
        try:
            set_devicePrama(mode=int(mode), distance=int(distance), opticalZoomLevel=opticalZoomLevel,
                            gain=int(gain), iris=int(iris), shutter=int(shutter), exposureMode=int(exposureMode))
            logger.info("=====摄像头设置完成=====")
        except Exception as e:
            logger.error(e)
            logger.error("摄像头参数设置错误！")


if __name__ == '__main__':
    from schema.db_robot_path_position_item import DBRobotPathPositionItem
    from schema.db_robot_position import DBRobotPosition

    position_data = DBRobotPosition().get_position(1, 2)
    for res in position_data:
        position_res = res
        robot_position_id = position_res.get("robot_position_id")
        core_cabinet_id = position_res.get("core_cabinet_id")
        task_data = {"inspect_project_detail_id": 22222222222, "robot_path_id": 2,
                     "robot_position_id": robot_position_id}
        elevator_ret = DBRobotPathPositionItem().get_robot_elevator_param(id=robot_position_id)
        if not elevator_ret:
            continue
        FourFoldLight().elevator_action(elevator_ret, task_data, core_cabinet_id)
