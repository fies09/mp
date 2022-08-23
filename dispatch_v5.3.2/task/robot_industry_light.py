import os
import time
import requests
import json
import copy
import datetime
from threading import Thread
from configs import industry_api
from configs.log import logger
from task.blend import lightBlend
from modules.robot_hardware_server import robot_lifter
from task.robot_alarm import IndustryAlarm
from schema.db_core_indcamera_param import DBCoreIndcameraParam
from schema.db_inspect_position_item_data import DBInspectPositionItemDatum
from utils.file_path import Path
from schema.db_core_device_location import DBCoreDeviceLocation
from schema.db_robot_item import DBRobotItem
from schema.db_robot_device import DBRobotDevice
from schema.robot_core_cabinet import DBCoreCabinet
from schema.robot_core_device import DBCoreDevice
from analysis.robot_analysis import task_analysis
from schema.db_alarm_manage import DBAlarmManage
from task.hardware_exception import schedule_test
from configs import db_rabbit_mq
from core.rabbitmq_db import RabbitPublisher
from modules.robot_hardware_server import RobotDeviceStatus

# 工业相机
from configs import analysis_api


class RobotIndustryLight(object):
    def __init__(self):
        self.db_core_indcamera_param = DBCoreIndcameraParam()
        # self.db_industry_alarm = IndustryAlarm()
        self.dlcw_l = None
        # self.db_robot_position = DBRobotPosition

    def industry_photo_action(self, data, dlcw_l):
        self.dlcw_l = dlcw_l
        robot_device_status=RobotDeviceStatus()
        # 取出该点对应算法参数 id
        db_robot_item = DBRobotItem()
        db_inspect_item_data = DBInspectPositionItemDatum()
        task_data = data.get("task_data")
        inspect_project_detail_id = task_data.get("inspect_project_detail_id")
        core_cabinet_id = data.get("core_cabinet_id")
        robot_position_id = task_data.get("robot_position_id")

        # 需要robot_cmd_id

        core_id_list, core_device_id_list, start_u_list, end_u_list = self.get_indcamera_param_id(core_cabinet_id)
        if not core_id_list:
            logger.info("无core_indcamera_param配置检测信息")
            return False

        # 获取上下工业相机的设备ID
        db_robot_device = DBRobotDevice()
        top_id = db_robot_device.get_device_name("工业相机上")["robot_device_id"]
        bottom_id = db_robot_device.get_device_name("工业相机下")["robot_device_id"]
        # 获取此机柜所有高度
        all_height_list = list()
        for core_id in core_id_list:
            try:
                core_id_height = DBCoreIndcameraParam().get_height(core_id)[0][0]
                all_height_list.append(core_id_height)
            except Exception as e:
                logger.error(e)
                continue
        all_height_list = list(set(all_height_list))
        all_height_list.sort()
        # logger.info(all_height_list)
        for height in all_height_list:
            main_state, position = robot_lifter(1, height)
            lift_status = robot_device_status.get_device_status("lifter")
            if lift_status == False or not main_state:
                # 开始向告警信息表插入数据
                data_value = "升降杆"
                schedule_test(data_value)
                logger.error("升降杆调用失败！")
            else:
                pass

            # 上摄像头
            top_core_id_list = list()
            top_core_device_id_list = list()
            top_start_u_list = list()
            top_end_u_list = list()
            # 下摄像头
            bottom_core_id_list = list()
            bottom_core_device_id_list = list()
            bottom_start_u_list = list()
            bottom_end_u_list = list()
            for core_id in core_id_list:
                # 根据core_id获取参数
                camera_data = self.db_core_indcamera_param.get_indcamera_data(core_id)
                camera_height = camera_data.get("elevator_height")
                device_id = camera_data.get("robot_device_id")
                # 如果高度相同且是上或下摄像头
                if height == camera_height:
                    index = core_id_list.index(core_id)
                    if top_id == device_id:
                        top_core_id_list.append(core_id)
                        top_core_device_id_list.append(core_device_id_list[index])
                        top_start_u_list.append(start_u_list[index])
                        top_end_u_list.append(end_u_list[index])
                    elif bottom_id == device_id:
                        bottom_core_id_list.append(core_id)
                        bottom_core_device_id_list.append(core_device_id_list[index])
                        bottom_start_u_list.append(start_u_list[index])
                        bottom_end_u_list.append(end_u_list[index])
                    else:
                        continue
            if not top_core_id_list and not bottom_core_id_list:
                logger.info("高度{}暂未发现配置".format(height))
                continue
            top_params = {"core_id_list": top_core_id_list, "core_device_id_list": top_core_device_id_list,
                          "start_u_list": top_start_u_list, "end_u_list": top_end_u_list,
                          "task_data": task_data, "db_robot_item": db_robot_item,
                          "core_cabinet_id": core_cabinet_id, "inspect_project_detail_id": inspect_project_detail_id,
                          "db_inspect_item_data": db_inspect_item_data, "robot_device_id": top_id, "data": data
                          }
            bottom_params = {"core_id_list": bottom_core_id_list, "core_device_id_list": bottom_core_device_id_list,
                             "start_u_list": bottom_start_u_list, "end_u_list": bottom_end_u_list,
                             "task_data": task_data, "db_robot_item": db_robot_item,
                             "core_cabinet_id": core_cabinet_id, "inspect_project_detail_id": inspect_project_detail_id,
                             "db_inspect_item_data": db_inspect_item_data, "robot_device_id": bottom_id, "data": data
                             }
            # 开启线程，让上下同高度的相机同时执行任务
            thread_list = list()
            if top_core_id_list:
                top_thread = Thread(target=self.analysis_insert, args=(top_params,))
                thread_list.append(top_thread)
            if bottom_core_id_list:
                bottom_thread = Thread(target=self.analysis_insert, args=(bottom_params,))
                thread_list.append(bottom_thread)
            for s in thread_list:
                s.start()
                data["work_list"][robot_position_id].append(s)
            for j in thread_list:
                j.join()
        robot_lifter(1, 0)

    def get_indcamera_param_id(self, id):
        core_id_list = []
        core_device_id_list = []
        start_u_list = []
        end_u_list = []
        res = DBCoreDeviceLocation().get_core_device_id(id)
        if not res:
            return False, None, None, None
        for device_data in res:
            core_device_id = device_data.get("core_device_id")
            get_core_param_id = self.db_core_indcamera_param.get_core_param_id(core_device_id)
            if get_core_param_id:
                core_id_list.append(get_core_param_id)
                core_device_id_list.append(core_device_id)
                start_u_list.append(device_data.get("start_u"))
                end_u_list.append(device_data.get("end_u"))

        return core_id_list, core_device_id_list, start_u_list, end_u_list

    def get_indcamera_param_data(self, id_list):
        """
        根据参数的id列表获取所有数据
        """
        data_list = list()
        for i in id_list:
            data = self.db_core_indcamera_param.get_indcamera_data(i)
            data_list.append(data)
        return data_list

    def get_merge_index(self, data):
        """
        获取合并后参数列表的索引
        """
        copy_data = copy.deepcopy(data)
        new_data_list = list()
        while True:
            if not copy_data:
                break
            data_list = []
            exm = copy_data[0]
            data_list.append(exm)
            for i in copy_data[1:]:
                if exm["robot_device_id"] == i["robot_device_id"] and exm["elevator_height"] == i["elevator_height"]:
                    data_list.append(i)
                    del copy_data[copy_data.index(i)]
            del copy_data[0]
            new_data_list.append(data_list)
        results = list()
        for i in new_data_list:
            result = list()
            for j in i:
                result.append(data.index(j))
            results.append(result)
        return results

    @staticmethod
    def get_show_data(core_id):
        # logger.info(int(core_id))
        camera_data = DBCoreIndcameraParam().get_industry_id(int(core_id))
        # logger.info(camera_data)
        core_device_id = camera_data.get("core_device_id")
        # 获取机柜数据
        core_cabinet_id = DBCoreDeviceLocation().get_core_cabinet_id(core_device_id)
        cabinet_data = DBCoreCabinet().get_cabinet_name(core_cabinet_id)
        if cabinet_data:
            cabinet_name = cabinet_data.get("name")
        else:
            cabinet_name = "00"

        # 获取设备ID
        ret = DBCoreDevice().get_device_id(core_device_id)
        if ret:
            device_name = ret.get("name")
        else:
            device_name = "00"

        # 获取起始U-结束U
        try:
            algorithm_param = camera_data.get("algorithm_param")
            u_params = algorithm_param.get("u_params")
            start_u = u_params.get("start_u")
            end_u = u_params.get("end_u")
        except Exception as e:
            logger.error(e)
            start_u = "--"
            end_u = "--"

        show_data = "P-{} U-{} D-{}-{}".format(cabinet_name, device_name, start_u, end_u)
        # logger.info(show_data)
        return show_data

    def industry_industry_photo(self, core_id):
        core_id_list = core_id.split(",")
        # logger.info(core_id_list)
        # 获取索引0处的工业相机参数
        core_id = core_id_list[0]
        os_path = Path().industry + datetime.datetime.now().strftime("%Y_%m_%d")
        photo_name = os_path + "/" + datetime.datetime.now().strftime("%H_%M_%S.%f") + ".jpg"
        if not os.path.exists(os_path):
            os.makedirs(os_path)
        try:
            camera_data = DBCoreIndcameraParam().get_industry_id(int(core_id))
        except Exception as e:
            camera_data = {}
            logger.error(e)
        if not camera_data:
            logger.info("该设备id{}无数据".format(core_id))
            return False
        # 获取摄像头id
        robot_device_id = camera_data.get("robot_device_id")
        davice_data = DBRobotDevice().get_device_id(robot_device_id)
        if not davice_data:
            logger.error("无此摄像头")
            return False
        Id = davice_data.get("address")
        # logger.info('使用id为{}摄像头'.format(Id))

        # 摄像头参数
        device_param = camera_data.get("device_param")
        # 设置工业相机参数
        camera_params = dict()
        camera_params["ExposureTime"] = device_param.get("ExposureTime")
        camera_params["BalanceRatio"] = device_param.get("BalanceRatio")
        camera_params["Gain"] = device_param.get("Gain")
        try:
            if not self.dlcw_l.modify_industry_parameter(Id, None, None, camera_params):
                logger.error("工业相机参数设置失败")
                return False
        except Exception as e:
            logger.error(e)
            logger.error("工业相机参数设置失败")
            return False
        # 遍历算法ID获取对应的参数
        rois = list()
        for param_id in core_id_list:
            show_data = self.get_show_data(param_id)
            # logger.info(show_data)
            # logger.info(param_id)
            algorithm_data = DBCoreIndcameraParam().get_industry_id(int(param_id))
            # logger.info(algorithm_data)
            algorithm_param_for = algorithm_data.get("algorithm_param", {})
            u_params_for = algorithm_param_for.get("u_params", None)
            u_params_for["id"] = param_id
            u_params_for["show_data"] = show_data
            # logger.info(u_params_for)
            rois.append(u_params_for)
        photo_num = device_param.get("PhotoNum")
        photo_interval_time = device_param.get("PhotoIntervalTime")
        try:
            if int(photo_num) == 1:
                try:
                    if not self.dlcw_l.industry_photo(photo_name, Id):
                        logger.error("工业相机拍照异常")
                        data_value = "工业相机"
                        schedule_test(data_value)
                        return False
                except Exception as e:
                    logger.error(e)
                    logger.error("工业相机拍照异常")
                    data_value="工业相机"
                    schedule_test(data_value)
                    return False
                # P机柜 U设备名称 D起始U-结束U
                logger.info('开始算法识别...')
                rustle_dict = {"rois": rois, "light_path": photo_name}
                return rustle_dict
            else:
                file_name_base = photo_name.split(".jpg")[0]
                logger.info('开始合成照片')
                for num in range(photo_num):
                    file_name = file_name_base + "_{}.jpg".format(num)
                    try:
                        if not self.dlcw_l.industry_photo(photo_name, Id):
                            logger.error("工业相机拍照异常")
                            data_value="工业相机"
                            schedule_test(data_value)
                            return False
                    except Exception as e:
                        logger.error(e)
                        logger.error("工业相机拍照异常")
                        data_value="工业相机"
                        schedule_test(data_value)
                        return False
                    time.sleep(photo_interval_time)
                blend_state, photo_name = lightBlend(photo_name, photo_num)
                rustle_dict = {"rois": rois, "light_path": photo_name}
                return rustle_dict
        except Exception as e:
            logger.error(e)
            return False


    def analysis_insert(self, params):
        robot_device_status = RobotDeviceStatus()
        # 这个函数用于获取一个高度一个相机的所有数据，并对比规则插入数据
        core_id_list = params.get("core_id_list")
        core_device_id_list = params.get("core_device_id_list")
        start_u_list = params.get("start_u_list")
        end_u_list = params.get("end_u_list")
        task_data = params.get("task_data")
        db_robot_item = params.get("db_robot_item")
        core_cabinet_id = params.get("core_cabinet_id")
        inspect_project_detail_id = params.get("inspect_project_detail_id")

        # logger.info(task_data.get("robot_position_id"))
        # 判断参数长度，重新定义参数类型
        if len(core_id_list) == 1:
            core_indcamera_param_id = str(core_id_list[0])
        else:
            core_indcamera_param_id = ",".join([str(i) for i in core_id_list])
        try:
            industry_camera_up_status = robot_device_status.get_device_status("industry_camera_up")
            industry_camera_down_status = robot_device_status.get_device_status("industry_camera_down")
            if industry_camera_up_status == False or industry_camera_down_status == False:
                # 开始向告警信息表插入数据
                data_value = "工业相机"
                schedule_test(data_value)
            else:
                params_data=self.industry_industry_photo(core_indcamera_param_id)
                # 拍照成功，并且正常返回数据
                if params_data:
                    # 参数获取
                    show_data = params_data.get("show_data", 'show_data')
                    rois = params_data.get("rois")
                    light_path = params_data.get("light_path")
                    params.pop("db_inspect_item_data")
                    params.pop("db_robot_item")
                    post_data = dict()
                    post_data["data"] = {"rois": rois, "show_data": show_data, "params": params, "file_path": light_path}
                    post_data["type"] = 5
                    # 开始指示灯异步解析
                    try:
                        analysis_work = Thread(target=task_analysis, args=([post_data],))
                        analysis_work.start()
                        robot_position_id = params["data"]["task_data"]["robot_position_id"]
                        params["data"]["work_list"][robot_position_id].append(analysis_work)
                    except Exception as e:
                        logger.error(e)
        except Exception as e:
            logger.error(e)
            # 初始化工业相机状态
            use_status_obj=RobotDeviceStatus()
            use_status_obj.set_device_status("industry_camera_up", False)
            use_status_obj.set_device_status("industry_camera_down", False)
            # 开始向告警信息表插入数据
            data_value = "工业相机"
            schedule_test(data_value)

def action_elevator_height(position):
    url = "http://{}:{}/elevator".format(industry_api.get("host"), industry_api.get("port"))
    logger.info("升降杆升高{}".format(position))
    params = {
        "type": 1,
        "position": position
    }

    res = requests.get(url=url, params=params)
    if res.status_code != 200:
        return False
    return True


if __name__ == '__main__':
    from schema.db_robot_position import DBRobotPosition

    obj = RobotIndustryLight()
    position_data = DBRobotPosition().get_position(1, 2)
    for res in position_data:
        position_res = res
        result = {"core_cabinet_id": position_res.get("core_cabinet_id"),
                  "task_data": {"inspect_project_detail_id": 1111111111,
                                "robot_id": 1,
                                "robot_position_id": position_res.get("robot_position_id")
                                },
                  "ret": position_res
                  }
        obj.industry_photo_action(result)

