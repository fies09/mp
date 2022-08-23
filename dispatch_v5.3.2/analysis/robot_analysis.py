#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/27 10:01
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : robot_analysis.py
# @description: "数据解析入口"

import traceback
from configs.log import logger
from analysis.robot_four import img_discernment
from analysis.robot_request import post_agx_plot
from analysis.robot_ocr import PointerDistinguishAL
from analysis.robot_industry import industry_img_discernment
from schema.db_power_inspect_data import DBPowerInspectDatum


def task_analysis(data_list):
    """
    检测值是否正确
    :param data_list:解析的相关数据json类型
    :param :解析的类型 0：表示无解析 1：指示灯解析(四倍) 2：数字仪表盘解析 3：指针仪表盘解析 4：开关解析 5：工业相机解析
    :return:
    """
    try:
        pointer_distinguish_al = PointerDistinguishAL()
        plot_list = list()
        all_power_inspect_data_id_list = list()
        photo = None
        # logger.info(data_list)
        for dict_data in data_list:
            data = dict_data.get("data")
            type = dict_data.get("type")
            ret = data.get("ret", None)
            elevator_list = data.get("elevator_list", None)
            task_data = data.get("task_data", None)
            robot_algorithm_param_id = data.get("robot_algorithm_param_id", None)
            core_cabinet_id = data.get("core_cabinet_id", None)
            # 共用字段
            file_path = data.get("file_path", None)

            rois = data.get("rois", None)
            show_data = data.get("show_data", None)
            params = data.get("params", None)

            camera_data = data.get("camera_data", None)
            power_device_id = data.get("power_device_id", None)
            power_cabinet_id = data.get("power_cabinet_id", None)
            robot_algorithm_id = data.get("robot_algorithm_id", None)
            photo_name = data.get("photo_name", None)
            photo = photo_name
            if type == 0:
                return False
            elif type == 1:
                # 四倍相机指示灯解析
                img_discernment(ret, elevator_list, task_data, robot_algorithm_param_id, file_path, core_cabinet_id)
            elif type == 2:
                # 数字仪表盘解析
                plot_data, power_inspect_data_id_list = pointer_distinguish_al.digital_dashboard_al(camera_data,
                                                                                                    power_device_id,
                                                                                                    power_cabinet_id,
                                                                                                    robot_algorithm_id,
                                                                                                    photo_name,
                                                                                                    task_data)
                logger.info(plot_data)
                logger.info(power_inspect_data_id_list)
                if plot_data:
                    plot_list.append(plot_data)
                all_power_inspect_data_id_list += power_inspect_data_id_list
            elif type == 3:
                # 指针仪表盘解析
                plot_data, power_inspect_data_id_list = pointer_distinguish_al.pointer_dashboard_al(camera_data,
                                                                                                    power_device_id,
                                                                                                    power_cabinet_id,
                                                                                                    robot_algorithm_id,
                                                                                                    photo_name,
                                                                                                    task_data)
                logger.info(plot_data)
                logger.info(power_inspect_data_id_list)
                if plot_data:
                    plot_list.append(plot_data)
                all_power_inspect_data_id_list += power_inspect_data_id_list
            elif type == 4:
                # 开关解析
                plot_data, power_inspect_data_id_list = pointer_distinguish_al.switch_dashboard_al(camera_data,
                                                                                                   power_device_id,
                                                                                                   power_cabinet_id,
                                                                                                   robot_algorithm_id,
                                                                                                   photo_name,
                                                                                                   task_data)
                logger.info(plot_data)
                logger.info(power_inspect_data_id_list)
                if plot_data:
                    plot_list.append(plot_data)
                all_power_inspect_data_id_list += power_inspect_data_id_list
            elif type == 5:
                # 工业相机指示灯解析
                industry_img_discernment(file_path, rois, show_data, params)
            else:
                continue

        # 访问绘图服务
        # logger.info(plot_list)
        # logger.info(photo)
        # logger.info(all_power_inspect_data_id_list)
        logger.info("开始访问算法绘图服务")
        if plot_list and photo:
            plot_photo_path = post_agx_plot(photo, plot_list, 2)
            db_power_inspect_data = DBPowerInspectDatum()
            logger.info("更新绘图数据")
            db_power_inspect_data.update(all_power_inspect_data_id_list, {"img_path": plot_photo_path})

        else:
            logger.info("绘图参数为空")
        return True

    except Exception as e:
        logger.error(e)
        # self.retry(exc=e, countdown=3, max_retries=5)
    return True




