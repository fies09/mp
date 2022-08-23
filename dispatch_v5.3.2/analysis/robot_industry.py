from datetime import datetime
from configs.log import logger
from core.dict_config import lamp_dic
from analysis.robot_request import post_agx_light, post_agx_light_industry
from schema.db_inspect_position_item_data import DBInspectPositionItemDatum
from schema.db_robot_algorithm_param import DBRobotAlgorithmParam
from schema.db_robot_item import DBRobotItem
from schema.db_core_device_part import DBCoreDevicePart
# 解析照片
from core.dict_config import gongye_lamp_dic
# from analysis.robot_alarm import IndustryAlarm


def industry_img_discernment(file_path, rois, show_data, params):
    core_id_list = params.get("core_id_list")
    core_device_id_list = params.get("core_device_id_list")
    start_u_list = params.get("start_u_list")
    end_u_list = params.get("end_u_list")
    task_data = params.get("task_data")
    db_robot_items = DBRobotItem()
    db_core_device_part = DBCoreDevicePart()
    core_cabinet_id = params.get("core_cabinet_id")
    inspect_project_detail_id = params.get("inspect_project_detail_id")
    # 调用算法服务
    light_detect_results, light_path = post_agx_light_industry(file_path, rois, show_data, 2)
    # 更新数据库img路径
    if not light_path:
        logger.info("算法解析出错")
        return None
    light_result = light_detect_results
    # logger.info(light_result)
    # 遍历算法结果
    for light_res in light_result:
        # logger.info(light_res)
        light_detect_result = light_res.get("res")
        crop_path = light_res.get("path")
        # 算法返回的id已自定义成算法id，根据索引对应关系得出资产ID、start_u的值、end_u的值
        # core_device_id、start_u、end_u
        # 获取结果Id的索引
        index = core_id_list.index(int(light_res["id"]))
        # 根据索引获取各个框的参数
        core_device_id = core_device_id_list[index]
        start_u = start_u_list[index]
        end_u = end_u_list[index]
        server_u = str(start_u) + "-" + str(end_u)
        device_part = db_core_device_part.get_core_device_part(task_data.get("robot_position_id"), start_u, end_u)
        # 遍历指示灯红黄蓝绿
        for k, v in light_detect_result.items():
            ret = {}
            light_en_name = gongye_lamp_dic.get(k)
            task_data["core_device_id"] = core_device_id
            task_data["img_path"] = light_path
            # 识别告警
            # IndustryAlarm().contrast_alarm(task_data, light_en_name, v)

            item_data = db_robot_items.get_item_name(light_en_name)
            if item_data:
                item_id = item_data.get("robot_item_id")
            else:
                item_id = 0

            ret["inspect_project_detail_id"] = inspect_project_detail_id
            ret['user_id'] = task_data.get("user_id")
            ret['robot_id'] = task_data.get("robot_id")
            ret['value'] = v
            ret['server_u'] = server_u
            ret["create_time"] = datetime.now()
            ret["robot_position_id"] = task_data.get("robot_position_id")
            ret["inspect_project_task_id"] = task_data.get("inspect_project_task_id")

            ret["core_cabinet_id"] = core_cabinet_id
            ret["core_device_id"] = core_device_id
            # 检测项
            ret["robot_item_id"] = item_id
            ret["img_path"] = light_path
            ret['type'] = 7
            ret['crop_path'] = crop_path
            ret["robot_device_id"] = params.get("robot_device_id", 0)
            # 插入巡检数据
            ret["device_part_id"] = device_part.get("core_device_part_id")
            uuid = DBInspectPositionItemDatum().insert(ret)
            logger.info("插入工业相机识别结果信息ID为{}".format(str(uuid)))

        # logger.info("{}工业相机检测成功".format(light_path))
        # logger.info("{}工业相机识别结果".format(light_detect_result))
