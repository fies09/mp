from datetime import datetime
from configs.log import logger
from core.dict_config import lamp_dic
from analysis.robot_alarm import FourFoldAlarm
from analysis.robot_request import post_agx_light
from schema.db_inspect_position_item_data import DBInspectPositionItemDatum
from schema.db_robot_algorithm_param import DBRobotAlgorithmParam
from schema.db_core_device_part import DBCoreDevicePart


# 解析照片

def img_discernment(ret, elevator_list, task_data, robot_algorithm_param_id, file_path, core_cabinet_id):
    param_ret = DBRobotAlgorithmParam().get_by_id(robot_algorithm_param_id)
    logger.info(param_ret)
    parameters = param_ret.get("parameters")
    u_params = parameters.get("u_params")
    start_u = u_params.get("start_u")
    end_u = u_params.get("end_u")
    start_imgRows = int(u_params.get("start_imgRows"))
    end_imgRows = int(u_params.get("end_imgRows"))
    start_imgCols = int(u_params.get("start_imgCols"))
    end_imgCols = int(u_params.get("end_imgCols"))

    logger.info("合成后的图片-------》{}".format(file_path))
    light_detect_results, light_path, narrow_path = post_agx_light(file_path, start_imgRows, end_imgRows,
                                                                  start_imgCols, end_imgCols, "", 1)
    if not narrow_path:
        narrow_path = light_path
    light_detect_result = light_detect_results["res"]
    crop_path = light_detect_results["path"]
    logger.info("照片的解析" + str(light_detect_result))
    logger.info("所属机柜id{}".format(core_cabinet_id))
    if not light_path:
        return False
    else:
        # 获取部件信息
        device_part_data = DBCoreDevicePart().get_core_device_part(task_data["robot_position_id"], start_u, end_u)
        task_data["img_path"] = light_path
        for elevator in elevator_list:
            # 插入数据
            lamp_name = elevator.get("name")
            lamp_name_en = lamp_dic.get(lamp_name)
            lamp_num = light_detect_result.get(lamp_name_en)
            # 获取
            task_data["img_path"] = light_path
            FourFoldAlarm().contrast_alarm(task_data, lamp_name, lamp_num, robot_algorithm_param_id)
            ret["server_u"] = str(start_u)
            ret["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
            ret["inspect_project_task_id"] = task_data.get("inspect_project_task_id")
            ret["robot_position_id"] = elevator.get("robot_position_id")
            ret["robot_path_id"] = task_data.get("robot_path_id")
            ret["core_cabinet_id"] = core_cabinet_id
            ret["robot_item_id"] = elevator.get("robot_item_id")
            ret["create_time"] = datetime.now()
            ret['value'] = lamp_num
            ret["img_path"] = light_path
            ret['type'] = 3
            ret['crop_path'] = crop_path
            ret["core_device_id"] = device_part_data.get("core_device_id", 0)
            ret["device_part_id"] = device_part_data.get("core_device_part_id", 0)
            # robot_device_id ret已存在
            DBInspectPositionItemDatum().insert(info=ret)

        return True
