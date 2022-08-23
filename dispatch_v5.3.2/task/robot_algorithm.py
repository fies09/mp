import json
import time
import base64
from schema.db_alarm_manage import DBAlarmManage
from utils import http_req
from configs import qrcode_identify_url,db_rabbit_mq
from configs.log import logger
from core.redis_interactive import RedisPipe
from task.light_al import post_voice_str
from modules.moving_ring_data import Dynamic_environment
from task.robot_door import AutoDoor
from task.robot_mq import publish_mq
from schema.db_robot_algorithm_param import DBRobotAlgorithmParam
from schema.db_robot_path_position_item import DBRobotPathPositionItem
from schema.db_robot_position import DBRobotPosition
from modules.robot_hardware_server import robot_move
from schema.db_inspect_project_detail import DBInspectProjectDetail

class RobotAlgorithm(object):
    def __init__(self):
        pass

    def alive_action_door(self, ret):
        door_rets = DBRobotPathPositionItem(). \
            get_door_param(id=ret.get("robot_position_id"))
        if door_rets:
            logger.info("该点存在开关门动作")
            door_ret = door_rets[0]
            logger.info(door_ret)
            al_data = DBRobotAlgorithmParam().get_by_id(door_ret.get("robot_algorithm_param_id"))
            if al_data:
                parameters = al_data.get("parameters")

                auto_door = AutoDoor(ip=parameters.get("ip"), channel=parameters.get("channel"))
                logger.info("开始进行该点的开关门动作,当前点动作为{}".format(door_ret.get("name")))
                if door_ret.get("name") == "开":
                    if auto_door.auto_open():
                        return True
                    else:
                        return False
                elif door_ret.get("name") == "关":
                    if auto_door.auto_close():
                        return True
                    else:
                        return False
                else:
                    logger.error("该点{}开关门动作配置错误！！！".format(ret.get("name")))
                    return False
            else:
                return False
        # else:
        #     logger.info("该点没有开关门动作")
        return True


    # 调取move算法
    def robot_back_al(self, ret, type_name):
        if not isinstance(ret.get("position"), dict):
            res = json.loads(ret.get("position"))
        else:
            res = ret.get("position")
        is_obstacle = ret.get("is_obstacle")
        if is_obstacle == 1:
            type_name = "detour_move"
        now_state = robot_move(res, type_name)
        if now_state:
            logger.info("机器人已到达目标点:{}({})".format(ret.get("name"), ret.get("robot_position_id")))
            # 给上层发送到底点位指令
            try:
                robot_position_id = ret.get("robot_position_id")
                body = RedisPipe("Robot_state").get_data()
                body = str(body, 'utf-8')
                data_dict = eval(body)
                cmd = DBInspectProjectDetail().get_top1()
                cmd_name = cmd.get("task_type_name")
                cmd_name_list = ["自动巡检", "设备盘点", "动力巡检", "参观"]
                if cmd_name not in cmd_name_list:
                    inspect_project_detail_id = data_dict.get("inspect_project_detail_id")
                    data = {"inspect_project_detail_id": inspect_project_detail_id,
                            "robot_position_id": robot_position_id}
                    publish_mq("robot_task", "rout_send_robotTask", "queue_send_robotTask", data)
            except Exception as e:
                logger.error(e)
                logger.error("推送到点MQ信息错误")
            try:
                post_voice_str(ret.get("name"), 2)
            except Exception as e:
                logger.error(e)
                logger.error("语音播报错误")
            if type_name == "back_charging":
                # 没有对上充电桩
                if not self.calibration_position(ret):
                    post_voice_str("没有到达充电桩", 1)
                    return False
            if not self.alive_action_door(ret=ret):
                logger.error("该点开关门动作没有完成")
                return False
            try:
                # logger.info("该达到巡检点位置为{}".format(ret.get("robot_position_id")))
                before_params = {'current_state': "0"}
                DBRobotPosition().update({'current_state': "1"}, before_params)
                # 更新此点为新的机器人所在点
                logger.info("更新此点 {}({}) 为新的机器人所在点".format(ret.get("name"),ret.get("robot_position_id")))
                params = {'current_state': '1'}
                DBRobotPosition().update({'robot_position_id': ret.get("robot_position_id")}, params)
                # logger.info("更新机器人位置点信息")
                return True
            except Exception as e:
                logger.error(e)
                return False
        else:
            logger.info("到达到点" + ret.get("name") + "超时")
        return False

    # 对接充电桩校准
    def calibration_position(self, ret):
        robot_path_id = ret.get("robot_path_id")
        num = 0
        nums = 0
        db_robot_position = DBRobotPosition()
        rets = db_robot_position.get_charging_point(ret.get("robot_id"), robot_path_id)
        while True:
            nums += 1
            if Dynamic_environment("battery_state") == 0:
                return True
            time.sleep(0.1)
            if nums > 20:
                break
        logger.info("开始多次对桩")
        post_voice_str("没有充电，开始多次对桩", 1)
        for i in range(5):
            logger.info("当前点的状态{}".format(Dynamic_environment("battery_state")))
            nums += 1
            if Dynamic_environment("battery_state") == 0:
                return True
            if num > 4:
                return False
            num += 1
            for ret in rets:
                if ret.get("type") == 3:
                    type_name = "forward_move"
                else:
                    type_name = "back_charging"
                if isinstance(ret.get("position"), dict):
                    res = ret.get("position")
                else:
                    res = json.loads(ret.get("position"))
                robot_move(res, type_name)
                time.sleep(3)

    # 七合一 （湿度，温度，甲醛，CO2,TVOC,PM25,PM1）
    # 调取温度算法
    def robot_seven_temperature(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前温度为" + str(data))

        return data

    # 调取湿度算法
    def robot_seven_humidity(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前湿度为" + str(data))

        return data

    # 调取甲醛算法
    def robot_seven_formaldehyde(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前甲醛浓度为" + str(data))

        return data

    # 调取CO2算法
    def robot_seven_co2(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前CO2浓度为" + str(data))

        return data

    # 调取TVOC算法
    def robot_seven_tvoc(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)
        if data:
            logger.info("获取当前TVOC为" + str(data))

        return data

    # 调取PM25算法
    def robot_seven_pm25(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前PM25为" + str(data))

        return data

    # 调取PM10算法
    def robot_seven_pm10(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前PM10为" + str(data))

        return data

    # 电池
    def robot_battery_power(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        return data

    # 电池温度
    def robot_battery_temp(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)
        return data

    # 二氧化硫
    def robot_so2(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前二氧化硫为" + str(data))

        return data

    # 硫化氢
    def robot_h2s(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前硫化氢为" + str(data))

        return data

    # PM1.0
    def robot_pm1(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前PM1.0为" + str(data))

        return data

    # PM1.0
    def robot_noise(self, result):
        name = result.get("name")
        data = Dynamic_environment(name)

        if data:
            logger.info("获取当前noise为" + str(data))

        return data


class QrcodeIdentify(object):
    """
    二维码盘点
    """
    def __init__(self):
        self.url = qrcode_identify_url

    def qrcode_req(self, file_path: str, pic_type: int):
        with open(file_path, 'rb') as f:  # 本地图片路径
            img = base64.b64encode(f.read()).decode('utf-8')
        req_param = {
            'picPath': None,      # 图片路径
            'picData': img,       # 原始图片数据(base64)
            'picType': pic_type,  # 数据类型,"1/2/3-普通/工业/web"
            'isSaved': True       # 是否保存识别结果
        }
        ok, res = http_req.HttpClient().http_client(method="GET", url=self.url, data=req_param, header={})
        if res.status_code == 200 and res:
            result_ok, device_id, img_data = self.result_pars(res)
            return result_ok, device_id, img_data
        else:
            return False, "", ""

    def result_pars(self, res):
        res_code = res.json()["code"]
        data = res.json()
        logger.info("算法识别结果：{}".format(data))
        if res_code == "1":
            logger.info("二维码识别完成,有结果输出")
            return True, [i["value"] for i in data["results"]], data["resPath"]
        elif res_code == "0":
            logger.error("二维码识别完成,未检测到二维码")
            print(data["resPath"])
            return False, [], data["resPath"]
        elif res_code == "-1":
            logger.error("请求参数错误(*缺少必要参数或者参数类型/值不合规)")
            return False, [], data["resPath"]





if __name__ == '__main__':
    ret = DBRobotPosition().get_position_id(25)[0]
    aa = RobotAlgorithm().alive_action_door(ret)
    print(aa)
