import threading
import time
import json
import traceback
from threading import Thread
from core.queue_config import err_rfid_quene, err_qrcocde_quene, check_qrcode_quene, check_rfid_quene
from configs import rfid_query_name, qrcode_query_name
from configs.log import logger
from core.dict_config import algorithm_dic
from core.queue_config import rfid_queue
from core.redis_interactive import RedisPipe
from core.thread_pool import Pool
from modules.robot_hardware_server import robot_lifter
from modules.moving_ring_data import Dynamic_environment
from task.light_al import post_voice_str
from task.robot_action import RobotAction, move_state
from task.robot_al_pointer import PointerDistinguishAL
from task.robot_alarm import EnvironmentAlarm, AssetsAlarm
from task.robot_algorithm import RobotAlgorithm
from task.robot_fourfold_light import FourFoldLight
from task.robot_industry_light import RobotIndustryLight
from task.robot_init import init_positions
from task.robot_mq import publish_mq
from task.robot_rfid import RFIdCheck, rfid_handler
from task.robot_rfid import RFIdCheck
from modules.camera_compatibility import Hikvision, Daheng
from schema.db_core_cabinet import DBCoreCabinet
from schema.db_core_inventory_data import DBCoreInventoryDatum
from schema.db_core_inventory_record import DBCoreInventoryRecord
from schema.db_core_inventory_type import DBCoreInventoryType
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_power_cabinet import DBPowerCabinet
from schema.db_robot import DBRobot
from schema.db_robot_path_position_relation import DBRobotPathPositionRelation
from schema.db_robot_position import DBRobotPosition
from schema.db_inspect_position_item_data import DBInspectPositionItemDatum
from schema.db_robot_path_position_item import DBRobotPathPositionItem
from task.robot_qrcode import QRcodeControl
from modules.robot_hardware_server import RobotDeviceStatus
from core.rabbitmq_db import RabbitPublisher
from schema.db_alarm_manage import DBAlarmManage
from task.hardware_exception import schedule_test
from configs import db_rabbit_mq


class RobotAutoInspection:
    def __init__(self):
        self.robot_id = "1"
        self.robot_room_id = 4
        self.yuntai_state = 0
        self.inventory_record_id = 0
        self.pool = Pool
        self.rotbot_action = RobotAction()
        self.task_data = {}
        self.robot_algorithm = RobotAlgorithm()
        self.environment_alarm = EnvironmentAlarm()
        self.robot_industry_light = RobotIndustryLight()
        self.db_inspect_project_detail = DBInspectProjectDetail()
        self.rf_id_list = RedisPipe("rf_id_list")
        self.dlcw_l = None
        self.error_device_count = 0
        self.work_list = {}

    # 提取7合一算法
    def get_seven_data(self, algorithm_ret, res):   
        self.task_data["robot_position_id"] = res.get("robot_position_id")
        try:
            self.task_data.pop("img_path")
        except:
            pass
        for ret in algorithm_ret:
            name = ret.get("name")
            if name not in list(algorithm_dic.keys()):
                continue
            # 判断传感器硬件状态
            device_status = self.get_transducer_status(name)
            if not device_status:
                continue
            # 检测的内容
            method_name = algorithm_dic.get(name)
            try:
                data = getattr(self.robot_algorithm, method_name)(ret)
            except Exception as e:
                data = 0
                logger.error(e)
                logger.info("获取{}的值为错误")
            self.task_data["value"] = str(data)
            # logger.info(self.task_data)
            # self.environment_alarm.contrast_alarm(self.task_data, name, data)
            # 获取巡检任务表信息
            ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
            ret["core_cabinet_id"] = res.get("core_cabinet_id")
            ret["robot_path_id"] = res.get("robot_path_id")
            ret["power_cabinet_id"] = res.get("power_cabinet_id")
            ret['value'] = data
            # 插入数据
            try:
                logger.info("插入{}({})动环数据".format(res.get("name"), res.get("robot_position_id")))
                DBInspectPositionItemDatum().insert(info=ret)
            except Exception as e:
                logger.error(e)
        return True

    @staticmethod
    def get_transducer_status(name):
        """
        判断传感器是否可用
        """
        robot_device_status = RobotDeviceStatus()
        if name in ["平均温度", "最高温度", "最低温度"]:
            status = robot_device_status.get_device_status("thermal_imagery")
            if not status:
                logger.info("热成像设备不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value = "热成像传感器"
                schedule_test(data_value)
                return False
        elif name in ["温度", "湿度"]:
            status = robot_device_status.get_device_status("temhum")
            if not status:
                logger.info("温湿度传感器不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value = "温湿度传感器"
                schedule_test(data_value)
                return False
        elif name in ["PM2.5", "CO2", "TVOC", "PM10", "PM1", "甲醛"]:
            status = robot_device_status.get_device_status("mp_all")
            if not status:
                logger.info("七和一传感器不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value = "七和一传感器"
                schedule_test(data_value)
                return False
        elif name == "SO2":
            status = robot_device_status.get_device_status("sdioxide")
            if not status:
                logger.info("二氧化硫传感器不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value = "二氧化硫传感器"
                schedule_test(data_value)
                return False
        elif name == "H2S":
            status = robot_device_status.get_device_status("hsulfide")
            if not status:
                logger.info("硫化氢传感器不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value = "硫化氢传感器"
                schedule_test(data_value)
                return False
        elif name == "噪声":
            status = robot_device_status.get_device_status("noise")
            if not status:
                logger.info("噪声传感器不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value = "噪声传感器"
                schedule_test(data_value)
                return False
        else:
            return True
        return True

    # 调取云台动作
    def yuntai_location(self, ptz_ret, position_res):
        robot_device_status=RobotDeviceStatus()
        # 判断热成像可用状态
        device_status = self.get_transducer_status("最高温度")
        if not device_status:
            return False
        self.yuntai_state = 1
        alive_ptz, ptz_type_list, algorithm_type_list, res_device_type_list = self.screen_ptz_data(
            ptz_ret)
        for robot_ptz_param_id, robot_algorithm_param_id, res_device_param_id in zip(ptz_type_list, algorithm_type_list, res_device_type_list):
            # 调用云台动作
            ptz_status = robot_device_status.get_device_status("ptz")
            if ptz_status == False:
                logger.info("云台不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value = "云台"
                schedule_test(data_value)
                return False
            else:
                yuntai_state = self.rotbot_action.robot_yuntai_action(robot_ptz_param_id, 1)
                ptz_list = self.screen_ptz_dict(alive_ptz, robot_ptz_param_id)
            # 热成像拍照+数据处理
            thermal_imagery_status = robot_device_status.get_device_status("thermal_imagery")
            if thermal_imagery_status == False:
                logger.info("热成像服务不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value = "热成像"
                schedule_test(data_value)
                return False
            else:
                self.rotbot_action.robot_temp_action(res_device_param_id, ptz_list, position_res, self.task_data)
                time.sleep(1.5)
        # 云台回位
        self.yuntai_state = 0
        self.rotbot_action.robot_yuntai_action(1, 0)

    # 整合数据库调动云台算法的数据
    def screen_ptz_data(self, res_dic):
        res_list = []
        res_ptz_param = []
        res_algorithm_param = []
        res_device_param = []
        for res in res_dic:
            # 没有云台动作的时候 不调取云台
            if res.get("robot_ptz_param_id") != 0:
                res_list.append(res)
                res_ptz_param.append(res.get("robot_ptz_param_id"))
                res_algorithm_param.append(res.get("robot_algorithm_param_id"))
                res_device_param.append(res.get("robot_device_param_id"))
        res_ptz_type_list = sorted(set(res_ptz_param), key=res_ptz_param.index)
        res_algorithm_type_list = sorted(set(res_algorithm_param), key=res_algorithm_param.index)
        res_device_type_list = sorted(set(res_device_param), key=res_device_param.index)

        return res_list, res_ptz_type_list, res_algorithm_type_list, res_device_type_list

    def insert_rf_id_data(self, robot_position_id, status, rf_id): # FIXME
        logger.info(robot_position_id)
        logger.info(status)
        logger.info(rf_id)
        logger.info(self.task_data)
        rfid_data = DBCoreInventoryType().get_by_rfid(rf_id)
        logger.info(rfid_data)

        if rfid_data:
            inventory_id = rfid_data.get("inventory_id", 0)
        else:
            inventory_id = 0
        info = {}
        info["robot_path_id"] = self.task_data.get("robot_path_id", 0)
        info["robot_id"] = self.task_data.get("robot_id", 0)
        info["inventory_status"] = 1

        info["inventory_id"] = inventory_id
        info["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id", 0)
        info["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")

        info["robot_position_id"] = robot_position_id
        info["inventory_status"] = 1
        info["status"] = status
        info["remark"] = ""
        info["inventory_record_id"] = self.inventory_record_id
        info["core_room_id"] = self.robot_room_id
        DBCoreInventoryDatum().insert_data(info)

    def update_core_inv_record(self, error_device_count, check_qrcode_quene, check_rfid_quene):
        info = {}
        db_core_inventory_type = DBCoreInventoryType()
        info["robot_id"] = self.task_data.get("robot_id")
        info["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
        info["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")
        
        info["cabinet_count"] = db_core_inventory_type.get_rfid_cabinet_all()
        # info["device_count"] = db_core_inventory_type.get_rfid_device_all()
        info["device_count"] = check_qrcode_quene.qsize() + check_rfid_quene.qsize()
        info["device_fail_count"] = error_device_count

        DBCoreInventoryRecord().update({"inventory_record_id": self.inventory_record_id}, info=info)

    def insert_core_inv_record(self):
        info = {}
        db_core_inventory_type = DBCoreInventoryType()
        info["robot_id"] = self.task_data.get("robot_id")
        info["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
        info["inspect_project_task_id"] = self.task_data.get("inspect_project_task_id")

        info["cabinet_count"] = db_core_inventory_type.get_rfid_cabinet_all()
        info["device_count"] = db_core_inventory_type.get_rfid_device_all()
        try:
            rf_id_list = json.loads(str(self.rf_id_list.get_data(), "utf-8").replace("'", '"'))
            lack_data_list = [y for y in filter(lambda x: x not in rf_id_list, self.rfid_alive_list)]
        except Exception as e:
            logger.error(traceback.format_exc(e))
            lack_data_list = []
            logger.error(e)
        info["device_fail_count"] = len(lack_data_list)
        if lack_data_list:
            for lack_dat in lack_data_list:
                self.insert_rf_id_data(0, 1, lack_dat)

        DBCoreInventoryRecord().update({"inventory_record_id": self.inventory_record_id}, info=info)

    def data_obtain(self, result, error_device_count):
        robot_device_status = RobotDeviceStatus()
        # 工业相机初始化
        logger.info("++++++++++++++++++++++ 初始化工业相机 ++++++++++++++++++++++")
        industrial_camera_type = RedisPipe("industrial_camera_type").get_data()
        if industrial_camera_type:
            if str(industrial_camera_type, 'utf-8') == "Hikvision":
                self.dlcw_l = Hikvision()
            elif str(industrial_camera_type, 'utf-8') == "Daheng":
                self.dlcw_l = Daheng()
            else:
                logger.error("未发现工业相机")
        # type 1为巡检点 3为充电预备点 4为充电点
        global start_rf_id_action, qrcode_check
        for res in result:
            robot_position_id = res.get("robot_position_id")
            robot_path_id = res.get("robot_path_id")
            self.work_list[robot_position_id] = list()
            position_res = DBRobotPosition().get_position_by_id(robot_position_id, robot_path_id)
            logger.info("========== 开始执行{}(id:{})点位任务 ==========".format(position_res["name"], robot_position_id))
            # logger.info(position_res)
            if not self.get_task_cmd():
                return False
            # 获取机柜id
            self.task_data["power_cabinet_id"] = position_res.get("power_cabinet_id", None)
            self.task_data["core_cabinet_id"] = position_res.get("core_cabinet_id", None)
            # 行走点
            if position_res.get("type") == 1:
                self.task_data["robot_position_id"] = position_res.get("robot_position_id")
                core_cabinet_id = position_res.get("core_cabinet_id")
                if core_cabinet_id:
                    core_room_id = DBCoreCabinet().get_room_id(core_cabinet_id)
                    if core_room_id:
                        self.core_room_id = core_room_id
                        self.task_data["core_room_id"] = core_room_id
                else:
                    robot_id = position_res.get("robot_id")
                    core_room_id = DBRobot().get_core_room_id(int(robot_id))
                    logger.info("当前机房id为{}".format(str(core_room_id)))
                    self.robot_room_id = core_room_id
                    self.task_data["core_room_id"] = core_room_id

                # 调取move算法
                move_status=robot_device_status.get_device_status("move")
                if move_status == False:
                    logger.info("移动服务不可用，跳过此检测项...")
                    # 开始向告警信息表插入数据
                    data_value = "移动服务"
                    schedule_test(data_value)
                    return False
                else:
                    move_state = self.robot_algorithm.robot_back_al(position_res, "forward_move")
                if move_state:
                    # 更新当前点
                    self.task_data["robot_position_id"] = position_res.get("robot_position_id")
                    try:
                        # 获取该点所设置的检测点采集项
                        algorithm_ret = DBRobotPathPositionItem().get_robot_position_id(
                            ids=position_res.get("robot_position_id"))
                        # logger.info(algorithm_ret)
                    except Exception as e:
                        logger.error(e)
                        algorithm_ret = None
                    # 存在动环检测，开始执行动环检测
                    # algorithm_ret = False
                    if algorithm_ret:
                        logger.info("---------- 开始执行该点的动环检测 ----------")
                        if not self.get_task_cmd():
                            return False
                        # self.pool.submit(self.get_seven_data, algorithm_ret, position_res)
                        seal_head_work = Thread(target=self.get_seven_data, args=(algorithm_ret, position_res))
                        seal_head_work.start()
                        self.work_list[robot_position_id].append(seal_head_work)
                    else:
                        logger.info("该点 {}({}) 没有需要配置动环检测".format(position_res["name"], robot_position_id))
                    # 调取该点所添加的RFID动作
                    if self.task_data.get("task_type_name") == "设备盘点":
                        # 查询当前点位机柜
                        # 判断当前点位是否存在
                        logger.info("---------- 开始执行该点的设备盘点 ----------")
                        position_cabinet = DBRobotPosition().get_by_position_id(robot_position_id=position_res.get("robot_position_id"))
                        if not position_cabinet:
                            return
                        # 判断当前点是否有rfid检测项目
                        rf_id_ret = DBRobotPathPositionItem().get_robot_rfid_param(id=position_res.get("robot_position_id"), query_name=rfid_query_name)
                        if rf_id_ret:
                            logger.info("开始rfid盘点")
                            # 查询当前点位需检测的全部设备
                            rfid_check_device, check_device_total = DBCoreInventoryType().get_list([], **{"core_cabinet_id":position_cabinet["core_cabinet_id"], "inventory_type":1})
                            if len(rfid_check_device) == 0:
                                return

                            # 开启2个线程分别执行rfid检测和qrcode检测
                            start_rf_id_action = Thread(target=rfid_handler, args=(rfid_check_device, self.task_data, position_res.get("robot_position_id"), self.robot_room_id, self.inventory_record_id,))  # rfid检测
                            start_rf_id_action.start()
                            self.work_list[robot_position_id].append(start_rf_id_action)

                        # 判断当前点是否有二维码检测项
                        qrcode_ids_ret = DBRobotPathPositionItem().get_robot_rfid_param(id=position_res.get("robot_position_id"), query_name=qrcode_query_name)
                        if qrcode_ids_ret != None and len(qrcode_ids_ret)> 0:
                            logger.info("开始二维码盘点")

                            # 查询当前点位需检测的全部设备
                            qrcode_check_device, check_device_total = DBCoreInventoryType().get_list([], **{"core_cabinet_id": position_cabinet["core_cabinet_id"], "inventory_type": 2})
                            if len(qrcode_check_device) == 0:
                                return

                            qrcode_check = Thread(target=QRcodeControl().qrcode_start, args=(qrcode_check_device, qrcode_ids_ret, self.dlcw_l, self.task_data, position_res.get("robot_position_id"), self.robot_room_id, self.inventory_record_id,)) # 二维码检测
                            qrcode_check.start()
                            self.work_list[robot_position_id].append(qrcode_check)

                        if rf_id_ret:
                            # 等待盘点结束
                            start_rf_id_action.join()
                            # 更新异常设备数量
                            self.error_device_count += err_rfid_quene.qsize()
                            # 清空quene
                            for i in range(err_rfid_quene.qsize()):
                                rf_id_err = err_rfid_quene.get()
                            logger.info("当前点rfid盘点结束")   
                            logger.info("rfid盘点异常设备数量: " + str(error_device_count))
                            
                        if qrcode_ids_ret:
                            # 等待盘点结束
                            qrcode_check.join()
                            # 更新异常设备数量
                            self.error_device_count += err_qrcocde_quene.qsize()
                            # 清空quene
                            for i in range(err_qrcocde_quene.qsize()):
                                qrcode_id_err = err_qrcocde_quene.get()
                            logger.info("当前点二维码盘点结束")    
                            logger.info("二维码盘点异常设备数量: " + str(error_device_count))

                        logger.info("当前点盘点异常设备数量: " + str(error_device_count))

                    # 调取该点所添加的云台动作
                    ptz_ret=DBRobotPathPositionItem().\
                        get_robot_ptz_param(id=position_res.get("robot_position_id"))

                    # 异步执行温度检测
                    # ptz_ret = False
                    if ptz_ret:
                        logger.info("---------- 开始执行该点的热成像采集 ----------")
                        if not self.get_task_cmd():
                            return False
                        logger.info("开始进行热成像检测")
                        logger.info(ptz_ret)
                        # self.pool.submit(self.yuntai_location, ptz_ret, position_res)
                        thermal_work = Thread(target=self.yuntai_location, args=(ptz_ret, position_res))
                        thermal_work.start()
                        self.work_list[robot_position_id].append(thermal_work)
                    else:
                        logger.info("该点 {}({}) 没有需要配置热成像检测".format(position_res["name"], robot_position_id))

                    # 调取该点所添加升降杆动作
                    # position_res["core_cabinet_id"] = 0
                    if position_res.get("core_cabinet_id") != 0:
                        logger.info("---------- 开始执行该点的工业指示灯采集 ----------")
                        if not self.get_task_cmd():
                            return False
                        self.task_data["core_cabinet_id"] = position_res.get("core_cabinet_id")
                        self.task_data["power_cabinet_id"] = position_res.get("power_cabinet_id")
                        result = {
                            "core_cabinet_id": position_res.get("core_cabinet_id"),
                            "task_data": self.task_data,
                            "ret": position_res,
                            "work_list": self.work_list
                        }
                        self.robot_industry_light.industry_photo_action(result, self.dlcw_l)
                    else:
                        logger.info("该检测点 {}({}) 没有工业指示灯采集项".format(position_res["name"], robot_position_id))
                    # 确定是否有四倍相机检测
                    elevator_ret = DBRobotPathPositionItem(). \
                        get_robot_elevator_param(id=position_res.get("robot_position_id"))
                    # elevator_ret = False
                    if elevator_ret:
                        logger.info("---------- 开始执行该点的四倍相机采集 ----------")
                        if not self.get_task_cmd():
                            return False
                        logger.info("获取该检测点id {}四倍相机采集项".format(position_res.get("robot_position_id")))
                        self.task_data["core_cabinet_id"] = position_res.get("core_cabinet_id")
                        self.task_data["power_cabinet_id"] = position_res.get("power_cabinet_id")
                        FourFoldLight().elevator_action(elevator_ret, self.task_data,
                                                        position_res.get("core_cabinet_id"), self.work_list)
                    else:
                        logger.info("该检测点 {}({}) 没有四倍相机采集项".format(position_res["name"], robot_position_id))
                    if position_res.get("power_cabinet_id") != 0:
                        logger.info("---------- 开始执行该点的动力设备采集 ----------")
                        if not self.get_task_cmd():
                            return False
                        power_cabinet_id = position_res.get("power_cabinet_id")
                        if power_cabinet_id:
                            core_room_id = DBPowerCabinet().get_room_id(power_cabinet_id)
                            if core_room_id:
                                self.task_data["core_room_id"] = core_room_id
                        result = {
                            "power_cabinet_id": position_res.get("power_cabinet_id"),
                            "task_data": self.task_data,
                            "ret": position_res,
                            "work_list": self.work_list
                        }
                        PointerDistinguishAL().power_photo_action(result)
                    else:
                        logger.info("该检测点 {}({}) 没有动力设备采集项".format(position_res["name"], robot_position_id))
                    try:
                        robot_lifter(1, 0)
                    except Exception as e:
                        logger.error(e)
                        # 开始向告警信息表插入数据
                        data_value = "升降杆"
                        schedule_test(data_value)

                    for num in range(10):
                        time.sleep(1)
                        if self.yuntai_state == 0:
                            break
                        if num > 10:
                            break
                else:
                    logger.info("没有到达点{}".format(str(position_res)))
                    return False
                self.rotbot_action.robot_yuntai_action(1, 0)

            # 返回充电预备点
            if position_res.get("type") == 3:
                try:
                    logger.info("准备到达充电预备点")
                    logger.info(position_res)
                    move_state = self.robot_algorithm.robot_back_al(position_res, "forward_move")
                    if not move_state:
                        return False
                except Exception as e:
                    logger.info("出现超时！停止自动巡检任务！")
                    logger.error(e)
                    # 开始向告警信息表插入数据
                    data_value = "移动服务"
                    schedule_test(data_value)
                    return False
            # 返回充电点
            if position_res.get("type") == 4:
                logger.info("准备到达充电点")
                type_name = "back_charging"
                try:
                    move_state = self.robot_algorithm.robot_back_al(position_res, type_name)
                    if not move_state:
                        # 开始向告警信息表插入数据
                        data_value="移动服务"
                        schedule_test(data_value)
                        return False
                except Exception as e:
                    logger.info("出现超时！停止自动巡检任务！")
                    logger.error(e)
                     # 开始向告警信息表插入数据
                    data_value="移动服务"
                    schedule_test(data_value)
                    return False
                return True
            logger.info("========== {}(id:{})点位任务已结束 ==========".format(position_res["name"], robot_position_id))
            # 开启点位线程监听
            logger.info("点位任务监听线程已开启......")
            Thread(target=self.listen_thread, args=(robot_position_id,)).start()
        # 设置云台在充电点时的位置
        self.rotbot_action.robot_yuntai_action(2, 0)

        return True

    def get_task_cmd(self):
        cmd = DBInspectProjectDetail().get_top1()
        if not cmd:
            return False
        cmd_name = cmd.get("task_type_name")
        # logger.info(cmd_name)
        cmd_name_list = ["自动巡检", "设备盘点", "动力巡检", "参观"]
        if cmd_name not in cmd_name_list:
            return False
        return True

    # 取出对应云台设置的值
    def screen_ptz_dict(self, dicts, keys):
        elevator_list = []
        for a_dict in dicts:
            if a_dict.get("robot_ptz_param_id") == keys:
                elevator_list.append(a_dict)

        return elevator_list

    # 自动巡检主入口
    def robot_start_inspection(self, task_data):
        robot_device_status=RobotDeviceStatus()
        logger.info("已进入主任务入口......")
        # 更改task执行状态
        self.rf_id_check = None
        self.rfid_alive_list = []

        self.task_data = task_data
        self.task_data['status'] = 1
        self.robot_inspect_project(task_data, 1)
        self.robot_id = self.task_data.get("robot_id")
        self.robot_path_id = task_data.get("robot_path_id")
        # 判断是否在充电的上
        now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)
        # logger.info("当前机器人位置为{}".format(str(now_location)))
        if not now_location:
            init_positions(self.robot_path_id)
            now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)
            logger.info("初始化机器人位置{}".format(str(now_location)))

        now_location_name = now_location.get("name")

        # 不在充电点时先返回到充电点
        if now_location.get("type") != 4:
            # 通过redis判断移动服务的状态是否正常
            move_status = robot_device_status.get_device_status("move")
            if move_status == False:
                logger.info("移动服务不可用，跳过此检测项...")
                # 开始向告警信息表插入数据
                data_value="移动服务"
                schedule_test(data_value)
                return False
            if not move_state(self.robot_id, task_data.get("robot_path_id")):
                logger.info("返回充电点没有成功！！！")
                return False

        # 判断是否需要设备盘点
        logger.info("当前任务数据")
        logger.info(task_data)
        if task_data.get("task_type_name") == "设备盘点":
            info = dict()
            info["robot_id"] = self.task_data.get("robot_id")
            info["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
            self.inventory_record_id = DBCoreInventoryRecord().insert_data(info=info)
        # 开始巡检 成功返回True
        # 获取点位信息
        position_relation_data = DBRobotPathPositionRelation().get_position_id(self.robot_path_id)
        # logger.info("-------------  进入移动循环 ----------------")
        logger.info("此次任务的移动路径ID为:{},途径{}个点".format(self.robot_path_id, len(position_relation_data)))
        inspection_state = self.data_obtain(position_relation_data, self.error_device_count)
        logger.info("inspection_state: " + str(inspection_state))
        # 设置云台在充电点时的位置
        logger.info("---------- 云台归位 ----------")
        self.rotbot_action.robot_yuntai_action(1, 1)
        if inspection_state:
            logger.info("此次盘点任务结束")
            logger.info("异常设备数量error_device_count: " + str(self.error_device_count))
            self.update_core_inv_record(self.error_device_count, check_qrcode_quene, check_rfid_quene)
            logger.info("资产盘点结束")
            # 语音播报:自动巡检完成
            # 结束后任务状态设置为2
            self.task_data["status"] = 2
            self.robot_inspect_project(task_data, 2)
            return True
        else:
            logger.info("没有执行完整个流程{}".format(str(task_data)))
            # logger.info("任务未完成开始返回到充电桩")
            # return_ret = DBRobotPosition().get_battery_point()
            # if move_state(return_ret.get("robot_id"), return_ret.get("robot_path_id"), move_state=1):
            #     post_voice_str("已返回到充电点", 1)
            RedisPipe("ActionTask").set_data({"ActionTask": 0})
            return False

    # 更新机器人状态巡检状态信息
    def robot_inspect_project(self, task_data, state):
        # logger.info(task_data)
        params = {}
        params['status'] = state
        if state == 6 or state == 2:
            start_battery_power = task_data.get("now_battery_power")
            logger.info(start_battery_power)

            end_battery_power = Dynamic_environment("battery_power")
            expend_battery_power = start_battery_power - end_battery_power

            params['power_consumption'] = expend_battery_power
        if state == 6:
            params['exception_info'] = "该任务被中断"
        inspect_project_detail_id = task_data.get("inspect_project_detail_id")
        DBInspectProjectDetail().update({'inspect_project_detail_id': inspect_project_detail_id}, params)
        # 更新mq内容
        data = {"inspect_project_detail_id": inspect_project_detail_id, "status": state}
        publish_mq("robot_task", "task_rout_key", "task_queue", data=data)

    def listen_thread(self, robot_position_id):
        """
        此方法监听了各个点位的所有线程
        :param robot_position_id:
        :return:
        """
        while True:
            time.sleep(1)
            if not self.work_list[robot_position_id]:
                del self.work_list[robot_position_id]
                logger.info("点{}所有监听线程已结束".format(robot_position_id))
                logger.info("推送点位任务已结束的信息")
                inspect_project_detail_id = self.task_data.get("inspect_project_detail_id")
                data = {"inspect_project_detail_id": inspect_project_detail_id,
                        "robot_position_id": robot_position_id}
                publish_mq("robot_task", "rout_send_robotTask", "queue_send_robotTask", data)
                break
            for i in self.work_list[robot_position_id]:
                if not i.isAlive():
                    self.work_list[robot_position_id].remove(i)

    # 更新自动巡检中断时数据库状态信息
    def __del__(self):
        logger.info("结束当前任务__del__{}".format(self.task_data))
        robot_task_status = self.task_data.get("status")
        logger.info("结束当前任务__del__状态{}".format(robot_task_status))
        # 设置云台在充电点时的位置
        logger.info("---------- 云台归位 ----------")
        self.rotbot_action.robot_yuntai_action(1, 1)
        # 关闭rf_id
        if self.rf_id_check:
            # self.rf_id_check.close()
            logger.info("开始对比RFID数据信息")
            self.insert_core_inv_record()
            try:
                rf_id_list = json.loads(str(self.rf_id_list.get_data(), "utf-8").replace("'", '"'))
                rfid_alarm_list = list(set(self.rfid_alive_list) ^ set(rf_id_list))
                assets_alarm = AssetsAlarm()
                for rfid_alarm in rfid_alarm_list:
                    assets_alarm.contrast_alarm(self.task_data, rfid_alarm)
            except Exception as e:
                logger.info("没有RFID告警信息")
                logger.error(e)
        self.rf_id_list.set_data(json.dumps([]))

        if robot_task_status == 1:
            self.robot_inspect_project(self.task_data, 6)

        RedisPipe("ActionTask").set_data({"ActionTask": 0})


def auto_ins(task_data):
    robot_auto_inspection = RobotAutoInspection()
    robot_auto_inspection.robot_start_inspection(task_data)
