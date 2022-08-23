# -*- encoding:utf-8 -*-
import json
import time
import traceback
from datetime import datetime
from threading import Thread
from configs import rfid_time, db_rabbit_mq
from configs.log import logger
from schema.db_robot import DBRobot
from core.queue_config import rfid_queue, err_rfid_quene, check_rfid_quene
from schema.db_alarm_manage import DBAlarmManage
from core.redis_interactive import RedisPipe
from core.rabbitmq_db import RabbitPublisher
from socket import socket, AF_INET, SOCK_STREAM
from schema.db_core_inventory_type import DBCoreInventoryType
from schema.db_core_inventory_data import DBCoreInventoryDatum

host = '192.168.10.136'
port = 20058
addr = (host, port)
buf_size = 1024


class RFIdCheck(object):
    """
    RFID扫描
    """
    def __init__(self):

        self.list = []
        self.rf_id_list = RedisPipe("rf_id_list")
        self.tcpClient = socket(AF_INET, SOCK_STREAM)

    def start(self):
        self.tcpClient.connect((host, port))  # 由于tcp三次握手机制，需要先连接
        data = '1B 39 01 FF 01 00 00 00'  # 开始询标命令
        data_byte = bytes(bytearray.fromhex(data))
        self.tcpClient.send(data_byte)
        data_recvice = self.tcpClient.recv(1024).hex()
        logger.info(data_recvice)
        if data_recvice[0:16] == '1b3901ff01000000':
            logger.info("开始接收FRID数据")
            try:
                while True:
                    data_recvice = self.tcpClient.recv(1024).hex()
                    time.sleep(0.1)
                    if data_recvice not in self.list and data_recvice[0:8] == "1b3901ff":
                        self.list.append(data_recvice)
                        for i in self.list:
                            sn = i[20:28]
                            if not sn:
                                continue
                            rf_id_list = json.loads(str(self.rf_id_list.get_data(), "utf-8").replace("'", '"'))
                            if sn not in rf_id_list:
                                rf_id_list.append(sn)
                                self.rf_id_list.set_data(rf_id_list)
                                logger.info(sn)
                                rfid_queue.put(sn)

                    if data_recvice == '1b3b01ff01000000':
                        logger.info('询标命令关闭成功{}'.format(str(data_recvice)))
                        self.tcpClient.close()
                        return None
            except Exception as e:
                logger.error(traceback.format_exc(e))
                logger.error(e)
        else:
            logger.info('询标命令开启失败{}'.format(str(data_recvice)))

    def close(self):
        data = '1B 3B 01 FF 01 00 00 00'  # 关闭询标命令
        data_byte = bytes(bytearray.fromhex(data))

        self.tcpClient.send(data_byte)

class RFIDAssetsThan(object):
    """
    资产盘点
    """
    def __init__(self):
        self.scan_rfid = [] # rfid扫描到的rfid
        self.exist_rfid = [] # 需盘点得rfid

    def assets_than(self, rfid_check_device_list, task_data, robot_position_id, robot_room_id, inventory_record_id):
        for detail in rfid_check_device_list:
            self.exist_rfid.append(detail.rfid)
            check_rfid_quene.put(detail.rfid)

        for i in range(rfid_queue.qsize()):
            rf_id = rfid_queue.get()
            self.scan_rfid.append(rf_id)

        # 资产盘点
        self.insert_rf_id_data(task_data, robot_position_id, robot_room_id, inventory_record_id)

    def insert_rf_id_data(self, task_data, robot_position_id, robot_room_id, inventory_record_id):
        data = {
            "robot_path_id": task_data.get("robot_path_id", 0),
            "robot_id": task_data.get("robot_id", 0),
            "inventory_status": 1,
            "inspect_project_detail_id": task_data.get("inspect_project_detail_id", 0),
            "inspect_project_task_id": task_data.get("inspect_project_task_id"),
            "robot_position_id": robot_position_id,
            "inventory_record_id": inventory_record_id,
            "core_room_id": robot_room_id,
        }

        # 异常数据
        print("self.exist_rfid: ", self.exist_rfid)
        print("self.scan_rfid: ", self.scan_rfid)
        unusual_rfid_list = list(set(self.exist_rfid) - set(self.scan_rfid))
        print("unusual_rfid_list: ", unusual_rfid_list)
        if len(unusual_rfid_list) > 0:
            logger.info("rfid异常设备数量: " + str(len(unusual_rfid_list)))

            for unusual_rfid in unusual_rfid_list:
                err_rfid_quene.put(err_rfid_quene)

                unusual_entity = DBCoreInventoryType().get_by_rfid(rfid=unusual_rfid)
                if unusual_entity != None:
                    data["inventory_id"] = unusual_entity["inventory_id"]
                    data["status"] = 1
                    DBCoreInventoryDatum().insert_data(data)

                # 写入告警数据, 发送告警消息
                info = {}
                robot_id = task_data.get("robot_id")
                core_room_id = DBRobot().get_core_room_id(int(robot_id))
                
                info["inspect_project_detail_id"] = task_data.get("inspect_project_detail_id")
                info["robot_id"] = robot_id
                info["core_room_id"] = core_room_id
                info["core_device_id"] = 0
                info["core_cabinet_id"] = 0
                info["alarm_type"] = 9
                info["robot_item_id"] = 31
                info["level"] = 5
                info["num"] = 1
                info["value"] = "没有检测到该设备"
                info["alarm_desc"] = "识别到的RFID{}没有在资产表中".format(unusual_rfid)
                alarm_manage_id = DBAlarmManage().insert_data(info)
                
                data_dic = {"alarm_manage_id": alarm_manage_id, "type": 9}
                rout = db_rabbit_mq.get("robot_warn")
                routing_ey = db_rabbit_mq.get("warn_rout_key")
                warn_queue = db_rabbit_mq.get("warn_queue")
                RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)

                alarm_data = {
                    "sourceAlarmId": alarm_manage_id,
                    "alarmContent": "机器人RFID盘点告警",
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

        # 正常数据
        normal_rfid_list = list(set(self.exist_rfid) & set(self.scan_rfid))
        if len(normal_rfid_list) > 0:
            for normal_rfid in normal_rfid_list:
                normal_entity = DBCoreInventoryType().get_by_rfid(rfid=normal_rfid)
                if normal_entity != None:
                    data["inventory_id"] = normal_entity["inventory_id"]
                    data["status"] = 0
                    DBCoreInventoryDatum().insert_data(data)

        logger.info("rfid盘点动作结束")

def rfid_handler(rfid_check_device_list, task_data, robot_position_id, robot_room_id, inventory_record_id):
    rfid_check_data = RFIDAssetsThan()

    # 开启rfid扫描
    rfid_control = RFIdCheck()
    rfid_check = Thread(target=rfid_control.start)
    rfid_check.start()
    time.sleep(rfid_time)
    rfid_control.close()

    # 开始资产盘点
    rfid_check_data.assets_than(rfid_check_device_list, task_data, robot_position_id, robot_room_id, inventory_record_id)


if __name__ == '__main__':
    rfid = RFIdCheck()
    t_return = Thread(target=rfid.start)
    t_return.start()

    # 需要判断while巡检关闭的情况
    time.sleep(3)
    rfid.close()
    for i in range(rfid_queue.qsize()):
        rf_id = rfid_queue.get()
