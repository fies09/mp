# -*- coding: utf-8 -*-
"""
@Time       : 2022/05/27 15:57
@Author     : fany
@FileName   : test02.py
@Description: "机器人硬件配置文件"
"""
from configs.log import logger
from std_msgs.msg import Float64MultiArray, String, Bool, UInt8, Bool, Int8
from sensor_msgs.msg import BatteryState
from schema.db_alarm_manage import DBAlarmManage
from datetime import datetime
from configs import db_rabbit_mq
from core.rabbitmq_db import RabbitPublisher

inspect_res = {
    "sensor": {
        "temhum": "正常",
        "noise": "正常",
        "battery": "正常",
        "mp_all": "正常",
        "sidoxide": "正常",
        "hsulfide": "正常",
        # "wind_velocity": "正常",
    },
    "camera": {
        "ptz": "正常",
        "fourfold": "正常",
        "lift": "正常",
        "thermal_imagery": "正常",
        "industrial": "正常"
    },
    "state": {
        "scan": "正常",
        "IMU": "正常",
        "fall": "正常",
        "avoidance": "正常"
    },
    "server": {
        "agx": "正常",
        "ips": "正常",
        "arithmetic": "正常",
    }
}
# 传感器
sensor_list = [
    ["/temhum", Float64MultiArray, "温湿度"],
    ["/noise", String, "噪音"],
    ["/battery", BatteryState, "电池"],
    ["/mp_all", Float64MultiArray, "七合一"],
    ["/sdioxide", String, "二氧化硫"],
    ["/hsulfide", String, "硫化氢"],
    # ["/wind_velocity", Float64MultiArray, "风速"],
]
# 算法
server_data = {
    "ip": "10.173.27.120",
    "all_port" : [[8086, "agx"], [8088, "ips"]],
    "arithmetic_ip" : "192.168.10.134",
    "arithmetic_port" : [8087, "arithmetic"]
}
def sensor_test():
    res_list = []
    for i in sensor_list:
        key = i[0][1:]
        inspect_res["sensor"][key]="异常"
        logger.info("{}检测异常".format(i[2]))
        res_list.append(False)
        # 将告警信息插入alarm_manage数据表
        res = DBAlarmManage()
        result = res.get_alarm_data(0, 0)
        if result:
            if result['value']==i[2]:
                if result['status'] != 0:
                    pass
                else:
                    # 告警信息更新
                    num=result['num']
                    sensor_update = {
                        'inspect_project_detail_id':0,
                        'robot_position_id':0,
                        'num': 2,
                        'value': i[2],
                        'undo_status':0,
                        'alarm_desc': i[2] + '传感器检测异常',
                        'status':0,
                        'assign':0,
                        'user_id': 0,
                        'update_time':datetime.now(),
                    }
                    logger.info(i[2] + "检测异常, 更新告警信息")
                    res.update({'num':num},sensor_update)
                    logger.info("告警信息更新成功")
            else:
                # 添加告警信息
                sensor_insert = {
                    'robot_id':1,
                    'core_room_id':0,
                    'inspect_project_detail_id':0,
                    'inspect_project_task_id':0,
                    'robot_position_id':0,
                    'robot_item_id':0,
                    'core_cabinet_id':0,
                    'robot_device_id':'0',
                    'alarm_type':10,
                    'level':5,
                    'num':1,
                    'value':i[2],
                    'alarm_desc':i[2]+'传感器检测异常',
                    'status':0,
                    'assign':0,
                    'create_time':datetime.now(),
                    'user_id':0,
                    'update_time':datetime.now(),
                }
                logger.info(i[2] + "检测异常, 添加告警信息")
                res.insert_data(sensor_insert)
                logger.info("告警信息添加成功")
        else:
            sensor_insert={
                'robot_id':1,
                'core_room_id':0,
                'inspect_project_detail_id':0,
                'inspect_project_task_id':0,
                'robot_position_id':0,
                'robot_item_id':0,
                'core_cabinet_id':0,
                'robot_device_id':'0',
                'alarm_type':10,
                'level':5,
                'num':1,
                'value':i[2],
                'alarm_desc':i[2]+'传感器检测异常',
                'status':0,
                'assign':0,
                'create_time':datetime.now(),
                'user_id':0,
                'update_time':datetime.now(),
            }
            logger.info(i[2] + "检测异常, 添加告警信息")
            res.insert_data(sensor_insert)
            logger.info("告警信息添加成功")

def server_tests():
    all_port = server_data.get("all_port")
    for i_port in all_port:
        inspect_res["server"][i_port[1]] = "异常"
        logger.error("{}服务检测异常".format("算法" if i_port[1]=="arithmetic" else i_port[1]))
        # 将告警信息插入alarm_manage数据表
        res=DBAlarmManage()
        result=res.get_alarm_data(0,0)
        if result:
            if result['value']==i_port[1]:
                if result['status'] == 0:
                   pass
                else:
                    num=result['num']
                    server_update={
                        'inspect_project_detail_id':0,
                        'robot_position_id':0,
                        'num': 2,
                        'value': i_port[1],
                        'undo_status':0,
                        'alarm_desc': i_port[1] + '检测异常',
                        'status':0,
                        'assign':0,
                        'user_id':0,
                        'update_time':datetime.now(),
                    }
                    logger.info(i_port[1] + "检测异常, 更新告警信息")
                    res.update({'num': num}, server_update)
                    logger.info("告警信息更新成功")
            else:
                # 服务添加数据
                server_insert={
                    'robot_id':1,
                    'core_room_id':0,
                    'inspect_project_detail_id':0,
                    'inspect_project_task_id':0,
                    'robot_position_id':0,
                    'robot_item_id':0,
                    'core_cabinet_id':0,
                    'robot_device_id':'0',
                    'alarm_type':10,
                    'level':5,
                    'num':1,
                    'value':i_port[1],
                    'alarm_desc': i_port[1] + '检测异常',
                    'status':0,
                    'assign':0,
                    'create_time':datetime.now(),
                    'user_id':0,
                    'update_time':datetime.now(),
                }
                logger.info(i_port[1] + "检测异常, 添加告警信息")
                res.insert_data(server_insert)
                logger.info("告警信息添加成功")
        else:
            # 服务异常数据
            server_insert={
                'robot_id':1,
                'core_room_id':0,
                'inspect_project_detail_id':0,
                'inspect_project_task_id':0,
                'robot_position_id':0,
                'robot_item_id':0,
                'core_cabinet_id':0,
                'robot_device_id':'0',
                'alarm_type':10,
                'level':5,
                'num':1,
                'value':i_port[1],
                'alarm_desc':i_port[1] + '检测异常',
                'status':0,
                'assign':0,
                'create_time':datetime.now(),
                'user_id':0,
                'update_time':datetime.now(),
            }
            logger.info(i_port[1] + "检测异常, 添加告警信息")
            res.insert_data(server_insert)
            logger.info("告警信息添加成功")
        pass

# 自检模块
def hart_inspect_test(data_value):
    # 将告警信息插入alarm_manage数据表
    res = DBAlarmManage()
    result = res.get_alarm_data(0,0)
    if result:
        if "value" == data_value:
            if result['status'] == 0:
                pass
            else:
                num = result['num']
                dic_update = {}
                dic_update['inspect_project_detail_id'] = 0
                dic_update['robot_position_id'] = 0
                dic_update['num'] = 2
                dic_update['undo_status'] = 0
                dic_update['value'] = data_value
                dic_update['alarm_desc'] = data_value + "检测异常"
                dic_update["status"] = 0
                dic_update["assign"] = 0
                dic_update["user_id"] = 0
                dic_update["update_time"] = datetime.now()
                logger.info(data_value + "检测异常, 更新告警信息")
                res.update({'num': num}, dic_update)
                logger.info("告警信息更新成功")
        else:
            dic_insert={}
            dic_insert["robot_id"] = 1
            dic_insert["core_room_id"] = 0
            dic_insert["inspect_project_detail_id"] = 0
            dic_insert["inspect_project_task_id"] = 0
            dic_insert["robot_position_id"] = 0
            dic_insert["robot_item_id"] = 0
            dic_insert["core_cabinet_id"] = 0
            dic_insert["robot_device_id"] = 0
            dic_insert["value"] = data_value
            dic_insert['alarm_desc'] = data_value+"检测异常"
            dic_insert["alarm_type"] = 10
            dic_insert["level"] = 5
            dic_insert["num"] = 1
            dic_insert["status"] = 0
            dic_insert["assign"] = 0
            dic_insert["create_time"] = datetime.now(),
            dic_insert["user_id"] = 0
            dic_insert["update_time"] = datetime.now(),
            logger.info(data_value+"检测异常, 添加告警信息")
            res.insert_data(dic_insert)
            logger.info("告警信息添加成功")
    else:
        dic_insert={}
        dic_insert["robot_id"]=1
        dic_insert["core_room_id"]=0
        dic_insert["inspect_project_detail_id"]=0
        dic_insert["inspect_project_task_id"]=0
        dic_insert["robot_position_id"]=0
        dic_insert["robot_item_id"]=0
        dic_insert["core_cabinet_id"]=0
        dic_insert["robot_device_id"]=0
        dic_insert["value"]=data_value
        dic_insert['alarm_desc']=data_value+"检测异常"
        dic_insert["alarm_type"]=10
        dic_insert["level"]=5
        dic_insert["num"]=1
        dic_insert["status"]=0
        dic_insert["assign"]=0
        dic_insert["create_time"]=datetime.now(),
        dic_insert["user_id"]=0
        dic_insert["update_time"]=datetime.now(),
        logger.info(data_value+"检测异常, 添加告警信息")
        res.insert_data(dic_insert)
        logger.info("告警信息添加成功")

# 调度模块异常处理
def schedule_test(data_value):
    # 将告警信息插入alarm_manage数据表
    res=DBAlarmManage()
    result=res.get_alarm_data(0,0)
    if result:
        if "value"==data_value:
            if result['status']==0:
                pass
            else:
                num=result['num']
                dic_update={}
                dic_update['inspect_project_detail_id']=0
                dic_update['robot_position_id']=0
                dic_update['num']=2
                dic_update['undo_status']=0
                dic_update['value']=data_value
                dic_update['alarm_desc']=data_value+"检测异常"
                dic_update["status"]=0
                dic_update["assign"]=0
                dic_update["user_id"]=0
                dic_update["update_time"]=datetime.now()
                logger.info(data_value+"检测异常, 更新告警信息")
                res.update({'num':num},dic_update)
                logger.info("告警信息更新成功")
                # 将异常信息推送到mq
                result=res.get_alarm_data(0,0)
                alarm_manage_id=result["alarm_manage_id"]
                alarm_desc=result["alarm_desc"]
                dic_data={'alarm_manage_id':alarm_manage_id,'alarm_desc':alarm_desc}
                rout=db_rabbit_mq.get("robot_warn")
                routing_ey=db_rabbit_mq.get("warn_rout_key")
                warn_queue=db_rabbit_mq.get("warn_queue")
                RabbitPublisher.run(rout,routing_ey,warn_queue,dic_data)
                logger.info(data_value + "异常信息推送成功")
        else:
            dic_insert={}
            dic_insert["robot_id"]=1
            dic_insert["core_room_id"]=0
            dic_insert["inspect_project_detail_id"]=0
            dic_insert["inspect_project_task_id"]=0
            dic_insert["robot_position_id"]=0
            dic_insert["robot_item_id"]=0
            dic_insert["core_cabinet_id"]=0
            dic_insert["robot_device_id"]=0
            dic_insert["value"]=data_value
            dic_insert['alarm_desc']=data_value+"检测异常"
            dic_insert["alarm_type"]=10
            dic_insert["level"]=5
            dic_insert["num"]=1
            dic_insert["status"]=0
            dic_insert["assign"]=0
            dic_insert["create_time"]=datetime.now(),
            dic_insert["user_id"]=0
            dic_insert["update_time"]=datetime.now(),
            logger.info(data_value+"检测异常, 添加告警信息")
            res.insert_data(dic_insert)
            logger.info("告警信息添加成功")
            # 将异常信息推送到mq
            result=res.get_alarm_data(0,0)
            alarm_manage_id=result["alarm_manage_id"]
            alarm_desc=result["alarm_desc"]
            dic_data={'alarm_manage_id':alarm_manage_id,'alarm_desc':alarm_desc}
            rout=db_rabbit_mq.get("robot_warn")
            routing_ey=db_rabbit_mq.get("warn_rout_key")
            warn_queue=db_rabbit_mq.get("warn_queue")
            RabbitPublisher.run(rout,routing_ey,warn_queue,dic_data)
            logger.info(data_value + "异常信息推送成功")
    else:
        dic_insert={}
        dic_insert["robot_id"]=1
        dic_insert["core_room_id"]=0
        dic_insert["inspect_project_detail_id"]=0
        dic_insert["inspect_project_task_id"]=0
        dic_insert["robot_position_id"]=0
        dic_insert["robot_item_id"]=0
        dic_insert["core_cabinet_id"]=0
        dic_insert["robot_device_id"]=0
        dic_insert["value"]=data_value
        dic_insert['alarm_desc']=data_value+"检测异常"
        dic_insert["alarm_type"]=10
        dic_insert["level"]=5
        dic_insert["num"]=1
        dic_insert["status"]=0
        dic_insert["assign"]=0
        dic_insert["create_time"]=datetime.now(),
        dic_insert["user_id"]=0
        dic_insert["update_time"]=datetime.now(),
        logger.info(data_value+"检测异常, 添加告警信息")
        res.insert_data(dic_insert)
        logger.info("告警信息添加成功")
        # 将异常信息推送到mq
        result = res.get_alarm_data(0,0)
        alarm_manage_id=result["alarm_manage_id"]
        alarm_desc=result["alarm_desc"]
        dic_data={'alarm_manage_id':alarm_manage_id,'alarm_desc':alarm_desc}
        rout = db_rabbit_mq.get("robot_warn")
        routing_ey = db_rabbit_mq.get("warn_rout_key")
        warn_queue=db_rabbit_mq.get("warn_queue")
        RabbitPublisher.run(rout,routing_ey,warn_queue,dic_data)
        logger.info(data_value + "异常信息推送成功")

