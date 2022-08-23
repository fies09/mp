import sys
import time
import json
import traceback
import logging
import datetime
import base64
import uuid
import requests
import os
sys.path.append("../")
from flask import Flask, request, Response
from flask_cors import CORS
from threading import Thread
from configs import fourfold_ip, file_path_base, agx_service, arithmetic_service, robotId
from configs.log import logger
from task.light_al import post_agx_light
from requests.auth import HTTPDigestAuth
from task.blend import lightBlend
from modules.client import Main
from modules.voice_state import voice_start, voice_stop
from schema.db_robot_path_position_relation import DBRobotPathPositionRelation
from schema.robot_core_cabinet import DBCoreCabinet
from schema.robot_core_device import DBCoreDevice
from schema.robot_core_device_location import DBCoreDeviceLocation
from schema.robot_core_indcamera_param import DBCoreIndcameraParam
from schema.robot_device import DBRobotDevice
from utils.file_path import Path
from task.robot_inspect2 import inspect_main
import os.path
from queue import Queue
from modules.robot_lifter import RobotLifer
sys.path.remove('/opt/moss_robot/lib/dispatch_ips_web')
from modules import hk_api_model
import rospy
from std_msgs.msg import Bool, Int8, UInt8, UInt16
from sensor_msgs.msg import BatteryState
from core.redis_interactive import RedisPipe
from utils.msg import Msg
from modules.camera_compatibility import Hikvision, Daheng, FindDevice
from modules.robot_move import RobotMove
from modules.robot_take_picture import ThermalImagery, FourCamera
from modules.robot_transducer import RobotTransducer
from modules.robot_voice import RobotVoice
from modules.robot_hardware_server import robot_lifter
from modules.robot_hardware_server import robot_ptz
from task.robot_algorithm import QrcodeIdentify
from modules.robot_hardware_server import RobotDeviceStatus
from configs.msg import robot_code
from configs import industrial_camera_position
from server.base_api import ApiBase
from task.hardware_exception import schedule_test

app = Flask(__name__)
CORS(app, resources=r'/*')
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

q = Queue()
queue_flag = True


class API:
    def __init__(self):

        routes = [
            # ========================== 硬件调用接口 ==========================
            # 移动
            {'r': '/robot_move', 'm': ['POST'], 'f': self.robot_move},
            # 热成像
            {'r': '/robot_thermal_imagery', 'm': ['GET'], 'f': self.robot_thermal_imagery},
            # 四倍、云台相机拍照
            {'r': '/four_camera_photo', 'm': ['GET'], 'f': self.four_camera_photo},
            # 传感器
            {'r': '/robot_transducer', 'm': ['GET'], 'f': self.robot_transducer},
            # 语音
            {'r': '/robot_voice', 'm': ['POST'], 'f': self.robot_voice},
            # 工业相机
            {'r': '/get_devicePrama', 'm': ['GET', 'POST'], 'f': self.get_industry_parameters},
            {'r': '/get_pic', 'm': ['GET', 'POST'], 'f': self.get_industry_photo},
            {'r': '/change_devicePrama', 'm': ['GET', 'POST'], 'f': self.set_industry_parameter},
            # 升降杆
            {'r': '/elevator', 'm': ['GET'], 'f': self.robot_lifter},
            # 海康四倍相机
            {'r': '/set_hk_camera', 'm': ['GET', 'POST'], 'f': self.set_hk_camera},
            {'r': '/get_hk_camera', 'm': ['GET', 'POST'], 'f': self.get_hk_camera},
            # 云台
            {'r': '/ptz', 'm': ['GET', 'POST'], 'f': self.ptz},
            # ========================== 硬件调用接口 ==========================

            # 原图接口
            {'r': '/get_before_data', 'm': ['GET', 'POST'], 'f': self.file_befor_data},
            # 机器人状态信息接口
            {'r': '/api/robot/robotState', 'm': ['POST'], 'f': self.robot_robot_state},
            {'r': '/get_data', 'm': ['GET', 'POST'], 'f': self.file_data},
            # 算法预览
            {'r': '/Industry_preview', 'm': ['GET'], 'f': self.industry_al_preview},
            # 调度接口
            {'r': '/dispatch/industry/photo', 'm': ['GET'], 'f': self.industry_industry_photo},

            {'r': '/visible_light_preview', 'm': ['GET', 'POST'], 'f': self.fourfold_al_preview},
            {'r': '/action_voice', 'm': ['GET'], 'f': self.robot_action_voice},
            {'r': '/az_status_light', 'm': ['GET'], 'f': self.status_listen},
            # 模板管理OCR算法预览接口
            {'r': '/algorithm_preview', 'm': ['POST'], 'f': self.algorithm_preview},
            # 机器人二维码盘点算法预览
            {'r': '/qr_code_preview', 'm': ['POST'], 'f': self.qr_code_preview},
            # 机器人自检接口
            {'r':'/robot_inspect','m':['GET','POST'],'f': self.robot_inspect},
            # 模板管理抓图接口
            {'r': '/get_template_pic', 'm': ['GET'], 'f': self.get_template_pic},
            # ========================= 海康四倍相机设置接口 =========================
            # redis
            {'r': '/visit_command', 'm': ['GET', 'POST'], 'f': self.visit_command},
            # 语音播报
            {'r': '/robot/voice/state', 'm': ['GET', "POST"], 'f': self.voice_state},
            {'r': '/robot/voice/action', 'm': ['GET', 'POST'], 'f': self.voice_action},
        ]
        for route in routes:
            self.addroute(route)
        try:
            # 开始查找本机工业相机设备
            industrial_camera_type = FindDevice().start()
            # 工业相机类型判断
            if industrial_camera_type.get("Hikvision", None):
                # 实例化海康相机
                logger.info("初始化海康相机")
                RedisPipe("industrial_camera_type").set_data("Hikvision")
                self.dlcw_l = Hikvision()
            elif industrial_camera_type.get("Daheng", None):
                # 实例化大恒相机
                logger.info("初始化大恒相机")
                RedisPipe("industrial_camera_type").set_data("Daheng")
                self.dlcw_l = Daheng()
            else:
                logger.error("未找到工业相机，请检查工业相机")
                # sys.exit()
        except Exception as e:
            logger.error(e)
            logger.error("工业相机未启动成功，请检查工业相机")
        self.modify_num = 0
        self.queue_flag = True
        self.image_upper_state = 0
        self.image_lower_state = 0

    @staticmethod
    def addroute(route):
        app.add_url_rule(route['r'], view_func=route['f'], methods=route['m'])

    def robot_move(self):
        """
        机器人移动服务
        """
        try:
            data = request.get_json()
            position, move_type = data.get("position"), data.get("move_type")
            result = RobotMove(position, move_type).start()
            if result:
                return Response(json.dumps({"code": 200, "message": "机器人移动成功"}), 200, mimetype='application/json')
            logger.error("硬件调用错误")
            # 开始向告警信息表插入数据
            data_value = "移动服务"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "移动调用错误"}), 510, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("服务程序错误")
            # 开始向告警信息表插入数据
            data_value="移动服务"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "服务程序错误"}), 500, mimetype='application/json')

    def robot_thermal_imagery(self):
        try:
            ip = request.args.get('ip')
            photo_path = request.args.get('photo_path')
            thermal_imagery = ThermalImagery()
            file_path = thermal_imagery.take_picture()
            result = thermal_imagery.get_results()
            if file_path:
                return Response(
                    json.dumps({"code": 200, "message": "热成像拍照成功", "file_path": file_path, "result": result}), 200,
                    mimetype='application/json')
            # 开始向告警信息表插入数据
            data_value="热成像"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "热成像拍照失败", "file_path": "", "result": {}}), 510,
                            mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("服务程序错误")
            # 开始向告警信息表插入数据
            data_value="热成像"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "服务程序错误", "file_path": "", "result": {}}), 500,
                            mimetype='application/json')

    def four_camera_photo(self):
        try:
            ip = request.args.get('ip')
            file_path = FourCamera(ip).start()
            if file_path:
                return Response(json.dumps({"code": 200, "message": "四倍相机拍照成功", "file_path": file_path}), 200,
                                mimetype='application/json')
             # 开始向告警信息表插入数据
            data_value="四倍相机"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "四倍相机拍照失败", "file_path": file_path}), 510,
                            mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("服务程序错误")
             # 开始向告警信息表插入数据
            data_value="四倍相机"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "服务程序错误"}), 500, mimetype='application/json')

    def robot_transducer(self):
        try:
            item_id = request.args.get('item_id')
            result = RobotTransducer(item_id).start()
            if result:
                return Response(json.dumps({"code": 200, "message": "获取成功", "res": result}), 200,
                                mimetype='application/json')
            data_value="传感器"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "获取失败", "res": 0}), 510, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("服务程序错误")
            data_value="传感器"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "服务程序错误"}), 500, mimetype='application/json')

    def robot_voice(self):
        try:
            data = request.get_json()
            text = data.get("text", "")
            result = RobotVoice(text).start()
            if result:
                return Response(json.dumps({"code": 200, "message": "播报成功"}), 200, mimetype='application/json')
            data_value="语音服务"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "播报失败"}), 510, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("服务程序错误")
            data_value="语音服务"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "服务程序错误"}), 500, mimetype='application/json')

    # 获取机器人喇叭状态信息
    def voice_state(self):
        msg = Msg()
        try:
            data_dict = {'robotId': robotId, 'voiceState': 0}
            return msg.success(msg='机器人喇叭状态获取成功', result=data_dict).json()
        except Exception as e:
            logger.error(e)
            return msg.fail(msg='机器人喇叭状态获取失败').json()

    # 设置机器人喇叭状态
    def voice_action(self):
        msg = Msg()
        try:
            res = request.get_json()
            logger.info(res)
            robot_id = res.get("robotId")
            voice_action = res.get("voiceAction")
            if voice_action == 1:
                voice_start()
                RedisPipe("VoiceAction").set_data({"VoiceAction": 1}, 1800)
                data_dict = {'robotId': robotId, 'voiceState': 1}
            elif voice_action == 0:
                voice_stop()
                RedisPipe("VoiceAction").set_data({"VoiceAction": 0})
                data_dict = {'robotId': robotId, 'voiceState': 0}
            else:
                return msg.fail(msg='机器人状态喇叭状态失败,输入状态值错误').json()

            return msg.success(msg='设置机器人喇叭状态成功', result=data_dict).json()
        except Exception as e:
            logger.error(e)
            return msg.fail(msg='机器人状态喇叭状态失败').json()

    # 获取文件视频文件
    def file_befor_data(self):
        '''
            type表示传输数据的类型；
            name表示传输数据的路径。
        '''

        type = str(request.args.get("type"))
        file_path = request.args.get("name")

        if type != "5":
            if not os.path.isfile(file_path):
                return json.dumps({"statusCode": 500, "message": "该文件不存在"})
        else:
            if not os.path.isfile(file_path):
                return json.dumps({"statusCode": 500, "message": "该文件不存在"})
        # 如果获取图片，则先进行缩略
        if type in ["1", "4", "5"]:
            with open(file_path, "rb") as f:
                image = f.read()
            resp = Response(image, mimetype="image/jpeg")
            return resp
        if type == "2":
            with open(file_path, "rb") as f:
                video = f.read()
                resp = Response(video, mimetype="video/mpeg4")
        elif type == "3":
            with open(file_path, "rb") as f:
                audio = f.read()
                resp = Response(audio, mimetype="audio/mpeg")
        else:
            return json.dumps({"statusCode": 500, "message": "参数type错误"})

        return resp

    # 更新机器人状态信息
    @staticmethod
    def robot_robot_state():
        msg = Msg()
        res = request.get_json()
        try:
            robot_id = res.get("robotId")
            robot_path_id = res.get("robotPathId")
            body = RedisPipe("Robot_state").get_data()
            if not body:
                return msg.fail(msg='机器人状态信息获取失败').json()
            body = str(body, 'utf-8')
            data_dict = eval(body)

            robot_position_id = data_dict.get("robot_position_id")

            if robot_path_id and robot_position_id:
                if DBRobotPathPositionRelation().get_alive_data(robot_path_id, robot_position_id):
                    data_dict["is_now_path"] = 0
                else:
                    data_dict["is_now_path"] = 1

            return msg.success(msg='机器人状态信息获取成功', result={"result": data_dict}).json()

        except Exception as e:
            logger.error(e)
            return msg.fail(msg='机器人状态信息获取失败').json()

    # 获取文件视频文件
    def file_data(self):
        '''
            type表示传输数据的类型；
            name表示传输数据的路径。
        '''

        type = str(request.args.get("type"))
        file_path = request.args.get("name")

        if type != "5":
            if not os.path.isfile(file_path):
                return json.dumps({"statusCode": 500, "message": "该文件不存在"})
        else:
            if not os.path.isfile(file_path):
                return json.dumps({"statusCode": 500, "message": "该文件不存在"})
        # 如果获取图片，则先进行缩略
        if type in ["1", "4", "5"]:
            with open(file_path, "rb") as f:
                image = f.read()
            resp = Response(image, mimetype="image/jpeg")
            return resp
        if type == "2":
            with open(file_path, "rb") as f:
                video = f.read()
                resp = Response(video, mimetype="video/mpeg4")
        elif type == "3":
            with open(file_path, "rb") as f:
                audio = f.read()
                resp = Response(audio, mimetype="audio/mpeg")
        else:
            return json.dumps({"statusCode": 500, "message": "参数type错误"})

        return resp

    def get_industry_parameters(self):
        if request.method == "GET":
            position_id = request.args.get("id")
        else:
            position_id = request.form.get("id")
        robot_device_status = RobotDeviceStatus()
        if int(position_id) == industrial_camera_position.get("down"):
            device_status = robot_device_status.get_device_status("industry_camera_down")
            if not device_status:
                data_value = "下工业相机"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "下工业相机不可用", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
            if device_use_status:
                data_value="下工业相机"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "下工业相机被占用", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        else:
            device_status = robot_device_status.get_device_status("industry_camera_up")
            if not device_status:
                data_value="上工业相机"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "上工业相机不可用", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
            if device_use_status:
                data_value="上工业相机"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "上工业相机被占用", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        if not position_id:
            return Response(json.dumps({"code": 505, "message": "参数id错误"}), 505, mimetype='application/json')
        photo_params = self.dlcw_l.get_camera_parameter(position_id)
        if not photo_params:
            return Response(json.dumps({"code": 500, "message": "参数错误"}), 500, mimetype='application/json')
        return Response(json.dumps({"code": 200, "data": photo_params, "message": "sucess"}), 200, mimetype='application/json')

    # 获取工业相机照片
    def get_industry_photo(self):
        try:
            if request.method == 'GET':
                position_id = request.args.get("id")
            else:
                position_id = request.form.get('id')
            robot_device_status = RobotDeviceStatus()
            if int(position_id) == industrial_camera_position.get("down"):
                device_status = robot_device_status.get_device_status("industry_camera_down")
                if not device_status:
                    # 开始向告警信息表插入数据
                    data_value="下工业相机"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "下工业相机不可用", "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
                device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
                if device_use_status:
                    response_data = {"code": 500, "message": "下工业相机被占用", "robot_code": robot_code.get("used")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            else:
                device_status = robot_device_status.get_device_status("industry_camera_up")
                if not device_status:
                    # 开始向告警信息表插入数据
                    data_value="上工业相机"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "上工业相机不可用", "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
                device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
                if device_use_status:
                    response_data = {"code": 500, "message": "上工业相机被占用", "robot_code": robot_code.get("used")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            file_path = Path().industry + datetime.datetime.now().strftime("%Y_%m_%d")
            if not os.path.exists(file_path):
                os.makedirs(file_path)
            photo_path = file_path + "/" + datetime.datetime.now().strftime("%H_%M_%S.%f") + ".jpg"
            if self.dlcw_l.industry_photo(photo_path, position_id):
                with open(photo_path, 'rb') as f:
                    image = f.read()
                resp = Response(image, mimetype="image/jpeg")
                return resp
            else:
                logger.error("工业相机拍照异常")
                 # 开始向告警信息表插入数据
                data_value="工业相机"
                schedule_test(data_value)
                rustle_dict = {"statusCode": 500, "message": "抓图失败"}
                return json.dumps(rustle_dict)
        except Exception as e:
            logger.error(e)
            logger.error("工业相机抓图失败")
            # 开始向告警信息表插入数据
            data_value="工业相机"
            schedule_test(data_value)

    def set_industry_parameter(self):
        if request.method == 'GET':
            parma = request.args.get('prama')
            parma_value = request.args.get('prama_value')
            device_param = request.args.get('device_param',{})
            Id = request.args.get("id")
        else:
            data = request.get_json()
            parma = data.get('prama')
            parma_value = data.get('prama_value')
            device_param = data.get('device_param', {})
            Id = data.get('id')
        robot_device_status = RobotDeviceStatus()
        if int(Id) == industrial_camera_position.get("down"):
            device_status = robot_device_status.get_device_status("industry_camera_down")
            if not device_status:
                response_data = {"code": 500, "message": "下工业相机不可用", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
            if device_use_status:
                response_data = {"code": 500, "message": "下工业相机被占用", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        else:
            device_status = robot_device_status.get_device_status("industry_camera_up")
            if not device_status:
                response_data = {"code": 500, "message": "上工业相机不可用", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
            if device_use_status:
                response_data = {"code": 500, "message": "上工业相机被占用", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        if device_param:
            camera_params = dict()
            camera_params["ExposureTime"] = device_param.get("ExposureTime")
            camera_params["BalanceRatio"] = device_param.get("BalanceRatio")
            camera_params["Gain"] = device_param.get("Gain")
            if not self.dlcw_l.modify_industry_parameter(Id, None, None, camera_params):
                rustle = {"code": 500, 'message': '设置工业相机参数失败'}
                return json.dumps(rustle)
            else:
                return {"code": 200, 'message': '设置工业相机参数成功'}
        else:
            if self.dlcw_l.modify_industry_parameter(Id, parma, parma_value, None):
                rustle = {"code": 200, 'message': '操作成功'}
                return json.dumps(rustle)
            rustle = {"code": 500, 'message': '操作失败'}
            return json.dumps(rustle)

    def industry_al_preview(self):
        if request.method == "GET":
            core_id = request.args.get("id")
        else:
            core_id = request.form.get('id')
        logger.info(core_id)
        camera_data = DBCoreIndcameraParam().get_industry_id(core_id)
        logger.info(camera_data)
        if not camera_data:
            logger.info("该设备id{}无数据".format(core_id))
            return json.dumps({"statusCode": 500, "message": "not find id data"})
        # 升降杆告诉
        elevator_height = camera_data.get("elevator_height")
        status, position = RobotLifer(1, elevator_height).start()
        if not status:
            return json.dumps({"statusCode": 500, "message": "控制升降杆异常"})
        logger.info("调用升降杆成功！")
        # 获取摄像头id
        robot_device_id = camera_data.get("robot_device_id")
        device_data = DBRobotDevice().get_device_id(robot_device_id)
        if not device_data:
            return json.dumps({"statusCode": 500, "message": "无此摄像头"})
        Id = device_data.get("address")
        logger.info('使用id为{}摄像头'.format(Id))

        # 摄像头参数
        device_param = camera_data.get("device_param")
        # device_param = {
        #     "ExposureTime": 40000,
        #     "BalanceRatio": 4000,
        #     "Gain": 0,
        #     "PhotoIntervalTime": 0,
        #     "PhotoNum": 2
        # }
        # 设置工业相机参数
        camera_params = dict()
        camera_params["ExposureTime"] = device_param.get("ExposureTime")
        camera_params["BalanceRatio"] = device_param.get("BalanceRatio")
        camera_params["Gain"] = device_param.get("Gain")
        if not self.dlcw_l.modify_industry_parameter(Id, None, None, camera_params):
            rustle = {"statusCode": 500, 'msg': '设置工业相机参数失败'}
            return json.dumps(rustle)
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
        algorithm_param = camera_data.get("algorithm_param")
        u_params = algorithm_param.get("u_params")
        start_u = u_params.get("start_u")
        end_u = u_params.get("end_u")

        show_data = "P-{} U-{} D-{}-{}".format(cabinet_name, device_name, start_u, end_u)
        # 开始拍照
        web_photo_path = Path().web_photo
        file_path_dir = web_photo_path + datetime.datetime.now().strftime("%H_%M_%S.%f")
        rois = list()
        algorithm_data = DBCoreIndcameraParam().get_industry_id(int(core_id))
        algorithm_param_for = algorithm_data.get("algorithm_param")
        u_params_for = algorithm_param_for.get("u_params")
        u_params_for["id"] = core_id
        rois.append(u_params_for)
        photo_num = device_param.get("PhotoNum", 0)
        photo_interval_time = device_param.get("PhotoIntervalTime", 0)
        logger.info(photo_num)
        if int(photo_num) == 1:
            file_path = file_path_dir + '.jpg'
            if not self.dlcw_l.industry_photo(file_path, Id):
                logger.info("工业相机拍照异常")
                return json.dumps({"statusCode": 500, "message": "工业相机拍照异常"})
            # P机柜 U设备名称 D起始U-结束U
            logger.info('开始识别')
            light_detect_result, light_path = post_agx_light(file_path, rois, show_data, 3)
            logger.info('识别完成')
            logger.info(light_detect_result)
            logger.info(light_path)
            # 更新数据库img路径
            if not light_path:
                logger.info("算法解析出错")
                return json.dumps({"statusCode": 500, "message": "算法解析出错"})
            DBCoreIndcameraParam().update({"core_indcamera_param_id": core_id},
                                          {"img_path": str(light_path),
                                           "img_pah_thumbnail": light_path})
            rustle_dict = {"code": 200, "message": "sucess"}
            return json.dumps(rustle_dict)
        else:
            file_path = file_path_dir + '.jpg'
            logger.info('开始合成照片')
            for num in range(photo_num):
                file_name = file_path_dir + "_{}.jpg".format(num)
                if not self.dlcw_l.industry_photo(file_name, Id):
                    logger.info("工业相机拍照异常")
                    return json.dumps({"statusCode": 500, "message": "工业相机拍照异常"})
                time.sleep(photo_interval_time)
            logger.info('开始识别')
            blend_state, blend_photo_path = lightBlend(file_path, photo_num)
            if blend_state:
                logger.info("照片合成成功")
                light_detect_result, light_path = post_agx_light(blend_photo_path, rois, show_data, 3)
                logger.info('识别完成')
            else:
                return json.dumps({"statusCode": 500, "message": "照片合成错误"})
            # 更新数据库img路径
            if not light_path:
                logger.info("算法解析出错")
                return json.dumps({"statusCode": 500, "message": "算法解析出错"})
            photo_path = light_path.split(file_path_base.get("file_path_base"))[-1]

            DBCoreIndcameraParam().update({"core_indcamera_param_id": core_id},
                                          {"img_path": str(photo_path),
                                           "img_pah_thumbnail": photo_path})
            rustle_dict = {"code": 200, "message": "sucess"}
            return json.dumps(rustle_dict)

    def industry_industry_photo(self):
        if request.method == "GET":
            core_id = request.args.get("id")
        else:
            core_id = request.form.get('id')
        logger.info(core_id)
        core_id_list = core_id.split(",")
        logger.info(core_id_list)
        # 获取索引0处的工业相机参数
        core_id = core_id_list[0]
        os_path = Path().industry + datetime.datetime.now().strftime("%Y_%m_%d")
        #

        photo_name = os_path + "/" + datetime.datetime.now().strftime(
            "%H_%M_%S.%f") + ".jpg"
        if not os.path.exists(os_path):
            os.mkdir(os_path)
        try:
            logger.info(int(core_id))
            camera_data = DBCoreIndcameraParam().get_industry_id(int(core_id))
            logger.info(camera_data)
        except Exception as e:
            camera_data = {}
            logger.error(e)
        if not camera_data:
            logger.info("该设备id{}无数据".format(core_id))
            return json.dumps({"code": 500, "message": "not find id data"})

        # 获取摄像头id
        robot_device_id = camera_data.get("robot_device_id")
        device_data = DBRobotDevice().get_device_id(robot_device_id)
        if not device_data:
            return json.dumps({"code": 500, "message": "无此摄像头"})
        Id = device_data.get("address")
        logger.info('使用id为{}摄像头'.format(Id))

        # 摄像头参数
        device_param = camera_data.get("device_param")
        # 设置工业相机参数
        camera_params = dict()
        camera_params["ExposureTime"] = device_param.get("ExposureTime")
        camera_params["BalanceRatio"] = device_param.get("BalanceRatio")
        camera_params["Gain"] = device_param.get("Gain")
        if not self.dlcw_l.modify_industry_parameter(Id, None, None, camera_params):
            rustle = {"statusCode": 500, 'msg': '设置工业相机参数失败'}
            return json.dumps(rustle)
        # 遍历算法ID获取对应的参数
        rois = list()
        for param_id in core_id_list:
            show_data = get_show_data(param_id)
            logger.info(show_data)
            logger.info(param_id)
            algorithm_data = DBCoreIndcameraParam().get_industry_id(int(param_id))
            logger.info(algorithm_data)
            algorithm_param_for = algorithm_data.get("algorithm_param", {})
            u_params_for = algorithm_param_for.get("u_params", None)
            u_params_for["id"] = param_id
            u_params_for["show_data"] = show_data
            logger.info(u_params_for)
            rois.append(u_params_for)
        photo_num = device_param.get("PhotoNum")
        photo_interval_time = device_param.get("PhotoIntervalTime")
        try:
            if int(photo_num) == 1:
                if not self.dlcw_l.industry_photo(photo_name, Id):
                    logger.info("工业相机拍照异常")
                    data_value="工业相机"
                    schedule_test(data_value)
                    return json.dumps({"statusCode": 500, "message": "工业相机拍照异常"})
                # P机柜 U设备名称 D起始U-结束U
                logger.info('开始识别')
                rustle_dict = {"code": 200, "message": "sucess",
                               "rois": rois,
                               "light_path": photo_name
                               }
            else:
                file_name_base = photo_name.split(".jpg")[0]
                logger.info('开始合成照片')
                for num in range(photo_num):
                    file_name = file_name_base + "_{}.jpg".format(num)
                    if not self.dlcw_l.industry_photo(file_name, Id):
                        logger.info("工业相机拍照异常")
                        data_value = "工业相机"
                        schedule_test(data_value)
                        return json.dumps({"statusCode": 500, "message": "工业相机拍照异常"})
                    time.sleep(photo_interval_time)
                blend_state, photo_name = lightBlend(photo_name, photo_num)
                rustle_dict = {"code": 200, "message": "sucess",
                               "rois": rois,
                               "light_path": photo_name
                               }
        except Exception as e:
            logger.error(e)
            data_value="工业相机"
            schedule_test(data_value)
            rustle_dict = {"code": 500, "message": "fail",
                           "rois": [],
                           "show_data": "",
                           "light_path": ""}
        logger.info(rustle_dict)
        return json.dumps(rustle_dict)

    def fourfold_al_preview(self):
        if request.method == 'GET':
            port = request.args.get("port")
            nums = int(request.args.get('nums'))
            time_data = float(request.args.get('time'))
        else:
            port = request.form.get('port')
            nums = int(request.form.get('nums'))
            time_data = float(request.form.get('time'))

        if port == '2007':
            fourfold_preview = fourfold_ip.get("lower_fourfold")
            ip = fourfold_preview.get("host")
            port = fourfold_preview.get("port")
            username = fourfold_preview.get("username")
            password = fourfold_preview.get("password")
        elif port == '2009':
            fourfold_preview = fourfold_ip.get("upper_fourfold")
            ip = fourfold_preview.get("host")
            port = fourfold_preview.get("port")
            username = fourfold_preview.get("username")
            password = fourfold_preview.get("password")
        else:
            logger.info("传入端口错误 {}".format(port))
            rustle_dict = {"statusCode": 500, "message": "传入端口错误"}
            return json.dumps(rustle_dict)

        try:
            web_photo_path = Path().web_photo
            name = web_photo_path + datetime.datetime.now().strftime("%H_%M_%S")
            if nums == 1:
                name = name + '.jpg'
                if not get_fourfold_photo(ip, username, password, name, port):
                    logger.info("四倍相机拍照异常")
                    data_value="四倍相机"
                    schedule_test(data_value)
                    return json.dumps({"statusCode": 500, "message": "四倍相机拍照异常"})

                light_detect_result, light_path = post_agx_light(name, 1, 5000, 1, 5000, "P-00 U-00 D-00")
                with open(light_path, 'rb') as f:
                    image = f.read()
                resp = Response(image, mimetype="image/jpeg")
                return resp
            else:
                for num in range(nums):
                    file_name = name + "_{}.jpg".format(num)
                    if not get_fourfold_photo(ip, username, password, file_name, port):
                        logger.info("四倍相机拍照异常")
                        data_value="四倍相机"
                        schedule_test(data_value)
                        return json.dumps({"statusCode": 500, "message": "四倍相机拍照异常"})
                    time.sleep(time_data)

                blend_state, photo_name = lightBlend(name, nums)
                light_detect_result, light_path = post_agx_light(photo_name, 1, 5000, 1, 5000, "P-00 U-00 D-00")
                with open(light_path, 'rb') as f:
                    image = f.read()
                resp = Response(image, mimetype="image/jpeg")
                return resp

        except Exception as e:
            logger.error(e)
            data_value="四倍相机"
            schedule_test(data_value)
            rustle_dict = {"statusCode": 500, "message": "fail"}
            return json.dumps(rustle_dict)

    def robot_lifter(self):
        # type 2为相对高度
        try:
            robot_device_status = RobotDeviceStatus()
            # 判断升降杆是否可用
            device_status = robot_device_status.get_device_status("lifter")
            if not device_status:
                # 开始向告警信息表插入数据
                data_value="升降杆"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "升降杆不可用", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            # 判断升降杆是否被占用
            device_use_status = robot_device_status.get_device_use_status("lifter")
            if device_use_status:
                response_data = {"code": 500, "message": "升降杆被占用", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            move_type = int(request.args.get("type", 0))
            position = int(request.args.get("position"))
            status, position = robot_lifter(move_type, position)
            if status:
                response_data = {"code": 200, "message": "升降杆调用成功", "position": position, "robot_code": robot_code.get("normal")}
                return Response(json.dumps(response_data), 200, mimetype='application/json')
            logger.error("硬件调用错误")
            # 开始向告警信息表插入数据
            data_value="升降杆"
            schedule_test(data_value)
            response_data = {"code": 500, "message": "硬件调用错误", "position": 0, "robot_code": robot_code.get("device_error")}
            return Response(json.dumps(response_data), 500, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("升降杆调用失败")
            # 开始向告警信息表插入数据
            data_value="升降杆"
            schedule_test(data_value)
            response_data = {"code": 500, "message": "升降杆调用失败", "position": 0, "robot_code": robot_code.get("server_error")}
            return Response(json.dumps(response_data), 500, mimetype='application/json')


    def robot_action_voice(self):
        type = request.args.get("name")
        try:
            app_path = Path().APP_PATH
            logger.info("===================" * 10)
            logger.info("sh {}voice.sh '{}'".format(app_path, type))
            # state = os.system("sh {}voice.sh '{}'".format(app_path, type))
            logger.info("rostopic pub /mp_voice/xf_tts std_msgs/String 'data:'{}'''".format(type))

            os.system("rostopic pub /mp_voice/xf_tts std_msgs/String 'data:'{}'''".format(type))

        #     if state == 0:
        #         return True
        #     return False
        except Exception as e:
            logger.error(e)
            return True

    def status_listen(self):
        # rospy.init_node('listener')
        try:
            battery = rospy.wait_for_message("/battery", BatteryState, timeout=5)
            if battery.charge is None:
                battery_status = False
            else:
                battery_status = int(battery.charge)
        except Exception as e:
            battery_status = False
            logger.info(e)

        try:
            avoidance = rospy.wait_for_message('/avoidance', Int8, timeout=5)
            avoidance_status = avoidance.data
        except Exception as e:
            avoidance_status = False
            logger.info(e)

        try:
            obstacle = rospy.wait_for_message('/obstacle', UInt16, timeout=5)
            if obstacle.data is None:
                obstacle_status = False
            elif obstacle.data != 0:
                obstacle_status = True
            else:
                obstacle_status = False
        except Exception as e:
            obstacle_status = False
            # logger.info(e)

        try:
            isStop = rospy.wait_for_message('/isStop', Bool, timeout=5)
            isStop_status = isStop.data
            # logger.info(isStop_status)
        except Exception as e:
            isStop_status = 'error'
            logger.info(e)

        # battery 电池 返回值 True False， avoidance 超声波 返回值 异常为False 前避障1 后避障4 无避障9 ， obstacle 激光  True False ， isStop 急停 True False
        resp = {'battery': battery_status, 'avoidance': avoidance_status, 'obstacle': obstacle_status,
                'isStop': isStop_status}
        return json.dumps(resp)

    def algorithm_preview(self):
        try:
            # 接收算法预览接口参数
            params = json.loads(request.get_data().decode("utf-8"))

            # 参数拼接
            pic_type = params["picType"]
            pic_data = params["picData"]

            # 临时测试数据
            # path = "/opt/moss_robot/lib/dispatch_ips_web/data_pkg/web_path/1.jpg"
            # pic_data = get_img_ase64(path)

            # 初始化数字仪表参数
            digit_params = {"isPreview": 0, "picPathStr": "", "picType": pic_type, "rois": [], "picData": pic_data}
            # 初始化指针仪表参数
            meter_params = {"isPreview": 0, "picPathStr": "", "picType": pic_type, "rois": [], "picData": pic_data}
            # 初始化旋转开关参数
            rswitch_params = {"isPreview": 0, "picPathStr": "", "picType": pic_type, "rois": [], "picData": pic_data}
            params_type_list = list()
            # 遍历各个类型算法预览的参数
            for par in params["rois"]:
                # 根据类型组合参数类型列表
                if par["legend"] == "数字仪表盘":
                    digit_params["rois"].append(par)
                    params_type_list.append("数字仪表盘")
                elif par["legend"] == "指针仪表盘":
                    meter_params["rois"].append(par)
                    params_type_list.append("指针仪表盘")
                elif par["legend"] == "开关":
                    rswitch_params["rois"].append(par)
                    params_type_list.append("开关")
                else:
                    continue
            # 获取算法服务访问ip端口
            ip = arithmetic_service.get("host")
            port = arithmetic_service.get("port")
            # 定义绘图服务参数列表
            plot_list = list()
            params_type_list = list(set(params_type_list))  # 去重避免多次访问
            # 分算法类型访问各个算法
            for params_type in params_type_list:
                if params_type == "数字仪表盘":
                    name = 'algoritmic_server/digit'
                    request_params = digit_params
                elif params_type == "指针仪表盘":
                    name = 'algoritmic_server/meter'
                    request_params = meter_params
                elif params_type == "开关":
                    name = 'algoritmic_server/rswitch'
                    request_params = rswitch_params
                else:
                    name = ""
                    request_params = ""
                url = "http://" + ip + ":" + port + "/" + name
                logger.info(url)
                if len(params_type_list) <= 1:
                    request_params["isPreview"] = 1
                data = requests.get(url, data=json.dumps(request_params))
                logger.info(data.status_code)
                # 访问成功，提取绘图服务参数
                if data.status_code == 200:
                    res = json.loads(data.text)
                    logger.info(res)
                    if res["code"] == "0":
                        logger.info("{} 算法未识别到图片内容,code={}".format(params_type, res["code"]))
                    elif res["code"] == "1":
                        logger.info("{} 算法识别完成,code={}".format(params_type, res["code"]))
                    else:
                        logger.error("{} 算法识别错误,code={}".format(params_type, res["code"]))
                    plot_list.append(res["plots"])
                else:
                    logger.error("算法访问错误！")

            # 定义绘图服务参数
            plot_params = {
                "picPathStr": "",
                "picData": pic_data,
                "picType": pic_type,
                "plotMsg": plot_list,
                "isSaved": True,
                "isReturnImg": False
            }
            logger.info(plot_list)
            plot_url = "http://" + ip + ":" + port + "/" + "algoritmic_server/plot"
            # 调用绘图接口获取整体数据
            plot_data = requests.get(plot_url, data=json.dumps(plot_params))
            logger.info(plot_data)
            # 绘图访问成功，返回图片地址
            if plot_data.status_code == 200:
                plot_res = json.loads(plot_data.text)
                img_path = plot_res.get("resPath")
                ip = agx_service.get("host")
                port = agx_service.get("port")
                agx_url = "http://" + ip + ":" + port + "/get_data"
                logger.info(img_path)
                agx_params = {
                    "type": 1,
                    "name": img_path
                }
                if img_path:
                    agx_response = requests.get(url=agx_url, params=agx_params)
                    logger.info(agx_response)
                    img_data = base64.b64encode(agx_response.content).decode('utf-8')
                else:
                    img_data = None
            else:
                img_data = ""
            return json.dumps({"code": 200, "message": "获取成功", "data": img_data}, ensure_ascii=False)
        except Exception as e:
            logger.error(e)
        return json.dumps({"code": 500, "message": "服务器错误", "data": None}, ensure_ascii=False)

    def qr_code_preview(self):
        """
        二维码盘点算法预览
        """
        try:
            data = request.get_json()
            robot_device_id = data.get("robot_device_id", 2)
            device_data = DBRobotDevice().get_device_id(robot_device_id)
            robot_device_status = RobotDeviceStatus()
            if device_data.get("name") in ["升降机相机上", "升降机相机下", "主云台光学相机"]:
                if device_data["name"] == "升降机相机上":
                    device_status = robot_device_status.get_device_status("fourfold_up")
                    if not device_status:
                        data_value="上四倍相机"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "上四倍相机不可用",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                else:
                    device_status = robot_device_status.get_device_status("fourfold_down")
                    if not device_status:
                        data_value="下四倍相机"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "下四倍相机不可用",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                photo_path = FourCamera(device_data.get("address")).start()
                logger.info(photo_path)
                pic_type = 1
            else:
                file_path = Path().industry + datetime.datetime.now().strftime("%Y_%m_%d")
                if not os.path.exists(file_path):
                    os.makedirs(file_path)
                photo_path = file_path + "/" + datetime.datetime.now().strftime("%H_%M_%S.%f") + ".jpg"
                position_id = device_data.get("address")
                if int(position_id) == industrial_camera_position.get("down"):
                    device_status = robot_device_status.get_device_status("industry_camera_down")
                    if not device_status:
                        data_value="下工业相机"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "下工业相机不可用",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                    device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
                    if device_use_status:
                        response_data = {"code": 500, "message": "下工业相机被占用", "robot_code": robot_code.get("used")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                else:
                    device_status = robot_device_status.get_device_status("industry_camera_up")
                    if not device_status:
                        data_value="上工业相机"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "上工业相机不可用",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                    device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
                    if device_use_status:
                        response_data = {"code": 500, "message": "上工业相机被占用", "robot_code": robot_code.get("used")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                self.dlcw_l.industry_photo(photo_path, position_id)
                pic_type = 2
            qr_code = QrcodeIdentify()
            status, res, res_img = qr_code.qrcode_req(photo_path, pic_type)
            if status:
                data = {"res": res, "file_path": res_img}
                return Response(json.dumps({"code": 200, "message": "二维码预览成功", "data": data}), 200,
                                mimetype='application/json')
        except Exception as e:
            logger.error(traceback.format_exc(e))
        data = {"res": [], "file_path": ""}
        return Response(json.dumps({"code": 500, "message": "未检测到二维码", "data": data}), 200, mimetype='application/json')

    def robot_inspect(self):
        try:      
            # 接收type参数
            param=ApiBase().get_input(request)
            inspect_type= int(param.status)         
            if inspect_type == 0 or inspect_type == 1:
                logger.info(inspect_type)
                result = inspect_main(inspect_type)
                # print(result)
            else:
                for i in [0, 1]:
                    result = inspect_main(i)
            status = 1
            # print(result)
            if "异常" in result:
                status=0
            return json.dumps({"code":200,"data": result,"status":status},ensure_ascii=False)
        except Exception as e:
            logger.error(traceback.format_exc(e))
            return json.dumps({"code":500,"data":{},"status":0},ensure_ascii=False)

    def get_template_pic(self):
        """
        模板管理抓图接口（可使用四倍或者工业相机进行转图）
        return base64数据
        """
        try:
            robot_device_id = request.args.get("robot_device_id", 2)
            # 根据camera_id获取相机类型和相机上下
            device_data = DBRobotDevice().get_device_id(robot_device_id)
            # 判断需要访问的设备
            if device_data["name"] in ["升降机相机上", "升降机相机下", "主云台光学相机"]:
                camera_type = "四倍相机"
            else:
                camera_type = "工业相机"
            camera_direction = device_data["name"][-1]
            logger.info(camera_type)
            logger.info(camera_direction)
            robot_device_status = RobotDeviceStatus()
            # 访问四倍相机
            if camera_type == "四倍相机":
                if device_data["name"] == "升降机相机上":
                    device_status = robot_device_status.get_device_status("fourfold_up")
                    if not device_status:
                        data_value = "上四倍相机"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "上四倍相机不可用",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                else:
                    device_status = robot_device_status.get_device_status("fourfold_down")
                    if not device_status:
                        data_value="下四倍相机"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "下四倍相机不可用",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                # 拍照
                file_path = hk_api_model.take_picture(camera_direction)
                logger.info("数据解析完成")
                logger.info(file_path)
                if file_path:
                    with open(file_path, 'rb') as f:
                        image = f.read()
                    img_data = base64.b64encode(image).decode('utf-8')
                    response = json.dumps({"code": 200, "message": "抓取成功", "img_path": file_path, "data": img_data},
                                          ensure_ascii=False)
                    return response
            else:
                # 访问工业相机
                photo_path = Path().template_pic
                file_path = photo_path + str(uuid.uuid1()) + '.jpg'
                if not os.path.exists(photo_path):
                    os.makedirs(photo_path)
                # 判断相机上下
                Id = 1 if camera_direction == "上" else 0
                if int(Id) == industrial_camera_position.get("down"):
                    device_status = robot_device_status.get_device_status("industry_camera_down")
                    if not device_status:
                        data_value="下工业相机"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "下工业相机不可用", "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                    device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
                    if device_use_status:
                        response_data = {"code": 500, "message": "下工业相机被占用", "robot_code": robot_code.get("used")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                else:
                    device_status = robot_device_status.get_device_status("industry_camera_up")
                    if not device_status:
                        data_value="上工业相机"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "上工业相机不可用", "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                    device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
                    if device_use_status:
                        response_data = {"code": 500, "message": "上工业相机被占用", "robot_code": robot_code.get("used")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                # 拍照，并判断是否成功
                if self.dlcw_l.industry_photo(file_path, Id):
                    with open(file_path, 'rb') as f:
                        image = f.read()
                    img_data = base64.b64encode(image).decode('utf-8')
                    response = json.dumps({"code": 200, "message": "抓取成功", "file_path": file_path, "data": img_data},
                                          ensure_ascii=False)
                    return response
        except Exception as e:
            logger.error(e)
        return json.dumps({"code": 500, "message": "抓取失败", "data": None}, ensure_ascii=False)

    def set_hk_camera(self):
        """
        设置海康摄像头
        :return:
        """
        try:
            if request.method == 'GET':
                param_value = request.args.get('prama_value')  # 参数值
                param = request.args.get('prama')  # 参数类型
                param_type = request.args.get('type')  # 设备类型
            else:
                param_value = request.form.get('prama_value')
                param = request.form.get('prama')
                param_type = request.args.get('type')
            robot_device_status = RobotDeviceStatus()
            if int(param_type) == 0:
                device_status = robot_device_status.get_device_status("ptz_camera")
                if not device_status:
                    data_value="云台相机"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "云台相机不可用",
                                     "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            elif int(param_type) == 1:
                device_status = robot_device_status.get_device_status("fourfold_up")
                if not device_status:
                    data_value="上四倍相机"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "上四倍相机不可用",
                                     "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            else:
                device_status = robot_device_status.get_device_status("fourfold_down")
                if not device_status:
                    data_value="下四倍相机"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "下四倍相机不可用",
                                     "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            # 调用海康设置程序
            result = hk_api_model.hk_all_api(param_value, param, param_type)
            # 判断是否设置成功
            if result:
                response_data = {"code": 200, "message": "相机设置成功！", "robot_code": robot_code.get("normal")}
                return Response(json.dumps(response_data), 200, mimetype='application/json')
            else:
                response_data = {"code": 500, "message": "相机设置失败！", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        except Exception as e:
            logger.error(e)
        response_data = {"code": 500, "message": "相机设置失败！", "robot_code": robot_code.get("server_error")}
        return Response(json.dumps(response_data), 500, mimetype='application/json')

    def get_hk_camera(self):
        """
        获取海康摄像头设置的参数值
        :return:
        """
        try:
            # 调用海康相机设置函数
            results = hk_api_model.get_camera_params()
            if results:
                response_data = {"code": 200, "message": "相机参数获取成功！", "robot_code": robot_code.get("normal")}
                return Response(json.dumps(response_data), 200, mimetype='application/json')
            else:
                response_data = {"code": 500, "message": "相机参数获取失败！", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        except Exception as e:
            logger.error(e)
        response_data = {"code": 500, "message": "相机参数获取失败！", "robot_code": robot_code.get("server_error")}
        return Response(json.dumps(response_data), 500, mimetype='application/json')

    def ptz(self):
        """
        海康预置位调用函数
        """
        try:
            if request.method == 'GET':
                method = request.args.get('method', None)
                ptz_val = request.args.get('ptz_val', None)
            else:
                method = request.form.get('method', None)
                ptz_val = request.form.get('ptz_val', None)
            robot_device_status = RobotDeviceStatus()
            # 判断云台是否可用
            device_status = robot_device_status.get_device_status("ptz")
            if not device_status:
                # 开始向告警信息表插入数据
                data_value="云台"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "云台控制不可用", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("ptz")
            if device_use_status:
                response_data = {"code": 500, "message": "云台控制被占用", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            result = robot_ptz(method, ptz_val)
            if result:
                response_data = {"code": 200, "message": "云台控制成功！", "robot_code": robot_code.get("normal")}
                return Response(json.dumps(response_data), 200, mimetype='application/json')
            else:
                response_data = {"code": 500, "message": "云台控制失败！", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("云台预置位控制失败")
            response_data = {"code": 500, "message": "服务程序错误！", "robot_code": robot_code.get("server_error")}
            return Response(json.dumps(response_data), 500, mimetype='application/json')

    def visit_command(self):
        try:
            redis_visit_state = RedisPipe("VisitState")
            redis_visit_state.set_data("1", 20)
            return json.dumps({"code": 200, "msg": "success！"}, ensure_ascii=False)
        except Exception as e:
            logger.error(e)
            logger.error("redis错误")
            return json.dumps({"code": 500, "msg": "fail！"}, ensure_ascii=False)

    @staticmethod
    def start():
        rospy.init_node("Mian_node", disable_signals=False)
        Thread(target=app.run, args=('0.0.0.0', 8088, False)).start()
        # app.run(host='0.0.0.0', port=8888, debug=False)


def get_fourfold_photo(ip, username, password, filename, port):
    StreamingUrl = "http://" + ip + ":" + str(port) + "/ISAPI/Streaming/channels/1"

    auth = HTTPDigestAuth(username, password)
    url = StreamingUrl + "/picture"
    xml_data = requests.get(url, auth=auth)
    if xml_data.status_code != 200:
        return False
    with open(filename, 'wb') as f:
        f.write(xml_data.content)
    return True


def decorate_queue(func):
    def wrapper(*args):
        q.put((func, *args))
        global queue_flag
        while True:
            if queue_flag:
                queue_flag = False
                fun, args = q.get()
                return_data = fun(args)
                queue_flag = True
                return return_data
            if queue_flag:
                break

    return wrapper


@decorate_queue
def action_elevator_height(position):
    logger.info("升降杆升起高度为{}".format(position))
    state = Main(1, position)
    if state:
        return True
    return False


def merge_dict(dict1, dict2, dict3, dict4, dict5):
    res = {**dict1, **dict2, **dict3, **dict4, **dict5}
    return res


def get_show_data(core_id):
    logger.info(int(core_id))
    camera_data = DBCoreIndcameraParam().get_industry_id(int(core_id))
    logger.info(camera_data)
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
    logger.info(show_data)
    return show_data


def get_img_ase64(path):
    if os.path.exists(path):
        with open(path, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            return img
    else:
        return False


if __name__ == '__main__':
    server = API()
    logger.info('[+] AGX API is running [%s]' % datetime.datetime.now())
    server.start()
