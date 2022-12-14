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
            # ========================== ?????????????????? ==========================
            # ??????
            {'r': '/robot_move', 'm': ['POST'], 'f': self.robot_move},
            # ?????????
            {'r': '/robot_thermal_imagery', 'm': ['GET'], 'f': self.robot_thermal_imagery},
            # ???????????????????????????
            {'r': '/four_camera_photo', 'm': ['GET'], 'f': self.four_camera_photo},
            # ?????????
            {'r': '/robot_transducer', 'm': ['GET'], 'f': self.robot_transducer},
            # ??????
            {'r': '/robot_voice', 'm': ['POST'], 'f': self.robot_voice},
            # ????????????
            {'r': '/get_devicePrama', 'm': ['GET', 'POST'], 'f': self.get_industry_parameters},
            {'r': '/get_pic', 'm': ['GET', 'POST'], 'f': self.get_industry_photo},
            {'r': '/change_devicePrama', 'm': ['GET', 'POST'], 'f': self.set_industry_parameter},
            # ?????????
            {'r': '/elevator', 'm': ['GET'], 'f': self.robot_lifter},
            # ??????????????????
            {'r': '/set_hk_camera', 'm': ['GET', 'POST'], 'f': self.set_hk_camera},
            {'r': '/get_hk_camera', 'm': ['GET', 'POST'], 'f': self.get_hk_camera},
            # ??????
            {'r': '/ptz', 'm': ['GET', 'POST'], 'f': self.ptz},
            # ========================== ?????????????????? ==========================

            # ????????????
            {'r': '/get_before_data', 'm': ['GET', 'POST'], 'f': self.file_befor_data},
            # ???????????????????????????
            {'r': '/api/robot/robotState', 'm': ['POST'], 'f': self.robot_robot_state},
            {'r': '/get_data', 'm': ['GET', 'POST'], 'f': self.file_data},
            # ????????????
            {'r': '/Industry_preview', 'm': ['GET'], 'f': self.industry_al_preview},
            # ????????????
            {'r': '/dispatch/industry/photo', 'm': ['GET'], 'f': self.industry_industry_photo},

            {'r': '/visible_light_preview', 'm': ['GET', 'POST'], 'f': self.fourfold_al_preview},
            {'r': '/action_voice', 'm': ['GET'], 'f': self.robot_action_voice},
            {'r': '/az_status_light', 'm': ['GET'], 'f': self.status_listen},
            # ????????????OCR??????????????????
            {'r': '/algorithm_preview', 'm': ['POST'], 'f': self.algorithm_preview},
            # ????????????????????????????????????
            {'r': '/qr_code_preview', 'm': ['POST'], 'f': self.qr_code_preview},
            # ?????????????????????
            {'r':'/robot_inspect','m':['GET','POST'],'f': self.robot_inspect},
            # ????????????????????????
            {'r': '/get_template_pic', 'm': ['GET'], 'f': self.get_template_pic},
            # ========================= ?????????????????????????????? =========================
            # redis
            {'r': '/visit_command', 'm': ['GET', 'POST'], 'f': self.visit_command},
            # ????????????
            {'r': '/robot/voice/state', 'm': ['GET', "POST"], 'f': self.voice_state},
            {'r': '/robot/voice/action', 'm': ['GET', 'POST'], 'f': self.voice_action},
        ]
        for route in routes:
            self.addroute(route)
        try:
            # ????????????????????????????????????
            industrial_camera_type = FindDevice().start()
            # ????????????????????????
            if industrial_camera_type.get("Hikvision", None):
                # ?????????????????????
                logger.info("?????????????????????")
                RedisPipe("industrial_camera_type").set_data("Hikvision")
                self.dlcw_l = Hikvision()
            elif industrial_camera_type.get("Daheng", None):
                # ?????????????????????
                logger.info("?????????????????????")
                RedisPipe("industrial_camera_type").set_data("Daheng")
                self.dlcw_l = Daheng()
            else:
                logger.error("?????????????????????????????????????????????")
                # sys.exit()
        except Exception as e:
            logger.error(e)
            logger.error("???????????????????????????????????????????????????")
        self.modify_num = 0
        self.queue_flag = True
        self.image_upper_state = 0
        self.image_lower_state = 0

    @staticmethod
    def addroute(route):
        app.add_url_rule(route['r'], view_func=route['f'], methods=route['m'])

    def robot_move(self):
        """
        ?????????????????????
        """
        try:
            data = request.get_json()
            position, move_type = data.get("position"), data.get("move_type")
            result = RobotMove(position, move_type).start()
            if result:
                return Response(json.dumps({"code": 200, "message": "?????????????????????"}), 200, mimetype='application/json')
            logger.error("??????????????????")
            # ????????????????????????????????????
            data_value = "????????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "??????????????????"}), 510, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("??????????????????")
            # ????????????????????????????????????
            data_value="????????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "??????????????????"}), 500, mimetype='application/json')

    def robot_thermal_imagery(self):
        try:
            ip = request.args.get('ip')
            photo_path = request.args.get('photo_path')
            thermal_imagery = ThermalImagery()
            file_path = thermal_imagery.take_picture()
            result = thermal_imagery.get_results()
            if file_path:
                return Response(
                    json.dumps({"code": 200, "message": "?????????????????????", "file_path": file_path, "result": result}), 200,
                    mimetype='application/json')
            # ????????????????????????????????????
            data_value="?????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "?????????????????????", "file_path": "", "result": {}}), 510,
                            mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("??????????????????")
            # ????????????????????????????????????
            data_value="?????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "??????????????????", "file_path": "", "result": {}}), 500,
                            mimetype='application/json')

    def four_camera_photo(self):
        try:
            ip = request.args.get('ip')
            file_path = FourCamera(ip).start()
            if file_path:
                return Response(json.dumps({"code": 200, "message": "????????????????????????", "file_path": file_path}), 200,
                                mimetype='application/json')
             # ????????????????????????????????????
            data_value="????????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "????????????????????????", "file_path": file_path}), 510,
                            mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("??????????????????")
             # ????????????????????????????????????
            data_value="????????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "??????????????????"}), 500, mimetype='application/json')

    def robot_transducer(self):
        try:
            item_id = request.args.get('item_id')
            result = RobotTransducer(item_id).start()
            if result:
                return Response(json.dumps({"code": 200, "message": "????????????", "res": result}), 200,
                                mimetype='application/json')
            data_value="?????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "????????????", "res": 0}), 510, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("??????????????????")
            data_value="?????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "??????????????????"}), 500, mimetype='application/json')

    def robot_voice(self):
        try:
            data = request.get_json()
            text = data.get("text", "")
            result = RobotVoice(text).start()
            if result:
                return Response(json.dumps({"code": 200, "message": "????????????"}), 200, mimetype='application/json')
            data_value="????????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 510, "message": "????????????"}), 510, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("??????????????????")
            data_value="????????????"
            schedule_test(data_value)
            return Response(json.dumps({"code": 500, "message": "??????????????????"}), 500, mimetype='application/json')

    # ?????????????????????????????????
    def voice_state(self):
        msg = Msg()
        try:
            data_dict = {'robotId': robotId, 'voiceState': 0}
            return msg.success(msg='?????????????????????????????????', result=data_dict).json()
        except Exception as e:
            logger.error(e)
            return msg.fail(msg='?????????????????????????????????').json()

    # ???????????????????????????
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
                return msg.fail(msg='?????????????????????????????????,?????????????????????').json()

            return msg.success(msg='?????????????????????????????????', result=data_dict).json()
        except Exception as e:
            logger.error(e)
            return msg.fail(msg='?????????????????????????????????').json()

    # ????????????????????????
    def file_befor_data(self):
        '''
            type??????????????????????????????
            name??????????????????????????????
        '''

        type = str(request.args.get("type"))
        file_path = request.args.get("name")

        if type != "5":
            if not os.path.isfile(file_path):
                return json.dumps({"statusCode": 500, "message": "??????????????????"})
        else:
            if not os.path.isfile(file_path):
                return json.dumps({"statusCode": 500, "message": "??????????????????"})
        # ???????????????????????????????????????
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
            return json.dumps({"statusCode": 500, "message": "??????type??????"})

        return resp

    # ???????????????????????????
    @staticmethod
    def robot_robot_state():
        msg = Msg()
        res = request.get_json()
        try:
            robot_id = res.get("robotId")
            robot_path_id = res.get("robotPathId")
            body = RedisPipe("Robot_state").get_data()
            if not body:
                return msg.fail(msg='?????????????????????????????????').json()
            body = str(body, 'utf-8')
            data_dict = eval(body)

            robot_position_id = data_dict.get("robot_position_id")

            if robot_path_id and robot_position_id:
                if DBRobotPathPositionRelation().get_alive_data(robot_path_id, robot_position_id):
                    data_dict["is_now_path"] = 0
                else:
                    data_dict["is_now_path"] = 1

            return msg.success(msg='?????????????????????????????????', result={"result": data_dict}).json()

        except Exception as e:
            logger.error(e)
            return msg.fail(msg='?????????????????????????????????').json()

    # ????????????????????????
    def file_data(self):
        '''
            type??????????????????????????????
            name??????????????????????????????
        '''

        type = str(request.args.get("type"))
        file_path = request.args.get("name")

        if type != "5":
            if not os.path.isfile(file_path):
                return json.dumps({"statusCode": 500, "message": "??????????????????"})
        else:
            if not os.path.isfile(file_path):
                return json.dumps({"statusCode": 500, "message": "??????????????????"})
        # ???????????????????????????????????????
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
            return json.dumps({"statusCode": 500, "message": "??????type??????"})

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
                data_value = "???????????????"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
            if device_use_status:
                data_value="???????????????"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        else:
            device_status = robot_device_status.get_device_status("industry_camera_up")
            if not device_status:
                data_value="???????????????"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
            if device_use_status:
                data_value="???????????????"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        if not position_id:
            return Response(json.dumps({"code": 505, "message": "??????id??????"}), 505, mimetype='application/json')
        photo_params = self.dlcw_l.get_camera_parameter(position_id)
        if not photo_params:
            return Response(json.dumps({"code": 500, "message": "????????????"}), 500, mimetype='application/json')
        return Response(json.dumps({"code": 200, "data": photo_params, "message": "sucess"}), 200, mimetype='application/json')

    # ????????????????????????
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
                    # ????????????????????????????????????
                    data_value="???????????????"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
                device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
                if device_use_status:
                    response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            else:
                device_status = robot_device_status.get_device_status("industry_camera_up")
                if not device_status:
                    # ????????????????????????????????????
                    data_value="???????????????"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
                device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
                if device_use_status:
                    response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
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
                logger.error("????????????????????????")
                 # ????????????????????????????????????
                data_value="????????????"
                schedule_test(data_value)
                rustle_dict = {"statusCode": 500, "message": "????????????"}
                return json.dumps(rustle_dict)
        except Exception as e:
            logger.error(e)
            logger.error("????????????????????????")
            # ????????????????????????????????????
            data_value="????????????"
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
                response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
            if device_use_status:
                response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        else:
            device_status = robot_device_status.get_device_status("industry_camera_up")
            if not device_status:
                response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
            if device_use_status:
                response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        if device_param:
            camera_params = dict()
            camera_params["ExposureTime"] = device_param.get("ExposureTime")
            camera_params["BalanceRatio"] = device_param.get("BalanceRatio")
            camera_params["Gain"] = device_param.get("Gain")
            if not self.dlcw_l.modify_industry_parameter(Id, None, None, camera_params):
                rustle = {"code": 500, 'message': '??????????????????????????????'}
                return json.dumps(rustle)
            else:
                return {"code": 200, 'message': '??????????????????????????????'}
        else:
            if self.dlcw_l.modify_industry_parameter(Id, parma, parma_value, None):
                rustle = {"code": 200, 'message': '????????????'}
                return json.dumps(rustle)
            rustle = {"code": 500, 'message': '????????????'}
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
            logger.info("?????????id{}?????????".format(core_id))
            return json.dumps({"statusCode": 500, "message": "not find id data"})
        # ???????????????
        elevator_height = camera_data.get("elevator_height")
        status, position = RobotLifer(1, elevator_height).start()
        if not status:
            return json.dumps({"statusCode": 500, "message": "?????????????????????"})
        logger.info("????????????????????????")
        # ???????????????id
        robot_device_id = camera_data.get("robot_device_id")
        device_data = DBRobotDevice().get_device_id(robot_device_id)
        if not device_data:
            return json.dumps({"statusCode": 500, "message": "???????????????"})
        Id = device_data.get("address")
        logger.info('??????id???{}?????????'.format(Id))

        # ???????????????
        device_param = camera_data.get("device_param")
        # device_param = {
        #     "ExposureTime": 40000,
        #     "BalanceRatio": 4000,
        #     "Gain": 0,
        #     "PhotoIntervalTime": 0,
        #     "PhotoNum": 2
        # }
        # ????????????????????????
        camera_params = dict()
        camera_params["ExposureTime"] = device_param.get("ExposureTime")
        camera_params["BalanceRatio"] = device_param.get("BalanceRatio")
        camera_params["Gain"] = device_param.get("Gain")
        if not self.dlcw_l.modify_industry_parameter(Id, None, None, camera_params):
            rustle = {"statusCode": 500, 'msg': '??????????????????????????????'}
            return json.dumps(rustle)
        core_device_id = camera_data.get("core_device_id")
        # ??????????????????
        core_cabinet_id = DBCoreDeviceLocation().get_core_cabinet_id(core_device_id)
        cabinet_data = DBCoreCabinet().get_cabinet_name(core_cabinet_id)
        if cabinet_data:
            cabinet_name = cabinet_data.get("name")
        else:
            cabinet_name = "00"

        # ????????????ID
        ret = DBCoreDevice().get_device_id(core_device_id)
        if ret:
            device_name = ret.get("name")
        else:
            device_name = "00"

        # ????????????U-??????U
        algorithm_param = camera_data.get("algorithm_param")
        u_params = algorithm_param.get("u_params")
        start_u = u_params.get("start_u")
        end_u = u_params.get("end_u")

        show_data = "P-{} U-{} D-{}-{}".format(cabinet_name, device_name, start_u, end_u)
        # ????????????
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
                logger.info("????????????????????????")
                return json.dumps({"statusCode": 500, "message": "????????????????????????"})
            # P?????? U???????????? D??????U-??????U
            logger.info('????????????')
            light_detect_result, light_path = post_agx_light(file_path, rois, show_data, 3)
            logger.info('????????????')
            logger.info(light_detect_result)
            logger.info(light_path)
            # ???????????????img??????
            if not light_path:
                logger.info("??????????????????")
                return json.dumps({"statusCode": 500, "message": "??????????????????"})
            DBCoreIndcameraParam().update({"core_indcamera_param_id": core_id},
                                          {"img_path": str(light_path),
                                           "img_pah_thumbnail": light_path})
            rustle_dict = {"code": 200, "message": "sucess"}
            return json.dumps(rustle_dict)
        else:
            file_path = file_path_dir + '.jpg'
            logger.info('??????????????????')
            for num in range(photo_num):
                file_name = file_path_dir + "_{}.jpg".format(num)
                if not self.dlcw_l.industry_photo(file_name, Id):
                    logger.info("????????????????????????")
                    return json.dumps({"statusCode": 500, "message": "????????????????????????"})
                time.sleep(photo_interval_time)
            logger.info('????????????')
            blend_state, blend_photo_path = lightBlend(file_path, photo_num)
            if blend_state:
                logger.info("??????????????????")
                light_detect_result, light_path = post_agx_light(blend_photo_path, rois, show_data, 3)
                logger.info('????????????')
            else:
                return json.dumps({"statusCode": 500, "message": "??????????????????"})
            # ???????????????img??????
            if not light_path:
                logger.info("??????????????????")
                return json.dumps({"statusCode": 500, "message": "??????????????????"})
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
        # ????????????0????????????????????????
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
            logger.info("?????????id{}?????????".format(core_id))
            return json.dumps({"code": 500, "message": "not find id data"})

        # ???????????????id
        robot_device_id = camera_data.get("robot_device_id")
        device_data = DBRobotDevice().get_device_id(robot_device_id)
        if not device_data:
            return json.dumps({"code": 500, "message": "???????????????"})
        Id = device_data.get("address")
        logger.info('??????id???{}?????????'.format(Id))

        # ???????????????
        device_param = camera_data.get("device_param")
        # ????????????????????????
        camera_params = dict()
        camera_params["ExposureTime"] = device_param.get("ExposureTime")
        camera_params["BalanceRatio"] = device_param.get("BalanceRatio")
        camera_params["Gain"] = device_param.get("Gain")
        if not self.dlcw_l.modify_industry_parameter(Id, None, None, camera_params):
            rustle = {"statusCode": 500, 'msg': '??????????????????????????????'}
            return json.dumps(rustle)
        # ????????????ID?????????????????????
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
                    logger.info("????????????????????????")
                    data_value="????????????"
                    schedule_test(data_value)
                    return json.dumps({"statusCode": 500, "message": "????????????????????????"})
                # P?????? U???????????? D??????U-??????U
                logger.info('????????????')
                rustle_dict = {"code": 200, "message": "sucess",
                               "rois": rois,
                               "light_path": photo_name
                               }
            else:
                file_name_base = photo_name.split(".jpg")[0]
                logger.info('??????????????????')
                for num in range(photo_num):
                    file_name = file_name_base + "_{}.jpg".format(num)
                    if not self.dlcw_l.industry_photo(file_name, Id):
                        logger.info("????????????????????????")
                        data_value = "????????????"
                        schedule_test(data_value)
                        return json.dumps({"statusCode": 500, "message": "????????????????????????"})
                    time.sleep(photo_interval_time)
                blend_state, photo_name = lightBlend(photo_name, photo_num)
                rustle_dict = {"code": 200, "message": "sucess",
                               "rois": rois,
                               "light_path": photo_name
                               }
        except Exception as e:
            logger.error(e)
            data_value="????????????"
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
            logger.info("?????????????????? {}".format(port))
            rustle_dict = {"statusCode": 500, "message": "??????????????????"}
            return json.dumps(rustle_dict)

        try:
            web_photo_path = Path().web_photo
            name = web_photo_path + datetime.datetime.now().strftime("%H_%M_%S")
            if nums == 1:
                name = name + '.jpg'
                if not get_fourfold_photo(ip, username, password, name, port):
                    logger.info("????????????????????????")
                    data_value="????????????"
                    schedule_test(data_value)
                    return json.dumps({"statusCode": 500, "message": "????????????????????????"})

                light_detect_result, light_path = post_agx_light(name, 1, 5000, 1, 5000, "P-00 U-00 D-00")
                with open(light_path, 'rb') as f:
                    image = f.read()
                resp = Response(image, mimetype="image/jpeg")
                return resp
            else:
                for num in range(nums):
                    file_name = name + "_{}.jpg".format(num)
                    if not get_fourfold_photo(ip, username, password, file_name, port):
                        logger.info("????????????????????????")
                        data_value="????????????"
                        schedule_test(data_value)
                        return json.dumps({"statusCode": 500, "message": "????????????????????????"})
                    time.sleep(time_data)

                blend_state, photo_name = lightBlend(name, nums)
                light_detect_result, light_path = post_agx_light(photo_name, 1, 5000, 1, 5000, "P-00 U-00 D-00")
                with open(light_path, 'rb') as f:
                    image = f.read()
                resp = Response(image, mimetype="image/jpeg")
                return resp

        except Exception as e:
            logger.error(e)
            data_value="????????????"
            schedule_test(data_value)
            rustle_dict = {"statusCode": 500, "message": "fail"}
            return json.dumps(rustle_dict)

    def robot_lifter(self):
        # type 2???????????????
        try:
            robot_device_status = RobotDeviceStatus()
            # ???????????????????????????
            device_status = robot_device_status.get_device_status("lifter")
            if not device_status:
                # ????????????????????????????????????
                data_value="?????????"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "??????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            # ??????????????????????????????
            device_use_status = robot_device_status.get_device_use_status("lifter")
            if device_use_status:
                response_data = {"code": 500, "message": "??????????????????", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            move_type = int(request.args.get("type", 0))
            position = int(request.args.get("position"))
            status, position = robot_lifter(move_type, position)
            if status:
                response_data = {"code": 200, "message": "?????????????????????", "position": position, "robot_code": robot_code.get("normal")}
                return Response(json.dumps(response_data), 200, mimetype='application/json')
            logger.error("??????????????????")
            # ????????????????????????????????????
            data_value="?????????"
            schedule_test(data_value)
            response_data = {"code": 500, "message": "??????????????????", "position": 0, "robot_code": robot_code.get("device_error")}
            return Response(json.dumps(response_data), 500, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("?????????????????????")
            # ????????????????????????????????????
            data_value="?????????"
            schedule_test(data_value)
            response_data = {"code": 500, "message": "?????????????????????", "position": 0, "robot_code": robot_code.get("server_error")}
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

        # battery ?????? ????????? True False??? avoidance ????????? ????????? ?????????False ?????????1 ?????????4 ?????????9 ??? obstacle ??????  True False ??? isStop ?????? True False
        resp = {'battery': battery_status, 'avoidance': avoidance_status, 'obstacle': obstacle_status,
                'isStop': isStop_status}
        return json.dumps(resp)

    def algorithm_preview(self):
        try:
            # ??????????????????????????????
            params = json.loads(request.get_data().decode("utf-8"))

            # ????????????
            pic_type = params["picType"]
            pic_data = params["picData"]

            # ??????????????????
            # path = "/opt/moss_robot/lib/dispatch_ips_web/data_pkg/web_path/1.jpg"
            # pic_data = get_img_ase64(path)

            # ???????????????????????????
            digit_params = {"isPreview": 0, "picPathStr": "", "picType": pic_type, "rois": [], "picData": pic_data}
            # ???????????????????????????
            meter_params = {"isPreview": 0, "picPathStr": "", "picType": pic_type, "rois": [], "picData": pic_data}
            # ???????????????????????????
            rswitch_params = {"isPreview": 0, "picPathStr": "", "picType": pic_type, "rois": [], "picData": pic_data}
            params_type_list = list()
            # ???????????????????????????????????????
            for par in params["rois"]:
                # ????????????????????????????????????
                if par["legend"] == "???????????????":
                    digit_params["rois"].append(par)
                    params_type_list.append("???????????????")
                elif par["legend"] == "???????????????":
                    meter_params["rois"].append(par)
                    params_type_list.append("???????????????")
                elif par["legend"] == "??????":
                    rswitch_params["rois"].append(par)
                    params_type_list.append("??????")
                else:
                    continue
            # ????????????????????????ip??????
            ip = arithmetic_service.get("host")
            port = arithmetic_service.get("port")
            # ??????????????????????????????
            plot_list = list()
            params_type_list = list(set(params_type_list))  # ????????????????????????
            # ?????????????????????????????????
            for params_type in params_type_list:
                if params_type == "???????????????":
                    name = 'algoritmic_server/digit'
                    request_params = digit_params
                elif params_type == "???????????????":
                    name = 'algoritmic_server/meter'
                    request_params = meter_params
                elif params_type == "??????":
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
                # ???????????????????????????????????????
                if data.status_code == 200:
                    res = json.loads(data.text)
                    logger.info(res)
                    if res["code"] == "0":
                        logger.info("{} ??????????????????????????????,code={}".format(params_type, res["code"]))
                    elif res["code"] == "1":
                        logger.info("{} ??????????????????,code={}".format(params_type, res["code"]))
                    else:
                        logger.error("{} ??????????????????,code={}".format(params_type, res["code"]))
                    plot_list.append(res["plots"])
                else:
                    logger.error("?????????????????????")

            # ????????????????????????
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
            # ????????????????????????????????????
            plot_data = requests.get(plot_url, data=json.dumps(plot_params))
            logger.info(plot_data)
            # ???????????????????????????????????????
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
            return json.dumps({"code": 200, "message": "????????????", "data": img_data}, ensure_ascii=False)
        except Exception as e:
            logger.error(e)
        return json.dumps({"code": 500, "message": "???????????????", "data": None}, ensure_ascii=False)

    def qr_code_preview(self):
        """
        ???????????????????????????
        """
        try:
            data = request.get_json()
            robot_device_id = data.get("robot_device_id", 2)
            device_data = DBRobotDevice().get_device_id(robot_device_id)
            robot_device_status = RobotDeviceStatus()
            if device_data.get("name") in ["??????????????????", "??????????????????", "?????????????????????"]:
                if device_data["name"] == "??????????????????":
                    device_status = robot_device_status.get_device_status("fourfold_up")
                    if not device_status:
                        data_value="???????????????"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "????????????????????????",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                else:
                    device_status = robot_device_status.get_device_status("fourfold_down")
                    if not device_status:
                        data_value="???????????????"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "????????????????????????",
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
                        data_value="???????????????"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "????????????????????????",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                    device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
                    if device_use_status:
                        response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                else:
                    device_status = robot_device_status.get_device_status("industry_camera_up")
                    if not device_status:
                        data_value="???????????????"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "????????????????????????",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                    device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
                    if device_use_status:
                        response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                self.dlcw_l.industry_photo(photo_path, position_id)
                pic_type = 2
            qr_code = QrcodeIdentify()
            status, res, res_img = qr_code.qrcode_req(photo_path, pic_type)
            if status:
                data = {"res": res, "file_path": res_img}
                return Response(json.dumps({"code": 200, "message": "?????????????????????", "data": data}), 200,
                                mimetype='application/json')
        except Exception as e:
            logger.error(traceback.format_exc(e))
        data = {"res": [], "file_path": ""}
        return Response(json.dumps({"code": 500, "message": "?????????????????????", "data": data}), 200, mimetype='application/json')

    def robot_inspect(self):
        try:      
            # ??????type??????
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
            if "??????" in result:
                status=0
            return json.dumps({"code":200,"data": result,"status":status},ensure_ascii=False)
        except Exception as e:
            logger.error(traceback.format_exc(e))
            return json.dumps({"code":500,"data":{},"status":0},ensure_ascii=False)

    def get_template_pic(self):
        """
        ???????????????????????????????????????????????????????????????????????????
        return base64??????
        """
        try:
            robot_device_id = request.args.get("robot_device_id", 2)
            # ??????camera_id?????????????????????????????????
            device_data = DBRobotDevice().get_device_id(robot_device_id)
            # ???????????????????????????
            if device_data["name"] in ["??????????????????", "??????????????????", "?????????????????????"]:
                camera_type = "????????????"
            else:
                camera_type = "????????????"
            camera_direction = device_data["name"][-1]
            logger.info(camera_type)
            logger.info(camera_direction)
            robot_device_status = RobotDeviceStatus()
            # ??????????????????
            if camera_type == "????????????":
                if device_data["name"] == "??????????????????":
                    device_status = robot_device_status.get_device_status("fourfold_up")
                    if not device_status:
                        data_value = "???????????????"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "????????????????????????",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                else:
                    device_status = robot_device_status.get_device_status("fourfold_down")
                    if not device_status:
                        data_value="???????????????"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "????????????????????????",
                                         "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                # ??????
                file_path = hk_api_model.take_picture(camera_direction)
                logger.info("??????????????????")
                logger.info(file_path)
                if file_path:
                    with open(file_path, 'rb') as f:
                        image = f.read()
                    img_data = base64.b64encode(image).decode('utf-8')
                    response = json.dumps({"code": 200, "message": "????????????", "img_path": file_path, "data": img_data},
                                          ensure_ascii=False)
                    return response
            else:
                # ??????????????????
                photo_path = Path().template_pic
                file_path = photo_path + str(uuid.uuid1()) + '.jpg'
                if not os.path.exists(photo_path):
                    os.makedirs(photo_path)
                # ??????????????????
                Id = 1 if camera_direction == "???" else 0
                if int(Id) == industrial_camera_position.get("down"):
                    device_status = robot_device_status.get_device_status("industry_camera_down")
                    if not device_status:
                        data_value="???????????????"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                    device_use_status = robot_device_status.get_device_use_status("industry_camera_down")
                    if device_use_status:
                        response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                else:
                    device_status = robot_device_status.get_device_status("industry_camera_up")
                    if not device_status:
                        data_value="???????????????"
                        schedule_test(data_value)
                        response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("device_error")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                    device_use_status = robot_device_status.get_device_use_status("industry_camera_up")
                    if device_use_status:
                        response_data = {"code": 500, "message": "????????????????????????", "robot_code": robot_code.get("used")}
                        return Response(json.dumps(response_data), 500, mimetype='application/json')
                # ??????????????????????????????
                if self.dlcw_l.industry_photo(file_path, Id):
                    with open(file_path, 'rb') as f:
                        image = f.read()
                    img_data = base64.b64encode(image).decode('utf-8')
                    response = json.dumps({"code": 200, "message": "????????????", "file_path": file_path, "data": img_data},
                                          ensure_ascii=False)
                    return response
        except Exception as e:
            logger.error(e)
        return json.dumps({"code": 500, "message": "????????????", "data": None}, ensure_ascii=False)

    def set_hk_camera(self):
        """
        ?????????????????????
        :return:
        """
        try:
            if request.method == 'GET':
                param_value = request.args.get('prama_value')  # ?????????
                param = request.args.get('prama')  # ????????????
                param_type = request.args.get('type')  # ????????????
            else:
                param_value = request.form.get('prama_value')
                param = request.form.get('prama')
                param_type = request.args.get('type')
            robot_device_status = RobotDeviceStatus()
            if int(param_type) == 0:
                device_status = robot_device_status.get_device_status("ptz_camera")
                if not device_status:
                    data_value="????????????"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "?????????????????????",
                                     "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            elif int(param_type) == 1:
                device_status = robot_device_status.get_device_status("fourfold_up")
                if not device_status:
                    data_value="???????????????"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "????????????????????????",
                                     "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            else:
                device_status = robot_device_status.get_device_status("fourfold_down")
                if not device_status:
                    data_value="???????????????"
                    schedule_test(data_value)
                    response_data = {"code": 500, "message": "????????????????????????",
                                     "robot_code": robot_code.get("device_error")}
                    return Response(json.dumps(response_data), 500, mimetype='application/json')
            # ????????????????????????
            result = hk_api_model.hk_all_api(param_value, param, param_type)
            # ????????????????????????
            if result:
                response_data = {"code": 200, "message": "?????????????????????", "robot_code": robot_code.get("normal")}
                return Response(json.dumps(response_data), 200, mimetype='application/json')
            else:
                response_data = {"code": 500, "message": "?????????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        except Exception as e:
            logger.error(e)
        response_data = {"code": 500, "message": "?????????????????????", "robot_code": robot_code.get("server_error")}
        return Response(json.dumps(response_data), 500, mimetype='application/json')

    def get_hk_camera(self):
        """
        ???????????????????????????????????????
        :return:
        """
        try:
            # ??????????????????????????????
            results = hk_api_model.get_camera_params()
            if results:
                response_data = {"code": 200, "message": "???????????????????????????", "robot_code": robot_code.get("normal")}
                return Response(json.dumps(response_data), 200, mimetype='application/json')
            else:
                response_data = {"code": 500, "message": "???????????????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        except Exception as e:
            logger.error(e)
        response_data = {"code": 500, "message": "???????????????????????????", "robot_code": robot_code.get("server_error")}
        return Response(json.dumps(response_data), 500, mimetype='application/json')

    def ptz(self):
        """
        ???????????????????????????
        """
        try:
            if request.method == 'GET':
                method = request.args.get('method', None)
                ptz_val = request.args.get('ptz_val', None)
            else:
                method = request.form.get('method', None)
                ptz_val = request.form.get('ptz_val', None)
            robot_device_status = RobotDeviceStatus()
            # ????????????????????????
            device_status = robot_device_status.get_device_status("ptz")
            if not device_status:
                # ????????????????????????????????????
                data_value="??????"
                schedule_test(data_value)
                response_data = {"code": 500, "message": "?????????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            device_use_status = robot_device_status.get_device_use_status("ptz")
            if device_use_status:
                response_data = {"code": 500, "message": "?????????????????????", "robot_code": robot_code.get("used")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
            result = robot_ptz(method, ptz_val)
            if result:
                response_data = {"code": 200, "message": "?????????????????????", "robot_code": robot_code.get("normal")}
                return Response(json.dumps(response_data), 200, mimetype='application/json')
            else:
                response_data = {"code": 500, "message": "?????????????????????", "robot_code": robot_code.get("device_error")}
                return Response(json.dumps(response_data), 500, mimetype='application/json')
        except Exception as e:
            logger.error(e)
            logger.error("???????????????????????????")
            response_data = {"code": 500, "message": "?????????????????????", "robot_code": robot_code.get("server_error")}
            return Response(json.dumps(response_data), 500, mimetype='application/json')

    def visit_command(self):
        try:
            redis_visit_state = RedisPipe("VisitState")
            redis_visit_state.set_data("1", 20)
            return json.dumps({"code": 200, "msg": "success???"}, ensure_ascii=False)
        except Exception as e:
            logger.error(e)
            logger.error("redis??????")
            return json.dumps({"code": 500, "msg": "fail???"}, ensure_ascii=False)

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
    logger.info("????????????????????????{}".format(position))
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
    # ??????????????????
    core_cabinet_id = DBCoreDeviceLocation().get_core_cabinet_id(core_device_id)
    cabinet_data = DBCoreCabinet().get_cabinet_name(core_cabinet_id)
    if cabinet_data:
        cabinet_name = cabinet_data.get("name")
    else:
        cabinet_name = "00"

    # ????????????ID
    ret = DBCoreDevice().get_device_id(core_device_id)
    if ret:
        device_name = ret.get("name")
    else:
        device_name = "00"

    # ????????????U-??????U
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
