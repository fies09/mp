import base64
import json
import os
import time

import requests
from configs import pointer_ips, db_rabbit_mq
from configs.log import logger
from core.redis_interactive import RedisPipe
from core.dict_config import voice_dict, switch_status
from schema.db_robot_voice import DBRobotVoice
from modules.voice_action import voice_client, voice_stop, voice_start
from core.rabbitmq_db import RabbitPublisher
# 获取工业相机的图片
def get_industry_photo(id, file_name):
    try:
        ret = requests.get("http://192.168.10.100:8088/get_pic?id={}&name=industry".format(id))
        with open(file_name, 'wb') as f:  # file_name 为本地图片要保存的路径
            f.write(ret.content)
        return True
    except:
        return False


# 设置工业相机的参数
def set_industry_parameter(Id, parameters):
    input_get = {'id': Id,  # 相机选择 *
                 'device_param': parameters,  # 参数选择 *
                 }
    set_parameter_url = "http://192.168.10.100:8088/change_devicePrama"
    img_data = requests.post(set_parameter_url, json=input_get)
    if img_data.status_code == 200:
        res_data = json.loads(img_data.content.decode("utf-8"))
        if res_data.get("statusCode") == 200:
            logger.info("工业相机{}参数设置成功".format(Id))
            return True
        logger.info(res_data.get("msg"))

    logger.info("工业相机{}参数错误或传参接口不同".format(Id))
    return False


# 调用AGX算法接口
def post_agx_light(photo_name, start_imgRows, end_imgRows,
                   start_imgCols, end_imgCols, show_data, type=0):
    image = []
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            image.append(img)
    input_get = {'picPathList': None,  # 图片路径 *
                 'picDataList': image,  # 原始图片数据 *
                 'picType': type,
                 'rois':
                     [
                         {
                             'startCol': start_imgCols,
                             'startRow': start_imgRows,
                             'endCol': end_imgCols,
                             'endRow': end_imgRows,
                             'legend': show_data,
                             'id': 1
                         }
                     ],  # 检测框
                 'isBlend': 0,  # 是否执行图片融合
                 'bNum': 1,
                 'isCrop': 0}

    # 访问服务
    try:
        ret = requests.post("http://192.168.10.134:8087/algoritmic_server/light", data=json.dumps(input_get))
        if ret.status_code == 200:
            data = json.loads(ret.text)
            print(data)
            light_detect_result = data['results'][0]["res"]
            light_path = data.get("resPath")
            narrow_path = data.get("narrowPath")
            return light_detect_result, light_path, narrow_path
        else:
            return None, None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None, None



# 语音接口
def post_voice_str(name_en_str, type=None):
    voice_start = 0
    if type == 1:
        name_str = name_en_str
    elif type == 2:
        voice_data = DBRobotVoice().get_by_name("到达路径点")
        # logger.info(voice_data)
        voice_str = voice_data.get("content")
        name_str = voice_str.replace("XX", name_en_str)
        voice_start = voice_data.get("is_play")
    else:
        voice_data = DBRobotVoice().get_by_name(voice_dict.get(name_en_str))
        # logger.info(voice_data)
        name_str = voice_data.get("content")
        voice_start = voice_data.get("is_play")

    if voice_start == 0:
        voice_action_state = RedisPipe("VoiceAction").get_data()
        # logger.info(voice_action_state)
        # logger.info(name_en_str)
        if voice_action_state:
            if eval(voice_action_state.decode('utf-8')).get("VoiceAction"):
                if name_en_str in ["obstacle", "battery_low", "urgent_stop"]:
                    voice_select(name_str)
                else:
                    logger.info("喇叭被占用，当前语音播报暂停")
            else:
                voice_client(name_str)
        else:
            voice_client(name_str)


def voice_select(name_str):
    # 推送打断信息
    voice_stop()
    time.sleep(1)
    data_dic = {"robotId": 1, "voiceState": 1, "voiceAction": 0}
    rout = db_rabbit_mq.get("rout_ex")
    routing_ey = db_rabbit_mq.get("rout_send_robotVoice")
    warn_queue = db_rabbit_mq.get("queue_send_robotVoice")
    logger.info("发送打断信息MQ数据！")
    logger.info(data_dic)

    RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)
    logger.info(name_str)

    voice_client(name_str)

    # 推送结束语音信息
    voice_start()

    data_dic = {"robotId": 1, "voiceState": 0, "voiceAction": 1}
    rout = db_rabbit_mq.get("rout_ex")
    routing_ey = db_rabbit_mq.get("rout_send_robotVoice")
    warn_queue = db_rabbit_mq.get("queue_send_robotVoice")
    logger.info("推送结束语音信息MQ数据！")
    logger.info(data_dic)
    RabbitPublisher.run(rout, routing_ey, warn_queue, data_dic)


# 数字仪表读数服务接口
def post_agx_digital(photo_name, u_params_list, type=1):
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            # image.append(img)

            input_get = {"Image": {'picPathStr': None,  # 图片路径 *
                                   'picData': img,  # 原始图片数据 *
                                   'picType': type,  # 图片类型（四倍1，工业2，web3）
                                   'rectangles': u_params_list}}

    try:
        ret = requests.post(
            "http://{}:{}/algoritmic_server/digit".format(pointer_ips.get("host"), pointer_ips.get("port")),
            data=json.dumps(input_get))
        if ret.status_code == 200:
            data = json.loads(ret.content.decode('utf-8'))
            logger.info("数字仪表算法返回结果")
            logger.info(data)
            if data.get("code") != "1":
                logger.info(data.get("msg"))
                return None, None
            light_detect_result = data.get("result")  # 解析结果
            # light_path = data.get("resPath")  # 图片存储路径
            narrow_path = data.get("narrowPath")  # 缩略路径
            return light_detect_result, narrow_path
        else:
            logger.error("访问算法接口没有成功！")
            return None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None


# 开关服务接口
def post_agx_switch(photo_name, u_params_list, type=1):
    try:
        rectangles = u_params_list[0]
        # rectangles["legend"] = u_params_list[0].get("legend")
        rectangles["legend"] = "switch_status"

    except Exception as e:
        logger.error(e)
        logger.error("传入开关算法数据正确")
        return None, None
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            rswitchKind = switch_status.get(u_params_list[0].get("rswitchKind"))
            rectangles.pop("rswitchKind")
            input_get = {'picPathStr': None,  # 图片路径 *
                         'picData': img,  # 原始图片数据 *
                         'picType': type,  # 图片类型（四倍1，工业2，web3）
                         # 检测的旋转开关的种类，⽤来读取配置⽂件中的相关参数。该参数必须准确， 空或者参数错误，返回的结果是报错。
                         'rswitchKind': rswitchKind,
                         'rectangle': rectangles}
    try:
        ret = requests.post(
            "http://{}:{}/algoritmic_server/rswitch".format(pointer_ips.get("host"), pointer_ips.get("port")),
            data=json.dumps(input_get))
        if ret.status_code == 200:
            data = json.loads(ret.content.decode('utf-8'))
            logger.info("开关服务算法返回结果")
            if data.get("code") != "1":
                logger.info(data.get("msg"))
                return None, None
            light_detect_result = data.get("result")  # 解析结果
            # light_path = data.get("resPath")  # 图片存储路径
            narrow_path = data.get("narrowPath")  # 缩略路径
            return light_detect_result, narrow_path
        else:
            logger.error("访问算法接口没有成功！")
            return None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None


# 指针仪表读数服务接口
def post_agx_pointer(photo_name, u_params_list, type=1):
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')

            input_get = {'picPathStr': photo_name,  # 图片路径 *
                         'picData': img,  # 原始图片数据 *
                         'picType': type,  # 图片类型（四倍1，工业2，web3）
                         'rectangles': u_params_list}
    try:
        ret = requests.post(
            "http://{}:{}/algoritmic_server/meter".format(pointer_ips.get("host"), pointer_ips.get("port")),
            data=json.dumps(input_get))
        if ret.status_code == 200:
            try:
                data = json.loads(ret.text, encoding="utf-8")
                logger.info("指针仪表算法返回结果")
                logger.info(data)
                if data.get("code") != "1":
                    logger.info(data.get("msg"))
                    return None, None
            except Exception as e:
                logger.error(e)
                return None, None
            light_detect_result = data.get("result")  # 解析结果
            # light_path = data.get("resPath")  # 图片存储路径
            narrow_path = data.get("narrowPath")  # 缩略路径
            return light_detect_result, narrow_path
        else:
            logger.error("访问算法接口没有成功！")
            return None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None


def post_video(state):
    input_get = {'action': state}
    # 访问服务
    try:
        ret = requests.get("http://192.168.10.134:8086/save_video", params=input_get, timeout=3)
        logger.info(ret.content)
        if ret.status_code == 200:
            if ret.content:
                data = eval(ret.content.decode('utf-8'))
                code_state = data.get("code")
                if code_state == 1:
                    return True, data.get("res_path")
                elif code_state == 0:
                    return True, data.get("res_path")
                else:
                    return False, None
            else:
                return None, None
        else:
            return None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问热成像接口错误")
        return None, None


def post_tem_photo():
    input_get = {'action': 1}
    # 访问服务
    try:
        ret = requests.get("http://192.168.10.134:8086/save_data", params=input_get, timeout=3)
        if ret.status_code == 200:
            data = eval(ret.content.decode('utf-8'))
            light_path = data.get("resPath")
            return light_path, None
        else:
            return None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问热成像接口错误")
        return None, None


def get_face_photo():
    input_get = {'action': 1}
    # 访问服务
    ret = requests.get("http://192.168.10.134:8086/face_regocation", params=input_get, timeout=30)
    try:
        if ret.status_code == 200:
            data = json.loads(ret.text)
            # logger.info("人脸接口返回值：{}".format(data))
            if data.get("code") == 0:
                face_path = data.get("resPath")
                res = data.get("res")
                return res, face_path, 0

            if data.get("code") == 1:
                res = data.get("res")
                face_path = data.get("resPath")
                return res, face_path, 1
            if data.get("code") == -1:
                logger.info("请求参数错误")
                return None, None, None
            if data.get("code") == -2:
                logger.info("图片错误")
                return None, None, None
            if data.get("code") == -3:
                # logger.info("图片没有检测出人脸")
                return None, None, None
            if data.get("code") == -4:
                logger.info("检测出多张人脸，但无法识别")
                return None, None, None
            if data.get("code") == -5:
                logger.info("检测/识别出错")
                return None, None, None
            else:
                return None, None, None
        else:
            return None, None, None
    except Exception as e:
        logger.error(e)


if __name__ == '__main__':
    # path_img = "/opt/moss_robot/lib/dispatch_v5.2/data_pkg/photo_path/2021_08_26/16_31_15.jpg"
    # a = post_agx_light(path_img, 1, 5000, 1, 5000, "P-00 U-00 D-00", 1)
    # import rospy
    #
    # rospy.init_node("Test", disable_signals=False)
    # post_voice_str('到达动力机柜后方', 2)
    # # post_video(1)
    # # time.sleep(10)
    # # post_video(0)
    # name_str = DBRobotVoice().get_by_name(voice_dict.get("obstacle"))
    # print(name_str)
    light_path, aaa = post_tem_photo()
    print(light_path, aaa)