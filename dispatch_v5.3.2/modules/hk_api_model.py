# -*- coding:utf-8 -*-
# @Author: WangL
# @file:  hk_api_model.py
# @Time: 2021/12/1 14:20
# @Description: 将海康设备调用添加至IPS,这里提供API接口调用

import os
import sys
import time
import uuid
import json
import base64
import requests
import traceback
import subprocess
import logging
from xml.dom.minidom import parseString
from requests.auth import HTTPDigestAuth
sys.path.append("/opt/moss_robot/lib/dispatch_v5.3.2/lib/")
from configs.log import logger
from lib.hikSDK import NET_DVR_Login_V30, Preview, NET_DVR_Logout_V30, NET_DVR_Cleanup, NET_DVR_PTZControl_Other, \
    NET_DVR_PTZPreset_Other, Get_JPEGpicture, FOCUSMODE, AEMODECFG, callCpp
# from hikSDK import NET_DVR_Login_V30, Preview, NET_DVR_Logout_V30, NET_DVR_Cleanup, NET_DVR_PTZControl_Other, \
#     NET_DVR_PTZPreset_Other, Get_JPEGpicture, FOCUSMODE, AEMODECFG, callCpp

IP_Upper = "192.168.10.212"  # 四倍相机——上
IP_Lower = "192.168.10.211"  # 四倍相机——下
IP_Ptz = "192.168.10.210"  # 主云台
PORT = 8000
username = "admin"
password = "Admin123"
ID = 1  # 通道号
filename = str(time.strftime('%Y-%m-%d_%H%I%M%S', time.localtime(time.time()))) + ".jpg"
videoname = str(time.strftime('%Y-%m-%d_%H%I%M', time.localtime(time.time()))) + ".mp4"
# StreamingUrl = "http://" + IP_Ptz + ":80/ISAPI/Streaming/channels/" + str(ID)
Header = {'Connection': 'keep-alive', 'Content-Type': 'application/x-www-form-urlencoded; charset=UTF-8'}


# 根据类型获取海康设备地址
def get_url(type):
    if int(type) == 0:
        ip = IP_Ptz
    elif int(type) == 1:
        ip = IP_Upper
    else:
        ip = IP_Lower
    # ImageUrl = "http://" + ip + ":80/ISAPI/Image/channels/" + str(ID)
    ImageUrl = "http://{}:80/ISAPI/Image/channels/{}".format(ip, str(ID))
    return ImageUrl


# 用户认证
def digest(url):
    time.sleep(2)
    auth = HTTPDigestAuth(username, password)
    return auth


# 四倍相机拍照
def take_picture(camera_direction):
    if camera_direction == "上":
        ip = IP_Upper
    elif camera_direction == "下":
        ip = IP_Lower
    else:
        ip = IP_Ptz
    StreamingUrl = "http://" + ip + ":80/ISAPI/Streaming/channels/" + str(ID)
    url = StreamingUrl + "/picture"
    auth = digest(url)
    response = requests.get(url, auth=auth)
    logger.info(response.status_code)
    photo_path = "/opt/moss_robot/lib/dispatch_ips_web/data_pkg/template_pic/"
    if not os.path.exists(photo_path):
        os.makedirs(photo_path)
    if response.status_code == 200:
        file_path = photo_path + str(uuid.uuid1()) + '.jpg'
        with open(file_path, 'wb') as f:
            f.write(response.content)
            return file_path
    return False


# 获取曝光模式：manual为手动（0），auto为自动（1），IrisFirst为光圈优先（2），ShutterFirst为快门优先（3）。
def get_exposure(type):
    try:
        url = get_url(type) + "/exposure"
        auth = digest(url)
        xml_data = requests.get(url, auth=auth).content
        exposure_data = parseString(xml_data)
        data_tree = exposure_data.documentElement
        exposure_result = data_tree.getElementsByTagName("ExposureType")[0].childNodes[0].data
        return str(exposure_result)
    except Exception as e:
        logger.error(e)
    return None


# 设置曝光模式：manual为手动（0），auto为自动（1），IrisFirst为光圈优先（2），ShutterFirst为快门优先（3）。
def set_exposure(val, type):
    try:
        mode_list = ["manual", "auto", "IrisFirst", "ShutterFirst"]
        mode = mode_list[int(val)]
        url = get_url(type) + "/exposure"
        auth = digest(url)
        body = '''<?xml version: "1.0" encoding="UTF-8"?><Exposure><ExposureType>{}</ExposureType></Exposure>'''.format(
            mode)
        response = requests.put(url, data=body, auth=auth, headers=Header)
        if response.status_code == 200:
            return True
    except Exception as e:
        logger.error(e)
    return False


# 获取光圈
def get_iris(type):
    try:
        url = get_url(type) + "/iris"
        auth = digest(url)
        xml_data = requests.get(url, auth=auth).content
        iris_data = parseString(xml_data)
        data_tree = iris_data.documentElement
        iris_result = data_tree.getElementsByTagName("IrisLevel")[0].childNodes[0].data
        return str(iris_result)
    except Exception as e:
        logger.error(e)
    return None


# 设置光圈
def set_iris(val, type):
    try:
        url = get_url(type) + "/iris"
        auth = digest(url)
        body = '''<?xml version: "1.0" encoding="UTF-8"?><Iris><IrisLevel>{}</IrisLevel></Iris>'''.format(str(val))
        response = requests.put(url, data=body, auth=auth, headers=Header)
        if response.status_code == 200:
            return True
    except Exception as e:
        logger.error(e)
    return False


# 获取快门
def get_shutter(type):
    try:
        url = get_url(type) + "/shutter"
        auth = digest(url)
        xml_data = requests.get(url, auth=auth).content
        shutter_data = parseString(xml_data)
        data_tree = shutter_data.documentElement
        shutter_result = data_tree.getElementsByTagName("ShutterLevel")[0].childNodes[0].data
        logger.info(str(shutter_result))
        return str(shutter_result)
    except Exception as e:
        logger.error(e)
    return None


# 设置快门: 快门映射值
ShutterData = {
    "17": "1/25", "19": "1/50", "21": "1/75", "23": "1/100", "24": "1/120", "26": "1/150", "50": "1/175", "28": "1/200",
    "52": "1/225", "30": "1/250", "31": "1/300", "33": "1/425", "35": "1/600", "37": "1/1000", "38": "1/1250",
    "40": "1/1750", "42": "1/2500", "44": "1/3500", "46": "1/6000", "47": "1/10000", "48": "1/30000"
}


# 设置快门
def set_shutter(val, type):
    try:
        shutter_val = ShutterData[str(val)]
        url = get_url(type) + "/shutter"
        auth = digest(url)
        body = '''<?xml version: "1.0" encoding="UTF-8"?><Shutter><ShutterLevel>{}</ShutterLevel></Shutter>'''.format(
            shutter_val)
        response = requests.put(url, data=body, auth=auth, headers=Header)
        if response.status_code == 200:
            return True
    except Exception as e:
        logger.error(e)
    return False


# 获取增益
def get_gain(type):
    try:
        url = get_url(type) + "/gain"
        auth = digest(url)
        xml_data = requests.get(url, auth=auth).content
        gain_data = parseString(xml_data)
        data_tree = gain_data.documentElement
        gain_level = data_tree.getElementsByTagName("GainLevel")[0].childNodes[0].data
        gain_limit = data_tree.getElementsByTagName("GainLimit")[0].childNodes[0].data
        return gain_level, gain_limit
    except Exception as e:
        logger.error(e)
    return None, None


# 设置增益 1-15
def set_gain(val, type):
    try:
        url = get_url(type) + "/gain"
        logger.info(url)
        auth = digest(url)
        gain_level, gain_limit = get_gain(type)
        body = '''<?xml version: "1.0" encoding="UTF-8"?><Gain><GainLevel>{}</GainLevel><GainLimit>{}</GainLimit></Gain>'''.format(
            val, gain_limit)
        response = requests.put(url, data=body, auth=auth, headers=Header)
        logger.info(response.status_code)
        if response.status_code == 200:
            return True
    except Exception as e:
        logger.error(e)
    return False


# 获取聚焦模式和最小聚焦距离：MANUAL为手动（1），AUTO为自动（0），SEMIAUTOMATIC为半自动（2）
def get_focus_configuration(type):
    try:
        url = get_url(type) + "/focusConfiguration"
        auth = digest(url)
        xml_data = requests.get(url, auth=auth).content
        focus_configuration_data = parseString(xml_data)
        data_tree = focus_configuration_data.documentElement
        focus_style = data_tree.getElementsByTagName("focusStyle")[0].childNodes[0].data
        focus_limited = data_tree.getElementsByTagName("focusLimited")[0].childNodes[0].data
        return str(focus_style), str(focus_limited)  # 聚焦模式,最小聚焦距离
    except Exception as e:
        logger.error(e)
    return None, None


# 设置聚焦模式: MANUAL为手动（1），AUTO为自动（0），SEMIAUTOMATIC为半自动（2）
def set_focus_style(val, type):
    try:
        mode_list = ["AUTO", "MANUAL", "SEMIAUTOMATIC"]
        focus_style = mode_list[int(val)]
        logger.info(focus_style)
        url = get_url(type) + "/focusConfiguration"
        auth = digest(url)
        # 获取最小聚焦距离
        focus_limited = get_focus_configuration(type)[1]
        body = '''<?xml version: "1.0" encoding="UTF-8"?><FocusConfiguration><focusStyle>{}</focusStyle><focusLimited>{}</focusLimited></FocusConfiguration>'''.format(
            focus_style, focus_limited)
        response = requests.put(url, data=body, auth=auth, headers=Header)
        if response.status_code == 200:
            return True
    except Exception as e:
        print(e)
        logger.error(e)
    return False


# 设置最小聚焦距离
def set_focus_limited(val, type):
    try:
        url = get_url(type) + "/focusConfiguration"
        auth = digest(url)
        # 获取聚焦模式
        focus_style = get_focus_configuration(type)[0]
        focus_limited = str(val)
        body = '''<?xml version: "1.0" encoding="UTF-8"?><FocusConfiguration><focusStyle>{}</focusStyle><focusLimited>{}</focusLimited></FocusConfiguration>'''.format(
            focus_style, focus_limited)
        response = requests.put(url, data=body, auth=auth, headers=Header)
        if response.status_code == 200:
            return True
    except Exception as e:
        logger.error(e)
    return False


# 获取变倍limit
def get_zoom(type):
    try:
        url = "http://192.168.9.120:1080/ISAPI/System/deviceInfo"
        auth = digest(url)
        url1 = "http://admin:Admin123@192.168.9.120:1080/ISAPI/System/Serial/ports/1/Transparent/channels/1/open"
        auth1 = digest(url1)
        xml_data = requests.get(url, auth=auth1).content
        zoom_limit_data = parseString(xml_data)
        data_tree = zoom_limit_data.documentElement
        zoom_limit_ratio = data_tree.getElementsByTagName("zoom")[0].childNodes[0].data
        return str(zoom_limit_ratio)
    except Exception as e:
        logger.error(e)
    return False


# 设置变倍值limit
def set_zoom(val, type):
    try:
        url = "http://192.168.9.120:1080/ISAPI/System/deviceInfo"
        auth = digest(url)
        url1 = "http://admin:Admin123@192.168.9.120:1080/ISAPI/System/Serial/ports/1/Transparent/channels/1/open"
        auth1 = digest(url1)
        body = '''okokok\r\n,<?xml version=\"1.0\" encoding=\"UTF-8\" ?><PTZData version=\"2.0\" xmlns=\"http://www.isapi.org/ver20/XMLSchema\"><pan>0</pan><tilt>0</tilt><zoom>{}</zoom></PTZData>"'''.format(
            str(val))
        response = requests.put(url1, data=body, auth=auth, headers=Header)
        if response.status_code == 200:
            return True
    except Exception as e:
        logger.error(e)
    return False


# 获取变倍限制
def get_zoom_limit(type):
    try:
        url = get_url(type) + "/zoomLimit"
        auth = digest(url)
        xml_data = requests.get(url, auth=auth).content
        zoom_limit_data = parseString(xml_data)
        data_tree = zoom_limit_data.documentElement
        zoom_limit_ratio = data_tree.getElementsByTagName("ZoomLimitRatio")[0].childNodes[0].data
        return str(zoom_limit_ratio)
    except Exception as e:
        logger.error(e)
    return None


# 获取全部设置的值

def get_all(params, type):
    results = dict()
    try:
        if params == "exposureMode":
            results["exposureMode"] = get_exposure(type)
        elif params == "iris":
            results["iris"] = get_iris(type)
        elif params == "shutter":
            results["shutter"] = get_shutter(type)
        elif params == "gain":
            results["gain"] = get_gain(type)[0]
        elif params == "mode":
            results["mode"] = get_focus_configuration(type)[0]
        elif params == "distance":
            results["distance"] = get_focus_configuration(type)[1]
        elif params == "zoom":
            results["zoom"] = get_camera_zoom()
        else:
            results["exposureMode"] = get_exposure(type)
            results["iris"] = get_iris(type)
            results["shutter"] = get_shutter(type)
            results["gain"] = get_gain(type)[0]
            results["mode"] = get_focus_configuration(type)[0]
            results["distance"] = get_focus_configuration(type)[1]
            results["zoom"] = get_camera_zoom()
        return results
    except Exception as e:
        logger.error(e)
    return None


# ================================================= 原海康SDK方式 =================================================
def login(type):
    """
    海康设备登录函数
    """
    if int(type) == 0:
        ip = IP_Ptz
    elif int(type) == 1:
        ip = IP_Upper
    else:
        ip = IP_Lower
    global m_lRealHandle
    try:
        NET_DVR_Logout_V30()
        NET_DVR_Cleanup()
        time.sleep(0.5)
        # 执行登陆操作

        UserID = NET_DVR_Login_V30(str(ip), int(PORT), str(username), str(password))
        # 相机预览
        if UserID == -1:
            return False
        m_lRealHandle = Preview()
        return True
    except:
        return False


def req_ptz_always(index, button, speed):
    """
    index表示宏定义指令；
    button表示开关--0表示执行，1表示关闭。
    """
    # ============== 创建云台控制对象 =================
    PTZControl = NET_DVR_PTZControl_Other()
    # 执行云台下俯命令，2S后停止
    if not speed:
        speed = 4
    res = PTZControl.get_PTZControl(int(index), int(button), int(speed))
    if res:
        return True
    return False


def get_camera_params():
    """
    获取摄像头参数值
    """
    # 创建聚焦对象
    focus = FOCUSMODE()
    # 获取聚焦模式   # 0-自动，1-手动，2-半自动
    mode = focus.get_focusMode()
    # 获取最小聚焦距离     # 范围10-2000
    distance = focus.get_minFocusDistance()
    # 获取光学变倍值      # 范围 1-4
    try:
        zoom_num = focus.get_CamZoom()
        zoom = round(zoom_num, 2)
    except Exception as e:
        logger.error(e)
        zoom = 1.0

    # 创建曝光对象
    ae_mode = AEMODECFG()
    # 获取增益值
    gain = ae_mode.get_Gain()  # 取值范围为0-15
    # 获取光圈值
    iris = ae_mode.get_Iris()  # 实际值*100
    # 获取快门等级
    shutter = ae_mode.get_Shutter()  # 为列表形式
    # 获取曝光模式
    exposure_mode = ae_mode.get_exposureMode()  # 0-手动模式，1-自动曝光，2-光圈优先，3-快门优先，4-增益优先
    device_params_dict = {
        "focusParams": {
            "focusMode": mode,
            "minFocusDistance": distance,
            "opticalZoomLevel": zoom
        },
        "exposureParams": {
            "gain": gain,
            "iris": iris,
            "shutter": shutter,
            "exposureMode": exposure_mode
        }
    }
    return device_params_dict


def set_camera_params(param_type, param, param_value):
    logger.info(param_type)
    logger.info(param)
    logger.info(param_value)
    try:
        if param_type == 'focus':
            # 创建聚焦对象
            focus = FOCUSMODE()
            if param == 'mode':
                # 获取聚焦模式   # 0-自动，1-手动，2-半自动
                mode = focus.Change_focusMode(int(param_value))
            elif param == 'distance':
                # 获取最小聚焦距离     # 范围10-2000
                distance = focus.Change_minFocusDistance(int(param_value))

        elif param_type == 'ae_mode':
            # 创建曝光对象
            ae_mode = AEMODECFG()
            exposure_mode = ae_mode.get_exposureMode()
            if exposure_mode == 0:
                if param == 'gain':
                    # 设置增益值
                    gain = ae_mode.Change_Gain(int(param_value))  # 取值范围为0-15
                elif param == 'iris':
                    # 获取光圈值
                    iris = ae_mode.Change_Iris(int(param_value) * 100)  # 实际值*100
                elif param == 'shutter':
                    # 获取快门等级
                    shutter = ae_mode.Change_Shutter(ShutterData[str(param_value)])  # 为列表形式
                elif param == 'exposureMode':
                    # 获取曝光模式
                    exposure_mode = ae_mode.Change_exposureMode(int(param_value))  # 0-手动模式，1-自动曝光，2-光圈优先，3-快门优先，4-增益优先
                    pass
            elif exposure_mode == 1:
                if param == 'exposureMode':
                    # 获取曝光模式
                    exposure_mode = ae_mode.Change_exposureMode(int(param_value))  # 0-手动模式，1-自动曝光，2-光圈优先，3-快门优先，4-增益优先
                else:
                    logger.info('自动模式下，参数不可设置')
        return True
    except Exception as e:
        logger.error(e)
        return False


def hk_all_api(param_value, param, param_type):
    """
    海康设备设置函数
    """
    try:
        if param in ["exposureMode", "iris", "shutter", "gain"]:
            # 设置聚焦参数
            result = set_camera_params("ae_mode", param, param_value)
        elif param in ["mode", "distance"]:
            # 设置曝光参数
            result = set_camera_params("focus", param, param_value)
        elif param == "login":
            logger.info("登录")
            result = login(type=param_type)
        elif param == "zoom_add_on":
            # 设置变倍zoom(调焦+启动)
            result = req_ptz_always(11, 0, param_value)
        elif param == "zoom_add_off":
            # 设置变倍zoom(调焦+停止)
            result = req_ptz_always(11, 1, param_value)
        elif param == "zoom_subtract_on":
            # 设置变倍zoom(调焦-启动)
            result = req_ptz_always(12, 0, param_value)
        elif param == "zoom_subtract_off":
            # 设置变倍zoom(调焦-停止)
            result = req_ptz_always(12, 1, param_value)
        elif param == "focus_add_on":
            # 设置变倍zoom(聚焦+启动)
            result = req_ptz_always(13, 0, param_value)
        elif param == "focus_add_off":
            # 设置变倍zoom(聚焦+停止)
            result = req_ptz_always(13, 1, param_value)
        elif param == "focus_subtract_on":
            # 设置变倍zoom(聚焦-启动)
            result = req_ptz_always(14, 0, param_value)
        elif param == "focus_subtract_off":
            # 设置变倍zoom(聚焦-停止)
            result = req_ptz_always(14, 1, param_value)
        elif param == "iris_add_on":
            # 设置变倍zoom(光圈+启动)
            result = req_ptz_always(15, 0, param_value)
        elif param == "iris_add_off":
            # 设置变倍zoom(光圈+停止)
            result = req_ptz_always(15, 1, param_value)
        elif param == "iris_subtract_on":
            # 设置变倍zoom(光圈-启动)
            result = req_ptz_always(16, 0, param_value)
        elif param == "iris_subtract_off":
            # 设置变倍zoom(光圈-停止)
            result = req_ptz_always(16, 1, param_value)
        elif param == "U":
            result = req_ptz_always(21, 0, param_value)
        elif param == "SU":
            result = req_ptz_always(21, 1, param_value)
        elif param == "D":
            result = req_ptz_always(22, 0, param_value)
        elif param == "SD":
            result = req_ptz_always(22, 1, param_value)
        elif param == "L":
            result = req_ptz_always(23, 0, param_value)
        elif param == "SL":
            result = req_ptz_always(23, 1, param_value)
        elif param == "R":
            result = req_ptz_always(24, 0, param_value)
        elif param == "SR":
            result = req_ptz_always(24, 1, param_value)
        else:
            return False
    except Exception as e:
        logger.error(e)
        return False
    return True


if __name__ == '__main__':
    a = login(type=0)
    req_ptz_always(22, 1, 7)
