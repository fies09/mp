#!/usr/bin/python3
# -*- coding:utf-8 -*-

# Response
import os
import time
import json
from configs.log import logger
from lib.hikSDK import NET_DVR_Login_V30, Preview, NET_DVR_Logout_V30, NET_DVR_Cleanup, Get_JPEGpicture, FOCUSMODE, AEMODECFG, callCpp

focusPramas_dict = {}
exposureParams_dict = {}
deviceParams_dict = {}

m_lRealHandle = {}


def getDevicePrama():
    # 创建聚焦对象
    focus = FOCUSMODE()
    # 获取聚焦模式   # 0-自动，1-手动，2-半自动
    mode = focus.get_focusMode()
    # 获取最小聚焦距离     # 范围10-2000
    distance = focus.get_minFocusDistance()
    # 获取光学变倍值      # 范围 1-4
    camZoom = focus.get_CamZoom()

    focusPramas_dict.update(focusMode=mode)
    focusPramas_dict.update(minFocusDistance=distance)
    focusPramas_dict.update(opticalZoomLevel=camZoom)
    deviceParams_dict.update(focusParams=focusPramas_dict)
    # 创建曝光对象
    AEmode = AEMODECFG()
    # 获取增益值
    gain = AEmode.get_Gain()  # 取值范围为0-15
    # 获取光圈值
    iris = AEmode.get_Iris()  # 实际值*100
    # 获取快门等级
    shutter = AEmode.get_Shutter()  # 为列表形式
    # 获取曝光模式
    exposureMode = AEmode.get_exposureMode()  # 0-手动模式，1-自动曝光，2-光圈优先，3-快门优先，4-增益优先
    exposureParams_dict.update(gain=gain)
    exposureParams_dict.update(iris=iris)
    exposureParams_dict.update(shutter=shutter)
    exposureParams_dict.update(exposureMode=exposureMode)
    deviceParams_dict.update(exposureParams=exposureParams_dict)
    return deviceParams_dict


# @app.route('/get_devicePrama/',methods=['GET','POST'])
def get_devicePrama():
    result = getDevicePrama()
    result_text = {"data": result, "statusCode": 200, "message": "success"}
    return json.dumps(result_text)


def set_devicePrama(mode, distance, opticalZoomLevel, gain, iris, shutter, exposureMode):
    # 创建聚焦对象
    focus = FOCUSMODE()

    # 获取聚焦模式   # 0-自动，1-手动，2-半自动
    if mode:
        mode = focus.Change_focusMode(mode)
        focusPramas_dict.update(focusMode=mode)
    # 获取最小聚焦距离     # 范围10-2000
    if distance:
        distance = focus.Change_minFocusDistance(distance)
        focusPramas_dict.update(minFocusDistance=distance)

    # 获取光学变倍值      # 范围 1-4 ，最小修改为0.5
    if opticalZoomLevel:
        camZoom = focus.Change_CamZoom(opticalZoomLevel)
        focusPramas_dict.update(opticalZoomLevel=camZoom)
    if focusPramas_dict:
        deviceParams_dict.update(focusParams=focusPramas_dict)
    # 创建曝光对象
    AEmode = AEMODECFG()
    # 获取增益值
    if gain:
        gain = AEmode.Change_Gain(gain)  # 取值范围为0-15
    # 获取光圈值
    if iris:
        iris = AEmode.Change_Iris(iris)  # 实际值*100
    # 获取快门等级
    if shutter:
        shutter = AEmode.Change_Shutter(shutter)  # 为列表形式

    # 获取曝光模式
    if exposureMode:
        exposureMode = AEmode.Change_exposureMode(exposureMode)  # 0-手动模式，1-自动曝光，2-光圈优先，3-快门优先，4-增益优先

    if exposureParams_dict:
        exposureParams_dict.update(gain=gain)
        exposureParams_dict.update(iris=iris)
        exposureParams_dict.update(shutter=shutter)
        exposureParams_dict.update(exposureMode=exposureMode)
        deviceParams_dict.update(exposureParams=exposureParams_dict)
    return deviceParams_dict


def changeDevicePrama(pramaType, prama, prama_value):
    if pramaType == 'focus':
        # 创建聚焦对象
        focus = FOCUSMODE()
        if prama == 'mode':
            # 获取聚焦模式   # 0-自动，1-手动，2-半自动
            mode = focus.Change_focusMode(prama_value)
        elif prama == 'distance':
            # 获取最小聚焦距离     # 范围10-2000
            distance = focus.Change_minFocusDistance(prama_value)
            pass
        pass

    elif pramaType == 'AEmode':
        # 创建曝光对象
        AEmode = AEMODECFG()
        exposureMode = AEmode.get_exposureMode()
        if exposureMode == 0:
            if prama == 'gain':
                # 设置增益值
                gain = AEmode.Change_Gain(prama_value)  # 取值范围为0-15
            elif prama == 'iris':
                # 获取光圈值
                iris = AEmode.Change_Iris(prama_value)  # 实际值*100
            elif prama == 'shutter':
                # 获取快门等级
                shutter = AEmode.Change_Shutter(prama_value)  # 为列表形式
            elif prama == 'exposureMode':
                # 获取曝光模式
                exposureMode = AEmode.Change_exposureMode(prama_value)  # 0-手动模式，1-自动曝光，2-光圈优先，3-快门优先，4-增益优先
                pass
        elif exposureMode == 1:
            if prama == 'exposureMode':
                # 获取曝光模式
                exposureMode = AEmode.Change_exposureMode(prama_value)  # 0-手动模式，1-自动曝光，2-光圈优先，3-快门优先，4-增益优先
            else:
                print('自动模式下，参数不可设置')
            pass
        pass
    pass


# @app.route('/get_cameraZoom/',methods=['GET','POST'])
def getCameraZoom():
    # 创建聚焦对象
    focus = FOCUSMODE()
    cameraZoom = focus.get_CamZoom()
    result_text = {"data": cameraZoom, "statusCode": 200, "message": "success"}
    return json.dumps(result_text)


def loging(ip, port, username, password):
    global m_lRealHandle
    try:
        NET_DVR_Logout_V30()
        NET_DVR_Cleanup()
        time.sleep(0.5)
        # 执行登陆操作
        UserID = NET_DVR_Login_V30(str(ip), int(port), str(username), str(password))
        # 相机预览
        if UserID == -1:
            return 0
        m_lRealHandle = Preview()
        return 1
    except:
        return 0


#  =============抓图控制接口 ================
def get_pic(file_name):
    # changeDevicePrama()
    Get_JPEGpicture(file_name)
    return "1"


#  =============随工录像接口 ================
def savevideo(voide_file_name, voide_state):
    '''
        cvoide_state为录像控制命令：start为开始，end为结束；
        voide_file_name表示录像名字（包括地址）。
    '''
    if voide_state == 'start':
        video_posi = bytes(voide_file_name, "ascii")
        try:
            callCpp("NET_DVR_SaveRealData", m_lRealHandle, video_posi)
            logger.info("开始录像")
            return True
        except Exception as e:
            logger.error(e)
            return False
    elif voide_state == 'end':
        try:
            callCpp("NET_DVR_StopRealPlay", m_lRealHandle)
            # output_voide_file_name = voide_file_name.split(".mp4")[0] + '_process.mp4'

            # trans_state = os.popen("ffmpeg -i {} -c:v libx264 -strict -2 -s 800x480 -b 1000k {}".format(voide_file_name,output_voide_file_name))
            a = change_voide(voide_file_name)
            logger.info(a)
            logger.info(output_voide_file_name)
            if trans_state == 0:
                logger.info("已经结束录像")
                return True
            return False
        except Exception as e:
            logger.error(e)
            return False


def change_voide(name):
    try:
        output_voide_file_name = name.split(".mp4")[0] + '_process.mp4'
        trans_state = os.popen(
            "ffmpeg -i {} -c:v libx264 -strict -2 -s 800x480 -b 1000k {}".format(name, output_voide_file_name))
        return True
    except:
        return False


if __name__ == '__main__':
    a = loging('192.168.10.210', '8000', 'admin', 'Admin123')
    b = savevideo('savavideo.mp4', 'start')
    time.sleep(10)
    c = savevideo('savavideo.mp4', 'end')

# savevideo('start', 'voide/', 'a3')
# time.sleep(20)
# savevideo('end', 'voide/', 'a7')
