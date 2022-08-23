# !/usr/bin/python3
# -*- coding:utf-8 -*-
"""
@file: manage1.py
@desc:主摄像头功能和服务
①实现云台方向控制：含单击和常按，预置位设置、调用、删除；
②摄像头抓图、指示灯算法参数、效果图；
③随工录像；
④视频流实时推送-- 通过摄像头。
@author: hugoway
@time: 2020/3/23 09:31
"""

import os
import ctypes
import time
# from flask import Flask

import sys

# app=Flask(__name__)
from configs.log import logger

UserID = 0
Channel = 1

# 获取所有的库文件到一个列表
path = "./lib"


def file_name(file_dir):
    pathss = []
    for root, dirs, files in os.walk(file_dir):
        for file in files:
            pathss.append("./" + file)
    return pathss


dll_list = file_name(path)


# ===========定义调用c++库函数===============
def callCpp(func_name, *args):
    logger.info(os.getcwd())
    if os.getcwd().split("/")[-1] != "lib":
        os.chdir("./lib")
    for HK_dll in dll_list:
        try:
            # 将动态库加入工作目录
            lib = ctypes.cdll.LoadLibrary(HK_dll)
            try:
                # value = getattr(lib, func_name)(*args)
                value = eval("lib.%s" % func_name)(*args)
                if os.getcwd().split("/")[-1] == "lib":
                    os.chdir("../")
                return value
            except Exception as e:
                # logger.error(e)
                continue
        except Exception as e:
            # logger.error(e)
            continue
    if os.getcwd().split("/")[-1] == "lib":
        os.chdir("../")
    return False


# region 登入
# 定义登入结构体
class LPNET_DVR_DEVICEINFO_V30(ctypes.Structure):
    _fields_ = [
        ("sSerialNumber", ctypes.c_byte * 48),
        ("byAlarmInPortNum", ctypes.c_byte),
        ("byAlarmOutPortNum", ctypes.c_byte),
        ("byDiskNum", ctypes.c_byte),
        ("byDVRType", ctypes.c_byte),
        ("byChanNum", ctypes.c_byte),
        ("byStartChan", ctypes.c_byte),
        ("byAudioChanNum", ctypes.c_byte),
        ("byIPChanNum", ctypes.c_byte),
        ("byZeroChanNum", ctypes.c_byte),
        ("byMainProto", ctypes.c_byte),
        ("bySubProto", ctypes.c_byte),
        ("bySupport", ctypes.c_byte),
        ("bySupport1", ctypes.c_byte),
        ("bySupport2", ctypes.c_byte),
        ("wDevType", ctypes.c_uint16),
        ("bySupport3", ctypes.c_byte),
        ("byMultiStreamProto", ctypes.c_byte),
        ("byStartDChan", ctypes.c_byte),
        ("byStartDTalkChan", ctypes.c_byte),
        ("byHighDChanNum", ctypes.c_byte),
        ("bySupport4", ctypes.c_byte),
        ("byLanguageType", ctypes.c_byte),
        ("byVoiceInChanNum", ctypes.c_byte),
        ("byStartVoiceInChanNo", ctypes.c_byte),
        ("byRes3", ctypes.c_byte * 2),
        ("byMirrorChanNum", ctypes.c_byte),
        ("wStartMirrorChanNo", ctypes.c_uint16),
        ("byRes2", ctypes.c_byte * 2)]


# linux系统中需要调用该函数增加到初始化方案中
def Init():
    strPathCom = "./lib"
    ptrByteArraySsl = "./lib/libssl.so"
    ptrByteArrayCrypto = "./lib/libcrypto.so"
    struComPath = callCpp("NET_DVR_LOCAL_SDK_PATH", strPathCom)
    callCpp("NET_DVR_SetSDKInitCfg", 2, struComPath)
    callCpp("NET_DVR_SetSDKInitCfg", 3, ptrByteArrayCrypto)
    callCpp("NET_DVR_SetSDKInitCfg", 4, ptrByteArraySsl)
    pass


def NET_DVR_Login_V30(sDVRIP, wDVRPort, sUserName, sPassword):
    NET_DVR_Logout_V30()
    Init()
    init_res = callCpp("NET_DVR_Init")  # SDK初始化
    if init_res:
        logger.info("SDK初始化成功")
        error_info = callCpp("NET_DVR_GetLastError")
    else:
        error_info = callCpp("NET_DVR_GetLastError")
        # logger.error("SDK初始化错误：" + str(error_info))
        return False

    set_overtime = callCpp("NET_DVR_SetConnectTime", 5000, 4)  # 设置超时
    if set_overtime:
        pass
        # logger.info("设置超时时间成功")
    else:
        error_info = callCpp("NET_DVR_GetLastError")
        logger.error("设置超时错误信息：" + str(error_info))
        return False

    # 用户注册设备
    # c++传递进去的是byte型数据，需要转成byte型传进去，否则会乱码
    sDVRIP = bytes(sDVRIP, "ascii")
    sUserName = bytes(sUserName, "ascii")
    sPassword = bytes(sPassword, "ascii")
    DeviceInfo = LPNET_DVR_DEVICEINFO_V30()
    UserID = callCpp("NET_DVR_Login_V30", sDVRIP, wDVRPort, sUserName, sPassword, ctypes.byref(DeviceInfo))

    logger.info("登录成功，用户ID：" + str(UserID))
    if UserID == -1:
        error_info = callCpp("NET_DVR_GetLastError")
        logger.error("登录错误信息：" + str(error_info))
        return UserID
    else:
        return UserID


# endregion

# 释放SDK 资源
def NET_DVR_Cleanup():
    res = callCpp("NET_DVR_Cleanup")
    if res < 0:
        logger.info("SDK资源释放失败:" + str(callCpp("NET_DVR_GetLastError")))
    else:
        logger.info("SDK资源成功释放！！")


# 用户注销
def NET_DVR_Logout_V30():
    if callCpp("NET_DVR_Logout_V30", UserID):
        logger.info("用户已经成功注销")
    else:
        pass
        # logger.info("注销失败：" + str(callCpp("NET_DVR_GetLastError")))
    init_res = callCpp("NET_DVR_Cleanup")  # 释放SDK资源
    if init_res:
        logger.info("SDK资源释放成功")
    else:
        error_info = callCpp("NET_DVR_GetLastError")
        # logger.info("错误信息：" + str(error_info))
        return False


# region 预览
# 定义预览结构体
class NET_DVR_PREVIEWINFO(ctypes.Structure):
    _fields_ = [
        ("Channel", ctypes.c_long),
        ("lLinkMode", ctypes.c_long),
        ("hPlayWnd", ctypes.c_void_p),
        ("sMultiCastIP", ctypes.c_char_p),
        ("byProtoType", ctypes.c_byte),
        ("byRes", ctypes.c_byte * 3)]


# 预览实现
def Preview():
    lpPreviewInfo = NET_DVR_PREVIEWINFO()
    # hPlayWnd需要输入创建图形窗口的handle,没有输入无法实现BMP抓图
    lpPreviewInfo.hPlayWnd = None
    lpPreviewInfo.Channel = 1
    lpPreviewInfo.dwLinkMode = 0
    lpPreviewInfo.sMultiCastIP = None
    m_lRealHandle = callCpp("NET_DVR_RealPlay_V30", UserID, ctypes.byref(lpPreviewInfo), None, None, True)
    if (m_lRealHandle < 0):
        error_info = callCpp("NET_DVR_GetLastError")
        logger.info("预览失败：" + str(error_info))
    else:
        logger.info("预览成功")
    return m_lRealHandle


# endregion

# ================ 定义带速度云台控制类 ==================
class NET_DVR_PTZControl_Other:
    def __init__(self):
        pass

    # 控制云台，传递两个参数，第一个为控制命令，第二个为开关命令(0表示开始，1表示停止),第三个为运行速度
    def get_PTZControl(self, dwPTZCommand, dwStop, dwSpeed):
        if (callCpp("NET_DVR_PTZControlWithSpeed_Other", UserID, Channel, dwPTZCommand, dwStop, dwSpeed) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("控制云台失败：" + str(error_info))
        else:
            logger.info("控制云台成功")
        pass

    pass


# ================ 定义抓图操作结构体 ==================
class NET_DVR_JPEGPARA(ctypes.Structure):
    _fields_ = [
        ("wPicSize", ctypes.c_ushort),
        ("wPicQuality", ctypes.c_ushort)]


# ================ 定义抓图函数 ==================
# jpeg抓图hPlayWnd显示窗口能为none，存在缺点采集图片速度慢
def Get_JPEGpicture(pic_path):
    sJpegPicFileName = bytes(pic_path, "ascii")  # 图片的名字和路径
    lpJpegPara = NET_DVR_JPEGPARA()
    lpJpegPara.wPicSize = 0
    lpJpegPara.wPicQuality = 1
    if (callCpp("NET_DVR_CaptureJPEGPicture", UserID, Channel, ctypes.byref(lpJpegPara),
                sJpegPicFileName) == False):
        error_info = callCpp("NET_DVR_GetLastError")
        logger.info("抓图失败：" + str(error_info))
    # else:
    #     logger.info("抓图成功")
    # endregion


# ================ 定义云台预置点操作类 ==================
class NET_DVR_PTZPreset_Other:
    def __init__(self):
        pass

    # 控制云台，传递两个参数，第一个为控制命令，第二个为开关命令(0表示开始，1表示停止)
    def get_PTZPreset(self, dwPTZPresetCmd, dwPresetIndex):
        if (callCpp("NET_DVR_PTZPreset_Other", UserID, Channel, dwPTZPresetCmd, dwPresetIndex) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("控制预置位失败：" + str(error_info))
        else:
            logger.info("控制预置位成功")
        pass

    pass


# ====================== 定义聚焦结构体 ==================
# 聚焦结构体
class NET_DVR_FOCUSMODE_CFG(ctypes.Structure):
    _fields_ = [
        # DWORD使用 c_uint32, WORD使用 c_uint16, BYTE使用c_byte， float使用c_float
        ("dwSize", ctypes.c_uint32),
        ("byFocusMode", ctypes.c_byte),
        ("byAutoFocusMode", ctypes.c_byte),
        ("wMinFocusDistance", ctypes.c_uint16),
        ("byZoomSpeedLevel", ctypes.c_byte),
        ("byFocusSpeedLevel", ctypes.c_byte),
        ("byOpticalZoom", ctypes.c_byte),
        ("byDigtitalZoom", ctypes.c_byte),
        ("fOpticalZoomLevel", ctypes.c_float),
        ("dwFocusPos", ctypes.c_uint32),
        ("byFocusDefinitionDisplay", ctypes.c_byte),
        ("byFocusSensitivity", ctypes.c_byte),
        ("byRes1", ctypes.c_byte * 2),
        ("dwRelativeFocusPos", ctypes.c_uint32),
        ("byRes", ctypes.c_byte * 48), ]


class FOCUSMODE:
    def __init__(self):
        pass

    # 获取聚焦模式
    def get_focusMode(self):
        m_struFocusModeCfg = NET_DVR_FOCUSMODE_CFG()
        dwReturned = ctypes.c_uint16(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3305, Channel, ctypes.byref(m_struFocusModeCfg),
                    sys.getsizeof(m_struFocusModeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("聚焦模式获取失败：" + str(error_info))
        else:
            logger.info("聚焦模式获取成功")
        logger.info("聚焦模式为：" + str(m_struFocusModeCfg.byFocusMode))
        return m_struFocusModeCfg.byFocusMode

    # 修改聚焦模式 ,0 -- 自动，1 -- 收到，2 -- 半自动
    def Change_focusMode(self, zoomScale):
        m_struFocusModeCfg = NET_DVR_FOCUSMODE_CFG()
        dwReturned = ctypes.c_uint16(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3305, Channel, ctypes.byref(m_struFocusModeCfg),
                    sys.getsizeof(m_struFocusModeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("最小聚焦距离获取失败：" + str(error_info))
        else:
            logger.info("最小聚焦距离获取成功")
            logger.info("当前最小聚焦距离值：" + str(m_struFocusModeCfg.byFocusMode))
            m_struFocusModeCfg.byFocusMode = zoomScale
            if (callCpp("NET_DVR_SetDVRConfig", UserID, 3306, Channel, ctypes.byref(m_struFocusModeCfg),
                        sys.getsizeof(m_struFocusModeCfg)) == False):
                error_info = callCpp("NET_DVR_GetLastError")
                logger.info("最小聚焦距离修改失败：" + str(error_info))
            else:
                logger.info("最小聚焦距离修改成功;修改后的数据为：" + str(m_struFocusModeCfg.byFocusMode))
            return m_struFocusModeCfg.byFocusMode

    # 获取最小聚焦距离
    def get_minFocusDistance(self):
        m_struFocusModeCfg = NET_DVR_FOCUSMODE_CFG()
        dwReturned = ctypes.c_uint16(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3305, Channel, ctypes.byref(m_struFocusModeCfg),
                    sys.getsizeof(m_struFocusModeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("最小聚焦距离获取失败：" + str(error_info) + "line324")
        else:
            logger.info("最小聚焦距离获取成功")
        logger.info("最小聚焦距离为：" + str(m_struFocusModeCfg.wMinFocusDistance))
        return m_struFocusModeCfg.wMinFocusDistance

    # 修改最小聚焦距离值 ,单位cm，取值范围0xffff - 无穷大
    def Change_minFocusDistance(self, zoomScale):
        m_struFocusModeCfg = NET_DVR_FOCUSMODE_CFG()
        dwReturned = ctypes.c_uint16(0)
        logger.info(callCpp("NET_DVR_GetDVRConfig", UserID, 3305, Channel, ctypes.byref(m_struFocusModeCfg),
                      sys.getsizeof(m_struFocusModeCfg), ctypes.byref(dwReturned)))
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3305, Channel, ctypes.byref(m_struFocusModeCfg),
                    sys.getsizeof(m_struFocusModeCfg), ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("最小聚焦距离获取失败：" + str(error_info) + "line337")
        else:
            logger.info("最小聚焦距离获取成功")
            logger.info("当前最小聚焦距离值：" + str(m_struFocusModeCfg.wMinFocusDistance))
            m_struFocusModeCfg.wMinFocusDistance = zoomScale
            if (callCpp("NET_DVR_SetDVRConfig", UserID, 3306, Channel, ctypes.byref(m_struFocusModeCfg),
                        sys.getsizeof(m_struFocusModeCfg)) == False):
                error_info = callCpp("NET_DVR_GetLastError")
                logger.info("最小聚焦距离修改失败：" + str(error_info))
            else:
                logger.info("最小聚焦距离修改成功;修改后的数据为：" + str(m_struFocusModeCfg.wMinFocusDistance))
            return m_struFocusModeCfg.wMinFocusDistance

    # 获取光学变倍值
    def get_CamZoom(self):
        m_struFocusModeCfg = NET_DVR_FOCUSMODE_CFG()
        dwReturned = ctypes.c_uint16(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3305, Channel, ctypes.byref(m_struFocusModeCfg),
                    sys.getsizeof(m_struFocusModeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("光学变倍获取失败：" + str(error_info))
        else:
            logger.info("光学变倍获取成功")
        logger.info("光学变倍值为：" + str(m_struFocusModeCfg.fOpticalZoomLevel))
        return m_struFocusModeCfg.fOpticalZoomLevel

    # 修改光学变倍值，取值范围[1,32],最小间隔0.5
    def Change_CamZoom(self, zoomScale):
        m_struFocusModeCfg = NET_DVR_FOCUSMODE_CFG()
        dwReturned = ctypes.c_uint16(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3305, Channel, ctypes.byref(m_struFocusModeCfg),
                    sys.getsizeof(m_struFocusModeCfg), ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("光学变倍获取失败：" + str(error_info))
        else:
            logger.info("光学变倍获取成功")
            logger.info("当前光学变倍值：" + str(m_struFocusModeCfg.fOpticalZoomLevel))
            m_struFocusModeCfg.fOpticalZoomLevel = zoomScale
            if (callCpp("NET_DVR_SetDVRConfig", UserID, 3306, Channel, ctypes.byref(m_struFocusModeCfg),
                        sys.getsizeof(m_struFocusModeCfg)) == False):
                error_info = callCpp("NET_DVR_GetLastError")
                logger.info("光学变倍修改失败：" + str(error_info))
            else:
                logger.info("光学变倍修改成功;修改后的数据为：" + str(m_struFocusModeCfg.fOpticalZoomLevel))
            return m_struFocusModeCfg.fOpticalZoomLevel

    pass


# ====================== 定义曝光结构体 ==================
# 曝光结构体
class NET_DVR_AEMODECFG(ctypes.Structure):
    _fields_ = [
        # DWORD使用 c_uint32, WORD使用 c_uint16, BYTE使用c_byte， float使用c_float
        ("dwSize", ctypes.c_uint32),
        ("iIrisSet", ctypes.c_int),
        ("iGainSet", ctypes.c_int),
        ("iGainLimit", ctypes.c_int),
        ("iExposureCompensate", ctypes.c_int),
        ("byExposureModeSet", ctypes.c_byte),
        ("byShutterSet", ctypes.c_byte),
        ("byImageStabilizeLevel", ctypes.c_byte),
        ("byCameraIrCorrect", ctypes.c_byte),
        ("byHighSensitivity", ctypes.c_byte),
        ("byInitializeLens", ctypes.c_byte),
        ("byChromaSuppress", ctypes.c_byte),
        ("byMaxShutterSet", ctypes.c_byte),
        ("byMinShutterSet", ctypes.c_byte),
        ("byMaxIrisSet", ctypes.c_byte),
        ("byMinIrisSet", ctypes.c_byte),
        ("byExposureLevel", ctypes.c_byte),
        ("byRes", ctypes.c_byte * 60),
    ]


class AEMODECFG:
    def __init__(self):
        pass

    # 获取增益
    def get_Gain(self):
        m_struAEmodeCfg = NET_DVR_AEMODECFG()
        dwReturned = ctypes.c_int(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3309, Channel, ctypes.byref(m_struAEmodeCfg),
                    sys.getsizeof(m_struAEmodeCfg), ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("增益值获取失败：" + str(error_info))
        else:
            logger.info("增益值获取成功")
        logger.info("增益值为：" + str(m_struAEmodeCfg.iGainSet))
        return m_struAEmodeCfg.iGainSet

    # 修改增益值
    def Change_Gain(self, gainScale):
        m_struAEmodeCfg = NET_DVR_AEMODECFG()
        dwReturned = ctypes.c_int(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3309, Channel, ctypes.byref(m_struAEmodeCfg),
                    sys.getsizeof(m_struAEmodeCfg), ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("增益值获取失败：" + str(error_info))
        else:
            logger.info("增益值获取成功")
            logger.info("当前增益值：" + str(m_struAEmodeCfg.iGainSet))
            m_struAEmodeCfg.iGainSet = gainScale

            if (callCpp("NET_DVR_SetDVRConfig", UserID, 3310, Channel, ctypes.byref(m_struAEmodeCfg),
                        sys.getsizeof(m_struAEmodeCfg)) == False):
                error_info = callCpp("NET_DVR_GetLastError")
                logger.info("增益值修改失败：" + str(error_info))
            else:
                logger.info("增益值修改成功;修改后的数据为：" + str(m_struAEmodeCfg.iGainSet))
        return m_struAEmodeCfg.iGainSet

    # 获取光圈值
    def get_Iris(self):
        m_struAEmodeCfg = NET_DVR_AEMODECFG()
        dwReturned = ctypes.c_uint16(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3309, Channel, ctypes.byref(m_struAEmodeCfg),
                    sys.getsizeof(m_struAEmodeCfg), ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("光圈值获取失败：" + str(error_info) + "line460")
        else:
            logger.info("光圈值获取成功")
        logger.info("光圈值为：" + str(m_struAEmodeCfg.iIrisSet))
        return m_struAEmodeCfg.iIrisSet

    # 修改光圈值
    def Change_Iris(self, gainScale):
        m_struAEmodeCfg = NET_DVR_AEMODECFG()
        dwReturned = ctypes.c_int(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3309, Channel, ctypes.byref(m_struAEmodeCfg),
                    sys.getsizeof(m_struAEmodeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("光圈值获取失败：" + str(error_info) + "line474")
        else:
            logger.info("光圈值获取成功")
            logger.info("当前光圈值：" + str(m_struAEmodeCfg.iIrisSet))
            m_struAEmodeCfg.iIrisSet = gainScale
            if (callCpp("NET_DVR_SetDVRConfig", UserID, 3310, Channel, ctypes.byref(m_struAEmodeCfg),
                        sys.getsizeof(m_struAEmodeCfg)) == False):
                error_info = callCpp("NET_DVR_GetLastError")
                logger.info("光圈值修改失败：" + str(error_info))
            else:
                logger.info("光圈值修改成功;修改后的数据为：" + str(m_struAEmodeCfg.iIrisSet))
            return m_struAEmodeCfg.iIrisSet

    # 获取快门等级
    def get_Shutter(self):
        m_struAEmodeCfg = NET_DVR_AEMODECFG()
        dwReturned = ctypes.c_uint16(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3309, Channel, ctypes.byref(m_struAEmodeCfg),
                    sys.getsizeof(m_struAEmodeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("快门等级获取失败：" + str(error_info) + "line495")
        else:
            logger.info("快门等级获取成功")
        logger.info("快门等级为：" + str(m_struAEmodeCfg.byShutterSet))
        return m_struAEmodeCfg.byShutterSet

    # 修改快门等级
    def Change_Shutter(self, gainScale):
        m_struAEmodeCfg = NET_DVR_AEMODECFG()
        dwReturned = ctypes.c_int(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3309, Channel, ctypes.byref(m_struAEmodeCfg),
                    sys.getsizeof(m_struAEmodeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("快门等级获取失败：" + str(error_info) + "line509")
        else:
            logger.info("快门等级获取成功")
            logger.info("当前快门等级：" + str(m_struAEmodeCfg.byShutterSet))
            m_struAEmodeCfg.byShutterSet = gainScale
            if (callCpp("NET_DVR_SetDVRConfig", UserID, 3310, Channel, ctypes.byref(m_struAEmodeCfg),
                        sys.getsizeof(m_struAEmodeCfg)) == False):
                error_info = callCpp("NET_DVR_GetLastError")
                logger.info("快门等级修改失败：" + str(error_info))
            else:
                logger.info("快门等级修改成功;修改后的数据为：" + str(m_struAEmodeCfg.byShutterSet))
            return m_struAEmodeCfg.byShutterSet

    # 获取曝光模式
    def get_exposureMode(self):
        m_struAEmodeCfg = NET_DVR_AEMODECFG()
        dwReturned = ctypes.c_uint16(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3309, Channel, ctypes.byref(m_struAEmodeCfg),
                    sys.getsizeof(m_struAEmodeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("曝光模式获取失败：" + str(error_info))
        else:
            logger.info("曝光模式获取成功")
        logger.info("曝光模式为：" + str(m_struAEmodeCfg.byExposureModeSet))
        return m_struAEmodeCfg.byExposureModeSet

    # 修改曝光模式
    def Change_exposureMode(self, gainScale):
        m_struAEmodeCfg = NET_DVR_AEMODECFG()
        dwReturned = ctypes.c_int(0)
        if (callCpp("NET_DVR_GetDVRConfig", UserID, 3309, Channel, ctypes.byref(m_struAEmodeCfg),
                    sys.getsizeof(m_struAEmodeCfg),
                    ctypes.byref(dwReturned)) == False):
            error_info = callCpp("NET_DVR_GetLastError")
            logger.info("曝光模式获取失败：" + str(error_info))
        else:
            logger.info("曝光模式获取成功")
            logger.info("当前曝光模式：" + str(m_struAEmodeCfg.byExposureModeSet))
            m_struAEmodeCfg.byExposureModeSet = gainScale
            if (callCpp("NET_DVR_SetDVRConfig", UserID, 3310, Channel, ctypes.byref(m_struAEmodeCfg),
                        sys.getsizeof(m_struAEmodeCfg)) == False):
                error_info = callCpp("NET_DVR_GetLastError")
                logger.info("曝光模式修改失败：" + str(error_info))
            else:
                logger.info("曝光模式修改成功;修改后的数据为：" + str(m_struAEmodeCfg.byExposureModeSet))
            return m_struAEmodeCfg.byExposureModeSet

    pass
