# -- coding: utf-8 --
import os
import sys
import cv2

sys.path.append("/opt/moss_robot/lib/dispatch_ips_web")
sys.path.append("MvImport")
from modules.Industrial_camera.MvImport.MvCameraControl_class import *
from configs.log import logger
from configs import industrial_id


class steparm():
    def __init__(self, stDevice_position):
        # ch:创建相机实例 | en:Creat Camera Object
        self.cam = MvCamera()
        self.stDevice_position = stDevice_position
        self.data_buf = None
        self.nPayloadSize = None
        self.stDeviceList = None
        # 是否使用opencv转存图片
        self.Equipment_dict = {1: industrial_id.get("upper_camera"), 0: industrial_id.get("lower_camera")}
        self.save_change_status = True
        self.init()

    # 连接设备
    def init(self):
        # 查看相机版本
        SDKVersion = self.cam.MV_CC_GetSDKVersion()
        # logger.info("SDKVersion[0x%x]" % SDKVersion)

        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

        # ch:枚举设备 | en:Enum device
        ret = self.cam.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            logger.error("enum devices fail! ret[0x%x]" % ret)
            logger.error("枚举设备相机出错，程序退出！")
            # sys.exit()

        if deviceList.nDeviceNum == 0:
            logger.error("find no device!")
            logger.error("没有查找到相机设备，程序退出！")
            # sys.exit()

        # logger.info("Find %d devices!" % deviceList.nDeviceNum)
        # logger.info("找到 %d 个相机设备!" % deviceList.nDeviceNum)

        mvcc_dev_info = cast(deviceList.pDeviceInfo[self.stDevice_position], POINTER(MV_CC_DEVICE_INFO)).contents
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            logger.info("\ngige device: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            logger.info("device model name: %s" % strModeName)

            nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            logger.info("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            # logger.info("\nu3v device: [%d]" % 0)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            logger.info("device model name: %s" % strModeName)

            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            logger.info("user serial number: %s" % strSerialNumber)

        self.create_handle(deviceList)

    # 选择设备创建句柄
    def create_handle(self, deviceList):
        # 摄像头
        status = None
        for i in range(2):
            stDevice_list = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            strSerialNumber = ''.join([chr(i) for i in stDevice_list.SpecialInfo.stUsb3VInfo.chSerialNumber]).rstrip(
                '\x00')
            # logger.info(strSerialNumber)
            # logger.info(self.Equipment_dict[int(self.stDevice_position)])
            if self.Equipment_dict[int(self.stDevice_position)] == str(strSerialNumber.strip()):
                status = True
                break
        if status:
            logger.info('海康工业相机初始化成功！')


        # 连接设备
        ret_lower = self.cam.MV_CC_CreateHandle(stDevice_list)
        if ret_lower != 0:
            logger.info("create handle fail! ret[0x%x]" % ret_lower)
            logger.info("打开下摄像头句柄失败，程序推出! ret[0x%x]" % ret_lower)
            sys.exit()

        self.connect()

    # 连接设备
    def connect(self):
        # ch:打开设备 | en:Open device
        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, self.stDevice_position)
        if ret != 0:
            logger.info("open device fail! ret[0x%x]" % ret)
            logger.info("打开摄像头失败，程序推出! ret[0x%x]" % ret)
            return False

            # sys.exit()
        # ch:设置触发模式为off | en:Set trigger mode as off
        ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            logger.info("set trigger mode fail! ret[0x%x]" % ret)
            return False

        # ch:获取数据包大小 | en:Get payload size
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            logger.info("get payload size fail! ret[0x%x]" % ret)
            return False
        self.nPayloadSize = stParam.nCurValue
        return True

    # ch:开始取流 | en:Start grab image
    def open_quliu(self):
        logger.info("开始取流")
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            logger.info("start grabbing fail! ret[0x%x]" % ret)
            return False
        self.stDeviceList = MV_FRAME_OUT_INFO_EX()
        memset(byref(self.stDeviceList), 0, sizeof(self.stDeviceList))

        self.data_buf = (c_ubyte * self.nPayloadSize)()
        return True

    # ch: 停止取流 | stop get
    def close_quliu(self):
        logger.info("停止取流")
        ret = self.cam.MV_CC_StopGrabbing()
        if ret != 0:
            logger.info("close deivce fail! ret[0x%x]" % ret)
            return False
        return True

    # 关闭设备 设备工作完成后调用 避免设备一直处于工作状态
    def shutdown(self):
        # ch:关闭设备 | Close device
        ret = self.cam.MV_CC_CloseDevice()
        if ret != 0:
            logger.info("close deivce fail! ret[0x%x]" % ret)
            # sys.exit()

        # ch:销毁句柄 | Destroy handle
        ret = self.cam.MV_CC_DestroyHandle()
        if ret != 0:
            logger.info("destroy handle fail! ret[0x%x]" % ret)
            # sys.exit()

    # 抓图
    def grap_img(self, name='./saveimg.jpg'):
        ret = self.cam.MV_CC_GetOneFrameTimeout(byref(self.data_buf), self.nPayloadSize, self.stDeviceList, 1000)
        if ret == 0:
            logger.info("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                self.stDeviceList.nWidth, self.stDeviceList.nHeight, self.stDeviceList.nFrameNum))
            # 创建构造体 赋值
            stConvertParam = MV_SAVE_IMAGE_PARAM_EX()
            stConvertParam.nWidth = self.stDeviceList.nWidth
            stConvertParam.nHeight = self.stDeviceList.nHeight
            stConvertParam.pData = self.data_buf
            stConvertParam.nDataLen = self.stDeviceList.nFrameLen
            stConvertParam.enPixelType = self.stDeviceList.enPixelType

            # MV_Image_Undefined  = 0, //未定义
            #   MV_Image_Bmp        = 1, //BMP图片
            #   MV_Image_Jpeg       = 2, //JPEG图片
            #   MV_Image_Png        = 3, //PNG图片，暂不支持
            #   MV_Image_Tif        = 4, //TIF图片，暂不支持

            # jpg参数
            stConvertParam.nJpgQuality = 70  # 压缩质量选择范围[50-99]
            img_path = name
            stConvertParam.enImageType = MV_Image_Jpeg
            bmpsize = self.nPayloadSize

            # file_path = "save.bmp"
            # stConvertParam.enImageType = MV_Image_Bmp
            # bmpsize = self.stDeviceList.nWidth * self.stDeviceList.nHeight * 3 + 54
            stConvertParam.nBufferSize = bmpsize
            bmp_buf = (c_ubyte * bmpsize)()
            stConvertParam.pImageBuffer = bmp_buf
            # 保存一张图片到内存中
            ret = self.cam.MV_CC_SaveImageEx2(stConvertParam)
            if ret != 0:
                logger.info("save file executed failed0:! ret[0x%x]" % ret)
                return False

            # 创建一个二进制可写入文件
            try:
                # 创建一个地址
                img_buff = (c_ubyte * stConvertParam.nDataLen)()
                # 将图片内存地址赋予创建的地址
                memmove(byref(img_buff), stConvertParam.pImageBuffer, stConvertParam.nDataLen)
                # 写入文件
                with open(img_path.encode('ascii'), 'wb+') as f:
                    f.write(img_buff, )
                if self.save_change_status:
                    self.save_change(img_path)
                # file_open.write(img_buff, )
            except Exception as e:
                logger.error(e)
                return False
            return True
        else:
            logger.info("get one frame fail, ret[0x%x]" % ret)
            return False

    # 将保存后的图片用opencv重新读写下
    def save_change(self, iamge_path='./saveimg.jpg'):
        logger.info("抓拍图片{}成功".format(iamge_path))
        img = cv2.imread(iamge_path)
        os.remove(iamge_path)
        cv2.imwrite(iamge_path, img)

    # 设定相机
    def setparm_int(self, key, value):
        # int类型相机参数改变
        ret = self.cam.MV_CC_SetIntValue(key, value)
        if ret != 0:
            logger.info("set int fail! ret[0x%x]" % ret)
            return False
        return True

    def getparm_int(self, key):
        # 获取int类型工业相机
        stFloatParam_FrameRate = MVCC_INTVALUE()
        memset(byref(stFloatParam_FrameRate), 0, sizeof(MVCC_INTVALUE))
        ret = self.cam.MV_CC_GetIntValue(key, stFloatParam_FrameRate)
        if ret != 0:
            logger.info("get int fail! ret[0x%x]" % ret)
            return None
        return stFloatParam_FrameRate.nCurValue

    def setparm_Enum(self, key, value):
        # 设置Enum型属性
        ret = self.cam.MV_CC_SetEnumValue(key, value)
        if ret != 0:
            logger.info("set enum fail! ret[0x%x]" % ret)
            return False
        return True

    def getparm_Enum(self, key):
        # 获取eum类型工业相机
        stFloatParam_FrameRate = MVCC_ENUMVALUE()
        memset(byref(stFloatParam_FrameRate), 0, sizeof(MVCC_ENUMVALUE))
        ret = self.cam.MV_CC_GetEnumValue(key, stFloatParam_FrameRate)
        if ret != 0:
            logger.info("get int fail! ret[0x%x]" % ret)
            return None
        return stFloatParam_FrameRate.nCurValue

    def setparm_EnumValueByString(self, key, value):
        # 设置Enum型属性
        ret = self.cam.MV_CC_SetEnumValueByString(key, value)
        if ret != 0:
            logger.info("set enumstr fail! ret[0x%x]" % ret)
            return False
        return True

    def setparm_Float(self, key, value):
        # 设置Float型属性
        ret = self.cam.MV_CC_SetFloatValue(key, value)
        if ret != 0:
            logger.info("set float fail! ret[0x%x]" % ret)
            return False
        return True

    def getparm_Float(self, key):
        # 获取float类型工业相机
        stFloatParam_FrameRate = MVCC_FLOATVALUE()
        memset(byref(stFloatParam_FrameRate), 0, sizeof(MVCC_FLOATVALUE))
        ret = self.cam.MV_CC_GetFloatValue(key, stFloatParam_FrameRate)
        if ret != 0:
            logger.info("get int fail! ret[0x%x]" % ret)
            return None
        return stFloatParam_FrameRate.fCurValue

    def setparm_Boolean(self, key, value):
        # 设置Boolean型属性
        ret = self.cam.MV_CC_SetBoolValue(key, value)
        if ret != 0:
            logger.info("set bool fail! ret[0x%x]" % ret)
            return False
        return True

    def getparm_Boolean(self, key):
        # 获取Bool类型工业相机
        stFloatParam_FrameRate = MVCC_INTVALUE()
        memset(byref(stFloatParam_FrameRate), 0, sizeof(MVCC_INTVALUE))
        ret = self.cam.MV_CC_GetBoolValue(key, stFloatParam_FrameRate)
        if ret != 0:
            logger.info("get int fail! ret[0x%x]" % ret)
            return None
        return stFloatParam_FrameRate.nCurValue

    def setparm_String(self, key, value):
        # 设置String型属性
        ret = self.cam.MV_CC_SetStringValue(key, value)
        if ret != 0:
            logger.info("set string fail! ret[0x%x]" % ret)
            return False
        return True

    def getparm_String(self, key):
        # 获取String类型工业相机
        stFloatParam_FrameRate = MVCC_STRINGVALUE()
        memset(byref(stFloatParam_FrameRate), 0, sizeof(MVCC_STRINGVALUE))
        ret = self.cam.MV_CC_GetStringValue(key, stFloatParam_FrameRate)
        if ret != 0:
            logger.info("get int fail! ret[0x%x]" % ret)
            return None
        return stFloatParam_FrameRate.chCurValue

    def setparm_Command(self, key="DeviceReset", val="Command"):
        # 设置Command型属性
        ret = self.cam.MV_CC_SetCommandValue(key, val)
        if ret != 0:
            logger.info("open device fail! ret[0x%x]" % ret)
            return False
        return True

    def set_acquisition_mode(self):
        ret = self.cam.MV_CC_SetAcquisitionMode()
        if ret != 0:
            logger.info("set device fail! ret[0x%x]" % ret)
            return False
        return True
if __name__ == '__main__':
    import time

    # image_upper = steparm(stDevice_position=0)
    # for i in range(100):
    #     image_upper.open_quliu()
    #     image_upper.grap_img()
    #     image_upper.close_quliu()
    # image_upper.shutdown()

    image_lower = steparm(stDevice_position=0)
    # action_state = image_lower.setparm_Command(key="Acquisition Mode", val="SingleFrame")
    action_state = image_lower.set_acquisition_mode()

    print(action_state)
    for i in range(100):
        image_lower.open_quliu()
        time.sleep(10)
        image_lower.grap_img("{}.jpg".format(str(i)))
        time.sleep(2)
        # time.sleep(0.1)
        image_lower.close_quliu()
    image_lower.shutdown()

