import sys
import time
import os
import cv2
import logging
from PIL import Image
import numpy as np
sys.path.append(os.path.abspath("./modules/industry_daheng/"))
import gxipy as gx

from configs import industrial_daheng_id
from configs.log import logger


# 大恒相机类，需实例化一个对象进行调用
class DH_CAMERA():
    def __init__(self) -> None:
        self.init_camera()
        self.upper_camera_x_y = 0
        self.lower_camera_x_y = 0

    # 初始化工业相机
    def init_camera(self):

        logger.info("initializing DAHENG industry camera.....")
        self.device_manager = gx.DeviceManager()
        logger.info("get device manager.")
        # 查找出几个相机，相机的详细类型列表
        self.dev_num, self.dev_info_list = self.device_manager.update_device_list()
        if self.dev_num == 0:
            logger.error("can not find a device, please check the hardware.")
            return False
        logger.info("device num: {},device info: {}.".format(str(self.dev_num), str(self.dev_info_list)))

    def open_camera(self, method, cam_type="HCL21080005"):
        """
        通过serial number打开工业相机
        INPUT:
        @method:"sn","use-id","index","mac","ip"
        目前连接方式选择sn模式
        """
        for dev_info in self.dev_info_list:
            str_sn = dev_info.get(method)
            if str_sn != cam_type:
                continue
            if str_sn is None:
                logger.info("can not find a device by {}, exit program".format(str_sn))
                continue
            if str_sn == industrial_daheng_id.get("lower_sn_id"):
                logger.info("use {} lower to open camera.".format(str_sn))
                self.camera = self.device_manager.open_device_by_sn(str_sn)
                if self.lower_camera_x_y == 0:
                    logger.info("旋转下工业相机180度")
                    self.camera.ReverseX.set(False)
                    self.camera.ReverseY.set(False)
                    self.lower_camera_x_y = 1

            elif str_sn == industrial_daheng_id.get("upper_sn_id"):
                logger.info("use {} upper to open camera.".format(str_sn))
                self.camera = self.device_manager.open_device_by_sn(str_sn)
                if self.upper_camera_x_y == 0:
                    logger.info("旋转上工业相机180度")
                    self.camera.ReverseX.set(False)
                    self.camera.ReverseY.set(False)
                    self.upper_camera_x_y = 1

            if self.camera is None:
                logger.error("can not open a device by {}, exit program".format(str_sn))
                return False
            logger.info("open camera success.")
            return True

    def set_exposure_time(self, time):
        """
        设置曝光时间
        INPUT：
        @time(float), 曝光时间,时间为ms
        """
        if not self.camera.ExposureTime.is_readable():
            logger.error("can not read the parameter, please check the device status.")
            return False
        # 获取当前的曝光时间
        current_exposure_time = self.camera.ExposureTime.get()
        # 获取曝光时间的可调节范围
        exposure_range = self.camera.ExposureTime.get_range()
        if time > exposure_range['min'] and time < exposure_range['max']:
            if not self.camera.ExposureTime.is_writable():
                logger.error("can not write the parameter, please check the device status.")
                return False
            self.camera.ExposureTime.set(time)
            logger.info("set exposure time from {} to {}.".format(current_exposure_time, time))
        else:
            logger.info("input parameter is out of range, please check the parameter.")

    def set_gain(self, gain):
        """
        设置增益
        INPUT：
        @gain(float),图像增益
        """
        if not self.camera.Gain.is_readable():
            logger.info("can not read the parameter, please check the device status.")
            return None
        # 获取当前的增益值
        current_gain = self.camera.Gain.get()
        # 获取增益值的可调节范围
        gain_range = self.camera.Gain.get_range()
        if gain > gain_range['min'] and gain < gain_range['max']:
            if not self.camera.Gain.is_writable():
                logger.info("can not write the parameter, please check the device status.")
                return None
            self.camera.Gain.set(gain)
            logger.info("set gain from {} to {}".format(current_gain, gain))
        else:
            logger.info('input parameter is out of range, please check the parameter')

    def set_white_balance(self, balance):
        '''
        设置白平衡
        INPUT:
        @balance(float),白平衡
        '''
        if not self.camera.BalanceRatio.is_readable():
            logger.warning("can not read the parameter, please check the device status.")
            return None
        # 获取当前的白平衡值
        current_balance = self.camera.BalanceRatio.get()
        # 获取白平衡的可调节范围
        balance_range = self.camera.BalanceRatio.get_range()
        if balance > balance_range['min'] and balance < balance_range['max']:
            if not self.camera.BalanceRatio.is_writable():
                logger.warning("can not write the parameter, please check the device status.")
                return None
            self.camera.BalanceRatio.set(balance)
            logger.info("set balance from {} to {}".format(current_balance, balance))
        else:
            logger.warning('input parameter is out of range, please check the parameter')

    def get_camera_parameter(self):
        """
        取出工业相机的当前数值：
        current_exposure_time 曝光时间
        current_gain 增益
        current_balance 白平衡
        :return: current_exposure_time ，current_gain，current_balance
        """
        current_exposure_time = self.camera.ExposureTime.get()
        current_gain = self.camera.Gain.get()
        current_balance = self.camera.BalanceRatio.get()

        return current_exposure_time, current_gain, current_balance

    # 取流
    def get_stream(self):
        self.camera.stream_on()
        logger.info("stream_on")

    # 关流
    def close_stream(self):
        self.camera.stream_off()
        logger.info("stream_of")

    # 关闭相机
    def close_camera(self):
        self.camera.close_device()
        logger.info("close camera")

    # 拍照
    # def take_a_picture(self, stream_index):
    #     # 从第 stream_index流通道获取一幅图像
    #     raw_image = self.camera.data_stream[stream_index].get_image()
    #     rgb_image = raw_image.convert("RGB")
    #     if rgb_image is None:
    #         logger.warning("the image is None, check the device status.")
    #         return False
    #     # 将图片转换为numpy格式
    #     numpy_image = rgb_image.get_numpy_array()
    #     if numpy_image is None:
    #         logger.warning("image format convert failed.")
    #         return False
    #     # 显示并保存获得的 RGB 图片
    #     image = Image.fromarray(numpy_image, 'RGB')
    #     return image
    def  take_a_picture(self,stream_index):
        try:
            '''
            拍照
            '''
            # 从第 stream_index流通道获取一幅图像
            raw_image = self.camera.data_stream[stream_index].get_image()
            rgb_image = raw_image.convert("RGB")
            cnt = 0
            while rgb_image is None and cnt < 5:
                    raw_image = self.camera.data_stream[stream_index].get_image()
                    rgb_image = raw_image.convert("RGB")
                    cnt += 1
                    time.sleep(0.2)
            if rgb_image is None:
                    logging.log(logging.WARNING,"the image is None, check the device status.")
                    return False
            # 将图片转换为numpy格式
            numpy_image = rgb_image.get_numpy_array()
            cnt = 0
            while numpy_image is None and cnt < 5:
                    numpy_image = rgb_image.get_numpy_array()
                    cnt += 1
                    time.sleep(0.1)
            if numpy_image is None:
                    logging.log(logging.WARNING,"image format convert failed.")
                    return False
            # 显示并保存获得的 RGB 图片
            image = Image.fromarray(numpy_image, 'RGB')
            dim = (3080, 1800)
            image = image.resize(dim)
            return image
        except Exception as e:
            print(e,'take_a_picture failed')
    


def take_picture():
    '''
    拍图测试
    '''
    folder_path = "imgs"
    continous_shot = 6
    loop = 1000
    counts = 0
    cam = DH_CAMERA()
    for i in range(loop):
        cam.open_camera('sn')
        # 更改曝光时间
        cam.set_exposure_time(80000.0)
        # 更改增益
        cam.set_gain(0.8)
        # 更改白平衡
        cam.set_white_balance(2.3)
        cam.get_stream()
        time_stamp = time.strftime("%Y_%m_%d_%H_%M", time.localtime())  # eg:2021_07_21

        # for j in range(continous_shot):
        j = 100
        sot = time.time()
        image = cam.take_a_picture(0)
        save_path = "{}_l{}_{}.jpg".format(time_stamp, i, j)
        print(save_path)
        eot = time.time()

        image.save(save_path)
        counts += 1

        cam.close_stream()
        cam.close_camera()


def play_video():
    '''
    播放视频
    '''
    fps = 25
    cv2.namedWindow('DAHENG', cv2.WINDOW_KEEPRATIO)
    cam = DH_CAMERA()
    cam.open_camera('sn')
    cam.get_stream()
    while (1):
        image = cam.take_a_picture(0)
        image = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        cv2.imshow("DAHENG", image)
        cv2.waitKey(int(1000 / int(fps)))
    cam.close_stream()
    cam.close_camera()


def test_parameter():
    '''
    测试更改参数
    '''
    fps = 25
    cv2.namedWindow('DAHENG', cv2.WINDOW_KEEPRATIO)
    cam = DH_CAMERA()
    cam.open_camera('sn')
    # 更改曝光时间
    cam.set_exposure_time(80000.0)
    # 更改增益
    cam.set_gain(0.8)
    # 更改白平衡
    cam.set_white_balance(2.3)
    cam.get_stream()
    while (1):
        image = cam.take_a_picture(0)
        image = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        cv2.imshow("DAHENG", image)
        cv2.waitKey(int(1000 / int(fps)))
    cam.close_stream()
    cam.close_camera()


if __name__ == '__main__':
    take_a_picture()
    # play_video()
    # test_parameter()
