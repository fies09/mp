from time import time
import time
import requests

from configs import auto_door
from configs.log import logger


class AutoDoor(object):
    def __init__(self, ip=None, channel=None):
        self.ip = ip
        self.channel = channel

    def get_door_state(self):
        url = 'http://' + self.ip + '/CN/httpapi.json?&sndtime=0.21345062273350623&CMD=UART_WRITE&UWHEXVAL=0'
        try:
            logger.info(url)
            response = requests.get(url, timeout=15)
            statusCode = response.text[0]  # 1为开门 0为关门
        except Exception as e:
            logger.error(e)
            return False
        if self.channel == 1:
            if statusCode == "1":
                return True
            else:
                return False
        elif self.channel == 2:
            if statusCode == "2":
                return True
            else:
                return False
        elif self.channel == 3:
            if statusCode == "4":
                return True
            else:
                return False
        elif self.channel == 4:
            if statusCode == "8":
                return True
            else:
                return False
        else:
            logger.error("没有配置信道")
            return False

    def door_action(self):
        url = 'http://' + self.ip + '/CN/httpapi.json?&sndtime=0.21345062273350623&CMD=UART_WRITE&UWHEXVAL={}'.format(self.channel)
        logger.info(url)

        ret = requests.get(url, timeout=15)
        if ret.status_code != 200:
            logger.info("开关门动作与开关门继电器没有通讯")
            return False
        return True

    def auto_open(self):
        logger.info("开始执行开门动作")
        num = 0
        try:
            while True:
                num += 1
                door_state = self.get_door_state()
                logger.info("当前门的状态{}".format(door_state))
                if num > 4:
                    logger.info("多次执行开门动作没有成功")
                    return False
                if door_state:
                    logger.info("该门状态为开启状态！")
                    logger.info("开门成功")
                    return True
                else:
                    # 执行门状态转换动作
                    self.door_action()
                time.sleep(5)
        except Exception as e:
            logger.error("执行开门动作出错")
            logger.error(e)
            return False

    def auto_close(self):
        logger.info("开始执行关门动作")
        num = 0
        try:
            while True:
                num += 1
                door_state = self.get_door_state()
                logger.info("当前门的状态{}".format(door_state))
                if num > 4:
                    logger.info("多次执行开门动作没有成功")
                    return True
                if door_state:
                    # 执行门状态转换动作，关门
                    self.door_action()
                else:
                    logger.info("该门状态为关闭状态！")
                    logger.info("关门成功")
                    return True
                time.sleep(5)
        except Exception as e:
            logger.error("执行关门动作出错")
            logger.error(e)
            return True


if __name__ == '__main__':
    from schema.db_robot_path_position_item import DBRobotPathPositionItem
    from schema.db_robot_algorithm_param import DBRobotAlgorithmParam

    door_rets = DBRobotPathPositionItem(). \
        get_door_param(id=7)
    door_ret = door_rets[0]
    logger.info(door_ret)
    al_data = DBRobotAlgorithmParam().get_by_id(door_ret.get("robot_algorithm_param_id"))
    if al_data:
        parameters = al_data.get("parameters")

        auto_door = AutoDoor(ip=parameters.get("ip"), channel=parameters.get("channel"))
        logger.info("开始进行该点的开关门动作,当前点动作为{}".format(door_ret.get("name")))
        if door_ret.get("name") == "开":
            if auto_door.auto_open():
                print("开门成功")
            else:
                print("开门失败")

        elif door_ret.get("name") == "关":
            if auto_door.auto_close():
                print("开门成功")
            else:
                print("开门失败")
