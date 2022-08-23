#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/9 14:33
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : robot_lifter.py
# @description: "机器人升降杆服务"
import sys
import rospy
sys.path.append("/opt/moss_robot/lib/dispatch_ips_web")
from configs.log import logger
from schema.db_robot_config import DBRobotConfig
from modules.client import Main


class RobotLifer:
    def __init__(self, move_type, position):
        self.move_type = int(move_type)
        self.position = position
        pass

    def start(self):
        db_robot_config = DBRobotConfig()
        tags = db_robot_config.get_height()
        height_data = tags.get("elevator")
        if height_data:
            now_height = int(height_data)
        else:
            now_height = 0
        # logger.info("升降杆当前高度{}".format(now_height))
        # logger.info("升降杆调整高度{}".format(self.position))
        # action_height运动高度， last_height最终高度
        if self.move_type == 2:
            if now_height + self.position > 830:
                action_height = 830 - now_height
                last_height = 830
            elif now_height + self.position < 0:
                action_height = -now_height
                last_height = 0
            else:
                logger.info(self.position)
                action_height = self.position
                last_height = now_height + action_height
        else:
            action_height = self.position
            last_height = action_height
        state = Main(self.move_type, action_height)
        if state:
            robot = {"robot_id": "1"}
            tags["elevator"] = last_height
            db_robot_config.update(robot, tags)
            return True, last_height
            # result_text = {"position": last_height, "message": "success"}
        return False, 0


if __name__ == '__main__':
    rospy.init_node('robot_lifter')
    RobotLifer(1, 600).start()

