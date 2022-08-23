#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/6 17:49
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : robot_transducer.py
# @description: "机器人传感器服务"
import sys
import rospy
from std_msgs.msg import Float64MultiArray, String, Float32
from sensor_msgs.msg import BatteryState
sys.path.append("/opt/moss_robot/lib/dispatch_ips_web")
from configs import transducer_map
from configs.log import logger


class RobotTransducer:
    def __init__(self, item_index):
        self.item_index = int(item_index)
        pass

    def start(self):
        """
        传感器
        """
        if transducer_map[self.item_index] == "温度":
            res = rospy.wait_for_message("/temhum", Float64MultiArray, timeout=5)
            return "%.2f" % float(res.data[0])
        elif transducer_map[self.item_index] == "湿度":
            res = rospy.wait_for_message("/temhum", Float64MultiArray, timeout=5)
            return "%.2f" % float(res.data[1])
        elif transducer_map[self.item_index] == "H2S":
            res = rospy.wait_for_message("/hsulfide", String, timeout=5)
            return res.data
        elif transducer_map[self.item_index] == "SO2":
            res = rospy.wait_for_message("/sdioxide", String, timeout=5)
            return res.data
        elif transducer_map[self.item_index] == "噪声":
            res = rospy.wait_for_message("/noise", String, timeout=5)
            return res.data
        elif transducer_map[self.item_index] == "PM1":
            res = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
            return "%.2f" % float(res.data[3])
        elif transducer_map[self.item_index] == "风速":
            return 666
        elif transducer_map[self.item_index] == "CO2":
            res = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
            return res.data[1]
        elif transducer_map[self.item_index] == "TVOC":
            res = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
            return "%.2f" % float(res.data[2])
        elif transducer_map[self.item_index] == "甲醛":
            res = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
            print(res.data[5] * 0.001)
            return str(res.data[5] * 0.001)
        elif transducer_map[self.item_index] == "PM10":
            res = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
            return "%.2f" % float(res.data[4])
        elif transducer_map[self.item_index] == "PM2.5":
            res = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
            return "%.2f" % float(res.data[0])
        elif transducer_map[self.item_index] == "电量":
            res = rospy.wait_for_message("/battery", BatteryState, timeout=5)
            voltage, electricity, temperature, battery = res.voltage, res.current, res.design_capacity, res.percentage
            return int(battery)
        else:
            return 0


