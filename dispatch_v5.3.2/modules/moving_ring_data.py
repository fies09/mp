#! /usr/bin/env python3
# coding:utf-8

import rospy
from std_msgs.msg import Float64MultiArray, String, Float32
from sensor_msgs.msg import BatteryState
from schema.db_robot_position import DBRobotPosition
from configs.log import logger


def Dynamic_environment(con):
    # rospy.init_node('listener_node_hsulfide')
    # 不能多次初始化
    # 温度
    # 温度湿度不在7合一传感器中有单独的传感器 暂时不测试
    if con == '温度':
        wendu = rospy.wait_for_message("/temhum", Float64MultiArray, timeout=5)
        return "%.2f" % float(wendu.data[0])

    # 湿度
    elif con == '湿度':
        temhum = rospy.wait_for_message("/temhum", Float64MultiArray, timeout=5)
        return "%.2f" % float(temhum.data[1])

    # 硫化氢
    if con == 'H2S':
        hsulfide = rospy.wait_for_message("/hsulfide", String, timeout=5)
        return hsulfide.data

    # 噪声
    elif con == '噪声':
        noise = rospy.wait_for_message("/noise", String, timeout=5)
        return noise.data

    # 二氧化硫
    elif con == 'SO2':
        sidoxide = rospy.wait_for_message("/sdioxide", String, timeout=5)
        return sidoxide.data

    # 电池state
    elif con == 'battery_state':
        try:
            battery = rospy.wait_for_message("/battery", BatteryState, timeout=5)
            if not battery:
                return 0
            return int(battery.charge)
        except Exception as e:
            logger.error(e)
            logger.error("电压是否充电获取失败，如果充电点则设为0，否则为1")
            return_ret = DBRobotPosition().get_battery_point()
            if return_ret.get("type") != 4:
                return 1
            return 0
    # 电池电压
    elif con == 'battery_voltage':
        try:
            battery = rospy.wait_for_message("/battery", BatteryState, timeout=5)
            return int(battery.voltage)
        except Exception as e:
            logger.error(e)
            logger.error("电压获取失败，暂定为25")
            return 25
    # 电池电流
    elif con == 'battery_current':
        try:
            battery = rospy.wait_for_message("/battery", BatteryState, timeout=5)
            return abs(int(battery.current))
        except Exception as e:
            logger.error(e)
            logger.error("电流获取失败，暂定为5")
            return 5

    # 电池温度
    elif con == 'battery_wendu':
        try:
            battery = rospy.wait_for_message("/battery", BatteryState, timeout=5)
            return int(battery.design_capacity)
        except Exception as e:
            logger.error(e)
            logger.error("电池温度获取失败，暂定为30")
            return 30

    # 电池剩余电量
    elif con == 'battery_power':
        try:
            battery = rospy.wait_for_message("/battery", BatteryState, timeout=3)
            return int(battery.percentage)
        except Exception as e:
            logger.error(e)
            logger.error("机器人电量获取错误，暂定为80")
            return 80



    # 7合1pm2.5
    elif con == 'PM2.5':
        mp_all = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
        return "%.2f" % float(mp_all.data[0])

    # 7合1CO2
    elif con == 'CO2':
        mp_all = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
        return mp_all.data[1]

    # 7合1TVOC
    elif con == 'TVOC':
        mp_all = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
        return "%.2f" % float(mp_all.data[2])
    # 7合1PM10
    elif con == 'PM10':
        mp_all = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
        return "%.2f" % float(mp_all.data[4])

    # 7合1PM1
    elif con == 'PM1':
        mp_all = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
        return "%.2f" % float(mp_all.data[3])

    # 7合1甲醛
    elif con == '甲醛':
        mp_all = rospy.wait_for_message("/mp_all", Float64MultiArray, timeout=5)
        return mp_all.data[5] * 0.001

    # 升降杆实时高度
    elif con == 'lift':
        lift_data = rospy.wait_for_message("/elevatorPosition", Float32, timeout=5)
        return lift_data.data
    else:
        return False


if __name__ == '__main__':
    rospy.init_node('listener_node_hsulfide1111')
    a = Dynamic_environment('battery_current')
    print(a)
    # b = Dynamic_environment('噪声')
    # print(b)
    # c = Dynamic_environment('温度')
    # print(c)
    # a,b,c = movstate()
    #     # print(a,b,c)
    # mp_all = rospy.wait_for_message("/obstacle", Float64MultiArray, timeout=None)
    # print(mp_all)
