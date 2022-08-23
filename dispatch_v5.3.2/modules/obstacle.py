#!/usr/bin/env python3
# coding:utf-8
import rospy
import json
import time
from std_msgs.msg import UInt8, Bool, Int8, UInt16


def obstacle():
    try:
        obs = rospy.wait_for_message("/obstacle", UInt16, timeout=1)
    except:
        return False
    if obs.data != 0:
        return True
    else:
        return False


# 防跌落
def obstacle_fall():
    try:
        obs = rospy.wait_for_message("/fall", Int8, timeout=1)
    except:
        return False
    if obs.data == 0:
        return True
    else:
        return False


# 超声波避障
def obstacle_avoidance():
    try:
        obs = rospy.wait_for_message("/avoidance", Int8, timeout=1)
    except:
        return False
    if obs.data == 1:
        return True
    if obs.data == 4:
        return True
    else:
        return False


# 急停
def urgent_stop():
    try:
        is_stop = rospy.wait_for_message("/isStop", Bool, timeout=1)

    except Exception as e:
        return False
    if is_stop.data:
        return True
    else:
        return False


if __name__ == '__main__':
    rospy.init_node('kjdsnvlk')
    while 1:
        time.sleep(0.3)

    # while 1:
    #     print(obstacle())
    #     time.sleep(3)
