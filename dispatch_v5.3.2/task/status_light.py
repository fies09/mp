#!/usr/bin python3
# coding:utf-8
import rospy
from std_msgs.msg import Float64MultiArray, Bool, Int8, UInt8, Int32MultiArray, UInt16
from sensor_msgs.msg import BatteryState
from ptz_service.srv import Greeting, GreetingRequest
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
# from configs.log import logger
import json
import time


# 发布数组类型 第一位为灯 第二位为闪烁否
# 两种想法 一 本程序自动订阅其状态信息然后判断发布 二 在其他程序检测到出问题时发布
from configs.log import logger


def status_listen():
    while True:
        try:
            battery = rospy.wait_for_message("/battery", BatteryState, timeout=5)
            pow_status = int(battery.charge)
            if battery.percentage is None:
                battery_status = False
            else:
                battery_status = True
        except Exception as e:
            pow_status = 1
            battery_status = False
            logger.error(e)

        try:
            avoidance = rospy.wait_for_message('/avoidance', Int8, timeout=5)
            avoidance_status = avoidance.data
        except Exception as e:
            avoidance_status = False
            logger.error(e)

        try:
            obstacle = rospy.wait_for_message('/obstacle', UInt16, timeout=5)
            if obstacle.data is None:
                obstacle_status = False
            elif obstacle.data != 0:
                obstacle_status = True
            else:
                obstacle_status = False
        except Exception as e:
            obstacle_status = False
            # logger.error(e)

        try:
            isStop = rospy.wait_for_message('/isStop', Bool, timeout=5)
            isStop_status = isStop.data
        except Exception as e:
            isStop_status = False
            logger.error(e)
        #print(obstacle_status, avoidance_status, battery_status, isStop_status) 
        if obstacle_status:
            main(3, 0)
            continue
        else:
            if avoidance_status !=9:
                main(3, 0)
                continue
            else:
                if battery_status == False:
                   main(1, 0)
                   continue
                else:
                    if isStop_status:
                       main(3, 0)
                       continue
                    else:
                       main(2, 0)




# 回调函数
def main(colour, twinkle):
    pub = rospy.Publisher('/status_light', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    status = [colour, twinkle]
    status = Int32MultiArray(data=status)
    # print(colour, twinkle)
    for i in range(3):
        pub.publish(status)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('status_light_main', anonymous=True)
    try:
        status_listen()
    except rospy.ROSInterruptException:
        pass
