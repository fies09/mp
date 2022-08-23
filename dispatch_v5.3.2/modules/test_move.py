#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from quaternions import Quaternion as Qt  
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from scipy.spatial.transform import Rotation as R # 用于求解欧拉角到四元数的变换结果

# from std_msgs.msg import String
from std_msgs.msg import Int8
import numpy as np


# from modules.move_stop import Stop_move
# from configs.log import logger


def DoneCb(state, result):
    rospy.loginfo('Finished in state %i' % state)
    rospy.loginfo('actionStatus: %i' % result.actionStatus)


def ActiveCb():
    rospy.loginfo("Goal just went active")


def FeedbackCb(feedback):
    global client
    '''
    uint8 PENDING=0
    uint8 ACTIVE=1
    uint8 PREEMPTED=2
    uint8 SUCCEEDED=3
    uint8 ABORTED=4
    uint8 REJECTED=5
    uint8 PREEMPTING=6
    uint8 RECALLING=7
    uint8 RECALLED=8
    uint8 LOST=9
    '''
    rospy.loginfo('Current in state %s' % client.get_state())


def move_main(x, y, yaw, type_name):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()
    goal = MoveBaseGoal()
    # 0x01:start->goal直线运动
    # 0x02:start->goal对准充电桩
    # 0x03:start->goal直线后退运动
    # 0x11:任意路径规划运动
    # 0x0X 转16进制
    start_type = {"forward_move": 0x01,
                  "back_charging": 0x02,
                  "back_move": 0x03,
                  "detour_move": 0x11}
    print("{},{},{}".format(x, y, yaw))
    print(type_name)
    if type_name == "forward_move":
        move_state = 1

    elif type_name == "back_charging":
        move_state = 2

    elif type_name == "back_move":
        move_state = 3

    elif type_name == "detour_move":
        move_state = 4
    else:
        move_state = 0

    talker(move_state)

    actionid = start_type.get(type_name)
    print(actionid)
    goal.target_pose.header.stamp.secs = actionid
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position = Point(float(x), float(y), 0.0)

    data = R.from_euler('zyx', [float(0.0), 0, 0]).as_quat()  # 返回的是 w x y z
    goal.target_pose.pose.orientation = Quaternion(*data)
    client.send_goal(goal)
    finished_within_time = client.wait_for_result(rospy.Duration(300))
    if not finished_within_time:
        client.cancel_goal()
        rospy.logerr("ERROR:Timed out achieving goal")
        return "2"
    else:
        state = client.get_state()
        if type_name == "back_charging":
            pass
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
            return "0"
        else:
            rospy.logerr("Goal failed with error code:back_charging:" + str(state))
            return "1"


def talker(state):
    # rate =rospy.Rate(40)# 10hz  设置发布频率
    for i in range(10):
        pub = rospy.Publisher('back_chatter', Int8, queue_size=10)  # 发布消息到话题 chatter 中,队列长度10
        pub.publish(state)
        # while not rospy.is_shutdown():  # 当没有异常关闭时候执行如下程序(防止ctrl+c 终止程序)
        # 发布字符串
        # rate.sleep()  # 配合发布频率的休眠


def move_states(x, y, yaw, type_name):
    try:
        # Stop_move()
        print("开始给底盘发送位置点")
        ret = move_main(x, y, yaw, type_name)
        print("底盘返回状态信息")
        print(ret)
        return ret
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")


if __name__ == '__main__':
    rospy.init_node('move_state')
    loc_list = [
        {"x": 1, "y": -0.3, "yaw": 0.02},
        # {"x": 1, "y": -0.3, "yaw": 0.02},
        # {"x": 1, "y": -0.3, "yaw": 0.02},
        # {"x": 1, "y": -0.3, "yaw": 0.02}
    ]
    # 模式自己切换 前进'forward_move'，后退back_move，返回充电桩 back_charging， 绕障detour_move
    for loc in loc_list:
        move_states(loc.get("x"), loc.get("y"), loc.get("yaw"), 'forward_move')
