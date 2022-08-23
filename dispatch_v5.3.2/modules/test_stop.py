#! /usr/bin/env python3
# coding:utf-8
import os

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from scipy.spatial.transform import Rotation as R  # 用于求解欧拉角到四元数的变换结果

from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import numpy as np


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


def Stop_move():
    print("先停止上次执行的任务！")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp.secs = 0x01
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position = Point(float(1.008), float(-1.316), 0.0)
    # {"x": "1.008", "y": "-1.316", "x_s": "0.000", "y_s": "0.000", "yaw": "-1.584", "yaw_s": "-0.712"}
    # {"x": "1.008", "y": "-1.316", "x_s": "0.000", "y_s": "0.000", "yaw": "-1.584", "yaw_s": "-0.712"}
    data = R.from_euler('zyx', [float(0.0), 0, 0]).as_quat()  # 返回的是 w x y z
    goal.target_pose.pose.orientation = Quaternion(*data)

    # client.cancel_goal()
    try:
        client.cancel_all_goals()  # 停止 删除当前目标点
        # client.send_goal(goal)
        # print("sendgoal")
        # client.cancel_goal()
        return True
    except Exception as e:
        print(e)
        return False


if __name__ == '__main__':
    rospy.init_node("Mian_node1", disable_signals=False)
    Stop_move()
