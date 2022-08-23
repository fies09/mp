#! /usr/bin/env python3
import sys
import time

import rospy
import actionlib

from configs.log import logger

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from actionlib_msgs.msg import GoalStatus
from mprobot_msgs.msg import commonAction, commonGoal
import json


def DoneCb(state, result):
    rospy.loginfo('Finished in state %i' % state)
    rospy.loginfo('Toal dish cleaned: %i' % result.actionStatus)
    return state


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


def Main(actionId, position):
    try:
        client = actionlib.SimpleActionClient('/elevator_ctl', commonAction)
        client.wait_for_server(rospy.Duration(5))

        goal = commonGoal()
        goal.actionId = actionId

        action_params = {}
        # 0:return 0  1:move to the position  2:move to to distance
        action_params['type'] = int(actionId)
        action_params['position'] = int(position)
        action_params['distance'] = int(position)

        goal.actionArg = json.dumps(action_params)
        client.send_goal(goal, done_cb=DoneCb, active_cb=ActiveCb, feedback_cb=FeedbackCb)

        client.wait_for_result(rospy.Duration(10))

        state = client.get_result().actionStatus
        if state == True:
            rospy.loginfo("Goal succeeded!")
            return True
        else:
            rospy.logerr("Goal failed with error code:升降杆:")
            return False
    except Exception as e:
        logger.error(e)
        # rospy.signal_shutdown('clear')
        # rospy.spin()
        return False


if __name__ == '__main__':
    rospy.init_node('elevator_driver_client123')
    # Main(2,-200)
    for i in range(10000):
        Main(1, i)

    '''
    time.sleep(6)
    Main(1,160)
    time.sleep(6) 
    Main(1,320)
    time.sleep(6)
    Main(1,480)
    time.sleep(6)
    Main(1,640)
    time.sleep(6)
    Main(1,800)
    '''

    # b = Main(1,400)
    # c = Main(1,500)
    # d = Main(1,600)

    # print(a,b,c,d)
    # list_a = [0, 410, 820]
    # for num in list_a:
    # a = Main(1, num)
    # print(a)
    # time.sleep(5)
