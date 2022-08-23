#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/6 14:37
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : robot_move.py
# @description: "机器人移动服务"
import sys
import time
import rospy
import actionlib
import func_timeout
from func_timeout import func_set_timeout
from modules.move_stop import Stop_move
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from scipy.spatial.transform import Rotation as R # 用于求解欧拉角到四元数的变换结果
from std_msgs.msg import Int8
sys.path.append("/opt/moss_robot/lib/dispatch_ips_web")
from configs.log import logger


class RobotMove:
    """
    机器人移动服务
    """
    def __init__(self, position, move_type):
        self.x, self.y, self.yaw, = position.get("x"), position.get("y"), position.get("yaw")
        self.move_type = move_type
        self.move_state = 0

    def start(self):
        """
        开始移动
        position（位置）: {"x": "0.42", "y": "0.02", "yaw": "-0.04"}
        obstacle（避障）: 0/1
        move_type（移动模式）: forward_move、detour_move、back_charging、back_move
        """
        now_state = self.move_states()
        if now_state == "0":
            logger.info("行走完毕...")
            return True
        return False

    def move_states(self):
        logger.info("机器人将前往({},{},{})，运动模式为:{}".format(self.x, self.y, self.yaw, self.move_type))
        # 移动失败时，加入超时重试
        for i in range(2):
            try:
                ret = self.move_main()
                return ret
            except func_timeout.exceptions.FunctionTimedOut as e:
                logger.error(e)
                logger.error("移动超时")
                if i:
                    return False
                else:
                    Stop_move()
                    logger.info("尝试重试")

    @func_set_timeout(50)
    def move_main(self):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        client.wait_for_server()
        goal = MoveBaseGoal()
        # 0x01:start->goal直线运动
        # 0x02:start->goal对准充电桩
        # 0x03:start->goal直线后退运动
        # 0x11:任意路径规划运动
        # 0x21 直线路过点向前运动
        # 0x23 直线路过点向后运动
        # 0x0X 转16进制
        start_type = {
            "forward_move": 0x01,
            "back_charging": 0x02,
            "back_move": 0x03,
            "detour_move": 0x11,
            "forward_skip_move": 0x21,
            "back_skip_move": 0x23
        }
        if self.move_type == "forward_move":
            self.move_state = 1

        elif self.move_type == "back_charging":
            self.move_state = 2

        elif self.move_type == "back_move":
            self.move_state = 3

        elif self.move_type == "detour_move":
            self.move_state = 4

        elif self.move_type == "forward_skip_move":
            self.move_state = 5
            pass

        elif self.move_type == "back_skip_move":
            self.move_state = 6
            pass

        else:
            self.move_state = 0

        self.talker()

        action_id = start_type.get(self.move_type)
        goal.target_pose.header.stamp.secs = action_id
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position = Point(float(self.x), float(self.y), 0.0)

        data = R.from_euler('zyx', [float(self.yaw), 0, 0]).as_quat()  # 返回的是 w x y z
        goal.target_pose.pose.orientation = Quaternion(*data)
        # logger.info(goal)
        logger.info("正在前往目标位置......")
        client.send_goal(goal, done_cb=self.done_cb)
        finished_within_time = client.wait_for_result(rospy.Duration(3600))
        if not finished_within_time:
            client.cancel_goal()
            rospy.logerr("ERROR:Timed out achieving goal")
            return "2"
        else:
            state = client.get_state()
            if self.move_type == "back_charging":
                pass
            if state == 3:
                rospy.loginfo("Goal succeeded!")
                return "0"
            else:
                rospy.logerr("Goal failed with error code:back_charging:" + str(state))
                return "1"

    def talker(self):
        # rate =rospy.Rate(40)# 10hz  设置发布频率
        for i in range(10):
            pub = rospy.Publisher('back_chatter', Int8, queue_size=10)  # 发布消息到话题 chatter 中,队列长度10
            pub.publish(self.move_state)
            # while not rospy.is_shutdown():  # 当没有异常关闭时候执行如下程序(防止ctrl+c 终止程序)
            # 发布字符串
            # rate.sleep()  # 配合发布频率的休眠

    @staticmethod
    def done_cb(state, result):
        rospy.loginfo('Finished in state %i' % state)
        # rospy.loginfo('actionStatus: %i' % result.actionStatus)


if __name__ == '__main__':
    rospy.init_node('move_state')
    robot_move = RobotMove({"x": "-0.15", "y": "0.02", "yaw": "-0.08"}, 0, "back_charging")
    robot_move.start()
