#!/usr/bin/env python3
# coding:utf-8
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
import math

def movstate():
    try:
        movstate = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=5)
        x = movstate.pose.pose.position.x
        y = movstate.pose.pose.position.y
        fx = movstate.pose.pose.orientation.x
        fy = movstate.pose.pose.orientation.y
        fz = movstate.pose.pose.orientation.z
        fw = movstate.pose.pose.orientation.w
        list1 = [fx, fy, fz, fw]
        r = R.from_quat(list1)
        w = r.as_euler('xyz', degrees=True)
        ow = math.radians(w[2])
        return x, y, ow
    except Exception as e:
        return 0.0, 0.0, 0.0


if __name__ == '__main__':
    rospy.init_node('listener_node_11')
    a, b, z = movstate()
    print(a, b, z)
