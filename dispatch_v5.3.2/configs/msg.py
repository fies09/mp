#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/18 15:00
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : msg.py
# @description: "机器人状态码"


# 机器人自定义错误码
robot_code = {
    # 服务正常
    "normal": 0,
    # 软件服务错误
    "server_error": 1,
    # 硬件调用错误
    "device_error": 2,
    # 硬件被占用
    "used": 3
}
