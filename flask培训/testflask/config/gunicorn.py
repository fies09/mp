#!/usr/bin/env python3
# coding:utf-8

# ------------------------------------
# Configs on runing V5.2.2 with Gunicorn
# Updated by lee on 2021.09.10
# DEV:
# ------------------------------------


import os
import gevent.monkey
import multiprocessing

# 配置
IP = '0.0.0.0'
PORT = '8082'

bind = str(IP)+':'+ str(PORT)  # 服务代理地址
workers = 2               # 启动的进程数 # multiprocessing.cpu_count()*2+1 
worker_class = 'gevent'   # 有eventlet, gevent, tornado, gthread, *缺省sync
worker_connections = 50   # 请求最大连接数 
# threads = 4             # 线程数字 仅在'gthread'有效，建议取值2-4x$(NUM_CORES)
daemon = False            # 是否守护进程
timeout = 60*2            # 超时时间(进程超过时间没有响应会重启 *缺省30s)
pidfile = './damon.pid'   # 进程号存储文件
proc_name = 'test'       # 进程名，查询状态ps -ef |grep [proc_name] 
debug = False             # 是否调试模式
loglevel = 'error'
logfile = './logs/error.log'
# x_forwarded_for_header = 'X-FORWARDED-FOR'
