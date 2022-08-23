# coding:utf-8

#---------------------
# 请求端脚本
# 依赖requests
#---------------------

import json
import requests

# 服务端地址
IP = '192.168.9.168'
PORT = '8082'


def requet_hello():
    """请求示例1: hello
    """
    # 服务地址
    URL = "http://" + IP + ":" + PORT + "/" 
    # 请求头
    headers = {'Content-Type': 'application/json'}
    # 执行请求
    sget = requests.get(URL,headers=headers)
    # 打印返回结果
    print("请求hellow返回结果: ", sget.text)
        

def requet_app():
    """请求示例2: 先后请求app的两个功能
    """
    # 服务地址1
    URL1 = "http://" + IP + ":" + PORT + "/" 
    # 请求头
    headers = {'Content-Type': 'application/json'}
    # 执行请求
    sget1 = requests.get(URL1,headers=headers)
    # 打印返回结果
    print("请求app返回结果: ", sget1.text)

    # 请求2--------------------------------------------------
    name = 'Jack'
    URL2 = "http://" + IP + ":" + PORT + "/user/%s" % name 
    sget2 = requests.get(URL2,headers=headers)
    print("请求app的hello返回结果: ", sget2.content)


        
def requet_app2():
    """请求示例3: 先后请求app的两个功能
    """
    # 服务地址1
    URL1 = "http://" + IP + ":" + PORT + "/" 
    # 请求头
    headers = {'Content-Type': 'application/json'}
    # 执行请求
    sget1 = requests.get(URL1,headers=headers)
    # 打印返回结果
    print("请求app2返回结果: ", sget1.text)

    # 请求2--------------------------------------------------
    name = 'Jack'
    URL2 = "http://" + IP + ":" + PORT + "/user/%s" % name 
    sget2 = requests.get(URL2,headers=headers)
    print("请求app2的hello返回结果: ", sget2.content)


    # 请求3--------------------------------------------------
    name = 'LEE'
    # 装填参数
    cmd = {'name': name}
    # cmd2 = {}     # 装填错误参数
    
    URL3 = "http://" + IP + ":" + PORT + "/login"
    sget3 = requests.get(URL3,headers=headers, data=json.dumps(cmd))
    print("请求app2的login返回结果: ", sget3.content)

if __name__=='__main__':
    
    # requet_hello()

    # # requet_app()

    requet_app2()
