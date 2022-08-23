db_mysql = {
    'host': '192.168.10.100',
    'port': '3306',
    'database': "v5.3_robot",
    'user': "root",
    'pwd': "mengpa@..",
}
db_redis = {
    'host': '192.168.10.100',
    'password': "123",
    'port': 6379,
    'db': 0,
    'max_connections': 5
}
temp_ip = {
    'host': '192.168.10.230'
}
fourfold_ip = {
    "lower_fourfold": {
        'host': '192.168.10.211',
        'port': '80',
        'username': 'admin',
        'password': 'Admin123',
    },
    "upper_fourfold": {
        'host': '192.168.10.212',
        'port': '80',
        'username': 'admin',
        'password': 'Admin123',
    }
}
optical_ip = {
    'host': '192.168.9.120',
    'port': '1080',
    'username': 'admin',
    'passwd': 'Admin123',
    'id': "1"
}
industry_api = {
    'host': '192.168.10.100',
    'port': "8088",

}
pointer_ips = {
    'host': '192.168.10.134',
    'port': '8087',
}
analysis_api = {
    'host': '192.168.10.100',
    'port': '8089',

}
auto_door = {
    "auto_door_ip": "192.168.9.10",
    "auto_door_port": "1"
}
db_rabbit_mq = {
    "host": "192.168.10.100",
    "port": 5672,
    "vhost": "/",
    "user": "admin",
    "passwd": "admin",
    "robot_exchange": "robot",
    "robot_warn": "warn",
    "warn_queue": "alarm",
    "warn_rout_key": "warn_info",

    #:robot，绑定队列taskState，routingKey为taskStatus

    "robot_task": "robot",
    "task_queue": "taskState",
    "task_rout_key": "taskStatus",

    "rout_send_robotStatus": "robotStatus",
    "queue_send_robotStatus": "robotStatus",
    "rout_send_robotTask": "robotTask",
    "queue_send_robotTask": "robotTask",

    "rout_ex": "robot",
    "rout_send_robotVoice": "robotVoice",
    "queue_send_robotVoice": "robotVoice",

    "test_rout_key": "robot_test",
    "test_queue": "robot_test",

    # 1)路由名为:warn，绑定队列 alarm,routingKey 为 warn_info
}

# 1)路由名为:robot，绑定队列send_serverid，routingKey为robotStatus

# 机器人主控密码
robot_password = "mengpa@.."

industrial_id = {
    "upper_camera": "00G32320516",
    "lower_camera": "00D72626140"
}
industrial_daheng_id = {
    "upper_sn_id": "HCM22130006",
    "lower_sn_id": "HCM22130003"
}

ips_data = {
    'host': '192.168.10.100',
    'port': '8089',
}

file_path_base = {
    "file_path_base": "/home/robot/Dev/dispatch_ips_web/",
    "file_dispatch_path": "/home/robot/Dev/dispatch_v5.3/"
}

# AGX服务
agx_service = {
    "host": "192.168.10.134",
    "port": "8086"
}

# 算法服务
arithmetic_service = {
    "host": "192.168.10.134",
    "port": "8087"
}


# 图片缩略倍数
multiple = 0.5

# 机器人类型
robotId = 1

# 工业相机类型
industrial_camera_type = {
    "Hikvision": True,  # 海康
    # "Daheng": True  # 大恒
}

# 工业相机上下
industrial_camera_position = {
    "up": 1,
    "down": 0
}

# 传感器映射关系
transducer_map = ["温度", "湿度", "H2S", "SO2", "噪声", "PM1", "风速", "CO2", "TVOC", "甲醛", "PM10", "PM2.5", "电量"]


# rfid扫描时间
rfid_time = 9

# 二维码识别算法服务URL
qrcode_identify_url = "http://10.173.26.253:8090/algoritmic_server/qrcode"

rfid_query_name = "RFID"
qrcode_query_name = "二维码"

# 热成像相机
hot_camera = {
    "hot_camera_type": "1" # 1:大力热成像; 2:海康热成像
}