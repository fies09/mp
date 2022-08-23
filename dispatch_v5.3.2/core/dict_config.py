algorithm_dic = {"温度": "robot_seven_temperature", "湿度": "robot_seven_humidity",
                 "甲醛": "robot_seven_formaldehyde", "CO2": "robot_seven_co2",
                 "TVOC": "robot_seven_tvoc", "PM2.5": "robot_seven_pm25",
                 "PM10": "robot_seven_pm10", "SO2": "robot_so2",
                 "H2S": "robot_h2s", "PM1": "robot_pm1", "噪声": "robot_noise",
                 "battery_voltage": "battery_voltage", "battery_current": "battery_current",
                 "battery_wendu": "battery_wendu", "battery_power": "battery_power"}

screen_list = ['battery_voltage', 'battery_current', 'battery_wendu', 'battery_power', '', '温度', '湿度', '噪声', 'PM2.5',
               'CO2', 'H2S', 'SO2', 'TVOC']

lamp_dic = {"绿灯数量": 'green', "红灯数量": "red", "黄灯数量": "yello", "蓝灯数量": "blue"}

temp_dic = {"最高温度": "high_temp", "最低温度": "mini_temp", "平均温度": "aver_temp"}

voice_dict = {"start_robot": "开机", "start_inspect": "开机自检",
              "auto_inspection": "自动巡检", "battery_low": "电量不足",
              "follow_engineer": "随工-选择机柜", "cmd_arrive": "到达指定路径点",
              "start_video": "随工-开始录像", "end_video": "随工-结束录像", "cmd_location": "随工-到达指定路",
              "robot_stop": "停止", "robot_return": "一键返回", "obstacle": "避障", "red_stop": "红色急停",
              "urgent_stop": "急停", "robot_move": "到达路径点"}

gongye_lamp_dic = {"green": '绿灯数量', "red": "红灯数量", "yello": "黄灯数量", "blue": "蓝灯数量"}
algorithm_model = {"LCD": 0, "LED": 1, "NUMBER": 2}
switch_status = {0: "2b", 1: "2c", 2: "3b"}

camera_type = {"industry_camera": ["工业相机上", "工业相机下"], "fourfold_camera": ["升降机相机上", "升降机相机下"]}

# 点位信息
# 数据采集点1,充电预备点3,充电点4,标志点5,开门点6,关门点7,巡检点8
position_type = {
    "null": 0,  # 空点
    "examine": 1,  # 巡检点
    "welcome": 9,  # 迎宾点
    "ready": 3,  # 充电预备点
    "charge": 4,  # 充电点
}
# 是否绕障: 0-不饶障碍  1-绕障
obstacle_type = {
    "YES": 1,  # 绕障
    "NO": 0  # 不饶障碍
}
# 运动状态
move_state = {
    "arrive": 0,  # 到达
    "overtime": 2  # 超时
}
# 机器人点位状态
current_state = {
    "YES": "1",  # 机器人在此点
    "NO": "0"  # 机器人不在此点
}
# 参观任务redis执行状态.
visit_state = {
    "wait": 0,  # 等待
    "start": 1,  # 开始
    "stop": 2,  # 停止
    "return": 3,  # 返回
}
# 机器人语音状态
voice_status = {
    "speaking": -1,  # 机器人正在讲话
    "dont_speak": 0  # 机器人没在讲话
}
# 机器人运动模式
robot_move_model = [
    "forward_move",  # 直线运动模式
    "back_charging"  # 返回充电桩模式
]
# 机器人任务执行状态
# 0 等待开始，1执行中，2 已完成，3 未执行， 4 正在录像，5 已到达随工点,6 执行失败
task_status = {
    "wait": 0,
    "action": 1,
    "finish": 2,
    "no_action": 3,
    "video": 4,
    "follow": 5,
    "fail": 6,
    "close_video": 7,
    "visit": 8,
    "welcome": 9
}
# 任务类型
task_type = {
    "auto": 1,  # 自动巡检
    "stop": 2,  # 停止
    "return": 3,  # 一键返回
    "return_charge": 4,  # 返回充电
    "follow": 5,  # 随工
    "appoint": 6,  # 指定到达
    "check": 7,  # 设备盘点
    "restart": 8,  # 机器人重启
    "visit": 9  # 参观
}
