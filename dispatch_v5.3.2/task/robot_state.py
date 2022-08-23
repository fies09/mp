import json
import time
from configs import db_rabbit_mq
from core.rabbitmq_db import RabbitPublisher
from core.redis_interactive import RedisPipe
from task.light_al import post_voice_str
from modules.move_state import movstate
from modules.obstacle import obstacle, urgent_stop, obstacle_fall, obstacle_avoidance
from task.robot_action import move_state
from task.robot_algorithm import RobotAlgorithm
from configs.log import logger
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_robot_config import DBRobotConfig
from schema.db_robot_position import DBRobotPosition
from utils.get_soc import start_client
from task.hardware_exception import schedule_test
# 时实更新机器人状态


class RobotState(object):
    def __init__(self):
        self.time_num = 0
        self.stop_time = 0
        self.is_stop = False
        self.robotalgorithm = RobotAlgorithm()
        self.redis_action_task = RedisPipe("ActionTask")
        self.redis_push_data = RedisPipe("Robot_state")
        self.db_robot_position = DBRobotPosition()
        self.db_inspect_project_detail = DBInspectProjectDetail()
        self.db_robot_config = DBRobotConfig()

    def robot_now_state(self):
        config_dict = {}
        battery_power_dic = {"name": "battery_power"}
        battery_temp_dic = {"name": "battery_wendu"}
        battery_state_dic = {"name": "battery_state"}

        # battery info{电池剩余电量，电池温度，电池是否在充电}
        try:
            battery_power = self.robotalgorithm.robot_battery_power(battery_power_dic)
            battery_temp = self.robotalgorithm.robot_battery_temp(battery_temp_dic)
            battery_state = self.robotalgorithm.robot_battery_temp(battery_state_dic)
        except Exception as e:
            logger.error(e)
            battery_power = 80
            battery_temp = 30
            battery_state = 1

        # robot now focus location
        position = self.db_robot_position.get_now_location_all("1")
        if not position:
            position = {"name": None}

        # robot now command data
        #
        cmd = self.db_inspect_project_detail.get_top1()
        if not cmd:
            cmd = {}
            cmd_name = ""
        else:
            cmd_name = cmd.get("task_type_name", "")
        # alive obstacle data
        obstacle_state = obstacle()
        fall = obstacle_fall()
        avoidance = obstacle_avoidance()
        stop_state = urgent_stop()
        if self.is_stop != stop_state:
            if stop_state:
                post_voice_str("urgent_stop")
                self.is_stop = True
            else:
                post_voice_str("结束急停", 1)
                self.is_stop = False

        # robot now coordinate data
        x, y, yaw = movstate()

        # robot hardware data
        soc_state = start_client()

        """缺少数据 用于自检
        是否充电 0未充电 1充电 is_charged
        避障雷达状态 0正常 1故障 radar_state
        电机状态 0正常 1故障 motor_state
        imu状态 0正常 1故障 imu_state
        气体传感器状态 0正常 1故障 gas_sensor_state
        激光雷达状态 0正常 1故障 lidar_state
        相机状态 0正常 1故障  thermal_state
        风速传感器状态 0正常 1故障 wind_state
        麦克风状态 0正常 1故障  mic_state
        放电仪状态 0正常 1故障  discharge_state
        """
        config_dict["battery_level"] = battery_power
        config_dict["battery_temperature"] = battery_temp
        config_dict["is_charged"] = battery_state
        config_dict["num"] = cmd.get("num", 0)
        config_dict["inspect_project_detail_id"] = cmd.get("inspect_project_detail_id", 0)

        config_dict["cpu_percent"] = int(float(soc_state.get("use_cpu")))
        config_dict["cpu_temperature"] = int(float(soc_state.get("use_temp")))
        config_dict["robot_temperature"] = int(float(soc_state.get("use_temp")))
        config_dict["disk_percent"] = int(float(soc_state.get("hdd_use")))
        config_dict["memory_percent"] = int(float(soc_state.get("memory_use")))

        config_dict["is_stop"] = stop_state
        config_dict["task_type"] = cmd_name

        config_dict["position_name"] = position.get("name")
        if obstacle_state or fall or avoidance:
            obstacle_state = 1
        else:
            obstacle_state = 0
        config_dict["obstacle_state"] = obstacle_state

        config_dict["position"] = {"x": x, "y": y, "yaw": yaw}

        # 判断当前计划任务的执行状态
        task_status = cmd.get("status", 0)
        # 计划执行详情状态
        config_dict["project_detail_status"] = task_status
        if task_status in [1, 4, 5, 7, 8, 9]:
            task_status = 1
        else:
            task_status = 2

        config_dict["task_state"] = task_status

        robot_cmd_position = RedisPipe("RobotCmdPosition").get_data()
        if robot_cmd_position:
            config_dict["robot_cmd_position"] = eval(robot_cmd_position.decode())


        return_ret = self.db_robot_position.get_battery_point()
        if return_ret:
            config_dict["position_type"] = return_ret.get("type")
            config_dict["robot_position_id"] = return_ret.get("robot_position_id")

        # logger.info(cmd)
        cmd_state = cmd.get("status")
        # config_dict["status"] = 2
        if cmd_state:
            if cmd_state in [1, 4, 7, 8, 9]:
                config_dict["status"] = 4
            else:
                config_dict["status"] = 2

        if obstacle_state or fall or avoidance:
            config_dict["status"] = 1
        if battery_state == 0:
            config_dict["status"] = 0
        if stop_state:
            config_dict["status"] = 3
            config_dict["is_stop"] = 1
        else:
            config_dict["is_stop"] = 0
        return config_dict

    def insert_data(self):
        while True:
            time.sleep(1)
            try:
                data_state = self.robot_now_state()
                # logger.info("============ 状态更新成功 ============")
                self.redis_push_data.set_data(data_state, 40)
                robot_exchange = db_rabbit_mq.get("robot_exchange")
                send_serverid = db_rabbit_mq.get("rout_send_robotStatus")
                quque = db_rabbit_mq.get("queue_send_robotStatus")
                RabbitPublisher.run(robot_exchange, send_serverid, quque, data_state)
                config_data = self.db_robot_config.get_by_robot_id("1")
                if config_data:
                    elevator = config_data.get("tags").get("elevator")
                    data_state["elevator"] = elevator
                    info = {"tags": data_state}
                    self.db_robot_config.update({"robot_id": 1}, info)
            except Exception as e:
                logger.info("============ 状态更新失败 ============")
                logger.error(e)
                time.sleep(5)


class RobotObstacle(object):
    def __init__(self):
        self.time_num = 0
        self.stop_time = 0
        self.is_stop = False
        self.db_robot_position = DBRobotPosition()

    def start(self):
        while True:
            time.sleep(1)
            obstacle_state = obstacle()
            fall = obstacle_fall()
            avoidance = obstacle_avoidance()
            # logger.info("obstacle_state:{},fall:{},avoidance:{}".format(obstacle_state, fall, avoidance))
            if obstacle_state or fall or avoidance:
                logger.info("出现避障！")
                if self.time_num >= 60 or self.time_num == 0:  # set broadcast interval time
                    self.time_num = 0
                    post_voice_str("obstacle")
                if self.stop_time >= 40:  # 设置避障3返回为40*3=120S，
                    return_ret = self.db_robot_position.get_battery_point()
                    if return_ret:
                        logger.info("避障时间过长，开始执行返回到充电点")
                        post_voice_str("避障时间过长，开始执行返回到充电点", 1)
                        # 开始向告警信息表插入数据
                        data_value = "避障"
                        schedule_test(data_value)
                        if move_state(return_ret.get("robot_id"), return_ret.get("robot_path_id"), move_state=1):
                            logger.info("已返回到充电点")
                            self.time_num = 0
                            self.stop_time = 0
                self.stop_time += 1
                self.time_num += 1
            else:
                self.time_num = 0
                self.stop_time = 0


def state_data():
    RobotState().insert_data()


def obstacle_listen():
    RobotObstacle().start()


if __name__ == '__main__':
    import rospy

    rospy.init_node("Mian_node122", disable_signals=False)
    state_data()

    # {'battery_level': 31, 'battery_temperature': 30, 'is_charged': 0, 'num': 0, 'inspect_project_detail_id': 1560287961,
    #  'cpu_percent': 26, 'cpu_temperature': 70, 'robot_temperature': 70, 'disk_percent': 63, 'memory_percent': 11,
    #  'is_stop': 0, 'task_type': '指定到达', 'position_name': '充电点', 'obstacle_state': 0,
    #  'position': {'x': -0.022146903862281547, 'y': -0.015502392050356147, 'yaw': -0.12229316732078979}, 'task_state': 2,
    #  'position_type': 4, 'status': 0}
    # return_ret = DBRobotPosition().get_battery_point()
    # print(return_ret)
