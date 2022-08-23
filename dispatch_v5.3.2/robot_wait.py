import json
from signal import alarm
import time
import rospy
from threading import Thread
from configs import db_rabbit_mq
from configs.log import logger
from core.redis_interactive import RedisPipe
from task.light_al import post_voice_str
from modules.moving_ring_data import Dynamic_environment
from task.robot_action import move_state
from task.robot_face import insert_face_alarm
from task.robot_implement import RobotImplement
from task.robot_init import init_positions
from task.robot_mq import RabbitComsumer
from task.robot_state import state_data
from task.robot_state import obstacle_listen
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_inspect_project_task import DBInspectProjectTask
from schema.db_robot_task_type import DBRobotTaskType
from schema.db_robot_position import DBRobotPosition
from schema.db_robot_config import DBRobotConfig
from schema.db_inspect_project import DBInspectProject
from schema.db_robot_path_position_relation import DBRobotPathPositionRelation
from server.API import API
from modules.robot_hardware_server import RobotDeviceStatus
from task.hardware_exception import schedule_test
from task.robot_inspect2 import inspect_main
rospy.init_node("Mian_node", disable_signals=False)

robot_id = 0
robot_path_id = 0
action = 0


class WaitTask(object):
    def __init__(self):
        self.redis_wait_task = RedisPipe("WaitTask")
        self.redis_action_task = RedisPipe("ActionTask")
        self.wait_task_state = Thread(target=self.wait_task).start()
        # 每次执行完对redis值进行撤销
        self.redis_wait_task.set_data({"task_num": 0})
        self.redis_action_task.set_data({"ActionTask": 0})

        self.redis_data = {"task_num": 0}
        self.robot_impolement = RobotImplement()
        self.task_data = {}

    # 判断任务是否执行
    def task_type(self, data):
        # 判断是否跨路径，并且正在执行任务。
        alive_action = eval(self.redis_action_task.get_data().decode()).get("ActionTask")
        alive_positon_path = self.alive_route(data)
        self.task_data = data.get("task_data")
        if data.get("task_type_name") not in ["停止", "一键返回", "指定到达"]:
            if alive_action and not alive_positon_path:
                logger.info("*" * 100)
                logger.info("在执行过程中，不同路径下不能切换")
                self.del_task(data, str_data="在执行过程中，不同路径下不能切换")
                return False

        # 0为手动任务1为循环任务
        if data.get("task_status") == 0 or data.get("task_status") == 2:
            if data.get("task_type_name") not in ["停止", "一键返回"]:
                now_battery = Dynamic_environment("battery_power")
                if data.get("minimum_battery") > now_battery:
                    logger.info("电量不足，无法执行当前任务！当前电量：{}".format(now_battery))
                    self.del_task(data, str_data="电量低，不执行当前{}任务".format(str(data.get("task_type_name"))))
                    return False
            action_now_task = Thread(target=self.execute_task, args=(data,))
            action_now_task.start()

            # 定位为没有等待任务
            # self.wait_task_del()
            self.redis_wait_task.set_data(str(self.redis_data))
            self.redis_action_task.set_data(str({"ActionTask": 1}), 7200)
        else:
            # 判断是否有任务执行
            # 有正在执行任务
            battery_power = Dynamic_environment("battery_power")
            logger.info(battery_power)
            data["minimum_battery"] = 0
            if data.get("minimum_battery") < Dynamic_environment("battery_power"):
                if eval(self.redis_action_task.get_data().decode()).get("ActionTask"):
                    logger.info("存在正在执行的任务")
                    self.del_task(data, str_data="存在正在执行的的任务，该任务被放弃")
                # 没有正在执行的任务
                else:
                    action_task = Thread(target=self.execute_task, args=(data,))
                    action_task.start()

                    # 定位为没有等待任务
                    self.redis_wait_task.set_data(str(self.redis_data))
                    self.redis_action_task.set_data(str({"ActionTask": 1}), 7200)
            else:
                if self.redis_wait_task.get_data():
                    redis_data = eval(self.redis_wait_task.get_data().decode())
                    logger.info(redis_data)
                    if redis_data.get("task_num") == 1:
                        self.del_task(data)
                    else:
                        if data.get("wait_time"):
                            wait_time = data.get("wait_time")
                        else:
                            wait_time = 600
                        self.redis_wait_task.set_data(str(data), wait_time)
                else:
                    if data.get("wait_time"):
                        wait_time = data.get("wait_time")
                    else:
                        wait_time = 600
                    self.redis_wait_task.set_data(str(data), wait_time)

    def wait_task_del(self):
        data = self.redis_wait_task.get_data()
        if not data:
            return False
        task_data = eval(str(data.decode('utf-8')))
        # logger.info(type(task_data))
        logger.info("等待任务抛弃的数据为{}".format(task_data))
        if task_data.get("task_num") == 1:
            inspect_project_detail_id = task_data.get("inspect_project_detail_id")
            info = {"status": 3, "exception_info": "该任务被抛弃"}

            DBInspectProjectDetail(). \
                update({'inspect_project_detail_id': inspect_project_detail_id}, info)
            logger.info("未执行任务的id为{}".format(inspect_project_detail_id))

    # 开始执行任务
    def execute_task(self, task_data=None):
        if not task_data:
            return None
        global robot_id, robot_path_id, action
        robot_id = task_data.get("robot_id")
        robot_path_id = task_data.get("robot_path_id")
        action = 1
        logger.info("开始执行任务")
        # logger.info(task_data)
        self.robot_impolement.select_task(task_data)

        # get_data(task_data)

    # 抛弃此次任务
    def del_task(self, data, str_data=None):
        logger.info(type(data))
        logger.info("当前抛弃的数据为{}".format(data))
        inspect_project_detail_id = data.get("inspect_project_detail_id")
        if str_data:
            info = {"status": 3, "exception_info": str_data}
        else:
            info = {"status": 3, "exception_info": "该任务被抛弃"}

        DBInspectProjectDetail(). \
            update({'inspect_project_detail_id': inspect_project_detail_id}, info)
        logger.info("抛弃任务的id为{}".format(inspect_project_detail_id))

    def alive_route(self, task):
        route_ret = DBRobotPosition().get_alive_action_route(task.get("robot_id"), task.get("robot_path_id"))
        # 如果在同一路径，可以打断
        if route_ret:
            return True
        # 如果在同一路径，不可打断
        return False

    def wait_task(self):
        while True:
            time.sleep(0.2)
            if eval(self.redis_action_task.get_data().decode()).get("ActionTask"):
                continue
            else:
                data = self.redis_wait_task.get_data()
                if not data:
                    continue
                task_data = eval(data.decode('utf-8'))
                if task_data.get("task_num") == 1:
                    if task_data.get("minimum_battery") < Dynamic_environment("battery_power"):
                        continue
                    logger.info(task_data)
                    action_auto_task = Thread(target=self.execute_task, args=(task_data,))
                    action_auto_task.start()
                    self.redis_wait_task.set_data(str(self.redis_data))
                else:
                    continue


def robot_monitor_action():
    db_robot_position = DBRobotPosition()
    while True:
        time.sleep(30)
        if action != 1:
            continue
        last_battery = Dynamic_environment("battery_power")
        logger.info("当前电量：{}".format(str(last_battery)))

        try:
            ret = DBRobotConfig().get_mini_charge()[0]
            mini_charge = ret.get("mini_charge")
        except Exception as e:
            logger.error(e)
            mini_charge = 20
        if last_battery < mini_charge:
            logger.info("=================================== 电量不足 ===================================")
            if Dynamic_environment("battery_state") == 0 or db_robot_position.get_now_position() == 4:
                time.sleep(3)
                continue
            post_voice_str("battery_low")
            # 电量异常处理
            # 开始向告警信息表插入数据
            data_value = "电量"
            schedule_test(data_value)
            # 更新参观监听状态
            redis_visit_state = RedisPipe("VisitState")
            redis_visit_state.set_data("2", 20)
            global robot_path_id, robot_id
            robot_id = int(RedisPipe("robot_id").get_data())
            robot_path_id = int(RedisPipe("robot_path_id").get_data())
            if move_state(robot_id, robot_path_id):
                post_voice_str("已返回到充电点", 1)


def get_data(detail_data):
    data = {}
    if detail_data.get("task_type_name") in ["自动巡检", "参观"]:
        try:
            inspect_project_id = detail_data.get("inspect_project_id")
            inspect_project_data = DBInspectProject().get_task_id(inspect_project_id)
            inspect_project_task_id = inspect_project_data.get("inspect_project_task_id")
            task_data = DBInspectProjectTask().get_by_num(detail_data.get("robot_id"), inspect_project_task_id)
        except Exception as e:
            logger.error(e)
            return False
    else:
        task_data = {}

    data["inspect_project_detail_id"] = detail_data.get("inspect_project_detail_id")
    data["robot_id"] = detail_data.get("robot_id")
    data["task_type_name"] = detail_data.get("task_type_name")

    # 所属路线类别
    data["robot_path_id"] = detail_data.get("robot_path_id")
    # 所属地点的id
    data["robot_position_id"] = detail_data.get("robot_position_id")
    # 避障状态
    data["obstacle"] = detail_data.get("obstacle")
    # data["wait_time"] = detail_data.get("wait_time")
    # '任务类型： 0 手动，1 计划，2.手动计划'
    data["task_status"] = detail_data.get("task_status")
    # 任务是否能被打断
    data["is_break"] = task_data.get("is_break", 0)
    # 执行最低电量
    data["minimum_battery"] = task_data.get("minimum_battery", 20)
    # 等待时长
    if task_data:
        data["wait_time"] = compute_time(task_data.get("wait_time"), task_data.get("unit"))
    # 任务类型id
    data["robot_task_type_id"] = task_data.get("robot_task_type_id")
    return data


def compute_time(wait_time, unit):
    if unit == "0":
        time_data = wait_time * 60
    else:
        time_data = wait_time * 3600
    return time_data


# 判断自动巡检是否在充电点上
def alive_init_point(task_data):
    db_robot_position = DBRobotPosition()
    try:
        alive_point_data = db_robot_position.get_init_point(task_data.get("robot_id"))
    except Exception as e:
        logger.error(e)
        logger.error("判断充电点错误")
        alive_point_data = False
    # logger.info(alive_point_data)
    now_location = db_robot_position.get_now_location_all(task_data.get("robot_id"))
    # 判断是否属于同路径
    if alive_point_data:
        return True
    else:
        # 判断是否在充点电
        if now_location.get("type") != 4:
            return False
        else:
            return True


def init_task_state():
    db_inspect_project_detail = DBInspectProjectDetail()
    rets = db_inspect_project_detail.get_task_state()
    if rets:
        for ret in rets:
            params = {"status": 3}
            inspect_project_detail_id = ret.get("inspect_project_detail_id")
            db_inspect_project_detail.update({'inspect_project_detail_id': inspect_project_detail_id}, params)


def task_main():
    # IPS硬件服务启动
    API().start()
    # 初始化硬件状态
    RobotDeviceStatus().init_device()
    Thread(target=state_data).start()
    Thread(target=obstacle_listen).start()
    Thread(target=wait_rabbit_mq).start()
    Thread(target=robot_monitor_action).start()
    # Thread(target=status_listen).start()
    # Thread(target=monitor_demo).start()
    Thread(target=insert_face_alarm).start()
    init_position_data = DBRobotPathPositionRelation().get_all_data()
    # logger.info(init_position_data)
    if init_position_data:
        robot_path_id = init_position_data.get("robot_path_id")
        # logger.info(robot_path_id)
        init_positions(robot_path_id)
    init_task_state()

    waitTask = WaitTask()
    redis_pipe = RedisPipe()
    db_inspect_project_data = DBInspectProjectDetail()
    # 执行自检功能
    post_voice_str("开始执行自检功能",1)
    for i in [0, 1]:
       inspect_main(i)
    post_voice_str("自检功能执行完毕",1)
    while True:
        time.sleep(1)
        body = redis_pipe.pop_data("TaskInfo")
        if not body:
            continue
        try:
            data = json.loads(body)
        except Exception as e:
            logger.error(e)
            continue
        logger.info("当前接收到的任务数据：{}".format(data))
        inspect_project_id = data.get("inspect_project_id", None)
        # 查看任务类型
        if inspect_project_id:
            logger.info("计划")
            detail_data = db_inspect_project_data.get_by_id(data.get("inspect_project_detail_id"))
            if not detail_data:
                detail_data = db_inspect_project_data.get_by_id(data.get("inspect_project_detail_id"))
            # logger.info("当前接收到的任务计划{}".format(detail_data))
            if not detail_data:
                logger.info("没有查询到{}在数据库中的数值".format(str(data.get("inspect_project_detail_id"))))
                logger.info("不处理该{}数据".format(str(data.get("inspect_project_detail_id"))))
            # task_data = get_data(data.get("inspect_project_detail_id"))
            if detail_data:
                task_data = get_data(detail_data)
                task_data["inspect_project_id"] = inspect_project_id
                inspect_project = DBInspectProject().get_task_id(inspect_project_id)
                task_data["inspect_project_task_id"] = inspect_project.get("inspect_project_task_id")
                # task_data["inspect_project_task_id"] = data.get("inspect_project_task_id")
            else:
                task_data = {}
                # 处理手动任务
        elif not inspect_project_id and not data.get("inspect_project_task_id"):
            logger.info("手动任务")
            # 更新参观任务监听
            redis_visit_state = RedisPipe("VisitState")
            redis_visit_state.set_data("2", 20)
            detail_data = db_inspect_project_data.get_by_id(data.get("inspect_project_detail_id"))
            if not detail_data:
                detail_data = db_inspect_project_data.get_by_id(data.get("inspect_project_detail_id"))
            logger.info("当前接收到的任务计划{}".format(detail_data))
            if not detail_data:
                logger.info("没有查询到{}在数据库中的数值".format(str(data.get("inspect_project_detail_id"))))
                logger.info("不处理该{}数据".format(str(data.get("inspect_project_detail_id"))))
            if detail_data:
                task_data = get_data(detail_data)
            else:
                task_data = {}
        else:
            logger.info("任务")
            inspect_project_task_id = data.get("inspect_project_task_id")
            task_data = DBInspectProjectTask().get_by_project_task(inspect_project_task_id)
            logger.info(task_data)
            robot_task_type_id = task_data.get("robot_task_type_id")
            logger.info(robot_task_type_id)
            task_type_data = DBRobotTaskType().get_data_by_id(robot_task_type_id)
            logger.info(task_type_data)
            # 添加任务类型名，用于后面判断
            task_type_name = task_type_data.get("name")
            task_data["task_type_name"] = task_type_name
            detail_data = db_inspect_project_data.get_by_id(data.get("inspect_project_detail_id"))
            task_data["task_status"] = detail_data.get("task_status")
            inspect_project_detail_id = data.get("inspect_project_detail_id")
            # 更新task_status 为 2，
            DBInspectProjectDetail().update({'inspect_project_detail_id': inspect_project_detail_id},
                                            {"task_status": 2})
            task_data["inspect_project_detail_id"] = inspect_project_detail_id
        if not task_data:
            continue
        # logger.info(task_data)
        RedisPipe("robot_id").set_data(str(task_data.get("robot_id")))
        RedisPipe("robot_path_id").set_data(str(task_data.get("robot_path_id")))
        if alive_init_point(task_data):
            # logger.info("收到任务数据")
            # logger.info(task_data)
            waitTask.task_type(task_data)
        else:
            waitTask.del_task(task_data, "机器人不在充电点上")


def wait_rabbit_mq():
    recv_serverid = db_rabbit_mq.get("rout_send_robotStatus")
    RabbitComsumer.run(recv_serverid)


if __name__ == '__main__':
    # init_positions()
    task_main()
