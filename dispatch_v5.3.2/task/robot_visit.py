# -*- coding:utf-8 -*-
# @Author: WangLiang
# @file:  robot_visit.py
# @Time: 2021/03/03 10:10
# @Description: 参观任务模块
import json
import time
import rospy
from configs.log import logger
from task.robot_mq import publish_mq
from sensor_msgs.msg import BatteryState
from task.robot_algorithm import RobotAlgorithm
from schema.db_robot_item import DBRobotItem
from core.redis_interactive import RedisPipe
from modules.voice_action import voice_client
from modules.robot_hardware_server import robot_move
from schema.db_inspect_project import DBInspectProject
from schema.db_robot_position import DBRobotPosition
from modules.moving_ring_data import Dynamic_environment
from schema.db_inspect_project_task import DBInspectProjectTask
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_inspect_position_item_data import DBInspectPositionItemDatum
from schema.db_robot_path_position_relation import DBRobotPathPositionRelation
from schema.db_inspect_project_task_path_relation import DBInspectProjectTaskPathRelation
from std_msgs.msg import Int8
from core.dict_config import position_type, obstacle_type, move_state, visit_state, current_state, voice_status, \
    robot_move_model, task_status


class RobotVisit:
    def __init__(self, task_data):
        self.task_data = task_data
        self.robot_id = task_data.get("robot_id")
        self.robot_path_id = task_data.get("robot_path_id")
        self.inspect_project_task_id = task_data.get("inspect_project_task_id")
        self.redis_wait_task = RedisPipe("WaitTask")
        self.robot_algorithm = RobotAlgorithm()
        # 初始化参观讲解状态
        self.redis_visit_state = RedisPipe("VisitState")
        # 初始化机器人是否到达迎宾点
        self.arrive_welcome = False
        # 讲解指令监听超时判定
        self.time_out = False
        # 初始化监听值
        self.visit_listen_val = None
        # 初始化机器人状态和电量
        logger.info("+++++++++++++++++++++++++++ 开始 +++++++++++++++++++++++++++++++")
        self.robot_inspect_project(task_status.get("action"))
        # 初始化其他任务计划状态
        self.inspect_project_detail = DBInspectProjectDetail()
        DBInspectProjectDetail().update({'status': 7}, {'status': 6})
        DBInspectProjectDetail().update({'status': 8}, {'status': 6})

    @staticmethod
    def robot_move(position_data, move_type):
        logger.info("当前运动模式>>>>>{}".format(move_type))
        position = position_data.get("position")
        if isinstance(position, str):
            position = json.loads(position)
        is_obstacle = position_data.get("is_obstacle")
        voice = position_data.get("voice")
        x, y, yaw, = position.get("x"), position.get("y"), position.get("yaw")
        if is_obstacle == obstacle_type.get("YES"):
            move_type = "detour_move"
        now_state = robot_move(position, move_type)
        # 因机器人移动的状态有不确定性，加一次重试
        if not now_state:
            now_state = robot_move(position, move_type)
        logger.info(now_state)
        logger.info(move_state.get("arrive"))
        if now_state:
            logger.info("到达" + position_data.get("name"))
            try:
                #  初始化所有点状态
                before_params = {'current_state': current_state.get("NO")}
                DBRobotPosition().update({'current_state': current_state.get("YES")}, before_params)
                # 更新此点为新的机器人所在点
                logger.info("更新此点{}为新的机器人所在点>>>".format(position_data.get("robot_position_id")))
                params = {'current_state': current_state.get("YES")}
                DBRobotPosition().update({'robot_position_id': position_data.get("robot_position_id")}, params)
                logger.info("机器人位置点信息已更新")
                return True
            except Exception as e:
                logger.error(e)
                return False
        else:
            logger.error("到达到点" + position_data.get("name") + "超时")
        return False

    @staticmethod
    def robot_speak(explain_word):
        """
        机器人语音函数同时监听是否播报完毕
        """
        try:
            voice_client(explain_word)
        except Exception as e:
            logger.error(e)
            logger.error("语音错误")
        while True:
            time.sleep(1)
            try:
                voice_status_data = rospy.wait_for_message('/voice_status', Int8, timeout=5)
            except Exception as e:
                logger.error(e)
                logger.error("ROS节点“voice_status”错误！")
                break
            #  判断机器人是否在讲话
            # 如果机器人没在讲话
            if voice_status_data.data == voice_status.get("dont_speak"):
                break
            logger.info("正在讲解中...")
            print("\n")

    def visit_state_listen(self, timeout):
        """
        参观任务状态监听函数
        value表示需识别的值
        timeout为监听超时时间
        """
        # 计时器变量 (单位为秒)
        timer = 0
        while True:
            logger.info("开始监听讲解指令中......")
            print("\n")
            state = self.redis_visit_state.get_data()
            time.sleep(2)
            if state:
                state_val = int(state.decode())
                if state_val == visit_state.get("start") or state_val == visit_state.get("stop"):
                    return state_val
            timer += 2
            if timer >= timeout:
                return 0

    def robot_inspect_project(self, state, update_time=None):
        # 初始化更新类容
        state_data = dict()
        logger.info(state_data)
        if state in [task_status.get("finish"), task_status.get("fail")]:
            start_battery_power = self.task_data.get("now_battery_power")
            logger.info(start_battery_power)
            end_battery_power = Dynamic_environment("battery_power")
            expend_battery_power = start_battery_power - end_battery_power
            state_data['power_consumption'] = expend_battery_power
        if state == task_status.get("fail"):
            if self.time_out:
                state_data['exception_info'] = "讲解指令等待超时"
            else:
                state_data['exception_info'] = "该任务被中断"
        if update_time:
            state_data['update_time'] = update_time
        logger.info(state_data)
        logger.info(state)
        state_data['status'] = state
        logger.info("更新计划任务状态信息")
        logger.info(self.task_data.get("inspect_project_detail_id"))
        inspect_project_detail_id = self.task_data.get("inspect_project_detail_id")
        self.task_data["status"] = state
        logger.info(state_data)
        logger.info(inspect_project_detail_id)
        DBInspectProjectDetail().update({'inspect_project_detail_id': inspect_project_detail_id}, state_data)
        # 更新mq内容
        data = {"inspect_project_detail_id": inspect_project_detail_id, "status": state}
        publish_mq("robot_task", "task_rout_key", "task_queue", data=data)

    def get_task_cmd(self):
        """
        非参观任务判断判断
        """
        cmd = DBInspectProjectDetail().get_top1()
        if not cmd:
            return False
        cmd_name = cmd.get("task_type_name")
        logger.info(cmd_name)
        cmd_name_list = ["自动巡检", "设备盘点", "动力巡检", "参观"]
        if cmd_name not in cmd_name_list:
            logger.info("参观任务已被终止，请稍后重新下发...")
            self.robot_speak("等待已被终止，请稍后重新下发参观任务")
            # 任务被终止，更新任务状态
            update_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
            logger.info("+++++++++++++++++++++++++++ 失败 +++++++++++++++++++++++++++++++")
            self.robot_inspect_project(task_status.get("fail"), update_time)
            return False
        return True

    def task_start(self):
        """
        参观任务主入口
        """
        # 获取该路径的点位ID
        try:
            position_relation_data = DBRobotPathPositionRelation().get_position_id(self.robot_path_id)
        except Exception as e:
            logger.error(e)
            logger.error("该路径>>>{}点位ID获取失败".format(self.robot_path_id))
            return False
        # 根据点位移动
        try:
            logger.info(self.task_data)
            for position_relation in position_relation_data:
                logger.info(position_relation)
                if not self.get_task_cmd():
                    return False
                try:
                    # 根据关联关系表 获取点位信息
                    robot_position_id = position_relation.get("robot_position_id")
                    robot_path_id = position_relation.get("robot_path_id")
                    position_data = DBRobotPosition().get_position_by_id(robot_position_id, robot_path_id)
                    logger.info(position_data)
                    logger.info(robot_position_id)
                    inspect_project_task_id = self.task_data.get("inspect_project_task_id")
                    task_path_relation_data = DBInspectProjectTaskPathRelation().get_play_desc(robot_position_id, inspect_project_task_id)
                    # 获取该点的播报内容
                    play_desc = task_path_relation_data.get("play_desc")
                    # 是否播报
                    play_status = task_path_relation_data.get("play_status")
                    position_data["word"] = play_desc
                except Exception as e:
                    logger.error(e)
                    logger.error("该点信息获取失败>>>{}".format(position_relation))
                    return False
                # 根据点位指定机器人运动模式
                if position_data.get("type") == position_type.get("charge"):
                    move_model = robot_move_model[1]
                else:
                    move_model = robot_move_model[0]
                logger.info("开始前往{}".format(position_data.get("name")))
                # 调用移动程序，开始移动...
                try:
                    # 在移动之前调用开关门
                    if not self.robot_algorithm.alive_action_door(position_data):
                        logger.error("该点开关门动作没有完成")
                        return False
                    move_feedback = self.robot_move(position_data, move_model)
                except Exception as e:
                    logger.error(e)
                    logger.error("该点移动失败>>>{}".format(position_relation))
                    move_feedback = False
                time.sleep(9)
                # 成功到达该点
                if move_feedback:
                    # 如果到达迎宾点
                    if position_data.get("type") == position_type.get("welcome"):
                        self.arrive_welcome = True
                        try:
                            # 获取任务数据
                            inspect_project_task_data = DBInspectProjectTask().get_by_project_task(
                                self.inspect_project_task_id)
                        except Exception as e:
                            logger.error(e)
                            logger.error("任务数据获取错误")
                            return False
                        # 超时时间
                        meet_time = inspect_project_task_data.get("meet_time")
                        # 更新任务状态，到达参观点
                        logger.info("+++++++++++++++++++++++++++ 迎宾 +++++++++++++++++++++++++++++++")
                        self.robot_inspect_project(task_status.get("welcome"))
                        # 到达迎宾点播放固定内容
                        self.robot_speak("已到达迎宾点，即将开始执行任务")
                        # 开始监听 是否开始讲解的指令
                        try:
                            logger.info(meet_time)
                            # 判断是任务还是计划（计划）
                            if self.task_data.get("inspect_project_id"):
                                # 获取是否人工干预
                                project_data = DBInspectProject().get_task_id(self.task_data.get("inspect_project_id"))
                                intervene = project_data.get("intervene", 0)
                                # 如果需要人工干预
                                if intervene:
                                    self.visit_listen_val = self.visit_state_listen(int(meet_time) * 60)
                                else:
                                    self.visit_listen_val = visit_state.get("start")
                            else:
                                self.visit_listen_val = self.visit_state_listen(int(meet_time) * 60)
                        except Exception as e:
                            logger.error(e)
                            logger.error("指令监听错误，判定为超时！")
                            self.visit_listen_val = None
                        # 判断是否等待超时
                        if self.visit_listen_val not in [visit_state.get("start"), visit_state.get("stop")]:
                            # 播报提示，自动返回充电桩
                            self.robot_speak("参观任务等待超时，即将返回充电桩")
                            logger.info("参观任务等待超时，即将返回充电桩")
                            # 自动返回
                            self.visit_listen_val = visit_state.get("return")
                            # 更新状态
                            update_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
                            self.robot_inspect_project(task_status.get("action"), update_time)
                            # 超时
                            self.time_out = True
                    # 如果机器人已到达过迎宾点
                    if self.arrive_welcome:
                        # 监听到开始讲解的指令，开始执行参观流程
                        if self.visit_listen_val == visit_state.get("start"):
                            # 讲解内容
                            explain_word = position_data.get("word", "")
                            # 开始讲解
                            # 修改状态为正在参观
                            logger.info("+++++++++++++++++++++++++++ 参观 +++++++++++++++++++++++++++++++")
                            self.robot_inspect_project(task_status.get("visit"))
                            logger.info("已到达{}，开始讲解".format(position_data.get("name")))
                            logger.info("讲解内容:{}".format(explain_word))
                            if not explain_word:
                                continue
                            # 该点播报完成，将该点的播报情况插入巡检数据表
                            try:
                                ret = dict()
                                robot_item = DBRobotItem().get_item_name("参观")
                                ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
                                ret["inspect_project_task_id"] = self.inspect_project_task_id
                                ret["user_id"] = self.task_data.get("user_id")
                                ret["robot_id"] = self.task_data.get("robot_id")
                                ret["robot_item_id"] = robot_item.get("robot_item_id")
                                ret["robot_position_id"] = robot_position_id
                                ret["core_cabinet_id"] = position_data.get("core_cabinet_id", 0)
                                ret["core_device_id"] = '0'
                                ret["server_u"] = '0'
                                ret["value"] = ""
                                ret["type"] = robot_item.get("type")
                                ret["status"] = 1
                                ret["create_time"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
                                ret["create_by"] = self.task_data.get("user_id")
                                ret["power_cabinet_id"] = position_data.get("power_cabinet_id", 0)
                                ret["play_desc"] = play_desc
                                ret["play_status"] = play_status
                                position_item_data_id = DBInspectPositionItemDatum().insert(info=ret)
                                condition = {"position_item_data_id": position_item_data_id}
                            except Exception as e:
                                logger.error(e)
                                logger.error("巡检数据插入错误")
                                continue
                            # 监听机器人是否在讲话，并且执行语音程序开始讲解
                            try:
                                if play_status:
                                    self.robot_speak(explain_word)
                                logger.info("{}讲解完毕!".format(position_data.get("name")))
                                # 更新时间
                                DBInspectPositionItemDatum().update(condition, {})
                            except Exception as e:
                                logger.error(e)
                                logger.error("{}讲解错误!".format(position_data.get("name")))
                                # 更新状态为讲解失败
                                info = {"status": 0}
                                DBInspectPositionItemDatum().update(condition, info)
                        # 收到停止指令
                        if self.visit_listen_val == visit_state.get("stop"):
                            logger.info("参观任务已被终止，请稍后重新下发...")
                            self.robot_speak("等待已被终止，请稍后重新下发参观任务")
                            # 任务被终止，更新任务状态
                            update_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
                            logger.info("+++++++++++++++++++++++++++ 失败 +++++++++++++++++++++++++++++++")
                            self.robot_inspect_project(task_status.get("fail"), update_time)
                            return False
                        # 收到返回指令
                        if self.visit_listen_val == visit_state.get("return"):
                            logger.info("正在返回充电桩...")
                else:
                    logger.error("未能成功到达该点>>>".format(position_data.get("robot_position_id")))
                    # 任务被终止，更新任务状态
                    update_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
                    logger.info("+++++++++++++++++++++++++++ 失败 +++++++++++++++++++++++++++++++")
                    self.robot_inspect_project(task_status.get("fail"), update_time)
                    return False
        except Exception as e:
            logger.error(e)
            logger.error("参观任务逻辑错误")

        # 成功完成参观任务,更新机器人状态
        update_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        logger.info("+++++++++++++++++++++++++++ 成功 +++++++++++++++++++++++++++++++")
        if self.time_out:
            self.robot_inspect_project(task_status.get("fail"), update_time)
        else:
            self.robot_inspect_project(task_status.get("finish"), update_time)

    def __del__(self):
        logger.info("结束当前任务__del__{}".format(self.task_data))
        robot_task_status = self.task_data.get("status")
        logger.info("结束当前任务__del__状态{}".format(robot_task_status))
        logger.info(robot_task_status)
        if robot_task_status in [task_status.get("action"), task_status.get("welcome")]:
            update_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
            logger.info("+++++++++++++++++++++++++++ 失败 +++++++++++++++++++++++++++++++")
            self.robot_inspect_project(task_status.get("fail"), update_time)

        RedisPipe("ActionTask").set_data({"ActionTask": 0})

    @staticmethod
    def return_charge(self):
        """
        返回充电点
        """

    @staticmethod
    def judgment_power():
        """
        判断电量是否可以执行此次任务
        """
        try:
            try:
                battery_data = rospy.wait_for_message("/battery", BatteryState, timeout=5)
                if not battery_data:
                    logger.error("未能正常订阅到电量信息，默认给80")
                    battery = 80
                else:
                    battery = int(battery_data.charge)
            except Exception as e:
                logger.info(e)
                logger.error("未能正常订阅到电量信息，默认给80")
                battery = 80
            # 获取保护电量的值
            protect_battery = 0  # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            if battery > int(protect_battery):
                return True
            else:
                return False
        except Exception as e:
            logger.error(e)
            logger.error("判断电量是否可以执行此次任务错误")
        return False

    @staticmethod
    def judgment_charge_position():
        """
        判断机器人是否在充电点
        """
        try:
            db_robot_position = DBRobotPosition()
            # 获取当前位置信息
            try:
                now_position_data = db_robot_position.get_now_position()
                now_position_type = now_position_data.get("type", position_type.get("null"))
            except Exception as e:
                logger.error(e)
                logger.error("当前点位获取错误")
                return False
            # 获取充电点type值
            charge_type = position_type.get("charge")
            if int(now_position_type) == charge_type:
                # 机器人在充点电
                return True
            else:
                # 机器人不在充点电
                return False
        except Exception as e:
            logger.error(e)
            logger.error("判断机器人是否在充电点错误")
        return False

    def judgment_execute_task(self):
        """
        判断是否在执行任务
        """

        try:
            alive_task = json.loads(self.redis_wait_task.get_data().get_data().decode()).get("ActionTask")
            if alive_task:
                return True
            else:
                return False
        except Exception as e:
            logger.error(e)
            logger.error("判断是否在执行任务错误")
            return False


def start_visit(task_data):
    test_task_data = {
        "create_by": 1,
        "create_time": "2022-02-24T19:20:45",
        "del_flag": 0,
        "inspect_project_task": 666,
        "inspect_project_task_id": 0,
        "is_break": 1,
        "minimum_battery": 21,
        "name": "参观测试",
        "remark": None,
        "robot_id": 1,
        "robot_path_id": 1,
        "robot_task_type_id": 1,
        "status": 0,
        "task_num": "RW10001",
        "unit": "0",
        "update_by": 1,
        "update_time": "2022-02-24T19:20:56",
        "wait_time": 30
    }
    robot_visit = RobotVisit(task_data)
    robot_visit.task_start()
    logger.info("任务结束")


if __name__ == '__main__':
    rospy.init_node('mp_voice11111', anonymous=True)
    visit = RobotVisit({"robot_id": 1, "robot_path_id": 1})
    visit.task_start()



