import os
from threading import Thread
from configs.log import logger
from configs import robot_password
from task.light_al import post_voice_str
from task.robot_mq import publish_mq
from modules.init_node import state_init
from modules.move_stop import Stop_move
from modules.moving_ring_data import Dynamic_environment
from task.robot_auto_inspection import auto_ins
from task.robot_cmd_arrive import robot_cmd_action
from task.robot_follow_engineer import robot_follow_engineer
from task.robot_return import robot_return
from task.robot_stop import robot_stop
from task.robot_visit import start_visit
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_inspect_project_task import DBInspectProjectTask


class RobotImplement(object):
    def __init__(self):
        pass

    def select_task(self, task_data):

        task_data["now_battery_power"] = Dynamic_environment("battery_power")

        if task_data.get("task_type_name") == "自动巡检" or task_data.get("task_type_name") == "设备盘点" \
                or task_data.get("task_type_name") == "动力巡检":
            self.auto_inspection(task_data)

            if task_data.get("task_type_name") == "设备盘点":
                post_voice_str("开始设备盘点", 1)
            elif task_data.get("task_type_name") == "动力巡检":
                post_voice_str("开始动力巡检", 1)
            else:
                post_voice_str("auto_inspection")

        elif task_data.get("task_type_name") == "指定到达":
            logger.info(task_data)
            self.cmd_arrive(task_data)
            post_voice_str("cmd_arrive")

        elif task_data.get("task_type_name") == "一键返回":
            logger.info(task_data)
            self.cmd_return(task_data)
            post_voice_str("robot_return")

        elif task_data.get("task_type_name") == "随工":
            self.follow_implement(task_data)
            post_voice_str("follow_engineer")

        elif task_data.get("task_type_name") == "停止":
            self.cmd_stop(task_data)
            post_voice_str("robot_stop")

        elif task_data.get("task_type_name") == "机器人重启":
            try:
                post_voice_str("机器人即将重启", 1)
                logger.info("重启中......")
                cmd = "echo '{}'|sudo -S init 6".format(robot_password)
                logger.info(cmd)
            except Exception as e:
                logger.error(e)
                cmd = "echo '{}'|sudo -S reboot".format(robot_password)
                logger.info(cmd)
            # 更新机器人重启任务状态
            inspect_project_detail_id = task_data.get("inspect_project_detail_id")
            params = dict()
            params["status"] = 2
            params['power_consumption'] = 0
            DBInspectProjectDetail().update({'inspect_project_detail_id': inspect_project_detail_id}, params)
            # 更新mq内容
            data = {"inspect_project_detail_id": inspect_project_detail_id, "status": 2}
            publish_mq("robot_task", "task_rout_key", "task_queue", data=data)
            system_res = os.popen(cmd)
            logger.info(system_res)

        elif task_data.get("task_type_name") == "参观":
            logger.info("============参观===========")
            self.visit(task_data)
        else:
            pass

    def auto_inspection(self, task_data):
        Stop_move()
        auto_inspection = Thread(target=auto_ins, args=(task_data,))
        auto_inspection.start()
        # logger.info(str(task_data.get("task_type_name")))

    def cmd_arrive(self, task_data):
        Stop_move()
        t_arrive = Thread(target=robot_cmd_action, args=(task_data,))
        t_arrive.start()
        logger.info("开始执行指令到达任务")

    def cmd_return(self, task_data):
        Stop_move()
        logger.info(task_data)
        t_return = Thread(target=robot_return, args=(task_data,))
        t_return.start()
        logger.info("开始执行一键返回任务")

    def follow_implement(self, task_data):
        Stop_move()
        t_follow = Thread(target=robot_follow_engineer, args=(task_data,))
        t_follow.start()
        logger.info("开始执行随工任务")

    def cmd_stop(self, task_data):
        robot_stop(task_data)
        logger.info("开始执行停止任务")

    def visit(self, task_data):
        Stop_move()
        auto_inspection = Thread(target=start_visit, args=(task_data,))
        auto_inspection.start()
        logger.info(str(task_data.get("task_type_name")))

    # def judge_positions(self, robot_path_id):
    #     db_robot_position = DBRobotPosition()
    #     ret = db_robot_position.get_alive_path(robot_path_id)
    #     if not ret:
    #         logger.info("判断该线路上没有位置点信息，将充电点设为机器人所在点")
    #         try:
    #             res = db_robot_position.get_position(1, robot_path_id)[-1]
    #             before_params = {'current_state': "0"}
    #             db_robot_position.update({'current_state': "1"}, before_params)
    #             data = {"current_state": 1}
    #             robot_position_id = res.get("robot_position_id")
    #             db_robot_position.update({'robot_position_id': robot_position_id}, data)
    #         except Exception as e:
    #             logger.error(e)


if __name__ == '__main__':
    def get_data(ids):
        data = {}
        detail_data = DBInspectProjectDetail().get_by_id(ids)
        task_data = DBInspectProjectTask().get_by_num(detail_data.get("robot_id"), detail_data.get("task_num"))

        data["inspect_project_detail_id"] = ids
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
        data["is_break"] = task_data.get("is_break")
        # 执行最低电量
        data["minimum_battery"] = task_data.get("minimum_battery")
        # 等待时长
        # 是否可以被打断
        data["is_break"] = task_data.get("is_break")
        # 任务类型id
        data["robot_task_type_id"] = task_data.get("robot_task_type_id")
        return data


    task_data = get_data("1930762811")

    state_init()
    robot_impolement = RobotImplement()
    robot_impolement.select_task(task_data)
