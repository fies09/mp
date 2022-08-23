from configs.log import logger
from core.redis_interactive import RedisPipe
from task.light_al import post_voice_str
from modules.moving_ring_data import Dynamic_environment
from task.robot_algorithm import RobotAlgorithm
from task.robot_mq import publish_mq
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_robot_position import DBRobotPosition
from task.robot_action import move_state
from task.robot_path.path_plan import RobotPathQuery


class RobotReturn(object):
    # 数据采集点1,充电预备点3,充电点4,autogate标志点5,autogate开门点6,autogate关门点7,autogate巡检点8
    def __init__(self):
        self.robot_id = 1
        self.robot_path_id = 0
        self.task_data = {}
        self.db_inspect_project_detail = DBInspectProjectDetail()

    def move_state_new(self, robot_id, robot_path_id):
        robot_path = []
        robot_algorithm = RobotAlgorithm()
        logger.info("目标点位：{}".format(self.robot_path_id))
        # 查询预设路径
        robot_location = DBRobotPosition().get_status(robot_id, robot_path_id)
        logger.info("robot_location *****: {}".format(robot_location))
        # 判断行走方式
        if self.task_data.get("obstacle") == 1:
            move_type_name = "detour_move"
        else:
            move_type_name = "forward_move"

        # 判断是否需要避障
        if self.task_data.get("obstacle") == 1:
            logger.info("此次为避障")
            logger.info("robot_location: {}".format(robot_location))
            robot_path = robot_location[-2:]
            logger.info("robot_path: {}".format(robot_path))
        else:
            # 调用算法
            logger.info("起始点: {}".format(robot_location[0]["robot_position_id"]))
            logger.info("目标点: {}".format(robot_location[-1]["robot_position_id"]))
            param_list = [robot_location[0]["robot_position_id"], robot_location[-1]["robot_position_id"]]
            path = RobotPathQuery().robot_path(param_list)
            if len(path) > 0:
                robot_path = path
            else:
                robot_path = robot_location

        logger.info("一键返回最终路径: {}".format(robot_path))
        for res in robot_path:
            if res.get("type") == 1 or res.get("type") == 9:
                # 调取move算法
                logger.info("到达返回点 -----》" + str(res))
                if not robot_algorithm.robot_back_al(res, move_type_name):
                    return False

            if res.get("type") == 3:
                # 调取move算法
                logger.info("返回充电预备点 -----》" + str(res))
                if not robot_algorithm.robot_back_al(res, move_type_name):
                    return False

            elif res.get("type") == 4:
                # 调取充电算法
                logger.info("返回充电点 -----》" + str(res))
                if not robot_algorithm.robot_back_al(res, "back_charging"):
                    return False

        return True

    def move_state(self, robot_id, robot_path_id):
        logger.info(robot_path_id)
        robot_algorithm = RobotAlgorithm()
        try:
            robot_location = DBRobotPosition().get_status(robot_id, robot_path_id)
            logger.info(robot_location)
        except Exception as e:
            logger.error(e)
            logger.info("没有找到该线路{}机器人所在点".format(robot_path_id))
            return False
        if self.task_data.get("obstacle") == 1:
            move_type_name = "detour_move"
        else:
            if len(robot_location) != 1:
                if robot_location[0].get("order_num") > robot_location[1].get("order_num"):
                    move_type_name = "back_move"
                else:
                    move_type_name = "forward_move"
            else:
                move_type_name = "forward_move"
        logger.info(robot_location)
        if self.task_data.get("obstacle") == 1:
            robot_location = robot_location[-2:]

        for res in robot_location:
            if res.get("type") == 1 or res.get("type") == 9:
                # 调取move算法
                logger.info("到达返回点 -----》" + str(res))
                if not robot_algorithm.robot_back_al(res, move_type_name):
                    return False

            if res.get("type") == 3:
                # 调取move算法
                logger.info("返回充电预备点 -----》" + str(res))
                if not robot_algorithm.robot_back_al(res, move_type_name):
                    return False

            elif res.get("type") == 4:
                # 调取充电算法
                logger.info("返回充电点 -----》" + str(res))
                if not robot_algorithm.robot_back_al(res, "back_charging"):
                    return False

        return True

    def robot_start_return(self, task_data):
        logger.info(self.task_data)
        self.task_data = task_data
        self.robot_id = task_data.get("robot_id")
        self.robot_path_id = task_data.get("robot_path_id") # 目标点
        self.task_data["status"] = 1

        self.robot_inspect_project(task_data, 1)

        # 获取当前点位，得到路径和点
        now_position = DBRobotPosition().get_now_location(task_data.get("robot_id"))
        task_data["robot_path_id"] = now_position.get("robot_path_id")
        if self.move_state_new(task_data.get("robot_id"), task_data.get("robot_path_id")):
            logger.info("已返回充电")
            # 语音播报:一键返回
            post_voice_str("已返回充电点", 1)
            self.task_data["status"] = 2
            self.robot_inspect_project(self.task_data, 2)
        else:
            RedisPipe("ActionTask").set_data({"ActionTask": 0})

    def robot_inspect_project(self, task_data, state):
        params = {}
        params['status'] = state
        if state == 6 or state == 2:
            start_battery_power = task_data.get("now_battery_power")
            logger.info(start_battery_power)

            end_battery_power = Dynamic_environment("battery_power")
            expend_battery_power = start_battery_power - end_battery_power

            params['power_consumption'] = expend_battery_power
        if state == 6:
            params['exception_info'] = "该任务被中断"
        logger.info("更新计划任务状态信息")
        logger.info(params)

        inspect_project_detail_id = task_data.get("inspect_project_detail_id")
        DBInspectProjectDetail().update({'inspect_project_detail_id': inspect_project_detail_id}, params)
        # 更新mq内容
        data = {"inspect_project_detail_id": inspect_project_detail_id, "status": state}
        publish_mq("robot_task", "task_rout_key", "task_queue", data=data)

    def __del__(self):
        robot_task_status = self.task_data.get("status")
        if robot_task_status == 1:
            self.robot_inspect_project(self.task_data, 6)
        RedisPipe("ActionTask").set_data({"ActionTask": 0})


def robot_return(task_data):
    RobotReturn().robot_start_return(task_data)


if __name__ == '__main__':
    # state_init()
    # robot_return(task_data)
    pass
