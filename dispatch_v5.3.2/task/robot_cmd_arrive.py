from configs.log import logger
from core.redis_interactive import RedisPipe
from task.light_al import post_voice_str
from modules.init_node import state_init
from modules.moving_ring_data import Dynamic_environment
from task.robot_algorithm import RobotAlgorithm
from task.robot_init import init_positions
from task.robot_mq import publish_mq
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_robot_position import DBRobotPosition
from task.robot_action import move_state
from task.robot_path.path_plan import RobotPathQuery
from schema.db_robot_path_position_relation import DBRobotPathPositionRelation


class RobotCmdArrive(object):
    def __init__(self):
        self.robot_id = "1"
        self.task_data = {}
        self.robot_path_id = "1"
        self.robot_position_id = "1"
        self.obstacle = 0

    def move_state_new(self):
        db_robotposition = DBRobotPosition()
        robot_algorithm = RobotAlgorithm()
        robot_path = []
        logger.info("指定到达目标点：{}".format(self.robot_position_id))
        # 查询当前坐标点
        logger.info("self.robot_id: {}".format(self.robot_id))
        logger.info("self.robot_path_id: {}".format(self.robot_path_id))
        now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)
        logger.info("目前坐标点为{}".format(str(now_location)))
        if not now_location:
            logger.info("初始化目标点到充电点")
            # 初始化点到充电点
            init_positions(self.robot_path_id)
            now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)

        logger.info("now_location: {}".format(now_location))
        robot_now_location_type = now_location.get("type")
        if robot_now_location_type != 1:
            res = db_robotposition.get_location_all(self.robot_id, self.robot_path_id)
            robot_now_location = {"position_num": res.get("order_num")}
        else:
            robot_now_location = {"position_num": now_location.get("order_num")}

        try:
            res = db_robotposition.get_position_id(self.robot_position_id)[0]
        except Exception as e:
            logger.error(e)
            logger.info("没有选择指令到达的点")
            res = {}
        position_num = res.get("order_num")
        logger.info(position_num)
        if not position_num:
            return False

        # 判断当前点和目标点是否相等
        if robot_now_location.get("order_num") == position_num:
            return True

        move_type_name = "forward_move"
        robot_appoint_location = self.gen_path(self.robot_id, now_location["robot_position_id"], self.robot_position_id)
        logger.info("robot_appoint_location: {}".format(robot_appoint_location))
        if self.task_data.get("obstacle") == 1:
            # new_now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)
            logger.info("此次为避障")
            robot_path = robot_appoint_location[-1:]

        else:
            logger.info("开始调用算法路径规划")
            logger.info("起始点： {}".format(now_location["robot_position_id"]))
            logger.info("目标点： {}".format(self.robot_position_id))
            param_list = [now_location["robot_position_id"], self.robot_position_id]
            path = RobotPathQuery().robot_path(param_list)
            if len(path) > 0:
                logger.info("path: {}".format(path))
                logger.info("此次路径为算法规划的路径\n")
                robot_path = path
                logger.info("robot_path {}".format(robot_path))

                if self.obstacle == 1:
                    move_type_name = "detour_move"
                else:
                    if len(robot_path) == 1:
                        move_type_name = "forward_move"
                    else:
                        move_type_name = "forward_move"
            else:
                logger.info("此次路径为预设路径")
                robot_path = self.gen_path(self.robot_path_id, now_location["robot_position_id"],
                                           self.robot_position_id)
                if self.obstacle == 1:
                    move_type_name = "detour_move"
                else:
                    if len(robot_path) == 1:
                        move_type_name = "forward_move"
                    else:
                        move_type_name = "forward_move"

        # 处理机器人充电点出发先走充电预备点，其他点出发只走数据采集点
        result_path_list = []
        if len(robot_path) > 0:
            if robot_path[0]["type"] == 3:
                logger.info("充电点出发, 先走充电预备点")
                result_path_list = robot_path
            else:
                # 过滤出数据采集点
                logger.info("非充电点出发, 只走数据采集点")
                for path_one in robot_path:
                    if path_one["type"] == 1:
                        result_path_list.append(path_one)

        logger.info("指定到达最终路径: {}".format(result_path_list))
        for res in result_path_list:
            # if res.get("type") == 1:
            #     # 调取move算法
            #     logger.info("移动点到 -----》" + str(res))
            #     if not robot_algorithm.robot_back_al(res, move_type_name):
            #         return False

            logger.info("移动点到 -----》" + str(res))
            if not robot_algorithm.robot_back_al(res, move_type_name):
                return False
        return True

    def move_state(self):
        db_robotposition = DBRobotPosition()
        robot_algorithm = RobotAlgorithm()

        now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)
        logger.info("目前坐标点为{}".format(str(now_location)))
        if not now_location:
            # 初始化点到充电点
            init_positions(self.robot_path_id)
            now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)

        robot_now_location_type = now_location.get("type")
        if robot_now_location_type != 1:
            res = db_robotposition.get_location_all(self.robot_id, self.robot_path_id)
            robot_now_location = {"position_num": res.get("order_num")}
        else:
            robot_now_location = {"position_num": now_location.get("order_num")}

        try:
            res = db_robotposition.get_position_id(self.robot_position_id)[0]
        except Exception as e:
            logger.error(e)
            logger.info("没有选择指令到达的点")
            res = {}
        position_num = res.get("order_num")
        logger.info(position_num)
        if not position_num:
            return False

        # 到达第一个采集点
        if robot_now_location.get("order_num") == position_num:
            rest = db_robotposition.get_position_num_one(position_num, self.robot_path_id)
            robot_algorithm.robot_back_al(rest, "forward_move")
            logger.info("已经到达指定点")
            return True
        logger.info("当前机器人所在位置{}".format(str(robot_now_location)))
        logger.info("当前机器人目标位置{}".format(str(position_num)))

        logger.info("self.robot_id: {} - position_num: {} - robot_now_location: {} - self.robot_path_id: {}".format(
            self.robot_id, position_num, robot_now_location, self.robot_path_id))
        robot_appoint_location = db_robotposition.get_appoint_location(self.robot_id, position_num, robot_now_location,
                                                                       self.robot_path_id)
        logger.info("对改目标点进行路径规划线路")
        logger.info(robot_appoint_location)
        if self.obstacle == 1:
            move_type_name = "detour_move"
        else:
            if len(robot_appoint_location) == 1:
                move_type_name = "forward_move"
            else:
                if robot_appoint_location[0].get("order_num") > robot_appoint_location[1].get("order_num"):
                    move_type_name = "back_move"
                else:
                    move_type_name = "forward_move"

        logger.info(robot_appoint_location)
        if self.task_data.get("obstacle") == 1:
            robot_appoint_location = robot_appoint_location[-1:]

        for res in robot_appoint_location:
            if res.get("type") == 1:
                # 调取move算法
                logger.info("移动点到 -----》" + str(res))
                if not robot_algorithm.robot_back_al(res, move_type_name):
                    return False
        return True

    def robot_start_action(self, task_data):
        self.task_data = task_data
        self.robot_id = task_data.get("robot_id")
        self.robot_path_id = task_data.get("robot_path_id")
        self.robot_position_id = task_data.get("robot_position_id")  # 目标点
        self.obstacle = task_data.get("obstacle")

        self.task_data["status"] = 1
        self.robot_inspect_project(self.task_data, 1)

        if self.move_state_new():
            logger.info("已到达指定位置点")
            post_voice_str("已到达指定点", 1)
            self.task_data["status"] = 2
            self.robot_inspect_project(self.task_data, 2)
        else:
            RedisPipe("ActionTask").set_data({"ActionTask": 0})

    # 更新机器人状态巡检状态信息
    def robot_inspect_project(self, task_data, state):
        params = {}
        logger.info(state)
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
        if state == 1:
            redis_visit_state = RedisPipe("RobotCmdPosition")
            redis_visit_state.set_data(self.robot_position_id, 1800)
            data = {"inspect_project_detail_id": inspect_project_detail_id, "status": state}

        else:
            data = {"inspect_project_detail_id": inspect_project_detail_id, "status": state}
            redis_visit_state = RedisPipe("RobotCmdPosition")
            redis_visit_state.set_data(0, 30)
        publish_mq("robot_task", "task_rout_key", "task_queue", data=data)

    def gen_path(self, robot_path_id, start_id, stop_id) -> list:
        """
        生成路径
        """
        logger.info("start_id: {}".format(start_id))
        logger.info("stop_id: {}".format(stop_id))

        path = []  # 机器人行走路径序列
        path_list = []  # 机器人最终路径
        start_order_num = 0  # 机器人起点序列
        stop_order_num = 0  # 机器人终点序列

        if start_id == stop_id:
            return path

        # 查询当前路径序列
        path_relation_list = DBRobotPathPositionRelation().get_position_id(robot_path_id)
        if len(path_relation_list) == 0:
            logger.error("查询路径点序列失败")
            return path

        # 查询起点, 终点对应的order_num
        for path_relation in path_relation_list:
            if path_relation["robot_position_id"] == start_id:
                start_order_num = path_relation["order_num"]
                logger.info("起点 start_order_num_1: {}".format(start_order_num))
                logger.info("起点 order_num_1: {}".format(path_relation["order_num"]))

            if path_relation["robot_position_id"] == stop_id:
                stop_order_num = path_relation["order_num"]
                logger.info("终点 stop_order_num_2: {}".format(stop_order_num))
                logger.info("终点 order_num_2: {}".format(path_relation["order_num"]))

        # 按照起点，终点截取路径序列
        logger.info("start_order_num: {}".format(start_order_num))
        logger.info("stop_order_num: {}".format(stop_order_num))
        if start_order_num < stop_order_num:
            # 正向截取
            logger.info("正向截取路径")
            for path_relation_detail in path_relation_list:
                if path_relation_detail["order_num"] <= start_order_num:
                    continue

                if path_relation_detail["order_num"] > stop_order_num:
                    continue

                path.append(path_relation_detail["robot_position_id"])

        else:
            # 反向截取
            logger.info("反向截取路径")
            path_relation_list_desc = DBRobotPathPositionRelation().get_position_id_desc(robot_path_id)
            if len(path_relation_list_desc) == 0:
                logger.error("查询路径点序列失败")
                return path

            for path_relation_detail in path_relation_list_desc:
                if path_relation_detail["order_num"] >= start_order_num:
                    continue

                if path_relation_detail["order_num"] < stop_order_num:
                    continue

                path.append(path_relation_detail["robot_position_id"])

        # 查询路径点数据
        logger.info("路径点顺序：{}".format(path))
        query_path_list, total = DBRobotPosition().get_list(path)

        # 按照路径点顺序进行最终路径排序
        for path_id in path:
            for posotion_detail in query_path_list:
                if path_id == posotion_detail["robot_position_id"]:
                    path_list.append(posotion_detail)

        logger.info("新的最终路径点个数是: {}".format(total))
        logger.info("新的最终路径是: {}".format(path_list))

        return path_list

    def __del__(self):
        robot_task_status = self.task_data.get("status")
        if robot_task_status == 1:
            self.robot_inspect_project(self.task_data, 6)

        RedisPipe("ActionTask").set_data({"ActionTask": 0})


def robot_cmd_action(task_data):
    RobotCmdArrive().robot_start_action(task_data)


if __name__ == '__main__':
    state_init()
    robot_cmd_action({})
