import time
from configs.log import logger
from core.redis_interactive import RedisPipe
from task.light_al import post_voice_str, post_video
from modules.init_node import state_init
from modules.moving_ring_data import Dynamic_environment
from task.robot_algorithm import RobotAlgorithm
from task.robot_init import init_positions
from task.robot_mq import publish_mq
from schema.db_inspect_position_item_data import DBInspectPositionItemDatum
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_robot_position import DBRobotPosition
from task.robot_path.path_plan import RobotPathQuery
from task.hardware_exception import schedule_test
from modules.robot_hardware_server import RobotDeviceStatus

class RobotFollowEngineer(object):
    def __init__(self):
        self.robot_id = "1"
        self.task_data = {}
        self.robot_path_id = "1"
        self.robot_position_id = "13"
        self.obstacle = 0

    def move_state_new(self):
        db_robotposition = DBRobotPosition()
        robot_algorithm = RobotAlgorithm()
        robot_path = []

        now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)
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
        if not position_num:
            return False

        # 判断当前点和目标点是否相等
        if robot_now_location.get("order_num") == position_num:
            return True

        # 查询预设路径
        move_type_name = "forward_move"
        robot_appoint_location = db_robotposition.get_appoint_location(self.robot_id, position_num, robot_now_location, self.robot_path_id)
        logger.info(robot_appoint_location)
        if self.task_data.get("obstacle") == 1:
            new_now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)
            logger.info("此次为避障")
            logger.info("起始点： {}".format(new_now_location["robot_position_id"]))
            logger.info("目标点： {}".format(self.robot_position_id))
            position_id_list = [new_now_location["robot_position_id"], self.robot_position_id]
            path = RobotPathQuery().query_path(position_id_list)
            if len(path) <= 0:
                return False

            robot_path = path
            logger.info("robot_path: {}".format(robot_path))

        else:
            # 调用算法
            logger.info("起始点: {}".format(now_location["robot_position_id"]))
            logger.info("目标点: {}".format(self.robot_position_id))
            param_list = [now_location["robot_position_id"], self.robot_position_id]
            path = RobotPathQuery().robot_path(param_list)
            if len(path) > 0:
                logger.info("此次路径为算法规划的路径")
                robot_path = path

                # 判断行走方式
                if self.obstacle == 1:
                    move_type_name = "detour_move"
                else:
                    if len(robot_path) == 1:
                        move_type_name = "forward_move"
                    else:
                        move_type_name = "forward_move"

            else:
                logger.info("此次路径为预设的路径")
                robot_path = robot_appoint_location

                # 判断行走方式
                if self.obstacle == 1:
                    move_type_name = "detour_move"
                else:
                    if len(robot_path) == 1:
                        move_type_name = "forward_move"
                    else:
                        move_type_name = "forward_move"

        for res in robot_path:
            if res.get("type") == 1:
                # 调取move算法
                logger.info("移动点到 -----》" + str(res))
                move_status=RobotDeviceStatus.get_device_status("move")
                if not robot_algorithm.robot_back_al(res, move_type_name) or move_status==False:
                    logger.info("移动服务不可用，跳过此检测项...")
                    data_value = "移动服务"
                    schedule_test(data_value)
                    return False
        return True

    def move_state(self):
        db_robotposition = DBRobotPosition()
        robot_algorithm = RobotAlgorithm()

        now_location = DBRobotPosition().get_now_location(self.robot_id, self.robot_path_id)
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
        if not position_num:
            return False

        # 到达第一个采集点
        if robot_now_location.get("order_num") == position_num:
            rest = db_robotposition.get_position_num_one(position_num, self.robot_path_id)
            robot_algorithm.robot_back_al(rest, "forward_move")
            logger.info("已经到达指定点")
            return True

        robot_appoint_location = db_robotposition \
            .get_appoint_location(self.robot_id, position_num, robot_now_location, self.robot_path_id)
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
                move_status=RobotDeviceStatus.get_device_status("move")
                if not robot_algorithm.robot_back_al(res, move_type_name) or move_status==False:
                    logger.info("移动服务不可用，跳过此检测项...")
                    data_value = "移动服务"
                    schedule_test(data_value)
                    return False
        return True

    def robot_start_action(self, task_data):
        self.video_path = None
        self.task_data = task_data
        self.robot_id = task_data.get("robot_id")
        self.robot_path_id = task_data.get("robot_path_id")
        self.robot_position_id = task_data.get("robot_position_id")
        self.obstacle = task_data.get("obstacle")
        self.task_data["status"] = 1
        self.robot_inspect_project(self.task_data, 1)
        if self.move_state_new():
            logger.info("已到达指定位置点")
            post_voice_str("已到达指定点", 1)
            self.robot_inspect_project(self.task_data, 5)
            db_insect_project_detail = DBInspectProjectDetail()
            while True:
                detail_data = db_insect_project_detail.get_top1()
                if detail_data.get("task_type_name") == "随工":
                    detail_state = db_insect_project_detail.get_by_id(self.task_data.get("inspect_project_detail_id"))
                    logger.info("等待随工指令下发")
                    logger.info(detail_state.get("status"))
                    if detail_state.get("status") == 4:
                        self.task_data["status"] = 4
                        post_voice_str("start_video")
                        # 此处为录像操作
                        logger.info("开始执行随工录像")
                        video_state, video_path = post_video(1)
                        if not video_state:
                            logger.error("开始录像接口调用失败")
                            return False
                        break
                else:
                    return False
                time.sleep(1)

            # 挂起等待随工录像结束
            while True:
                last_battery = Dynamic_environment("battery_power")
                detail_data = db_insect_project_detail.get_top1()
                logger.info("等待结束随工录像")
                # logger.info(detail_data.get("task_type_name"))
                detail_state = db_insect_project_detail.get_by_id(self.task_data.get("inspect_project_detail_id"))
                # 结束随工录像
                if detail_state.get("status") == 7 or last_battery < 20 or detail_data.get("task_type_name") == "停止":
                    self.task_data["status"] = 7
                    post_voice_str("end_video")
                    # 发送结束随工录像指令
                    logger.info("开始执行结束随工录像")
                    video_state, video_path = post_video(0)
                    self.video_path = video_path
                    if not video_state:
                        logger.error("开始录像接口调用失败")
                        return False
                    self.insert_data(video_path)
                    self.task_data["status"] = 2
                    self.robot_inspect_project(self.task_data, 2)
                    return True
                time.sleep(1)
        else:
            RedisPipe("ActionTask").set_data({"ActionTask": 0})

    # get_position_id
    def insert_data(self, video_path):
        if not video_path:
            return False
        ret = {}
        ret["inspect_project_detail_id"] = self.task_data.get("inspect_project_detail_id")
        ret["user_id"] = 0
        ret["robot_id"] = 1
        ret["robot_item_id"] = 0
        ret["robot_position_id"] = self.task_data.get("robot_position_id")
        core_device = DBRobotPosition().get_by_position_id(self.task_data.get("robot_position_id"))
        if core_device:
            core_cabinet_id = core_device.get("core_cabinet_id")
            power_cabinet_id = core_device.get("power_cabinet_id")
        else:
            core_cabinet_id = 0
            power_cabinet_id = 0
        ret["core_device_id"] = 0
        ret["core_cabinet_id"] = core_cabinet_id
        ret["power_cabinet_id"] = power_cabinet_id

        ret["value"] = "0"
        ret["status"] = 0

        ret["video_path"] = video_path
        ret['type'] = 0  # 0为随工
        # # 对数据进行存库
        DBInspectPositionItemDatum().insert(info=ret)

    # 更新机器人状态巡检状态信息
    def robot_inspect_project(self, task_data, state):
        params = {}
        params['status'] = state
        if state == 6 or state == 2:
            start_battery_power = task_data.get("now_battery_power")

            end_battery_power = Dynamic_environment("battery_power")
            expend_battery_power = start_battery_power - end_battery_power
            logger.info("该任务耗费电量")
            logger.info(expend_battery_power)

            params['power_consumption'] = expend_battery_power
        if state == 6:
            params['exception_info'] = "该任务被中断"
        if state == 4:
            params['exception_info'] = "录像被中断"
            state = 2
            params['status'] = 2
            self.insert_data(self.video_path)

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
        if robot_task_status == 4 or robot_task_status == 7:
            self.robot_inspect_project(self.task_data, 4)

        RedisPipe("ActionTask").set_data({"ActionTask": 0})


def robot_follow_engineer(task_data):
    RobotFollowEngineer().robot_start_action(task_data)


if __name__ == '__main__':
    state_init()
    robot_follow_engineer({})
