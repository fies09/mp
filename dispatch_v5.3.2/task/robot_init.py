from configs.log import logger
from schema.db_robot_position import DBRobotPosition
from schema.db_robot_path_position_relation import DBRobotPathPositionRelation


# 机器人初始点
def init_positions(robot_path_id):
    logger.info("初始化坐标点")
    db_robot_position = DBRobotPosition()
    before_params = {'current_state': "0"}
    db_robot_position.update({'current_state': "1"}, before_params)
    params = {'current_state': '1'}
    position_relation = DBRobotPathPositionRelation().get_position_id(robot_path_id)
    all_position_data = db_robot_position.get_battery_position()
    for position_data in all_position_data:
        if position_data.get("robot_position_id") in [i.get("robot_position_id") for i in position_relation]:
            robot_position_id = position_data.get("robot_position_id")
            db_robot_position.update({'robot_position_id': robot_position_id}, params)


def judge_location(robot_path_id):
    position_data = DBRobotPosition().get_position_name(robot_path_id)
    if not position_data:
        return False
    return True

