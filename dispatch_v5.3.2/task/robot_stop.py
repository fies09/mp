from modules.move_stop import Stop_move
# from robot_action_voice import push_voice
from core.redis_interactive import RedisPipe
from modules.init_node import state_init
from schema.db_inspect_project_detail import DBInspectProjectDetail


def robot_stop(task_data):
    Stop_move()
    update_cmd_status(task_data, 2)
    RedisPipe("ActionTask").set_data({"ActionTask": 0})


def update_cmd_status(task_data, state):
    params = {}
    params['status'] = state

    inspect_project_detail_id = task_data.get("inspect_project_detail_id")
    DBInspectProjectDetail().update({'inspect_project_detail_id': inspect_project_detail_id}, params)


if __name__ == '__main__':
    state_init()
    robot_stop({})
