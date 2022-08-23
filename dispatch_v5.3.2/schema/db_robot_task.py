import time
from datetime import datetime

from sqlalchemy import desc
from schema.db_base import DbBase
from schema.models import RobotTask
from utils.alchemy_encoder import parse_orm


class DBRobotTask(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotTask

    def insert(self, info):
        data = RobotTask(
            robot_id=info.get("robot_id", 1),
            robot_task_type_id=info.get("robot_task_type_id", 0),
            robot_path_id=info.get("robot_path_id", 0),
            num=info.get("num"),
            status=info.get("status", 0),
            del_flag=info.get("del_flag", 0),
            create_time=info.get("create_time", datetime.now()),
        )
        self.session.add(data)
        self.commit()

        return True

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:
            if info.get('status'):
                result.status = info.get('status')
            if info.get('update_time'):
                result.update_time = datetime.now()

            self.commit()

        return results

    def get_desc_top1(self):
        query = self.session.query(self.obj)
        result = query.order_by(desc(self.obj.robot_task_id)).limit(1).all()
        self.commit()
        results = parse_orm(result)

        return results


__all__ = ['DBRobotTask']
