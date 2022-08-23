import time
from datetime import datetime

from sqlalchemy import desc
from schema.db_base import DbBase
from schema.models import RobotTaskType
from utils.alchemy_encoder import parse_orm


class DBRobotTaskType(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotTaskType

    def get_data_by_id(self, robot_task_type_id):
        result = self.session.query(self.obj). \
            filter(self.obj.robot_task_type_id == robot_task_type_id).all()
        self.commit()

        results = parse_orm(result)
        if results:
            return results[0]
        return {}


__all__ = ['DBRobotTaskType']

