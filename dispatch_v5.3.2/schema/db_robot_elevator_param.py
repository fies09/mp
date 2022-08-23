import time

from sqlalchemy import desc
from schema.db_base import DbBase
from schema.models import RobotElevatorParam
from utils.alchemy_encoder import parse_orm


class DBRobotElevatorParam(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotElevatorParam

    def get_robot_elevator_high(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_elevator_param_id == id)
        result = query.all()
        self.commit()

        results = parse_orm(result)
        return results

    def get_robot_elevator_detail(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_elevator_param_id == id)
        result = query.all()
        self.commit()

        if result:
            results = parse_orm(result)
            self.commit()
            return results[0]
        return {}

__all__ = ['DBRobotElevatorParam']
