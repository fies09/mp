import time

from schema.db_base import DbBase
from schema.models import RobotAlgorithm
from utils.alchemy_encoder import parse_orm


class DBRobotAlgorithm(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotAlgorithm

    def get_type_data(self, robot_algorithm_id):
        query = self.session.query(self.obj). \
            filter(self.obj.robot_algorithm_id == robot_algorithm_id)
        result = query.all()
        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_name_data(self, name):
        query = self.session.query(self.obj). \
            filter(self.obj.name == name)
        result = query.all()
        self.commit()

        if result:
            results = parse_orm(result)
            results_id = results[0].get("robot_algorithm_id")
            return results_id
        return -1


__all__ = ['DBRobotAlgorithm']
