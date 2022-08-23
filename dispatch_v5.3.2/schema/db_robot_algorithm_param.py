import time

from schema.db_base import DbBase
from schema.models import RobotAlgorithmParam
from utils.alchemy_encoder import parse_orm


class DBRobotAlgorithmParam(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotAlgorithmParam

    def get_by_id(self, param_id):
        query = self.session.query(self.obj). \
            filter(self.obj.robot_algorithm_param_id == param_id)
        result = query.all()
        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBRobotAlgorithmParam']
