import time

from sqlalchemy import desc
from schema.db_base import DbBase
from schema.models import RobotPtzParam
from utils.alchemy_encoder import parse_orm


class DBRobotPtzParam(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotPtzParam

    def get_yuntai_preset_num(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_ptz_param_id == id)
        result = query.all()
        self.commit()

        results = parse_orm(result)
        return results


__all__ = ['DBRobotPtzParam']
