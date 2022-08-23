import time

from sqlalchemy import desc
from schema.db_base import DbBase
from schema.models import RobotDeviceParam
from utils.alchemy_encoder import parse_orm


class DBRobotDeviceParam(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotDeviceParam

    def get_robot_ptz_location(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_device_param_id == id)
        result = query.all()
        self.commit()

        results = parse_orm(result)
        return results

    def get_device_param(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_device_param_id == id)
        result = query.all()
        self.commit()
        if result:
            results = parse_orm(result)
            return results[0]
        return {}


__all__ = ['DBRobotDeviceParam']
