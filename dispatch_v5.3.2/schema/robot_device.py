from schema.db_base import DbBase
from schema.models import RobotDevice
from utils.alchemy_encoder import parse_orm


class DBRobotDevice(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotDevice

    def get_device_id(self, robot_device_id):
        query = self.session.query(self.obj).filter(self.obj.robot_device_id == robot_device_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            results = res[0]
            return results
        else:
            return ""


__all__ = ['DBRobotDevice']
