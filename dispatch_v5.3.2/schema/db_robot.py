from schema.db_base import DbBase
from schema.models import Robot
from utils.alchemy_encoder import parse_orm


class DBRobot(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = Robot

    def get_core_room_id(self, robot_id):
        query = self.session.query(self.obj).filter(self.obj.robot_id == robot_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res[0].get("core_room_id")
        else:
            return []


__all__ = ['DBRobot']
