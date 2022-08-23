from sqlalchemy import or_

from schema.db_base import DbBase
from schema.models import RobotPathPositionItem
from utils.alchemy_encoder import parse_orm


class DBRobotPathPositionItem(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotPathPositionItem

    def get_robot_position_id(self, ids):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == ids)
        result = query.all()
        self.commit()

        if result:
            results = parse_orm(result)
            return results
        return None

    def get_robot_elevator_param(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == id) \
            .filter(self.obj.robot_elevator_param_id != 0) \
            .filter(self.obj.name != "二维码") \
            .order_by(self.obj.robot_elevator_param_id)

        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results
        return None

    def get_robot_ptz_param(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == id) \
            .filter(self.obj.robot_ptz_param_id != 0) \
            .order_by(self.obj.robot_ptz_param_id)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results
        return None

    def get_robot_rfid_param(self, id, query_name:str):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == id) \
            .filter(self.obj.name == query_name)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results
        return None

    def get_door_param(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == id) \
            .filter(or_(self.obj.name == "开", self.obj.name == "关"))
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results
        return None


__all__ = ['DBRobotPathPositionItem']
