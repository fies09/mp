from operator import and_

from schema.db_base import DbBase
from schema.models import PowerRobotItem
from utils.alchemy_encoder import parse_orm


class DBPowerRobotItem(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerRobotItem

    def get_item_data(self, power_robot_item_id):
        query = self.session.query(self.obj).filter(self.obj.power_robot_item_id == power_robot_item_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res[0]
        else:
            return ""

    def get_item_id(self, robot_algorithm_id, name):
        query = self.session.query(self.obj).filter(self.obj.robot_algorithm_id == robot_algorithm_id,
                                                    self.obj.name == name)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res[0]
        else:
            return ""

    def get_model_item_id(self, robot_algorithm_id, config_algorithm_model_id):
        query = self.session.query(self.obj).filter(self.obj.robot_algorithm_id == robot_algorithm_id,
                                                    self.obj.config_algorithm_model_id == config_algorithm_model_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res[0]
        else:
            return ""


__all__ = ['DBPowerRobotItem']
