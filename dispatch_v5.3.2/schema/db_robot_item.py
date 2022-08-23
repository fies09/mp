import time

from schema.db_base import DbBase
from schema.models import RobotItem
from utils.alchemy_encoder import parse_orm


class DBRobotItem(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotItem

    def get_item_name(self, name):
        query = self.session.query(self.obj).filter(self.obj.name == name)
        result = query.all()

        if result:
            results = parse_orm(result)
            self.commit()
            return results[0]
        return {}


__all__ = ['DBRobotItem']
