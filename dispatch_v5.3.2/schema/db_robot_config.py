import time
from datetime import datetime

from schema.db_base import DbBase
from schema.models import RobotConfig
from utils.alchemy_encoder import parse_orm


class DBRobotConfig(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotConfig

    def get_battery(self):
        query = self.session.query(self.obj)
        results = parse_orm(query.all())
        self.commit()

        return results

    def get_mini_charge(self):
        query = self.session.query(self.obj) \
            .filter(self.obj.robot_id == 1)

        result = query.all()

        self.commit()
        if result:
            results = parse_orm(result)
            return results
        return {}

    def update(self, condition, info):
        try:
            results = self.judge_exist(condition)

            for result in results:
                if info.get('tags'):
                    result.tags = info.get('tags')
                result.update_time = datetime.now()

                self.commit()

            return results

        except:
            pass

    def get_height(self):
        query = self.session.query(self.obj)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            results = res[0].get("tags")
            return results
        else:
            return {}


__all__ = ['DBRobotConfig']
