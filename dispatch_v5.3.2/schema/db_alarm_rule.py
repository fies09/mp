from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmRule
from utils.alchemy_encoder import parse_orm


class DBAlarmRule(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmRule

    def get_by_rule_id(self, rule_id):
        query = self.session.query(self.obj).filter(self.obj.rule_id == rule_id,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_by_device_num(self, device_num):
        query = self.session.query(self.obj).filter(self.obj.device_num == device_num,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBAlarmRule']
